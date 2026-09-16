"""From a SITL ``.bin`` to the ``TASK-040`` bundle (``TASK-052`` "record
extraction", ``TASK-046`` D5, ``TASK-052`` D3).

The runner logs the harness record quantities directly (``HREC``, ``HEST``,
``HALG``; ``HAIR`` for the wind frame), so extraction is a **column read**:
each ``HREC`` row is one history sample with the harness's field names and
units, ``HEST`` and ``HALG`` fill the estimate and ``algorithm_state``
columns, and the result is handed to the **unchanged** Python pipeline
(``experiment.write_bundle`` with ``metrics.py`` and ``series.py`` behind it).
A SITL bundle is therefore comparable to a Python one by construction
(`VR-011`); the fields SITL adds are appended under ``record["sitl"]`` and
``sitl_extras.json``, never substituted (`TASK-052` scope).

What is different, and recorded rather than hidden:

* **Time.** The harness is fixed-step; SITL's scheduler jitters. Samples are
  recorded at their logged ``t`` and the metrics use the spec's ``dt_s`` as
  they do for a Python bundle; the measured tick spacing is in the ``sitl``
  block so a reader can judge the approximation.
* **The stop.** A refusal (``SHR_DONE = 2``) is logged as a row with NaN
  guidance; it becomes ``stopped_reason`` and ``partial = true``, exactly the
  harness's "records the failure and stops" (`VR-012`).
* **No ``HREC`` at all** is a bundle with an empty history, ``partial = true``
  and the reason named: an absent record is never an absent row.
* **The replay check** is off (``verify=False``): a SITL run is not
  reproducible from its spec, which is the whole point of running it, and
  the bundle says so in ``replay_verification.why``.
"""

import argparse
import json
import math
import os

from . import paths

from py_harness import experiment, scenario
from py_harness import zone as zone_mod
from py_harness.config import HarnessConfig

RECORD_MESSAGES = ("HANC", "HREC", "HEST", "HALG", "HAIR")

PHASE_FROM_CODE = {0: "approach", 1: "orbit"}
DIR_FROM_CODE = {1: "cw", -1: "ccw"}

#: ``HREC`` columns -> history keys. Everything else the harness records is
#: filled from ``HEST``/``HALG`` or left ``None``.
HREC_FIELDS = (("PN", "plane_n_m"), ("PE", "plane_e_m"), ("PHdg", "plane_hdg_rad"),
               ("TN", "target_n_m"), ("TE", "target_e_m"),
               ("TVN", "target_vn_ms"), ("TVE", "target_ve_ms"),
               ("GN", "guidance_n_m"), ("GE", "guidance_e_m"))


class ExtractionError(RuntimeError):
    """The log cannot be read as a cell. The message names why."""


def _tkey(t):
    return int(round(float(t) * 1000.0))


def read_messages(path, types=RECORD_MESSAGES):
    """Every message of the given types, in log order, as plain dicts."""
    try:
        from pymavlink import DFReader
    except ImportError as exc:                          # pragma: no cover
        raise ExtractionError("pymavlink is required to read a .bin: %s" % exc)
    reader = DFReader.DFReader_binary(path, zero_time_base=True)
    out = dict((t, []) for t in types)
    while True:
        m = reader.recv_match(type=list(types))
        if m is None:
            break
        out[m.get_type()].append(m.to_dict())
    return out


def _finite(value):
    return value is not None and not (isinstance(value, float) and
                                      (math.isnan(value) or math.isinf(value)))


def history_from_messages(messages):
    """Build the harness history from the logged rows.

    Returns ``(history, stopped_reason, extras)``. ``extras`` carries the
    per-tick SITL-only columns (airspeed, ground velocity, EKF wind, yaw,
    roll, course) keyed by the same ``t`` for :mod:`kangaroo_follow.wind_frame`.
    """
    est_by_t = dict((_tkey(m["t"]), m) for m in messages.get("HEST", []))
    alg_by_t = dict((_tkey(m["t"]), m) for m in messages.get("HALG", []))
    air_by_t = dict((_tkey(m["t"]), m) for m in messages.get("HAIR", []))
    history = []
    extras = []
    stopped_reason = None
    for m in messages.get("HREC", []):
        t = float(m["t"])
        if not (_finite(m["GN"]) and _finite(m["GE"])):
            stopped_reason = ("no solution at t=%.1f s (runner refused; "
                              "SHR_DONE=2)" % t)
            break
        sample = {"t_s": t}
        for column, key in HREC_FIELDS:
            sample[key] = float(m[column])
        sample["infeasible"] = False
        alg = alg_by_t.get(_tkey(t))
        state = {}
        if alg is not None:
            phase = PHASE_FROM_CODE.get(int(round(alg["Ph"])))
            if phase is not None:
                state["phase"] = phase
            direction = DIR_FROM_CODE.get(int(round(alg["Dir"])))
            if direction is not None:
                state["direction"] = direction
            state["curvature"] = float(alg["Cur"])
            state["replanned"] = bool(round(alg["Rep"]))
            state["ticks_since_replan"] = int(round(alg["Tsr"]))
            state["sense_held"] = bool(round(alg["Held"]))
            state["sense_switched"] = bool(round(alg["Sw"]))
            if alg.get("K"):
                state["k_steps"] = int(round(alg["K"]))
            if alg.get("Ring"):
                state["ring_angle_rad"] = float(alg["Ring"])
        sample["algorithm_state"] = state
        est = est_by_t.get(_tkey(t))
        if est is not None:
            sample["target_est_n_m"] = float(est["EN"])
            sample["target_est_e_m"] = float(est["EE"])
            sample["target_est_raw_n_m"] = float(est["RN"])
            sample["target_est_raw_e_m"] = float(est["RE"])
            sample["target_est_raw_vn_ms"] = float(est["RVN"])
            sample["target_est_raw_ve_ms"] = float(est["RVE"])
        else:
            for key in ("target_est_n_m", "target_est_e_m", "target_est_raw_n_m",
                        "target_est_raw_e_m", "target_est_raw_vn_ms",
                        "target_est_raw_ve_ms"):
                sample[key] = None
        history.append(sample)
        air = air_by_t.get(_tkey(t))
        extras.append({
            "t_s": t,
            "airspeed_ms": float(air["AS"]) if air else None,
            "ground_vn_ms": float(air["GVN"]) if air else None,
            "ground_ve_ms": float(air["GVE"]) if air else None,
            "wind_n_ms": float(air["WN"]) if air else None,
            "wind_e_ms": float(air["WE"]) if air else None,
            "yaw_rad": float(air["Yaw"]) if air else None,
            "roll_rad": float(air["Roll"]) if air else None,
            "course_rad": float(air["Crs"]) if air else None,
        })
    if not history and stopped_reason is None:
        stopped_reason = "no HREC messages in the log: the runner never opened the window"
    return history, stopped_reason, extras


def anchor_from_messages(messages):
    rows = messages.get("HANC", [])
    if not rows:
        return None
    a = rows[0]
    return {"t0_ms": int(a["t0ms"]), "lat_deg": a["Lat"] * 1e-7,
            "lng_deg": a["Lng"] * 1e-7, "alt_m": float(a["Alt"]),
            "heading_rad": float(a["Hdg"]), "yaw_rad": float(a["Yaw"]),
            "alt_cmd_m": float(a["AltCmd"])}


def tick_spacing(history):
    if len(history) < 2:
        return {"samples": len(history), "mean_s": None, "max_s": None,
                "min_s": None}
    gaps = [b["t_s"] - a["t_s"] for a, b in zip(history, history[1:])]
    return {"samples": len(history), "mean_s": sum(gaps) / len(gaps),
            "max_s": max(gaps), "min_s": min(gaps)}


class RecordedSession:
    """What ``experiment.write_bundle`` needs from a session, from a recorded
    history: the same attributes ``scenario.ScenarioSession`` exposes, with the
    two report methods borrowed unchanged so the numbers are the harness's."""

    reconvergence = scenario.ScenarioSession.reconvergence
    zone_report = scenario.ScenarioSession.zone_report

    def __init__(self, spec, history, stopped_reason=None, legs=None):
        self.config = experiment.config_from_spec(spec)
        self.algorithm_name = spec["algorithm"]["name"]
        self.history = list(history)
        self.stopped_reason = stopped_reason
        self.t_s = self.history[-1]["t_s"] if self.history else 0.0
        self.legs = [experiment.leg_tuple(l) for l in
                     (legs if legs is not None else spec["kangaroo"]["legs"])]
        # The schedule's changes, in the shape the Python session records
        # them: every leg after the first is a change at its start time.
        self.change_log = []
        t = 0.0
        for i, leg in enumerate(self.legs):
            if i > 0:
                self.change_log.append({
                    "t_s": t, "duration_s": leg[0], "mode": leg[1],
                    "heading_deg": leg[2], "speed_ms": leg[3],
                    "source": "schedule"})
            t += leg[0]
        # SITL has no zone actor: the target is replayed, never turned, so
        # there are no containment events. The aircraft's breaches are still
        # measured against the Python zone.
        self.containment_events = []
        zone_spec = spec.get("zone") or {}
        self.zone = (None if zone_spec.get("side_m") is None
                     else zone_mod.InclusionZone(side_m=zone_spec["side_m"]))

    def export_legs(self):
        return list(self.legs)


def extract(bin_path, spec, directory, cell=None, sitl_block=None, legs=None,
            render=False):
    """Read ``bin_path`` and write the bundle into ``directory``.

    Args:
        bin_path: The DataFlash log the cell produced.
        spec: The cell's Python spec (copied, not re-typed).
        directory: Bundle directory.
        cell: Identity for the ``ticks.csv`` columns.
        sitl_block: Provenance to merge under ``record["sitl"]`` (ArduPilot
            commit, hashes, wind, repeat, ...). The extraction adds the
            anchor, tick spacing and the record source to it.
        legs: The legs the vehicle flew (``legs_flown``), so the bundle's
            ``kangaroo.legs_flown`` is the schedule that was carried.
        render: Draw the PNGs as a Python bundle would.

    Returns:
        ``{name: path}`` from ``experiment.write_bundle`` plus ``"extras"``.
    """
    messages = read_messages(bin_path)
    history, stopped_reason, extras = history_from_messages(messages)
    session = RecordedSession(spec, history, stopped_reason, legs=legs)
    block = dict(sitl_block or {})
    block.update({
        "record_source": "HREC/HEST/HALG logged by sitl_harness_runner.lua",
        "log": os.path.basename(bin_path),
        "anchor": anchor_from_messages(messages),
        "tick_spacing": tick_spacing(history),
        "messages": dict((k, len(v)) for k, v in messages.items()),
        "stopped_reason": stopped_reason,
    })
    written = experiment.write_bundle(
        spec, session, verify=False, render=render and bool(history),
        directory=directory, cell=cell, extra_record={"sitl": block})
    # The replay check is not "disabled by a flag" here; say what it is.
    record_path = written["record"]
    with open(record_path) as handle:
        record = json.load(handle)
    record["replay_verification"] = {
        "attempted": False,
        "why": "SITL run: not reproducible from the spec by construction "
               "(TASK-046 D1); repeats and their spread are the check",
    }
    with open(record_path, "w") as handle:
        json.dump(record, handle, indent=2, default=str)
        handle.write("\n")
    extras_path = os.path.join(directory, "sitl_extras.json")
    with open(extras_path, "w") as handle:
        json.dump({"schema_version": experiment.SCHEMA_VERSION,
                   "columns": extras}, handle)
        handle.write("\n")
    written["extras"] = extras_path
    return written


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bin")
    parser.add_argument("--spec", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args(argv)
    with open(args.spec) as handle:
        spec = json.load(handle)
    written = extract(args.bin, spec, args.out, render=args.render)
    for name, path in sorted(written.items()):
        print("%-10s %s" % (name, path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
