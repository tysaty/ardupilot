"""The SITL campaign runner, mirroring ``Py_Sweep_Experiment`` (``TASK-052``).

One command plans a SITL campaign **from an existing Python campaign**, one
runs it cell by cell (resumable, ``N`` repeats, one ``autotest.py`` process
per cell with a timeout), one aggregates it to ``master.csv`` with the Python
columns, and one writes the Python-versus-SITL comparison. ``--sub wind`` is
the same runner with a wind axis over the same cells (`TASK-052` D6,
`TASK-051`). From ``Tools/autotest``::

    python3 -m kangaroo_follow.campaign --plan --from <repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst --reference --repeats 3
    python3 -m kangaroo_follow.campaign --dry-run --from ...          # no SITL needed
    python3 -m kangaroo_follow.campaign --all --from ... [--resume] [--only ID ...]
    python3 -m kangaroo_follow.campaign --aggregate --from ...
    python3 -m kangaroo_follow.campaign --compare --from ...
    python3 -m kangaroo_follow.campaign --plan --sub wind --from ... --reference

``--all`` flies each cell through the plane test suite itself::

    KANGAROO_FOLLOW_PLAN=<cell>/plan.json ./autotest.py test.Plane.KangarooFollowCell

which is the ``AutoTestPlane.KangarooFollowCell`` test in ``arduplane.py``,
beside the mid-term ``DubinsSweep``; ``test.Plane.KangarooFollowCampaign``
flies every planned cell of a manifest in one SITL process (reboot per cell)
when the campaign is driven from ``autotest.py`` directly.

Layout, beside the Python campaign it repeats::

    <python campaign>/sitl/            --sub main   MANIFEST.json
    <python campaign>/sitl-wind/       --sub wind   MANIFEST-wind.json
        <cell>-r<k>/                   or <cell>-wind-<N|E|S|W|calm>-r<k>/
            plan.json result.json spec.json log.bin
            record.json history.json series.json ticks.csv sitl_extras.json
        master.csv compare.csv [wind-frame.csv wind-compare.csv]

The spec is the input (P4): a SITL cell is the Python cell's ``spec.json``
and its ``legs_flown``, nothing re-typed. A cell whose arm has no Lua entry
point is planned as ``unflyable`` with the reason: a finding, never a fix
(`VR-014`). A failed or timed-out cell is a bundle with ``partial = true``
or a manifest row with its error, never an absent row (`VR-012`).
"""

import argparse
import datetime
import json
import os
import subprocess
import sys
import time

from . import paths, schedule, stage_scripts, check_env, extract_bundle, wind_frame

from py_harness import experiment, plotter
from py_harness import Py_Sweep_Experiment as sweep

SUB_MAIN = "main"
SUB_WIND = "wind"
SUBS = (SUB_MAIN, SUB_WIND)

STATUS_PLANNED = "planned"
STATUS_COMPLETE = "complete"
STATUS_PARTIAL = "partial"
STATUS_ERROR = "error"
STATUS_UNFLYABLE = "unflyable"
STATUS_DRY_RUN_OK = "dry_run_ok"

#: `TASK-046` D1 recommendation, until answered.
DEFAULT_REPEATS = 3
#: `TASK-051` D1 proposal (0.4 V at V = 25 m/s) and D3 (no turbulence).
DEFAULT_WIND_SPD_MS = 10.0
DEFAULT_WIND_TURB = 0.0
#: The direction the wind blows FROM (libraries/SITL/SITL.cpp), asserted by
#: the wind-frame closure check rather than assumed.
WIND_DIRECTIONS = (("calm", None), ("N", 0.0), ("E", 90.0), ("S", 180.0),
                   ("W", 270.0))
#: Start-up plus the window: the cell subprocess is killed past this.
CELL_OVERHEAD_S = 420.0
DEFAULT_ALT_M = 100.0

SITL_COLUMNS = (
    "sitl_python_cell_id", "sitl_repeat", "sitl_wind_label", "sitl_wind_spd_ms",
    "sitl_wind_dir_from_deg", "sitl_wind_turb", "sitl_speedup",
    "sitl_ardupilot_commit", "sitl_record_source", "sitl_heading_source",
    "sitl_start_heading_error_deg", "sitl_tick_mean_s", "sitl_tick_max_s",
    "sitl_wall_clock_s", "sitl_error",
)
MASTER_COLUMNS = sweep.MASTER_COLUMNS + SITL_COLUMNS

COMPARE_METRICS = (
    ("rms_ring_error_m", ("ring", "target", "rms_ring_error_m")),
    ("max_ring_error_m", ("ring", "target", "max_ring_error_m")),
    ("mean_radius_m", ("ring", "target", "mean_radius_m")),
    ("t_contact_s", ("post_contact", "target", "t_contact_s")),
    ("post_contact_mean_radial_m", ("post_contact", "target", "mean_radial_m")),
    ("plane_velocity_error_rms_ms", ("velocity", "plane", "rms_ms")),
    ("max_curvature_1pm", ("max_curvature_1pm",)),
    ("zone_plane_breaches", ("zone", "plane_breaches")),
    ("zone_plane_max_depth_m", ("zone", "plane_max_depth_m")),
)


# --------------------------------------------------------------------------
# Paths and manifest
# --------------------------------------------------------------------------

def sitl_dir(python_dir, sub=SUB_MAIN):
    return os.path.join(python_dir, "sitl" if sub == SUB_MAIN else "sitl-wind")


def manifest_path(out_dir, sub=SUB_MAIN):
    return os.path.join(out_dir, "MANIFEST.json" if sub == SUB_MAIN
                        else "MANIFEST-wind.json")


def load_manifest(out_dir, sub=SUB_MAIN):
    path = manifest_path(out_dir, sub)
    if not os.path.isfile(path):
        return None
    with open(path) as handle:
        return json.load(handle)


def save_manifest(manifest, out_dir, sub=SUB_MAIN):
    os.makedirs(out_dir, exist_ok=True)
    path = manifest_path(out_dir, sub)
    with open(path, "w") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    return path


def _utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def python_cells(python_dir):
    """Every complete Python cell across the campaign's manifests, as
    ``{cell_id: (manifest_dir, entry)}``. The main manifest first, then the
    composite subs, so an id names one cell."""
    out = {}
    for sub_dir in ("", "composite", "composite-box350"):
        directory = os.path.join(python_dir, sub_dir) if sub_dir else python_dir
        manifest = sweep.load_manifest(directory)
        if manifest is None:
            continue
        for cid, entry in manifest["cells"].items():
            if cid in out:
                continue
            if entry.get("bundle") and entry.get("status") == "complete":
                out[cid] = (directory, entry)
    return out


def reference_subset(python_dir, cells):
    """`TASK-052` first run: ``<arm>-point``, ``<arm>-straight-constant-half``
    for the baseline arm, then the composite cell at ratio 0.5 per arm."""
    manifest = sweep.load_manifest(python_dir)
    arms = list((manifest or {}).get("arms", {}).keys())
    if not arms:
        return []
    baseline = arms[0]
    wanted = ["%s-point" % baseline, "%s-straight-constant-half" % baseline]
    wanted += ["%s-composite-half" % arm for arm in arms]
    return [c for c in wanted if c in cells]


def lua_flyable():
    """Algorithm names the Lua registry can fly, from the registry itself."""
    from py_harness import luadiff
    sandbox = luadiff.LuaSandbox()
    sandbox.execute('package.path = "%s/?.lua;" .. package.path'
                    % paths.LUA_DIR.replace("\\", "/"))
    names = sandbox.call("sitl_arms", "flyable")
    reasons = sandbox.eval('require("sitl_arms").NOT_PORTED')
    return set(names), dict(reasons or {})


def wind_cases(sub, speed_ms, turb):
    if sub != SUB_WIND:
        return [None]
    return [{"label": label, "spd_ms": 0.0 if deg is None else speed_ms,
             "dir_from_deg": 0.0 if deg is None else deg, "turb": 0.0 if deg is None else turb}
            for label, deg in WIND_DIRECTIONS]


def cell_id_for(python_cell_id, repeat, wind=None):
    if wind is None:
        return "%s-r%d" % (python_cell_id, repeat)
    return "%s-wind-%s-r%d" % (python_cell_id, wind["label"], repeat)


def plan(python_dir, sub=SUB_MAIN, only=None, reference=False,
         repeats=DEFAULT_REPEATS, wind_speed_ms=DEFAULT_WIND_SPD_MS,
         wind_turb=DEFAULT_WIND_TURB, speedup=1, location=None,
         heading_source=schedule.DEFAULT_HEADING_SOURCE, alt_m=DEFAULT_ALT_M):
    """Write the SITL manifest from the Python campaign. Runs nothing."""
    cells = python_cells(python_dir)
    if not cells:
        raise SystemExit("no complete Python cells under %s" % python_dir)
    if only:
        missing = [c for c in only if c not in cells]
        if missing:
            raise SystemExit("not complete Python cells: %s" % ", ".join(missing))
        selected = list(only)
    elif reference:
        selected = reference_subset(python_dir, cells)
    else:
        selected = sorted(cells)
    flyable, not_ported = lua_flyable()
    env = check_env.load_environment()
    out_dir = sitl_dir(python_dir, sub)
    manifest = load_manifest(out_dir, sub) or {
        "campaign": "SITL-001-%s" % os.path.basename(os.path.normpath(python_dir)),
        "task": "TASK-052",
        "sub": sub,
        "python_campaign": paths.rel(python_dir),
        "module": "Tools/autotest/kangaroo_follow.campaign; test.Plane.KangarooFollowCell",
        "created_utc": _utc(),
        "not_a_flight_configuration": (
            "SITL test configuration (SR-004). No value here is an approved "
            "flight-safety limit; the SITL result is tracked evidence for "
            "A-VAL-001, not flight evidence."),
        "cells": {},
    }
    manifest.update({
        "planned_utc": _utc(),
        "repeats": repeats,
        "speedup": speedup,
        "heading_source": heading_source,
        "alt_m": alt_m,
        "location": location or env["location"]["name"],
        "param_file": paths.rel(paths.PARAM_FILE),
        "ardupilot_commit_pinned": env["ardupilot"]["commit"],
        "wind": None if sub != SUB_WIND else {
            "spd_ms": wind_speed_ms, "turb": wind_turb,
            "directions_from_deg": dict((l, d) for l, d in WIND_DIRECTIONS),
            "convention": "SIM_WIND_DIR is the direction the wind blows from "
                          "(libraries/SITL/SITL.cpp); asserted by wind_frame.py",
        },
        "start_pose_tolerances": {"heading_deg": 5.0, "airspeed_ms": 3.0},
        "selected": selected,
    })
    for pid in selected:
        directory, entry = cells[pid]
        algorithm = entry["algorithm"]
        for wind in wind_cases(sub, wind_speed_ms, wind_turb):
            for k in range(1, repeats + 1):
                cid = cell_id_for(pid, k, wind)
                existing = manifest["cells"].get(cid)
                if existing and existing.get("status") in (STATUS_COMPLETE, STATUS_PARTIAL):
                    continue
                row = {
                    "status": STATUS_PLANNED,
                    "python_cell_id": pid,
                    "python_dir": paths.rel(directory),
                    "python_spec": entry["spec"],
                    "python_bundle": entry["bundle"],
                    "sub": entry["sub"], "arm_set": entry.get("arm_set"),
                    "arm": entry["arm"], "algorithm": algorithm,
                    "mode_base": entry["mode_base"], "mode_pace": entry["mode_pace"],
                    "ratio_name": entry["ratio_name"], "speed_ratio": entry["speed_ratio"],
                    "target_speed_ms": entry["target_speed_ms"], "seed": entry["seed"],
                    "n_a_max_steps": entry.get("n_a_max_steps"),
                    "feasible": entry.get("feasible"), "unreachable": entry.get("unreachable"),
                    "repeat": k, "wind": wind,
                    "bundle": None, "bin": None, "partial": None,
                    "wall_clock_s": None, "error": None,
                }
                if algorithm not in flyable:
                    row["status"] = STATUS_UNFLYABLE
                    row["error"] = not_ported.get(
                        algorithm, "no Lua entry point for %s" % algorithm)
                manifest["cells"][cid] = row
    path = save_manifest(manifest, out_dir, sub)
    return manifest, path


# --------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------

def _python_bundle(python_dir, entry):
    directory = os.path.join(paths.REPO_ROOT, entry["python_dir"])
    with open(os.path.join(directory, entry["python_spec"])) as handle:
        spec = json.load(handle)
    with open(os.path.join(directory, entry["python_bundle"], "record.json")) as handle:
        record = json.load(handle)
    return spec, record, directory


def _provenance(manifest, entry, staging, env_rows, allow_commit, result=None):
    live_commit = next((r["live"] for r in env_rows if r["what"] == "ArduPilot commit"), None)
    dirty = next((r["live"] == "dirty" for r in env_rows if r["what"] == "ArduPilot tree clean"), None)
    return {
        "campaign": manifest["campaign"],
        "sub": manifest["sub"],
        "python_cell_id": entry["python_cell_id"],
        "python_bundle": os.path.join(entry["python_dir"], entry["python_bundle"]),
        "repeat": entry["repeat"],
        "wind": entry.get("wind"),
        "speedup": manifest["speedup"],
        "location": manifest["location"],
        "alt_m": manifest["alt_m"],
        "heading_source": manifest["heading_source"],
        "ardupilot_commit": live_commit,
        "ardupilot_commit_pinned": manifest["ardupilot_commit_pinned"],
        "ardupilot_allow_commit": bool(allow_commit),
        "ardupilot_dirty": dirty,
        "param_file": manifest["param_file"],
        "param_file_sha256": stage_scripts.sha256(paths.PARAM_FILE),
        "lua_sha256": (staging or {}).get("hashes"),
        "legs_source": (staging or {}).get("cell_table", {}).get("legs_source"),
        "python_provenance": experiment.provenance(),
        "driver": None if result is None else {
            k: result.get(k) for k in ("status", "reason", "start_pose", "timings",
                                       "shr_done", "shr_t_s", "shr_tick",
                                       "tolerances", "statustexts")},
        "wall_clock_utc": _utc(),
    }


def _cell_params(manifest, entry):
    params = {"SHR_ALT_M": manifest["alt_m"], "SHR_REPORT": 5}
    wind = entry.get("wind")
    if wind is not None:
        params.update({"SIM_WIND_SPD": wind["spd_ms"], "SIM_WIND_DIR": wind["dir_from_deg"],
                       "SIM_WIND_TURB": wind["turb"]})
    return params


def run_cell(cid, entry, manifest, out_dir, dry_run=False, allow_commit=False,
             timeout_s=None, sandbox=None, progress=print):
    """Stage, fly (or dry-run) and extract one cell; return the updated entry."""
    t_wall = time.time()
    cell_dir = os.path.join(out_dir, cid)
    os.makedirs(cell_dir, exist_ok=True)
    spec, py_record, _py_dir = _python_bundle(manifest, entry)
    legs = py_record["kangaroo"]["legs_flown"]
    experiment.save_spec(spec, os.path.join(cell_dir, "spec.json"))

    env_rows = check_env.run_checks(dry_run=dry_run, allow_commit=allow_commit,
                                    location_name=manifest["location"])
    summary = check_env.summarise(env_rows)
    if not summary["ok"]:
        entry.update({"status": STATUS_ERROR,
                      "error": "environment: " + "; ".join(
                          r["what"] for r in env_rows if r["level"] == check_env.FAIL)})
        return entry

    duration_s = float(spec["run"]["duration_s"])
    plan_doc = {
        "cell_id": cid,
        "location": manifest["location"],
        "frame": "plane",
        # Paths relative to the repository root (P2): a plan.json under
        # experiments/ must not carry a machine's home directory.
        "binary": paths.rel(paths.SITL_BINARY),
        "param_file": paths.rel(paths.PARAM_FILE),
        "params": _cell_params(manifest, entry),
        "alt_m": manifest["alt_m"],
        "plane_heading_deg": float(spec["initial_conditions"].get("plane_heading_deg", 0.0)),
        "airspeed_ms": float(spec["aircraft"]["airspeed_ms"]),
        "duration_s": duration_s,
        "margin_s": 30.0,
        "speedup": manifest["speedup"],
        "logs_dir": paths.rel(os.path.join(cell_dir, "autotest-logs")),
        "bin_out": paths.rel(os.path.join(cell_dir, "log.bin")),
        "result_out": paths.rel(os.path.join(cell_dir, "result.json")),
    }
    with open(os.path.join(cell_dir, "plan.json"), "w") as handle:
        json.dump(plan_doc, handle, indent=2)
        handle.write("\n")

    staging = None
    result = None
    try:
        staging = stage_scripts.stage(spec, cid, heading_source=manifest["heading_source"])
        # The schedule the vehicle carries must be the Python cell's, tick for
        # tick, against the recorded history, before anything flies.
        if sandbox is not None:
            _name, history, _meta = plotter.load_run(os.path.join(
                paths.REPO_ROOT, entry["python_dir"], entry["python_bundle"], "history.json"))
            schedule.check_against_lua(spec, sandbox, legs=legs, history=history)
        # Regenerate the staged cell module with the legs actually flown.
        schedule.write_cell_module(
            spec, cid, os.path.join(paths.MODULES_DIR, paths.CELL_MODULE),
            manifest["heading_source"], legs=legs)
        staging["hashes"]["modules/%s" % paths.CELL_MODULE] = stage_scripts.sha256(
            os.path.join(paths.MODULES_DIR, paths.CELL_MODULE))
        staging["cell_table"]["legs_source"] = "legs_flown"
        if dry_run:
            entry.update({"status": STATUS_DRY_RUN_OK, "error": None,
                          "wall_clock_s": time.time() - t_wall,
                          "provenance": _provenance(manifest, entry, staging, env_rows,
                                                    allow_commit)})
            return entry
        timeout = timeout_s or (duration_s / max(1, manifest["speedup"]) + CELL_OVERHEAD_S)
        # The cell is flown by the plane test suite: autotest.py resolves
        # the built binary itself and runs the one named test.
        cmd = [sys.executable, paths.AUTOTEST_PY, "--no-clean", "--no-configure",
               "--speedup", str(manifest["speedup"]),
               "test.Plane.KangarooFollowCell"]
        env = dict(os.environ)
        env[paths.PLAN_ENV] = os.path.join(cell_dir, "plan.json")
        progress("  flying %s (timeout %.0f s)" % (cid, timeout))
        with open(os.path.join(cell_dir, "driver.log"), "w") as log:
            try:
                proc = subprocess.run(cmd, cwd=paths.AUTOTEST_DIR, stdout=log,
                                      stderr=subprocess.STDOUT, timeout=timeout,
                                      env=env)
                rc = proc.returncode
            except subprocess.TimeoutExpired:
                rc = None
        result_path = os.path.join(paths.REPO_ROOT, plan_doc["result_out"])
        bin_path = os.path.join(paths.REPO_ROOT, plan_doc["bin_out"])
        if os.path.isfile(result_path):
            with open(result_path) as handle:
                result = json.load(handle)
        if result is None:
            result = {"status": STATUS_PARTIAL if rc is None else STATUS_ERROR,
                      "reason": ("killed at the cell timeout" if rc is None
                                 else "driver exited %s with no result.json" % rc),
                      "bin": plan_doc["bin_out"] if os.path.isfile(bin_path) else None}
    except (schedule.ScheduleError, RuntimeError, FileNotFoundError) as exc:
        entry.update({"status": STATUS_ERROR, "error": str(exc),
                      "wall_clock_s": time.time() - t_wall})
        return entry
    finally:
        stage_scripts.restore()

    provenance = _provenance(manifest, entry, staging, env_rows, allow_commit, result)
    bin_path = (os.path.join(paths.REPO_ROOT, result["bin"])
                if result.get("bin") else None)
    entry["bin"] = result.get("bin")
    if bin_path and os.path.isfile(bin_path):
        try:
            extract_bundle.extract(bin_path, spec, cell_dir,
                                   cell=_identity(entry), sitl_block=provenance,
                                   legs=legs, render=True)
            with open(os.path.join(cell_dir, "record.json")) as handle:
                record = json.load(handle)
            entry["bundle"] = cid
            entry["partial"] = bool(record["metrics"].get("partial"))
            entry["status"] = (STATUS_PARTIAL if entry["partial"] or
                               result["status"] == STATUS_PARTIAL else STATUS_COMPLETE)
            entry["error"] = result.get("reason")
        except Exception as exc:
            entry.update({"status": STATUS_ERROR,
                          "error": "extraction: %s: %s" % (type(exc).__name__, exc)})
    else:
        entry.update({"status": STATUS_ERROR,
                      "error": result.get("reason") or "no log produced"})
    entry["wall_clock_s"] = time.time() - t_wall
    entry["driver_status"] = result.get("status")
    return entry


def _identity(entry):
    return {"arm": entry["arm"], "mode_base": entry["mode_base"],
            "mode_pace": entry["mode_pace"], "speed_ratio": entry["speed_ratio"],
            "seed": entry["seed"]}


def run_cells(manifest, out_dir, sub, only=None, resume=False, dry_run=False,
              allow_commit=False, timeout_s=None, progress=print):
    from py_harness import luadiff
    sandbox = luadiff.LuaSandbox()
    for cid in sorted(manifest["cells"]):
        entry = manifest["cells"][cid]
        if only and cid not in only:
            continue
        if entry["status"] == STATUS_UNFLYABLE:
            progress("  %s: unflyable (%s)" % (cid, entry["error"]))
            continue
        if resume and entry["status"] in (STATUS_COMPLETE, STATUS_PARTIAL):
            continue
        progress("%s %s" % ("dry-run" if dry_run else "run", cid))
        manifest["cells"][cid] = run_cell(cid, entry, manifest, out_dir, dry_run=dry_run,
                                          allow_commit=allow_commit, timeout_s=timeout_s,
                                          sandbox=sandbox, progress=progress)
        save_manifest(manifest, out_dir, sub)
        progress("  -> %s%s" % (manifest["cells"][cid]["status"],
                                 (" (%s)" % manifest["cells"][cid]["error"])
                                 if manifest["cells"][cid]["error"] else ""))
    return manifest


# --------------------------------------------------------------------------
# Aggregation and comparison
# --------------------------------------------------------------------------

def _sweep_entry(entry):
    """The Python manifest entry shape ``Py_Sweep_Experiment.master_row`` reads."""
    row = dict(entry)
    row.setdefault("arm_set", sweep.DEFAULT_ARM_SET)
    row.setdefault("replay_ok", None)
    row.setdefault("curvature_ok", None)
    return row


def master_row(cid, entry, record):
    row = sweep.master_row(cid, _sweep_entry(entry), record)
    sitl = (record or {}).get("sitl") or {}
    wind = entry.get("wind") or {}
    driver = sitl.get("driver") or {}
    pose = driver.get("start_pose") or {}
    tick = sitl.get("tick_spacing") or {}
    row.update({
        "sitl_python_cell_id": entry["python_cell_id"],
        "sitl_repeat": entry["repeat"],
        "sitl_wind_label": wind.get("label"),
        "sitl_wind_spd_ms": wind.get("spd_ms"),
        "sitl_wind_dir_from_deg": wind.get("dir_from_deg"),
        "sitl_wind_turb": wind.get("turb"),
        "sitl_speedup": sitl.get("speedup"),
        "sitl_ardupilot_commit": sitl.get("ardupilot_commit"),
        "sitl_record_source": sitl.get("record_source"),
        "sitl_heading_source": sitl.get("heading_source"),
        "sitl_start_heading_error_deg": pose.get("heading_error_deg"),
        "sitl_tick_mean_s": tick.get("mean_s"),
        "sitl_tick_max_s": tick.get("max_s"),
        "sitl_wall_clock_s": entry.get("wall_clock_s"),
        "sitl_error": entry.get("error"),
    })
    row["status"] = entry["status"]
    return row


def aggregate(manifest, out_dir, sub=SUB_MAIN):
    import csv
    rows = []
    bundles = []
    for cid in sorted(manifest["cells"]):
        entry = manifest["cells"][cid]
        record = None
        if entry.get("bundle"):
            path = os.path.join(out_dir, entry["bundle"], "record.json")
            if os.path.isfile(path):
                with open(path) as handle:
                    record = json.load(handle)
                bundles.append((cid, os.path.join(out_dir, entry["bundle"])))
        rows.append(master_row(cid, entry, record))
    path = os.path.join(out_dir, "master.csv")
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(MASTER_COLUMNS)
        for row in rows:
            writer.writerow([experiment._csv_cell(row.get(c)) for c in MASTER_COLUMNS])
    written = {"master": path}
    if sub == SUB_WIND and bundles:
        wf_rows = [wind_frame.analyse(d) for _cid, d in bundles]
        written["wind_frame"] = wind_frame.write_csv(
            wf_rows, os.path.join(out_dir, "wind-frame.csv"))
        py_records = {}
        for entry in manifest["cells"].values():
            pid = entry["python_cell_id"]
            if pid not in py_records:
                _spec, record, _d = _python_bundle(manifest, entry)
                py_records[pid] = record
        written["wind_compare"] = wind_frame.write_compare_csv(
            wind_frame.compare_table(wf_rows, py_records),
            os.path.join(out_dir, "wind-compare.csv"))
    return written, rows


def _get(d, *path):
    for key in path:
        if not isinstance(d, dict):
            return None
        d = d.get(key)
    return d


def compare(manifest, out_dir):
    """Python-versus-SITL per Python cell: value, SITL mean and range over
    repeats (calm cells only in the wind sub). Writes ``compare.csv``."""
    import csv
    groups = {}
    py_records = {}
    for cid, entry in manifest["cells"].items():
        if entry.get("wind") and entry["wind"]["label"] != "calm":
            continue
        pid = entry["python_cell_id"]
        if pid not in py_records:
            _spec, record, _d = _python_bundle(manifest, entry)
            py_records[pid] = record
        record = None
        if entry.get("bundle"):
            path = os.path.join(out_dir, entry["bundle"], "record.json")
            if os.path.isfile(path):
                with open(path) as handle:
                    record = json.load(handle)
        groups.setdefault(pid, []).append((entry, record))
    columns = ["python_cell_id", "arm", "algorithm", "n_planned", "n_with_record",
               "n_partial", "unflyable_reason"]
    for name, _path in COMPARE_METRICS:
        columns += ["python_" + name, "sitl_mean_" + name, "sitl_min_" + name,
                    "sitl_max_" + name]
    rows = []
    for pid in sorted(groups):
        entries = groups[pid]
        py = py_records[pid].get("metrics") or {}
        row = {"python_cell_id": pid, "arm": entries[0][0]["arm"],
               "algorithm": entries[0][0]["algorithm"], "n_planned": len(entries),
               "n_with_record": sum(1 for _e, r in entries if r),
               "n_partial": sum(1 for _e, r in entries
                                if r and (r.get("metrics") or {}).get("partial")),
               "unflyable_reason": next((e["error"] for e, _r in entries
                                         if e["status"] == STATUS_UNFLYABLE), None)}
        for name, path in COMPARE_METRICS:
            row["python_" + name] = _get(py, *path)
            values = [_get(r.get("metrics") or {}, *path) for _e, r in entries if r]
            values = [v for v in values if isinstance(v, (int, float))]
            row["sitl_mean_" + name] = sum(values) / len(values) if values else None
            row["sitl_min_" + name] = min(values) if values else None
            row["sitl_max_" + name] = max(values) if values else None
        rows.append(row)
    path = os.path.join(out_dir, "compare.csv")
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for row in rows:
            writer.writerow([experiment._csv_cell(row.get(c)) for c in columns])
    return path, rows


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--plan", action="store_true")
    action.add_argument("--dry-run", action="store_true",
                        help="plan if needed, then stage, check and generate every "
                             "cell without starting SITL")
    action.add_argument("--all", action="store_true")
    action.add_argument("--aggregate", action="store_true")
    action.add_argument("--compare", action="store_true")
    parser.add_argument("--from", dest="python_dir", required=True,
                        help="the Python campaign directory to repeat")
    parser.add_argument("--sub", default=SUB_MAIN, choices=SUBS)
    parser.add_argument("--only", nargs="+", default=None,
                        help="Python cell ids (plan) or SITL cell ids (run)")
    parser.add_argument("--reference", action="store_true",
                        help="plan the TASK-052 reference subset")
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS)
    parser.add_argument("--wind-speed-ms", type=float, default=DEFAULT_WIND_SPD_MS)
    parser.add_argument("--wind-turb", type=float, default=DEFAULT_WIND_TURB)
    parser.add_argument("--speedup", type=int, default=1)
    parser.add_argument("--location", default=None)
    parser.add_argument("--alt-m", type=float, default=DEFAULT_ALT_M)
    parser.add_argument("--heading-source", default=schedule.DEFAULT_HEADING_SOURCE,
                        choices=schedule.HEADING_SOURCES)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--allow-commit", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=None)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    python_dir = os.path.abspath(args.python_dir)
    out_dir = sitl_dir(python_dir, args.sub)
    if args.plan or (args.dry_run and load_manifest(out_dir, args.sub) is None):
        manifest, path = plan(python_dir, sub=args.sub,
                              only=None if args.dry_run and not args.plan else args.only,
                              reference=args.reference, repeats=args.repeats,
                              wind_speed_ms=args.wind_speed_ms, wind_turb=args.wind_turb,
                              speedup=args.speedup, location=args.location,
                              heading_source=args.heading_source, alt_m=args.alt_m)
        counts = {}
        for entry in manifest["cells"].values():
            counts[entry["status"]] = counts.get(entry["status"], 0) + 1
        print("planned %s: %s" % (paths.rel(path), ", ".join(
            "%d %s" % (n, s) for s, n in sorted(counts.items()))))
        if args.plan:
            return 0
    manifest = load_manifest(out_dir, args.sub)
    if manifest is None:
        raise SystemExit("no manifest at %s; run --plan first" % paths.rel(out_dir))
    if args.dry_run or args.all:
        run_cells(manifest, out_dir, args.sub, only=args.only, resume=args.resume,
                  dry_run=args.dry_run, allow_commit=args.allow_commit,
                  timeout_s=args.timeout_s)
        counts = {}
        for entry in manifest["cells"].values():
            counts[entry["status"]] = counts.get(entry["status"], 0) + 1
        print("done: %s" % ", ".join("%d %s" % (n, s) for s, n in sorted(counts.items())))
        return 0
    if args.aggregate:
        written, rows = aggregate(manifest, out_dir, args.sub)
        for name, path in written.items():
            print("%-12s %s" % (name, paths.rel(path)))
        return 0
    if args.compare:
        path, rows = compare(manifest, out_dir)
        print("wrote %s (%d Python cells)" % (paths.rel(path), len(rows)))
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
