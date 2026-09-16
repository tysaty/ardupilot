"""Wind-frame analysis of a SITL bundle (``TASK-052`` ``--sub wind``;
``TASK-051`` D5 and the from/to convention).

Per wind bundle, from the recorded history and the SITL extras:

* **closure** -- ``v_air = v_ground - w`` reconstructed from the position
  differences (the harness's ground velocity) and the **commanded** wind
  vector, checked against the reported airspeed. It closes only if the
  ``SIM_WIND_DIR`` convention was read correctly (the direction the wind
  blows *from*), so the check settles the convention in writing: the row
  reports the closure under both readings and names the one that closes;
* **crab** -- course minus yaw around the ring;
* **groundspeed and ring radius by bearing relative to the wind** -- four
  bins (downwind of the target, upwind, and the two cross-wind sides), which
  is the ring deformation the wind imposes on a constant-airspeed follower
  (`A-ENV-002`), before any guidance error;
* **the harness quantities under wind** -- ``plane_velocity_error_ms`` (zero
  in the harness, the wind here) decomposed along and across the wind,
  contact time and zone breaches from the record.

Output: one row per bundle (:func:`analyse`), ``wind-frame.csv`` over a
campaign (:func:`write_csv`) and the planned-versus-actual table per
direction against the calm SITL cell and the Python cell
(:func:`compare_table`). Nothing here alters a metric the Python pipeline
already computed; it reads them.
"""

import csv
import json
import math
import os

from . import paths  # noqa: F401

from py_harness import experiment, plotter

#: Closure tolerance on the RMS of ``|v_ground - w| - airspeed``, m/s
#: (`TASK-051` D5 proposed here; recorded in every row).
CLOSURE_TOL_MS = 1.0

BINS = ("downwind", "cross_right", "upwind", "cross_left")

COLUMNS = (
    "cell_id", "python_cell_id", "wind_label", "wind_spd_ms", "wind_dir_from_deg",
    "wind_turb", "repeat", "samples", "closure_tol_ms",
    "closure_from_rms_ms", "closure_from_max_ms", "closure_to_rms_ms",
    "closure_ok", "convention",
    "ekf_wind_mean_n_ms", "ekf_wind_mean_e_ms", "ekf_wind_error_ms",
    "crab_mean_deg", "crab_rms_deg", "crab_max_abs_deg",
    "groundspeed_min_ms", "groundspeed_max_ms", "airspeed_mean_ms",
    "gs_downwind_ms", "gs_cross_right_ms", "gs_upwind_ms", "gs_cross_left_ms",
    "radius_downwind_m", "radius_cross_right_m", "radius_upwind_m",
    "radius_cross_left_m", "radius_spread_wind_m",
    "vel_err_rms_ms", "vel_err_along_wind_rms_ms", "vel_err_cross_wind_rms_ms",
    "rms_ring_error_m", "t_contact_s", "zone_plane_breaches",
    "zone_plane_max_depth_m", "partial",
)


def wind_vector(spd_ms, dir_deg, blows_from=True):
    """The wind's ``(n, e)`` velocity. ``blows_from`` is ArduPilot's
    ``SIM_WIND_DIR`` reading (0 = from the North, i.e. blowing South)."""
    r = math.radians(dir_deg)
    n, e = spd_ms * math.cos(r), spd_ms * math.sin(r)
    return (-n, -e) if blows_from else (n, e)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _rms(values):
    values = [v for v in values if v is not None]
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def _mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def load(directory):
    bundle = experiment.load_bundle(directory)
    _name, history, _meta = plotter.load_run(os.path.join(directory, "history.json"))
    extras = []
    path = os.path.join(directory, "sitl_extras.json")
    if os.path.isfile(path):
        with open(path) as handle:
            extras = json.load(handle).get("columns", [])
    return bundle, history, extras


def analyse(directory, closure_tol_ms=CLOSURE_TOL_MS):
    """One :data:`COLUMNS` row for the bundle at ``directory``."""
    bundle, history, extras = load(directory)
    record = bundle["record"]
    spec = bundle["spec"]
    sitl = record.get("sitl") or {}
    wind = sitl.get("wind") or {"spd_ms": 0.0, "dir_from_deg": 0.0, "turb": 0.0,
                                "label": "calm"}
    metrics = record.get("metrics") or {}
    airspeed_cmd = float(spec["aircraft"]["airspeed_ms"])
    R = float(spec["aircraft"]["orbit_radius_m"])
    row = dict((c, None) for c in COLUMNS)
    row.update({
        "cell_id": record.get("experiment_id"),
        "python_cell_id": sitl.get("python_cell_id"),
        "wind_label": wind.get("label"),
        "wind_spd_ms": wind.get("spd_ms"),
        "wind_dir_from_deg": wind.get("dir_from_deg"),
        "wind_turb": wind.get("turb"),
        "repeat": sitl.get("repeat"),
        "samples": len(history),
        "closure_tol_ms": closure_tol_ms,
        "partial": metrics.get("partial"),
        "rms_ring_error_m": ((metrics.get("ring") or {}).get("target") or {}).get(
            "rms_ring_error_m"),
        "t_contact_s": ((metrics.get("post_contact") or {}).get("target") or {}).get(
            "t_contact_s"),
        "zone_plane_breaches": (metrics.get("zone") or {}).get("plane_breaches"),
        "zone_plane_max_depth_m": (metrics.get("zone") or {}).get("plane_max_depth_m"),
        "vel_err_rms_ms": ((metrics.get("velocity") or {}).get("plane") or {}).get("rms_ms"),
    })
    if len(history) < 2:
        return row

    w_from = wind_vector(wind["spd_ms"], wind["dir_from_deg"], blows_from=True)
    w_to = wind_vector(wind["spd_ms"], wind["dir_from_deg"], blows_from=False)
    wind_to_bearing = math.atan2(w_from[1], w_from[0]) if wind["spd_ms"] > 0 else 0.0
    if wind["spd_ms"] > 0:
        w_unit = (w_from[0] / wind["spd_ms"], w_from[1] / wind["spd_ms"])
    else:
        w_unit = (1.0, 0.0)
    w_cross = (-w_unit[1], w_unit[0])

    by_t = dict((round(x["t_s"], 3), x) for x in extras)
    closure_from, closure_to, crab, gs_all, as_all = [], [], [], [], []
    ekf_n, ekf_e = [], []
    err_along, err_cross, err_all = [], [], []
    contact_tick = ((metrics.get("post_contact") or {}).get("target") or {}).get(
        "contact_tick")
    bins = dict((b, {"gs": [], "r": []}) for b in BINS)

    prev = history[0]
    for i in range(1, len(history)):
        s = history[i]
        dt = s["t_s"] - prev["t_s"]
        if dt <= 0:
            prev = s
            continue
        vn = (s["plane_n_m"] - prev["plane_n_m"]) / dt
        ve = (s["plane_e_m"] - prev["plane_e_m"]) / dt
        gs = math.hypot(vn, ve)
        gs_all.append(gs)
        x = by_t.get(round(s["t_s"], 3))
        aspd = x.get("airspeed_ms") if x else None
        if aspd is not None:
            as_all.append(aspd)
            closure_from.append(math.hypot(vn - w_from[0], ve - w_from[1]) - aspd)
            closure_to.append(math.hypot(vn - w_to[0], ve - w_to[1]) - aspd)
        if x and x.get("yaw_rad") is not None and x.get("course_rad") is not None:
            crab.append(math.degrees(_wrap(x["course_rad"] - x["yaw_rad"])))
        if x and x.get("wind_n_ms") is not None:
            ekf_n.append(x["wind_n_ms"])
            ekf_e.append(x["wind_e_ms"])
        # velocity error vector, the harness definition (V along the recorded
        # heading), decomposed on the wind axes
        cn = airspeed_cmd * math.cos(s["plane_hdg_rad"])
        ce = airspeed_cmd * math.sin(s["plane_hdg_rad"])
        en, ee = vn - cn, ve - ce
        err_all.append(math.hypot(en, ee))
        err_along.append(en * w_unit[0] + ee * w_unit[1])
        err_cross.append(en * w_cross[0] + ee * w_cross[1])
        # ring bins relative to the wind, post contact
        if contact_tick is not None and i >= contact_tick:
            dn, de = s["plane_n_m"] - s["target_n_m"], s["plane_e_m"] - s["target_e_m"]
            bearing = math.atan2(de, dn)
            rel = _wrap(bearing - wind_to_bearing)
            idx = int(((rel + math.pi / 4) % (2 * math.pi)) // (math.pi / 2))
            b = BINS[idx]
            bins[b]["gs"].append(gs)
            bins[b]["r"].append(math.hypot(dn, de))
        prev = s

    row["closure_from_rms_ms"] = _rms(closure_from)
    row["closure_from_max_ms"] = max((abs(c) for c in closure_from), default=None)
    row["closure_to_rms_ms"] = _rms(closure_to)
    if row["closure_from_rms_ms"] is not None:
        row["closure_ok"] = row["closure_from_rms_ms"] <= closure_tol_ms
        if wind["spd_ms"] > 0 and row["closure_to_rms_ms"] is not None:
            row["convention"] = ("from" if row["closure_from_rms_ms"] <=
                                 row["closure_to_rms_ms"] else "to")
        else:
            row["convention"] = "calm"
    row["ekf_wind_mean_n_ms"] = _mean(ekf_n)
    row["ekf_wind_mean_e_ms"] = _mean(ekf_e)
    if ekf_n:
        row["ekf_wind_error_ms"] = math.hypot(row["ekf_wind_mean_n_ms"] - w_from[0],
                                              row["ekf_wind_mean_e_ms"] - w_from[1])
    row["crab_mean_deg"] = _mean(crab)
    row["crab_rms_deg"] = _rms(crab)
    row["crab_max_abs_deg"] = max((abs(c) for c in crab), default=None)
    row["groundspeed_min_ms"] = min(gs_all) if gs_all else None
    row["groundspeed_max_ms"] = max(gs_all) if gs_all else None
    row["airspeed_mean_ms"] = _mean(as_all)
    radii = []
    for b in BINS:
        row["gs_%s_ms" % b] = _mean(bins[b]["gs"])
        row["radius_%s_m" % b] = _mean(bins[b]["r"])
        if row["radius_%s_m" % b] is not None:
            radii.append(row["radius_%s_m" % b])
    row["radius_spread_wind_m"] = (max(radii) - min(radii)) if len(radii) > 1 else None
    row["vel_err_along_wind_rms_ms"] = _rms(err_along)
    row["vel_err_cross_wind_rms_ms"] = _rms(err_cross)
    if row["vel_err_rms_ms"] is None:
        row["vel_err_rms_ms"] = _rms(err_all)
    return row


def write_csv(rows, path):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        for row in rows:
            writer.writerow([experiment._csv_cell(row.get(c)) for c in COLUMNS])
    return path


COMPARE_COLUMNS = (
    "python_cell_id", "wind_label", "n_repeats",
    "python_rms_ring_error_m", "calm_rms_ring_error_m", "rms_ring_error_m_mean",
    "rms_ring_error_m_min", "rms_ring_error_m_max",
    "python_t_contact_s", "calm_t_contact_s", "t_contact_s_mean",
    "python_zone_plane_breaches", "calm_zone_plane_breaches",
    "zone_plane_breaches_max",
    "groundspeed_min_ms_mean", "groundspeed_max_ms_mean",
    "radius_downwind_m_mean", "radius_upwind_m_mean", "radius_spread_wind_m_mean",
    "closure_ok_all", "convention",
)


def compare_table(rows, python_records):
    """Planned-versus-actual per (Python cell, direction).

    Args:
        rows: :func:`analyse` rows for a wind sub-campaign (calm included).
        python_records: ``{python_cell_id: record.json dict}``.
    """
    groups = {}
    for r in rows:
        groups.setdefault((r["python_cell_id"], r["wind_label"]), []).append(r)
    calm = {}
    for (pid, label), rs in groups.items():
        if label == "calm":
            calm[pid] = rs
    out = []
    for (pid, label) in sorted(groups):
        rs = groups[(pid, label)]
        py = (python_records.get(pid) or {}).get("metrics") or {}
        py_ring = ((py.get("ring") or {}).get("target") or {})
        py_pc = ((py.get("post_contact") or {}).get("target") or {})
        py_zone = py.get("zone") or {}
        calm_rs = calm.get(pid, [])

        def m(key, source=rs, fn=_mean):
            return fn([x.get(key) for x in source])

        out.append({
            "python_cell_id": pid, "wind_label": label, "n_repeats": len(rs),
            "python_rms_ring_error_m": py_ring.get("rms_ring_error_m"),
            "calm_rms_ring_error_m": m("rms_ring_error_m", calm_rs),
            "rms_ring_error_m_mean": m("rms_ring_error_m"),
            "rms_ring_error_m_min": min((x["rms_ring_error_m"] for x in rs
                                         if x.get("rms_ring_error_m") is not None),
                                        default=None),
            "rms_ring_error_m_max": max((x["rms_ring_error_m"] for x in rs
                                         if x.get("rms_ring_error_m") is not None),
                                        default=None),
            "python_t_contact_s": py_pc.get("t_contact_s"),
            "calm_t_contact_s": m("t_contact_s", calm_rs),
            "t_contact_s_mean": m("t_contact_s"),
            "python_zone_plane_breaches": py_zone.get("plane_breaches"),
            "calm_zone_plane_breaches": max((x["zone_plane_breaches"] for x in calm_rs
                                             if x.get("zone_plane_breaches") is not None),
                                            default=None),
            "zone_plane_breaches_max": max((x["zone_plane_breaches"] for x in rs
                                            if x.get("zone_plane_breaches") is not None),
                                           default=None),
            "groundspeed_min_ms_mean": m("groundspeed_min_ms"),
            "groundspeed_max_ms_mean": m("groundspeed_max_ms"),
            "radius_downwind_m_mean": m("radius_downwind_m"),
            "radius_upwind_m_mean": m("radius_upwind_m"),
            "radius_spread_wind_m_mean": m("radius_spread_wind_m"),
            "closure_ok_all": all(bool(x.get("closure_ok")) for x in rs),
            "convention": ",".join(sorted(set(str(x.get("convention")) for x in rs))),
        })
    return out


def write_compare_csv(table, path):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(COMPARE_COLUMNS)
        for row in table:
            writer.writerow([experiment._csv_cell(row.get(c)) for c in COMPARE_COLUMNS])
    return path


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bundles", nargs="+")
    parser.add_argument("--out", default=None, help="wind-frame.csv path")
    parser.add_argument("--closure-tol-ms", type=float, default=CLOSURE_TOL_MS)
    args = parser.parse_args(argv)
    rows = [analyse(d, args.closure_tol_ms) for d in args.bundles]
    if args.out:
        write_csv(rows, args.out)
        print("wrote %s (%d rows)" % (args.out, len(rows)))
    else:
        for row in rows:
            print(json.dumps(row, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
