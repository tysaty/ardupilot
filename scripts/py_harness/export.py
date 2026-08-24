"""Track and waypoint export (``TASK-029`` U1).

Three artefacts, deliberately separate because they answer different questions:

* :func:`write_tracks_csv` — the **flown tracks**. Per-sample position, speed and
  heading for both the aircraft and the kangaroo, for analysis outside the harness
  and for the thesis. CSV, so it opens in a spreadsheet without post-processing.
* :func:`write_waypoints_wpl` — the kangaroo's **change points** as a QGC WPL 110
  mission, for flying a second SITL vehicle as the target.
* :func:`write_target_stream` — the same change points as a location / heading /
  speed stream, matching what ``scripts/kangaroo_MAV.lua`` already consumes
  (``ADSB_VEHICLE`` messages built from lat/lon/heading/speed, with
  ``KBUS_LAT``/``KBUS_LNG``).

Both waypoint forms are provided because the right one depends on how the target
is injected, and that decision belongs to whoever runs SITL rather than to this
module. They share the same change points, so neither is the "real" one.

Heading convention
------------------
The harness records the aircraft heading it **arrived on**: ``Harness.step()``
turns, then moves, then records, so a sample's ``plane_hdg_rad`` describes the
segment *ending* at that sample, not the one leaving it. At 10 Hz the difference is
3.2 degrees; at 1 Hz it is 31.8 degrees, which is large enough to mislead. The CSV
therefore emits **both** — the recorded heading and the heading actually travelled
over the following segment — and says so in its header, rather than picking one
silently.

Kangaroo heading and speed are derived from the recorded target velocity
(``TASK-029``), which is exact.

Privacy (``AGENTS.md``)
-----------------------
Waypoint exports carry absolute coordinates. The origin defaults to the fictional
one in :mod:`py_harness.geodetic`; supplying a real site is a conscious act, and
the writers stamp which was used in the file.
"""

import csv
import json
import math

from . import geodetic


#: Default target altitude for exported waypoints, metres relative to home.
DEFAULT_ALT_M = 50.0


def _speed_heading(vn_ms, ve_ms):
    """``(speed, heading_deg)`` from a North/East velocity. Heading 0 = North."""
    speed = math.hypot(vn_ms, ve_ms)
    if speed < 1e-9:
        return 0.0, 0.0
    return speed, math.degrees(math.atan2(ve_ms, vn_ms)) % 360.0


def track_rows(history):
    """Per-sample rows for both actors, as plain dicts.

    Pure, so the shape is testable without writing a file.
    """
    rows = []
    for i, s in enumerate(history):
        nxt = history[i + 1] if i + 1 < len(history) else None
        # Heading actually travelled over the segment leaving this sample.
        if nxt is not None:
            dn = nxt["plane_n_m"] - s["plane_n_m"]
            de = nxt["plane_e_m"] - s["plane_e_m"]
            travelled = (math.degrees(math.atan2(de, dn)) % 360.0
                         if math.hypot(dn, de) > 1e-9 else None)
        else:
            travelled = None
        t_speed, t_hdg = _speed_heading(s.get("target_vn_ms", 0.0),
                                        s.get("target_ve_ms", 0.0))
        rows.append({
            "t_s": s["t_s"],
            "plane_n_m": s["plane_n_m"],
            "plane_e_m": s["plane_e_m"],
            "plane_hdg_deg_recorded": math.degrees(s["plane_hdg_rad"]) % 360.0,
            "plane_hdg_deg_travelled": travelled,
            "target_n_m": s["target_n_m"],
            "target_e_m": s["target_e_m"],
            "target_speed_ms": t_speed,
            "target_hdg_deg": t_hdg,
            "range_m": math.hypot(s["plane_n_m"] - s["target_n_m"],
                                  s["plane_e_m"] - s["target_e_m"]),
        })
    return rows


TRACK_FIELDS = ("t_s", "plane_n_m", "plane_e_m", "plane_hdg_deg_recorded",
                "plane_hdg_deg_travelled", "target_n_m", "target_e_m",
                "target_speed_ms", "target_hdg_deg", "range_m")


def write_tracks_csv(path, history, meta=None):
    """Write the flown tracks to CSV. Returns ``path``.

    Leading ``#`` comment lines carry the units and the heading convention, so the
    file is self-describing when it is opened months later in a spreadsheet.
    """
    rows = track_rows(history)
    with open(path, "w", newline="") as handle:
        handle.write("# py_harness track export (TASK-029)\n")
        handle.write("# positions and range in metres, local planar North/East; "
                     "speeds m/s; headings degrees from North, clockwise\n")
        handle.write("# plane_hdg_deg_recorded: heading as recorded, describing "
                     "the segment ENDING at this sample\n")
        handle.write("# plane_hdg_deg_travelled: heading actually flown over the "
                     "segment LEAVING this sample (blank on the last row)\n")
        handle.write("# the two differ by up to one turn-rate step: ~3.2 deg at "
                     "10 Hz, ~31.8 deg at 1 Hz\n")
        for key, value in sorted((meta or {}).items()):
            handle.write("# %s: %s\n" % (key, value))
        writer = csv.DictWriter(handle, fieldnames=list(TRACK_FIELDS))
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_tracks_json(path, history, meta=None):
    """The same track data as JSON, for programmatic reuse."""
    with open(path, "w") as handle:
        json.dump({"meta": dict(meta or {}), "rows": track_rows(history)},
                  handle, indent=2)
    return path


# --------------------------------------------------------------------------
# Waypoints — the kangaroo's change points, for SITL
# --------------------------------------------------------------------------

def change_points(session):
    """The kangaroo's manoeuvre points: where it was when each leg began.

    A waypoint per 10 Hz tick would be useless, so the export follows the
    **schedule**, not the samples: one point per leg, with straight travel between
    them. Returns ``[{t_s, n_m, e_m, mode, heading_deg, speed_ms}, ...]``.
    """
    history = session.history
    if not history:
        return []

    times = [0.0] + [c["t_s"] for c in session.change_log]
    legs = [session.legs[0] if session.legs else (0.0, "point", 0.0, 0.0)]
    legs += [(c["duration_s"], c["mode"], c["heading_deg"], c["speed_ms"])
             for c in session.change_log]

    points = []
    for t_mark, leg in zip(times, legs):
        sample = next((s for s in history if s["t_s"] >= t_mark), history[-1])
        points.append({
            "t_s": sample["t_s"],
            "n_m": sample["target_n_m"],
            "e_m": sample["target_e_m"],
            "mode": leg[1],
            "heading_deg": leg[2],
            "speed_ms": leg[3],
        })
    # Close with the final position so the last leg has an end point.
    last = history[-1]
    points.append({
        "t_s": last["t_s"], "n_m": last["target_n_m"], "e_m": last["target_e_m"],
        "mode": "point", "heading_deg": 0.0, "speed_ms": 0.0,
    })
    return points


def write_waypoints_wpl(path, session, origin=None, alt_m=DEFAULT_ALT_M):
    """Kangaroo change points as a **QGC WPL 110** mission file.

    For flying a *second* SITL vehicle as the target. Item 0 is home, as the format
    requires; the rest are ``MAV_CMD_NAV_WAYPOINT`` (16) in
    ``MAV_FRAME_GLOBAL_RELATIVE_ALT`` (3).

    Note this format carries **position only** — a waypoint has no speed. Where the
    kangaroo's speed matters, :func:`write_target_stream` is the better export.
    """
    origin = origin or geodetic.DEFAULT_ORIGIN
    points = change_points(session)
    with open(path, "w") as handle:
        handle.write("QGC WPL 110\n")
        lat0, lon0, alt0 = origin
        handle.write("0\t1\t0\t16\t0\t0\t0\t0\t%.8f\t%.8f\t%.6f\t1\n"
                     % (lat0, lon0, alt0))
        for i, p in enumerate(points, start=1):
            lat, lon = geodetic.ne_to_latlon(p["n_m"], p["e_m"], origin)
            handle.write("%d\t0\t3\t16\t0\t0\t0\t0\t%.8f\t%.8f\t%.6f\t1\n"
                         % (i, lat, lon, alt_m))
    return path


def write_target_stream(path, session, origin=None, alt_m=DEFAULT_ALT_M):
    """Kangaroo change points as a location / heading / speed stream (JSON).

    Matches what ``scripts/kangaroo_MAV.lua`` already consumes: it injects the
    virtual target as ``ADSB_VEHICLE`` messages built from lat/lon, heading and
    speed, with ``KBUS_LAT``/``KBUS_LNG`` parameters — not from a mission file.
    Unlike WPL this carries **speed and mode**, so it reproduces the manoeuvre
    rather than only the geometry.
    """
    origin = origin or geodetic.DEFAULT_ORIGIN
    lat0, lon0, alt0 = origin
    items = []
    for p in change_points(session):
        lat, lon = geodetic.ne_to_latlon(p["n_m"], p["e_m"], origin)
        items.append({
            "t_s": p["t_s"], "lat_deg": lat, "lng_deg": lon, "alt_m": alt_m,
            "heading_deg": p["heading_deg"], "speed_ms": p["speed_ms"],
            "mode": p["mode"], "n_m": p["n_m"], "e_m": p["e_m"],
        })
    payload = {
        "format": "py_harness target stream v1",
        "consumer": "scripts/kangaroo_MAV.lua (ADSB_VEHICLE / KBUS_LAT / KBUS_LNG)",
        "origin": {"lat_deg": lat0, "lng_deg": lon0, "alt_m": alt0,
                   "fictional": geodetic.is_default_origin(origin)},
        "algorithm": getattr(session, "algorithm_name", None),
        "duration_s": session.t_s,
        "points": items,
    }
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2)
    return path


def export_all(session, stem, origin=None, alt_m=DEFAULT_ALT_M, meta=None):
    """Write every artefact under one path stem. Returns the paths written."""
    meta = dict(meta or {})
    meta.setdefault("algorithm", getattr(session, "algorithm_name", ""))
    meta.setdefault("origin_is_fictional", geodetic.is_default_origin(origin))
    return {
        "tracks_csv": write_tracks_csv(stem + "-tracks.csv", session.history, meta),
        "tracks_json": write_tracks_json(stem + "-tracks.json", session.history,
                                         meta),
        "waypoints_wpl": write_waypoints_wpl(stem + "-kangaroo.waypoints",
                                             session, origin, alt_m),
        "target_stream": write_target_stream(stem + "-kangaroo-stream.json",
                                             session, origin, alt_m),
        "schedule": session.export(stem + "-schedule.json"),
    }
