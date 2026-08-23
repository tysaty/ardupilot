"""Speed x heading sweep for the CS-onto-orbit Dubins, plotted (``TASK-028``).

Sweeps the **circle-straight-onto-orbit** algorithm (``dubins_target_orbit``:
one initial arc at ``turn_radius_m``, one straight into the ring tangent, then the
ramp-free orbit continuation) against a **straight-travelling kangaroo**, over the
grid

    target speed  x  initial aircraft heading

and renders **both** plot forms already in the harness for every cell: the 2D
static top-down (``plotter.plot_runs``, ``show_3d=False``) and the 2D time
animation (``plotter.animate_runs``, ``TASK-011``).

Target speeds are expressed **relative to the aircraft**, because that ratio — not
the absolute speed — is what decides whether the geometry can hold station:

* ``default``  — the harness kangaroo default, 5 m/s;
* ``half``     — half aircraft speed;
* ``equal``    — aircraft speed (the marginal case);
* ``double``   — twice aircraft speed, which **cannot be caught**: the expected
  outcome is trailing pursuit, not an orbit (``TASK-003``).

The refresh rate is a first-class axis of this sweep and defaults to **1 Hz**
(``dt_s = 1.0``), ten times slower than the shipping 100 ms loop. That is
deliberate: it shows how the receding-horizon solve degrades when the plan is
refreshed less often. At 25 m/s the aircraft covers 25 m between solves, half the
default 50 m look-ahead, so the carrot is materially stale by the time it is
re-derived (``A-VAL-005`` and the ``TASK-027`` residual are both rate-sensitive).

Structure follows ``TASK-016``'s batch runner: a base front-end spec plus labelled
variants, each run through :func:`frontend.run_spec` with ``--save-run`` and
``--no-plot`` forced, then handed as *files* to the **read-only** plotter
(``DEC-2026-07-22-01``). Nothing here drives an algorithm.

Out of scope (``A-VAL-001``): this is offline geometry, not a controller sweep.
No claim about tracking quality follows from it.
"""

import argparse
import glob
import os

from . import batch
from . import frontend
from . import plotter
from .config import HarnessConfig


#: Target speeds as a multiple of aircraft speed. ``None`` means the harness
#: kangaroo default (5 m/s) rather than a ratio.
SPEED_CASES = (
    ("default", None),
    ("half", 0.5),
    ("equal", 1.0),
    ("double", 2.0),
)

#: Initial aircraft headings, degrees from North clockwise. -90 (West) and 90
#: (East) start across the bearing to the target, 0 (North) runs straight in and
#: 180 (South) starts pointing away, forcing the largest turn-in.
HEADINGS_DEG = (-90.0, 0.0, 90.0, 180.0)

#: Algorithm refresh interval, seconds. 1.0 = 1 Hz, the directed rate.
DEFAULT_DT_S = 1.0

#: The circle-straight-onto-orbit algorithm (TASK-024 approach + TASK-025 orbit).
ALGORITHM = "dubins_target_orbit"


def speed_for(case_ratio, airspeed_ms, default_speed_ms=5.0):
    """Resolve a speed case to m/s. ``None`` -> the kangaroo default."""
    if case_ratio is None:
        return default_speed_ms
    return airspeed_ms * case_ratio


def cell_label(speed_name, heading_deg):
    """Filesystem-safe label for one grid cell, e.g. ``half_hdg-090``."""
    sign = "-" if heading_deg < 0 else ""
    return "%s_hdg%s%03d" % (speed_name, sign, abs(int(round(heading_deg))))


def build_spec(algorithm, speed_ms, heading_deg, dt_s, duration_s,
               kang_heading_deg, start_range_m):
    """The front-end spec for one cell (no output paths; the runner adds those)."""
    return {
        "algorithms": [algorithm],
        "config": {"dt_s": float(dt_s)},
        "plane": {"heading_deg": float(heading_deg)},
        "target": {"range_m": float(start_range_m), "bearing_deg": 0.0},
        "kangaroo": {"mode": "straight",
                     "speed_ms": float(speed_ms),
                     "heading_deg": float(kang_heading_deg)},
        "run": {"duration_s": float(duration_s)},
    }


def run_sweep(out_dir, algorithm=ALGORITHM, dt_s=DEFAULT_DT_S, duration_s=240.0,
              kang_heading_deg=0.0, start_range_m=300.0, frames=120, fps=20,
              static=True, animate=True):
    """Run the speed x heading grid and render both plot forms per cell.

    Args:
        out_dir: Directory for the run JSON, PNGs and GIFs. Created if absent.
        algorithm: Registry name; defaults to the CS-onto-orbit algorithm.
        dt_s: Algorithm refresh interval, s (1.0 = 1 Hz).
        duration_s: Simulated seconds per cell.
        kang_heading_deg: Kangaroo track, degrees from North clockwise.
        start_range_m: Initial aircraft-to-target range, m.
        frames, fps: Animation sampling.
        static, animate: Render the static PNG / animated GIF.

    Returns:
        List of records, one per cell: ``{speed_case, speed_ms, heading_deg,
        label, json, png, gif}``. Plot paths are ``None`` when not rendered.
    """
    os.makedirs(out_dir, exist_ok=True)
    airspeed = HarnessConfig().airspeed_ms
    records = []

    for speed_name, ratio in SPEED_CASES:
        speed_ms = speed_for(ratio, airspeed)
        for heading_deg in HEADINGS_DEG:
            label = cell_label(speed_name, heading_deg)
            spec = build_spec(algorithm, speed_ms, heading_deg, dt_s,
                              duration_s, kang_heading_deg, start_range_m)
            stem = os.path.join(out_dir, label)
            spec = batch.merge_specs(
                spec, {"run": {"save_run": stem + ".json", "no_plot": True}})
            frontend.run_spec(spec)

            # The plotter takes (name, history) pairs; meta is kept separately
            # so the ring radius can be read back from the saved run.
            runs, metas, json_path = [], [], None
            for path in sorted(glob.glob(stem + "-*.json")):
                name, history, meta = plotter.load_run(path)
                runs.append((name, history))
                metas.append(meta)
                json_path = path

            png = gif = None
            if runs:
                orbit_radius_m = (metas[0] or {}).get("orbit_radius_m")
                if static:
                    png = stem + ".png"
                    plotter.plot_runs(runs, orbit_radius_m=orbit_radius_m,
                                      show_3d=False, save_path=png)
                if animate:
                    gif = stem + ".gif"
                    plotter.animate_runs(runs, orbit_radius_m=orbit_radius_m,
                                         save_path=gif, frames=frames, fps=fps)

            records.append({
                "speed_case": speed_name,
                "speed_ms": speed_ms,
                "heading_deg": heading_deg,
                "label": label,
                "json": json_path,
                "png": png,
                "gif": gif,
            })
    return records


def main(argv=None):
    """Entry point: ``python3 -m py_harness.cs_orbit_sweep [options]``."""
    parser = argparse.ArgumentParser(
        description="Speed x heading sweep of the circle-straight-onto-orbit "
        "Dubins against a straight-travelling kangaroo, at a 1 Hz refresh "
        "(TASK-028). Renders a 2D static PNG and a 2D animated GIF per cell.")
    parser.add_argument("--out-dir", required=True,
                        help="Directory for run JSON, PNGs and GIFs.")
    parser.add_argument("--algorithm", default=ALGORITHM,
                        help="Registry name (default: %s)." % ALGORITHM)
    parser.add_argument("--dt-s", type=float, default=DEFAULT_DT_S,
                        help="Refresh interval, s (default 1.0 = 1 Hz).")
    parser.add_argument("--duration-s", type=float, default=240.0,
                        help="Simulated seconds per cell (default 240).")
    parser.add_argument("--kang-heading-deg", type=float, default=0.0,
                        help="Kangaroo track, deg from North (default 0 = North).")
    parser.add_argument("--start-range-m", type=float, default=300.0,
                        help="Initial aircraft-to-target range, m (default 300).")
    parser.add_argument("--frames", type=int, default=120,
                        help="Animation frames per cell (default 120).")
    parser.add_argument("--fps", type=int, default=20,
                        help="Animation frames per second (default 20).")
    parser.add_argument("--no-static", action="store_true",
                        help="Skip the static PNGs.")
    parser.add_argument("--no-animate", action="store_true",
                        help="Skip the animated GIFs (much faster).")
    args = parser.parse_args(argv)

    records = run_sweep(
        args.out_dir, algorithm=args.algorithm, dt_s=args.dt_s,
        duration_s=args.duration_s, kang_heading_deg=args.kang_heading_deg,
        start_range_m=args.start_range_m, frames=args.frames, fps=args.fps,
        static=not args.no_static, animate=not args.no_animate)

    print("\n%d cells -> %s" % (len(records), os.path.abspath(args.out_dir)))
    print("%-18s %10s %9s  %s" % ("cell", "target m/s", "heading", "outputs"))
    for r in records:
        outs = " ".join(os.path.basename(p) for p in (r["png"], r["gif"]) if p)
        print("%-18s %10.1f %8.0f°  %s"
              % (r["label"], r["speed_ms"], r["heading_deg"], outs or "(none)"))
    return records


if __name__ == "__main__":
    main()
