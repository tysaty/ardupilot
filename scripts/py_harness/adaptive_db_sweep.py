"""Horizon x replan-interval sweep for the adaptive CS-orbit (``TASK-033``).

The `(k_horizon, n_replan)` grid `TASK-033` exists to produce, reproducible from a single
command, rendering for **every** cell both plot forms the harness already has —
the 2D static top-down (``TASK-002``) and the 2D time animation (``TASK-011``) —
plus a per-cell metrics record and one combined JSON summary.

Structural sibling of :mod:`py_harness.cs_orbit_sweep` (``TASK-028``), which
sweeps target speed x initial heading for the *unpredicted* CS-orbit. The axes
here are the two variables `TASK-033` introduces:

* ``k_horizon`` — prediction horizon in whole control ticks (``--lookahead-steps``,
  ``TASK-017``); the horizon in seconds is ``k_horizon * dt_s``.
* ``n_replan`` — commitment interval in whole control ticks (``--replan-every``); the
  replan period is ``m * dt_s`` seconds.

Unlike ``TASK-028`` this sweep holds ``dt_s`` at the harness default 0.1 s and
varies **only** the planning rate, because varying ``dt_s`` coarsens the
aircraft's kinematic integration as well as the planning rate and so cannot
attribute the result to planning.

Every cell is run against a **baseline** ``dubins_target_orbit`` under identical
conditions, so each figure shows the extension against what it extends rather
than in isolation.

Metrics are reported about **both** ring centres (:func:`metrics.dual_centre_stats`):
the true kangaroo, and the centre the algorithm actually held. Against a
prediction-planning algorithm those differ by the designed lead
``k * dt_s * |v_target|``, so quoting either alone is misleading — in opposite
directions.

Drives geometry through the ordinary front-end spec, and draws through the
read-only plotter (``DEC-2026-07-22-01``). It contains no geometry and steers
nothing.
"""

import argparse
import glob
import json
import os

from . import batch
from . import frontend
from . import metrics
from . import plotter
from .config import HarnessConfig

#: Prediction horizons in control ticks. 0 is the present-position behaviour;
#: at dt_s = 0.1 these are 0.0, 1.0, 2.5 and 5.0 seconds.
HORIZON_STEPS = (0, 10, 25, 50)

#: Commitment intervals in control ticks. 1 replans every tick (unchanged
#: behaviour); at dt_s = 0.1 these are 10, 2, 1 and 0.4 Hz.
REPLAN_EVERY = (1, 5, 10, 25)

#: The algorithm under study, and the baseline it extends.
ALGORITHM = "adaptive_db_circle"
BASELINE = "dubins_target_orbit"

#: Integration step, s. Held fixed; only the planning rate varies.
DT_S = 0.1


def cell_label(k_horizon, n_replan, hold_policy):
    """Filesystem-safe label for one grid cell, e.g. ``k025_m005_plan``.

    The label keeps the compact ``m`` shorthand for ``n_replan`` deliberately.
    The symbol was renamed on 2026-08-31 — a bare ``m`` collides with the metres
    suffix used throughout this repository — but figures and saved runs are
    already published under these names and cited from ``TASK-033``'s completion
    record. Renaming them would break those citations to gain nothing, and ``m``
    is unambiguous inside a fixed-width filename in a way it is not in prose.
    """
    return "k%03d_m%03d_%s" % (k_horizon, n_replan, hold_policy)


def build_spec(k_horizon, n_replan, hold_policy, kang_mode, kang_speed_ms, duration_s,
               plane_heading_deg, start_range_m):
    """The front-end spec for one cell (no output paths; the runner adds those).

    Both algorithms are listed, so each cell renders the adaptive run against its
    baseline on shared axes. ``estimate`` is mandatory — ``adaptive_db_circle``
    refuses without it.
    """
    return {
        "algorithms": [ALGORITHM, BASELINE],
        "config": {"dt_s": DT_S, "lookahead_steps": int(k_horizon),
                   "replan_every": int(n_replan), "hold_policy": hold_policy},
        "plane": {"heading_deg": float(plane_heading_deg)},
        "target": {"range_m": float(start_range_m), "bearing_deg": 0.0},
        "kangaroo": {"mode": kang_mode, "speed_ms": float(kang_speed_ms),
                     "heading_deg": 0.0},
        "run": {"duration_s": float(duration_s), "estimate": True},
    }


def cell_metrics(history, orbit_radius_m):
    """Per-cell numbers: both centres, curvature and the replan step.

    ``max_curvature`` and the replan-step figures come from ``algorithm_state``,
    so they are absent for the baseline, which reports no replan.
    """
    out = {"dual_centre": metrics.dual_centre_stats(history, orbit_radius_m)}
    curvatures, steps = [], []
    for s in history:
        st = s.get("algorithm_state") or {}
        if "curvature" in st:
            curvatures.append(st["curvature"])
        if st.get("replanned") and "replan_step_m" in st:
            steps.append(st["replan_step_m"])
    out["max_curvature"] = max(curvatures) if curvatures else None
    out["replans"] = len(steps)
    out["max_replan_step_m"] = max(steps) if steps else None
    out["mean_replan_step_m"] = (sum(steps) / len(steps)) if steps else None
    out["samples"] = len(history)
    return out


def run_sweep(out_dir, hold_policy="plan", kang_mode="straight",
              kang_speed_ms=12.5, duration_s=240.0, plane_heading_deg=0.0,
              start_range_m=300.0, horizons=HORIZON_STEPS,
              intervals=REPLAN_EVERY, frames=120, fps=20, static=True,
              animate=True):
    """Run the ``k_horizon`` x ``n_replan`` grid and render both plot forms per cell.

    Returns:
        List of one record per cell: the axes, the output paths, the per-algorithm
        metrics and the curvature-bound verdict.
    """
    os.makedirs(out_dir, exist_ok=True)
    cfg = HarnessConfig()
    curvature_limit = 1.0 / cfg.turn_radius_m
    records = []

    for k_horizon in horizons:
        for n_replan in intervals:
            label = cell_label(k_horizon, n_replan, hold_policy)
            spec = build_spec(k_horizon, n_replan, hold_policy, kang_mode, kang_speed_ms,
                              duration_s, plane_heading_deg, start_range_m)
            stem = os.path.join(out_dir, label)
            spec = batch.merge_specs(
                spec, {"run": {"save_run": stem + ".json", "no_plot": True}})
            frontend.run_spec(spec)

            runs, per_algorithm, orbit_radius_m = [], {}, None
            for path in sorted(glob.glob(stem + "-*.json")):
                name, history, meta = plotter.load_run(path)
                runs.append((name, history))
                orbit_radius_m = (meta or {}).get("orbit_radius_m",
                                                  cfg.orbit_radius_m)
                per_algorithm[name] = cell_metrics(history, orbit_radius_m)
                per_algorithm[name]["saved_run"] = path
                per_algorithm[name]["meta"] = meta

            png = gif = None
            if runs:
                if static:
                    png = stem + ".png"
                    plotter.plot_runs(runs, orbit_radius_m=orbit_radius_m,
                                      show_3d=False, save_path=png)
                if animate:
                    gif = stem + ".gif"
                    plotter.animate_runs(runs, orbit_radius_m=orbit_radius_m,
                                         save_path=gif, frames=frames, fps=fps)

            # FR-005/SR-002 asserted per cell rather than assumed across the grid.
            worst = max((v["max_curvature"] for v in per_algorithm.values()
                         if v["max_curvature"] is not None), default=0.0)
            records.append({
                "lookahead_steps": k_horizon,
                "horizon_s": k_horizon * DT_S,
                "replan_every": n_replan,
                "replan_period_s": n_replan * DT_S,
                "replan_rate_hz": 1.0 / (n_replan * DT_S),
                "hold_policy": hold_policy,
                "label": label,
                "png": png,
                "gif": gif,
                "algorithms": per_algorithm,
                "max_curvature": worst,
                "curvature_limit": curvature_limit,
                "curvature_ok": worst <= curvature_limit + 1e-12,
            })
    return records


def summarise(records):
    """One text table of the grid. Both centres, because either alone misleads."""
    lines = [
        "%-18s %8s %8s | %9s %9s %8s | %9s %9s | %9s %9s"
        % ("cell", "horizon", "rate", "RMS(tgt)", "max(tgt)", "settled",
           "RMS(ring)", "lead m", "max|k|", "mean step"),
        "-" * 118,
    ]
    for r in records:
        a = r["algorithms"].get(ALGORITHM)
        if a is None or a["dual_centre"] is None:
            continue
        d = a["dual_centre"]
        lines.append(
            "%-18s %7.1fs %7.1fHz | %9.2f %9.2f %8s | %9.2f %9.2f | %9.6f %9s"
            % (r["label"], r["horizon_s"], r["replan_rate_hz"],
               d["target"]["rms_ring_error_m"], d["target"]["max_ring_error_m"],
               d["target"]["settled"], d["ring"]["rms_ring_error_m"],
               d["mean_prediction_lead_m"], r["max_curvature"],
               "-" if a["mean_replan_step_m"] is None
               else "%.2f" % a["mean_replan_step_m"]))
    breaches = [r["label"] for r in records if not r["curvature_ok"]]
    lines.append("")
    lines.append("curvature bound %.6f — %s"
                 % (records[0]["curvature_limit"] if records else 0.0,
                    "respected in every cell" if not breaches
                    else "BREACHED in: " + ", ".join(breaches)))
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Prediction-horizon x replan-interval sweep for the adaptive "
                    "CS-orbit (TASK-033). Renders a 2D static PNG and a 2D "
                    "animated GIF per cell, against the dubins_target_orbit "
                    "baseline, and writes a metrics summary.")
    parser.add_argument("out_dir", help="Directory for runs, figures and summary.")
    parser.add_argument("--hold-policy", choices=("plan", "centre_only"),
                        default="plan",
                        help="What is held between replans. Default 'plan'.")
    parser.add_argument("--kang-mode", default="straight",
                        help="Kangaroo motion mode. Default straight.")
    parser.add_argument("--kang-speed-ms", type=float, default=12.5,
                        help="Kangaroo speed, m/s. Default 12.5 (half airspeed).")
    parser.add_argument("--duration-s", type=float, default=240.0,
                        help="Simulated seconds per cell. Default 240.")
    parser.add_argument("--plane-heading-deg", type=float, default=0.0,
                        help="Initial aircraft heading, deg from North.")
    parser.add_argument("--start-range-m", type=float, default=300.0,
                        help="Initial aircraft-to-target range, m.")
    parser.add_argument("--horizons", type=int, nargs="+", default=None,
                        help="Prediction horizons k, in ticks. Default %s."
                             % (list(HORIZON_STEPS),))
    parser.add_argument("--intervals", type=int, nargs="+", default=None,
                        help="Commitment intervals m, in ticks. Default %s."
                             % (list(REPLAN_EVERY),))
    parser.add_argument("--no-static", action="store_true",
                        help="Skip the static PNGs.")
    parser.add_argument("--no-animate", action="store_true",
                        help="Skip the animated GIFs (much faster).")
    args = parser.parse_args(argv)

    records = run_sweep(
        args.out_dir, hold_policy=args.hold_policy, kang_mode=args.kang_mode,
        kang_speed_ms=args.kang_speed_ms, duration_s=args.duration_s,
        plane_heading_deg=args.plane_heading_deg,
        start_range_m=args.start_range_m,
        horizons=tuple(args.horizons) if args.horizons else HORIZON_STEPS,
        intervals=tuple(args.intervals) if args.intervals else REPLAN_EVERY,
        static=not args.no_static, animate=not args.no_animate)

    summary_path = os.path.join(args.out_dir, "summary.json")
    with open(summary_path, "w") as fh:
        json.dump({
            "task": "TASK-033",
            "algorithm": ALGORITHM,
            "baseline": BASELINE,
            "dt_s": DT_S,
            "hold_policy": args.hold_policy,
            "kangaroo": {"mode": args.kang_mode, "speed_ms": args.kang_speed_ms},
            "duration_s": args.duration_s,
            "cells": records,
        }, fh, indent=1)

    text = summarise(records)
    with open(os.path.join(args.out_dir, "summary.txt"), "w") as fh:
        fh.write(text + "\n")
    print(text)
    print("\n%d cells -> %s" % (len(records), args.out_dir))
    return 0 if all(r["curvature_ok"] for r in records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
