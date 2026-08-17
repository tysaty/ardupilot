"""Look-ahead convergence sweep (``TASK-019``).

Runs the harness across a grid of **plane speed × start position × kangaroo mode ×
look-ahead ticks**, records the minimum on-orbit distance (`TASK-018`) for each,
and hands the records to the convergence plot (`plotter.plot_convergence`,
`TASK-020`) — so the look-ahead that best converges the plane onto the kangaroo's
ring (in phase) can be found. Because the optimal look-ahead is mode-dependent
(`A-TGT-002`), mode is a first-class sweep axis.

Each grid point is a front-end spec run with the trajectory plots **skipped**
(`run.no_plot`), so a large sweep is not slowed by per-run rendering (`TASK-020`);
only the single convergence plot is drawn at the end. The metric is read back from
the saved-run metadata (`min_orbit_distance_m`, stored by `run_harness`).

The algorithm swept must consume the estimate/prediction via ``target_est`` for the
look-ahead to have any effect — i.e. the ``heading_a``/``heading_a_orbit`` family
(the sweep forces ``--estimate``). ``base_spec`` should select such an algorithm.
"""

import argparse
import glob
import json
import os

from . import batch
from . import frontend
from . import plotter


def default_lookahead_range():
    """Default look-ahead axis: ``0..45`` steps, increment ``3``."""
    return list(range(0, 46, 3))


def run_sweep(base_spec, speeds, starts, modes, lookahead_steps=None,
              out_dir="sweepruns"):
    """Run the speed×start×mode×look-ahead grid and collect the metric.

    Args:
        base_spec: Shared front-end spec (must select a ``target_est``-consuming
            algorithm, e.g. ``heading_a_orbit``).
        speeds: Plane airspeeds, m/s (each → ``config.airspeed_ms``).
        starts: List of dicts, each a unique ``label`` plus ``target`` fields
            (``range_m``/``bearing_deg`` or ``n_m``/``e_m``).
        modes: List of dicts, each a unique ``label`` plus ``kangaroo`` fields
            (``mode`` + params).
        lookahead_steps: Iterable of horizons (default ``0..45`` step ``3``).
        out_dir: Directory for the saved run JSON.

    Returns:
        List of records ``{speed, start, mode, lookahead_steps,
        min_orbit_distance_m}``.
    """
    if lookahead_steps is None:
        lookahead_steps = default_lookahead_range()
    os.makedirs(out_dir, exist_ok=True)

    records = []
    for speed in speeds:
        for start in starts:
            start_label = start["label"]
            start_fields = {k: v for k, v in start.items() if k != "label"}
            for mode in modes:
                mode_label = mode["label"]
                mode_fields = {k: v for k, v in mode.items() if k != "label"}
                for n in lookahead_steps:
                    spec = batch.merge_specs(base_spec, {
                        "config": {"airspeed_ms": float(speed),
                                   "lookahead_steps": int(n)},
                        "target": start_fields,
                        "kangaroo": mode_fields,
                        "run": {"estimate": True},
                    })
                    label = "s%g_%s_%s_la%d" % (speed, start_label, mode_label, n)
                    stem = os.path.join(out_dir, label)
                    spec = batch.merge_specs(
                        spec, {"run": {"save_run": stem + ".json", "no_plot": True}})
                    frontend.run_spec(spec)

                    min_distance = None
                    for path in sorted(glob.glob(stem + "-*.json")):
                        _name, _history, meta = plotter.load_run(path)
                        min_distance = meta.get("min_orbit_distance_m")
                    records.append({
                        "speed": speed, "start": start_label, "mode": mode_label,
                        "lookahead_steps": n, "min_orbit_distance_m": min_distance,
                    })
    return records


def main(argv=None):
    """Entry point: ``python3 -m py_harness.sweep --config S.json --out-dir D``."""
    parser = argparse.ArgumentParser(
        description="Look-ahead convergence sweep (TASK-019): min on-orbit "
        "distance over speed x start x mode x look-ahead.")
    parser.add_argument("--config", required=True,
                        help="Sweep JSON: base, speeds, starts, modes, "
                        "[lookahead_steps].")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--plot", default=None,
                        help="PNG path for the convergence plot (TASK-020).")
    parser.add_argument("--show", action="store_true",
                        help="Show the convergence plot.")
    args = parser.parse_args(argv)

    with open(args.config) as handle:
        data = json.load(handle)
    for required in ("speeds", "starts", "modes"):
        if required not in data:
            print("INVALID SWEEP: needs '%s'" % required)
            return 2

    records = run_sweep(
        data.get("base", {}), data["speeds"], data["starts"], data["modes"],
        data.get("lookahead_steps"), args.out_dir)

    with open(os.path.join(args.out_dir, "sweep_records.json"), "w") as handle:
        json.dump(records, handle, indent=2)
    print("sweep: %d runs -> %s" % (len(records),
                                    os.path.join(args.out_dir, "sweep_records.json")))

    if args.plot or args.show:
        plotter.plot_convergence(records, save_path=args.plot, show=args.show)
        if args.plot:
            print("convergence plot -> %s" % args.plot)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
