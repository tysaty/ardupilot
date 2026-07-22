"""Entry point: run one algorithm through the harness and plot the result.

Selecting a different algorithm is a change to ``--algorithm`` and nothing
else. That is the whole point of the structure, and it is the acceptance
criterion for ``VR-014``.

Usage (once the stubs are implemented)::

    python3 -m py_harness.run_harness --algorithm continuous_weave
    python3 -m py_harness.run_harness --algorithm dubins

Status: SKELETON. Runs, parses arguments and builds an algorithm, then fails
at the first ``step()`` because no geometry is implemented.
"""

import argparse

from . import algorithms
from .state import Harness, PlaneState, TargetState, plot_history

#: Placeholder configuration. These are **not** requirements or safety limits.
#: Real values, units and provenance belong in the task record and ultimately
#: in a configuration file (``SR-004`` forbids unexplained constants).
DEFAULT_CONFIG = {
    "lambda_m": 750.0,
    "r_min_m": 20.0,
    "a_cap_m": 150.0,
    "d_start_m": 1000.0,
    "d_full_m": 300.0,
    "eta": 1.0,
    "u": 1.0,
    "phase_rad": 0.0,
}


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--algorithm",
        default="continuous_weave",
        choices=sorted(algorithms.REGISTRY),
        help="Geometric algorithm to run. The only difference between runs.",
    )
    parser.add_argument(
        "--duration-s", type=float, default=60.0, help="Simulated seconds."
    )
    parser.add_argument(
        "--dt-s",
        type=float,
        default=0.1,
        help="Fixed time step, seconds. 0.1 matches the controller's 100 ms loop.",
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="Skip the plot; print history only."
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    algorithm = algorithms.build(args.algorithm, DEFAULT_CONFIG)
    harness = Harness(
        plane=PlaneState(n_m=0.0, e_m=0.0, hdg_rad=0.0, speed_ms=25.0),
        target=TargetState(n_m=1000.0, e_m=0.0, vn_ms=0.0, ve_ms=0.0),
        algorithm=algorithm,
        dt_s=args.dt_s,
    )
    harness.run(args.duration_s)

    if not args.no_plot:
        plot_history(harness.history)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
