"""Entry point: run one algorithm through the harness and plot the result.

Selecting a different geometry is a change to ``--algorithm`` and nothing else.
That is the whole point of the structure, and it is the acceptance criterion
for ``VR-014``.

Usage::

    python3 -m py_harness.run_harness --algorithm dubins
    python3 -m py_harness.run_harness --algorithm orbit
    python3 -m py_harness.run_harness --algorithm dubins_orbit

The configured 45 m turn radius is **infeasible** at 25 m/s under the governing
45-degree bank limit, which needs 63.73 m. Runs are refused unless
``--allow-infeasible`` is passed, in which case every history sample and the
plot title are stamped infeasible. See :mod:`py_harness.config`.
"""

import argparse
import sys

from . import algorithms
from .config import HarnessConfig, InfeasibleConfiguration
from .state import Harness, PlaneState, TargetState, plot_history


def build_parser():
    parser = argparse.ArgumentParser(
        description="Run one geometric algorithm through the validation harness."
    )
    parser.add_argument(
        "--algorithm",
        default="dubins_orbit",
        choices=sorted(algorithms.REGISTRY),
        help="Geometric algorithm to run. The only difference between runs.",
    )
    parser.add_argument(
        "--duration-s", type=float, default=60.0, help="Simulated seconds."
    )
    parser.add_argument(
        "--airspeed-ms", type=float, default=None, help="Override airspeed, m/s."
    )
    parser.add_argument(
        "--turn-radius-m", type=float, default=None, help="Override turn radius, m."
    )
    parser.add_argument(
        "--orbit-radius-m", type=float, default=None, help="Override orbit radius, m."
    )
    parser.add_argument(
        "--allow-infeasible",
        action="store_true",
        help="Run even when the configuration violates the bank limit. "
        "Output is marked infeasible.",
    )
    parser.add_argument(
        "--start-range-m",
        type=float,
        default=800.0,
        help="Initial aircraft-to-target range, m.",
    )
    parser.add_argument(
        "--target-vn-ms", type=float, default=0.0, help="Target North velocity, m/s."
    )
    parser.add_argument(
        "--target-ve-ms", type=float, default=0.0, help="Target East velocity, m/s."
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="Skip the plot; print a summary only."
    )
    return parser


def build_config(args):
    overrides = {}
    if args.airspeed_ms is not None:
        overrides["airspeed_ms"] = args.airspeed_ms
    if args.turn_radius_m is not None:
        overrides["turn_radius_m"] = args.turn_radius_m
    if args.orbit_radius_m is not None:
        overrides["orbit_radius_m"] = args.orbit_radius_m
    return HarnessConfig(**overrides)


def main(argv=None):
    args = build_parser().parse_args(argv)
    cfg = build_config(args)

    print(cfg.summary())
    print()

    try:
        problems = cfg.validate(allow_infeasible=args.allow_infeasible)
    except InfeasibleConfiguration as exc:
        print("REFUSED: %s" % exc, file=sys.stderr)
        return 2

    if problems:
        print("PROCEEDING UNDER --allow-infeasible; output is not a flyable path:")
        for problem in problems:
            print("  - %s" % problem)
        print()

    if args.algorithm not in algorithms.IMPLEMENTED:
        print(
            "NOTE: %r is a stub; expect NotImplementedError. Implemented: %s"
            % (args.algorithm, ", ".join(algorithms.IMPLEMENTED))
        )

    algorithm = algorithms.build(args.algorithm, algorithms.config_dict(cfg))
    harness = Harness(
        plane=PlaneState(
            n_m=0.0, e_m=0.0, hdg_rad=0.0, speed_ms=cfg.airspeed_ms
        ),
        target=TargetState(
            n_m=args.start_range_m,
            e_m=0.0,
            vn_ms=args.target_vn_ms,
            ve_ms=args.target_ve_ms,
        ),
        algorithm=algorithm,
        dt_s=cfg.dt_s,
        turn_radius_m=cfg.turn_radius_m,
        infeasible=bool(problems),
    )
    harness.run(args.duration_s)

    if not harness.history:
        print("%s: no steps completed" % args.algorithm)
        return 1

    last = harness.history[-1]
    final_range = (
        (last["plane_n_m"] - last["target_n_m"]) ** 2
        + (last["plane_e_m"] - last["target_e_m"]) ** 2
    ) ** 0.5
    print(
        "%s: %d steps, %.1f s, final range %.1f m"
        % (args.algorithm, len(harness.history), last["t_s"], final_range)
    )

    achieved = harness.achieved_orbit_radius_m()
    if achieved is not None and args.algorithm in ("orbit", "dubins_orbit"):
        # Commanded versus achieved, the same gap A-DEC-009 records for the
        # weave. A carrot placed ahead on the ring is chased from inside it.
        print(
            "  commanded orbit radius %.1f m, achieved %.1f m (%.0f%%)"
            % (
                cfg.orbit_radius_m,
                achieved,
                100.0 * achieved / cfg.orbit_radius_m,
            )
        )

    if harness.stopped_reason:
        print("  stopped early: %s" % harness.stopped_reason)

    if not args.no_plot:
        import matplotlib.pyplot as plt

        plot_history(
            harness.history,
            title="%s — %.0f m turn radius, %.0f m orbit, %.0f m/s"
            % (
                args.algorithm,
                cfg.turn_radius_m,
                cfg.orbit_radius_m,
                cfg.airspeed_ms,
            ),
            orbit_radius_m=cfg.orbit_radius_m,
        )
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
