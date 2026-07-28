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
import math
import sys

from . import algorithms, kangaroo, plotter
from .config import HarnessConfig, InfeasibleConfiguration
from .estimator import KalmanFilter
from .geometry import dubins_orbit as dubins_orbit_geom
from .state import Harness, PlaneState, TargetState


def build_parser():
    parser = argparse.ArgumentParser(
        description="Run one geometric algorithm through the validation harness."
    )
    parser.add_argument(
        "--algorithm",
        action="append",
        dest="algorithms",
        choices=sorted(algorithms.REGISTRY),
        help="Geometric algorithm to run. The only difference between runs. "
        "Repeat to run several and overlay them on shared axes.",
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
        help="Initial aircraft-to-target range, m. Places the target due North "
        "of the aircraft unless --target-n-m/--target-e-m override it.",
    )
    parser.add_argument(
        "--target-n-m",
        type=float,
        default=None,
        help="Target North position, m. Overrides the --start-range-m placement.",
    )
    parser.add_argument(
        "--target-e-m",
        type=float,
        default=None,
        help="Target East position, m. Defaults to 0.",
    )
    parser.add_argument(
        "--no-3d",
        action="store_true",
        help="Plot the 2D top-down view only; omit the z = time 3D view.",
    )
    parser.add_argument(
        "--kang-mode", choices=kangaroo.MODES, default=None,
        help="Kangaroo target-motion mode (TASK-009). Overrides the --target-* "
        "placement and velocity when set.",
    )
    parser.add_argument("--kang-heading-deg", type=float, default=0.0,
                        help="Kangaroo mode heading, deg clockwise from North.")
    parser.add_argument("--kang-fwd-m", type=float, default=300.0,
                        help="Forward offset of the kangaroo start/centre, m.")
    parser.add_argument("--kang-disp-m", type=float, default=0.0,
                        help="Lateral displacement of the kangaroo start/centre, m.")
    parser.add_argument("--kang-radius-m", type=float, default=150.0,
                        help="Circle radius, m (circle mode).")
    parser.add_argument("--kang-length-m", type=float, default=300.0,
                        help="Rectangle length, m (rectangle mode).")
    parser.add_argument("--kang-width-m", type=float, default=150.0,
                        help="Rectangle width, m (rectangle mode).")
    parser.add_argument("--kang-speed-ms", type=float, default=5.0,
                        help="Kangaroo speed, m/s (straight/circle/rectangle).")
    parser.add_argument(
        "--estimate", action="store_true",
        help="Run the state estimator (TASK-012) and feed algorithms the "
        "estimated target via the snapshot. Required for the heading_a family.",
    )
    parser.add_argument(
        "--plane-heading-deg",
        type=float,
        default=0.0,
        help="Initial aircraft heading, degrees from North clockwise. Default 0 "
        "(North). Set it away from the bearing to the target to force a turning "
        "Dubins approach rather than a straight run-in.",
    )
    parser.add_argument(
        "--target-vn-ms", type=float, default=0.0, help="Target North velocity, m/s."
    )
    parser.add_argument(
        "--target-ve-ms", type=float, default=0.0, help="Target East velocity, m/s."
    )
    parser.add_argument(
        "--target-speed-ms",
        type=float,
        default=None,
        help="Target ground speed, m/s (TASK-003 moving target). Defaults to "
        "5 m/s when only a heading is given. Overrides "
        "--target-vn-ms/--target-ve-ms.",
    )
    parser.add_argument(
        "--target-heading-deg",
        type=float,
        default=None,
        help="Target heading, degrees from North clockwise (default 45 when a "
        "target speed is given). Overrides --target-vn-ms/--target-ve-ms.",
    )
    parser.add_argument(
        "--eta",
        type=float,
        default=None,
        help="Weave curvature safety factor, 0 < eta <= 1 (TASK-004). Overrides "
        "the configured default.",
    )
    parser.add_argument(
        "--weave-lambda", type=float, default=None,
        help="Weave wavelength, m (TASK-004). Larger = larger, gentler weaves "
        "(amplitude grows as lambda^2); smaller = more frequent but smaller.",
    )
    parser.add_argument(
        "--weave-a-cap", type=float, default=None,
        help="Maximum desired weave amplitude far from the curvature limit, m.",
    )
    parser.add_argument(
        "--weave-d-start", type=float, default=None,
        help="Distance at which the weave begins, m (d_start > d_full).",
    )
    parser.add_argument(
        "--weave-d-full", type=float, default=None,
        help="Distance at which the weave reaches full desired amplitude, m.",
    )
    parser.add_argument(
        "--compare-eta",
        action="store_true",
        help="For amplitude algorithms, run twice — with the safety factor and "
        "with eta = 1 — and overlay both on one plot (TASK-004).",
    )
    parser.add_argument(
        "--no-plot", action="store_true", help="Skip the plot; print a summary only."
    )
    parser.add_argument(
        "--save-run",
        default=None,
        help="Write each run to JSON, e.g. runs/out.json -> runs/out-dubins.json. "
        "Plot them later with `python3 -m py_harness.plotter <files>`.",
    )
    parser.add_argument(
        "--save-plot", default=None, help="Write the plot to PNG instead of showing it."
    )
    parser.add_argument(
        "--animate", default=None,
        help="Write a 2D time animation (GIF) of the plane and kangaroo over the "
        "run instead of the static plot (TASK-011), e.g. --animate out.gif.",
    )
    return parser


def _new_estimator(args, target_n0, target_e0):
    """A fresh state estimator per run when --estimate is set, else None.

    Each run gets its own filter (the estimator is stateful), initialised at the
    target's start position.
    """
    if not args.estimate:
        return None
    kf = KalmanFilter()
    kf.init(target_n0, target_e0)
    return kf


def target_velocity(speed_ms, heading_deg):
    """North/East velocity components for a straight-line target (TASK-003).

    Heading is degrees from North, clockwise (``IR-002``): ``vn = v cos hdg``,
    ``ve = v sin hdg``. Returns ``(vn_ms, ve_ms)``.
    """
    hdg = math.radians(heading_deg)
    return speed_ms * math.cos(hdg), speed_ms * math.sin(hdg)


def resolve_target_velocity(args, airspeed_ms):
    """Pick the target velocity from either speed+heading or raw components.

    Speed+heading wins when either is given; the speed defaults to 5 m/s
    (``TASK-003`` clarification, 2026-07-27) and the heading to 45 degrees.
    Otherwise the raw ``--target-vn-ms``/``--target-ve-ms`` are used (both
    default 0 — a static target).
    """
    if args.target_speed_ms is not None or args.target_heading_deg is not None:
        speed = 5.0 if args.target_speed_ms is None else args.target_speed_ms
        heading = 45.0 if args.target_heading_deg is None else args.target_heading_deg
        return target_velocity(speed, heading)
    return args.target_vn_ms, args.target_ve_ms


def follow_stats(history):
    """Aircraft-to-target range statistics over a run (TASK-003).

    Returns ``{"initial", "min", "final", "target_moved"}`` in metres, or
    ``None`` for an empty history. ``target_moved`` is the straight-line distance
    the target travelled over the run.
    """
    if not history:
        return None
    def rng(s):
        return math.hypot(
            s["plane_n_m"] - s["target_n_m"], s["plane_e_m"] - s["target_e_m"]
        )
    ranges = [rng(s) for s in history]
    first, last = history[0], history[-1]
    moved = math.hypot(
        last["target_n_m"] - first["target_n_m"],
        last["target_e_m"] - first["target_e_m"],
    )
    return {
        "initial": ranges[0],
        "min": min(ranges),
        "final": ranges[-1],
        "target_moved": moved,
    }


def build_config(args, weave_eta=None):
    overrides = {}
    if args.airspeed_ms is not None:
        overrides["airspeed_ms"] = args.airspeed_ms
    if args.turn_radius_m is not None:
        overrides["turn_radius_m"] = args.turn_radius_m
    if args.orbit_radius_m is not None:
        overrides["orbit_radius_m"] = args.orbit_radius_m
    if args.eta is not None:
        overrides["weave_eta"] = args.eta
    if args.weave_lambda is not None:
        overrides["weave_lambda_m"] = args.weave_lambda
    if args.weave_a_cap is not None:
        overrides["weave_a_cap_m"] = args.weave_a_cap
    if args.weave_d_start is not None:
        overrides["weave_d_start_m"] = args.weave_d_start
    if args.weave_d_full is not None:
        overrides["weave_d_full_m"] = args.weave_d_full
    if weave_eta is not None:  # explicit override wins (the compare-eta pass)
        overrides["weave_eta"] = weave_eta
    return HarnessConfig(**overrides)


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        cfg = build_config(args)
    except ValueError as exc:
        print("INVALID CONFIGURATION: %s" % exc, file=sys.stderr)
        return 2

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

    selected = args.algorithms or ["dubins_orbit"]

    # Each entry is (algorithm_name, display_label, config). --compare-eta adds a
    # second pass for amplitude algorithms with eta = 1, overlaid on one plot.
    amplitude_family = ("amplitude", "amplitude_orbit", "continuous_weave")
    pass_specs = []
    for name in selected:
        if args.compare_eta and name in amplitude_family:
            pass_specs.append((name, "%s (eta=%.2f)" % (name, cfg.weave_eta), cfg))
            pass_specs.append(
                (name, "%s (eta=1)" % name, build_config(args, weave_eta=1.0))
            )
        else:
            pass_specs.append((name, name, cfg))

    plane_hdg0 = math.radians(args.plane_heading_deg)
    target_n0 = args.start_range_m if args.target_n_m is None else args.target_n_m
    target_e0 = 0.0 if args.target_e_m is None else args.target_e_m
    target_vn, target_ve = resolve_target_velocity(args, cfg.airspeed_ms)

    # Kangaroo motion mode (TASK-009) overrides the --target-* placement/velocity.
    kang = None
    if args.kang_mode is not None:
        try:
            kang = kangaroo.build(
                args.kang_mode, heading_deg=args.kang_heading_deg,
                fwd_m=args.kang_fwd_m, disp_m=args.kang_disp_m,
                radius_m=args.kang_radius_m, length_m=args.kang_length_m,
                width_m=args.kang_width_m, speed_ms=args.kang_speed_ms,
            )
            target_n0, target_e0, target_vn, target_ve = kang(0.0)
        except ValueError as exc:
            print("INVALID CONFIGURATION: %s" % exc, file=sys.stderr)
            return 2
        print("kangaroo mode: %s" % args.kang_mode)
        print()

    if target_vn or target_ve:
        print(
            "moving target: %.1f m/s on heading %.0f deg (vn=%.1f, ve=%.1f)"
            % (math.hypot(target_vn, target_ve),
               math.degrees(math.atan2(target_ve, target_vn)) % 360.0,
               target_vn, target_ve)
        )
        print()

    runs = []
    planned = []
    for algo_name, label, algo_cfg in pass_specs:
        harness = Harness(
            plane=PlaneState(
                n_m=0.0,
                e_m=0.0,
                hdg_rad=plane_hdg0,
                speed_ms=cfg.airspeed_ms,
            ),
            target=TargetState(
                n_m=target_n0,
                e_m=target_e0,
                vn_ms=target_vn,
                ve_ms=target_ve,
            ),
            algorithm=algorithms.build(algo_name, algorithms.config_dict(algo_cfg)),
            dt_s=cfg.dt_s,
            turn_radius_m=cfg.turn_radius_m,
            infeasible=bool(problems),
            kangaroo=kang,
            estimator=_new_estimator(args, target_n0, target_e0),
        )
        harness.run(args.duration_s)

        if not harness.history:
            print("%s: no steps completed" % label)
            continue

        last = harness.history[-1]
        final_range = math.hypot(
            last["plane_n_m"] - last["target_n_m"], last["plane_e_m"] - last["target_e_m"]
        )
        print(
            "%s: %d steps, %.1f s, final range %.1f m"
            % (label, len(harness.history), last["t_s"], final_range)
        )

        if target_vn or target_ve:
            fs = follow_stats(harness.history)
            outcome = "closing" if fs["final"] < fs["initial"] else "trailing"
            print(
                "  moving-target follow: range %.1f -> %.1f m (min %.1f), "
                "target travelled %.1f m [%s]"
                % (fs["initial"], fs["final"], fs["min"], fs["target_moved"], outcome)
            )

        achieved = harness.achieved_orbit_radius_m()
        if achieved is not None and algo_name in ("orbit", "dubins_orbit", "heading_a_orbit",
                                                  "amplitude_orbit"):
            print(
                "  commanded orbit radius %.1f m, achieved %.1f m (%.0f%%)"
                % (cfg.orbit_radius_m, achieved, 100.0 * achieved / cfg.orbit_radius_m)
            )

        if harness.stopped_reason:
            print("  stopped early: %s" % harness.stopped_reason)

        if args.save_run:
            stem, _, ext = args.save_run.rpartition(".")
            path = "%s-%s.%s" % (stem or args.save_run, label, ext or "json")
            plotter.save_run(path, label, harness.history, meta={
                "turn_radius_m": cfg.turn_radius_m,
                "orbit_radius_m": cfg.orbit_radius_m,
                "airspeed_ms": cfg.airspeed_ms,
            })
            print("  saved %s" % path)

        runs.append((label, harness.history))

        # The exact planned Dubins geometry from the initial pose, for the plotter
        # to overlay on the flown track (TASK-002). Computed here, not in the
        # read-only plotter, which receives only points.
        if algo_name in ("dubins", "dubins_orbit"):
            planned.append(
                dubins_orbit_geom.planned_path(
                    0.0, 0.0, plane_hdg0, target_e0, target_n0,
                    cfg.orbit_radius_m, cfg.turn_radius_m,
                    cfg.delta_psi_rad, cfg.delta_d_m,
                )
            )
        else:
            planned.append(None)

    if not runs:
        print("no run produced a trajectory")
        return 1

    # Only draw the ring when an orbit-family algorithm is present, so a pure
    # amplitude run is not overlaid with a ring it does not use.
    orbit_present = any(
        n in ("orbit", "dubins_orbit", "amplitude_orbit", "heading_a_orbit") for n in selected
    )
    orbit_m = cfg.orbit_radius_m if orbit_present else None

    if args.animate:
        plotter.animate_runs(runs, orbit_radius_m=orbit_m, save_path=args.animate)
        print("  animation saved %s" % args.animate)
    elif not args.no_plot:
        plotter.plot_runs(
            runs,
            orbit_radius_m=orbit_m,
            show_3d=not args.no_3d,
            planned=planned,
            save_path=args.save_plot,
            show=args.save_plot is None,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
