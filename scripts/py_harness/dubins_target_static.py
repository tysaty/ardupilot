"""Single-shot altered Dubins to the target ring, and its orbit continuation.

``TASK-024`` (approach) and ``TASK-025`` (approach + orbit), the **plan-once, no
look-ahead** form. The altered Dubins is a **CS** path — one initial turn (radius
``turn_radius_m``) then one straight — that **flies into the tangent** on the
kangaroo ring (radius ``orbit_radius_m``). It is computed **once** from the
configuration and committed to; there is **no ``look_ahead`` parameter** anywhere in
this module and no receding-horizon replanning. ``TASK-025`` appends the **exact**
orbit continuation on the ring (radius ``orbit_radius_m`` precisely — dropping the
carrot removes the ``A-VAL-005`` pull-in that the flown variant, ``TASK-026``,
settles at).

This mirrors the plan-once approach of :mod:`py_harness.dubins_static` (``TASK-023``):
:func:`build` reuses the geometry (``dubins_target_circle.shortest_path`` and
``orbit``) to assemble a plain construction dict, which the **read-only** plotter
draws (``DEC-2026-07-22-01``). Stateless, no ``numpy`` (``VR-015``); frame
``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).

The receding-horizon *flown* target-circle / orbit algorithms are a **separate
task** (``TASK-026``); this module does not use or modify them.
"""

import argparse
import math

from .geometry import dubins_target_circle as dtc
from .geometry import orbit as orbit_geom


def build(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
          delta_psi, delta_d, orbit_turns=0.0):
    """Assemble the single-shot construction as a plain, drawable dict.

    Note there is **no** ``look_ahead`` parameter: the committed path is the
    guidance, not a point chased ahead of the aircraft.

    Args:
        px, py: Start position (East, North), m.
        psi_i: Start heading, rad (0 = North, +CW).
        tx, ty: Target position (East, North), m.
        orbit_radius_m: Kangaroo ring radius, m (>= ``turn_radius_m``).
        turn_radius_m: Initial-turn radius, m.
        delta_psi: Arc-point spacing, rad.
        delta_d: Straight-point spacing, m.
        orbit_turns: Revolutions of orbit continuation to append from the tangency
            (``0`` = ``TASK-024`` approach only; ``> 0`` = ``TASK-025``).

    Returns:
        A dict with keys: ``start`` ``(px, py, psi_i)``; ``target`` ``(tx, ty)``;
        ``orbit_radius_m``; ``turn_radius_m``; ``approach`` ``[(e, n), ...]`` (arc
        then straight, ending at the tangency point); ``arrival`` ``(e, n)`` the
        tangency point on the ring; ``direction`` ``"cw"``/``"ccw"``;
        ``reach_length_m`` the turn-in cost; and ``orbit`` ``[(e, n), ...]`` (empty
        when ``orbit_turns == 0``), every point exactly ``orbit_radius_m`` from the
        target.

    Raises:
        ValueError: If ``orbit_turns < 0``, or propagated from
            :func:`dubins_target_circle.shortest_path` (ring below the turn radius,
            aircraft inside the ring, or no tangent).
    """
    if orbit_turns < 0.0:
        raise ValueError("orbit_turns must be >= 0, got %r" % (orbit_turns,))

    appr, reach, direction, arrival = dtc.shortest_path(
        px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m, delta_psi, delta_d)

    result = {
        "start": (px, py, psi_i),
        "target": (tx, ty),
        "orbit_radius_m": orbit_radius_m,
        "turn_radius_m": turn_radius_m,
        "approach": [(p[0], p[1]) for p in appr],
        "arrival": arrival,
        "direction": direction,
        "reach_length_m": reach,
        "orbit": [],
    }

    if orbit_turns > 0.0:
        psi0 = orbit_geom.entry_angle(arrival[0], arrival[1], tx, ty)
        sense = 1 if direction == "cw" else -1
        total = orbit_turns * 2.0 * math.pi
        n = max(2, int(math.ceil(total / delta_psi)))
        result["orbit"] = [
            orbit_geom.orbit_point(tx, ty, orbit_radius_m,
                                   psi0 + sense * total * i / n)
            for i in range(n + 1)
        ]
    return result


def main(argv=None):
    """Entry point: ``python3 -m py_harness.dubins_target_static [options]``.

    Places the start at the origin and the target due North at ``--start-range-m``
    unless overridden, mirroring ``run_harness``. Defaults for the radii and
    sampling come from :class:`~py_harness.config.HarnessConfig`.
    """
    from .config import HarnessConfig
    from . import plotter

    defaults = HarnessConfig()
    parser = argparse.ArgumentParser(
        description="Single-shot altered Dubins to the target ring (TASK-024) and "
        "its orbit continuation (TASK-025) — plan-once, no look-ahead.")
    parser.add_argument("--start-range-m", type=float, default=300.0,
                        help="Start-to-target range, m (target due North).")
    parser.add_argument("--target-n-m", type=float, default=None,
                        help="Target North, m (overrides --start-range-m).")
    parser.add_argument("--target-e-m", type=float, default=0.0,
                        help="Target East, m.")
    parser.add_argument("--plane-heading-deg", type=float, default=60.0,
                        help="Start heading, deg (0 = North, +CW).")
    parser.add_argument("--turn-radius-m", type=float,
                        default=defaults.turn_radius_m, help="Turn radius rho, m.")
    parser.add_argument("--orbit-radius-m", type=float,
                        default=defaults.orbit_radius_m, help="Ring radius, m.")
    parser.add_argument("--delta-psi-deg", type=float,
                        default=math.degrees(defaults.delta_psi_rad),
                        help="Arc-point spacing, deg.")
    parser.add_argument("--delta-d-m", type=float, default=defaults.delta_d_m,
                        help="Straight-point spacing, m.")
    parser.add_argument("--orbit-turns", type=float, default=0.0,
                        help="Orbit revolutions to append (0 = TASK-024 approach "
                        "only; > 0 = TASK-025 approach + orbit).")
    parser.add_argument("--save-plot", default=None,
                        help="Write the static figure to PNG instead of showing it.")
    parser.add_argument("--animate", default=None,
                        help="Write a GIF animating the committed path over time.")
    parser.add_argument("--show-animation", action="store_true",
                        help="Show the animation in a matplotlib window.")
    args = parser.parse_args(argv)

    ty = args.target_n_m if args.target_n_m is not None else args.start_range_m
    try:
        construction = build(
            0.0, 0.0, math.radians(args.plane_heading_deg),
            args.target_e_m, ty, args.orbit_radius_m, args.turn_radius_m,
            math.radians(args.delta_psi_deg), args.delta_d_m,
            orbit_turns=args.orbit_turns)
    except ValueError as exc:
        print("no single-shot path: %s" % exc)
        return 2

    print("single-shot: direction=%s, turn-in reach=%.1f m, arrival=(%.1f, %.1f), "
          "orbit_turns=%.2f" % (construction["direction"],
          construction["reach_length_m"], construction["arrival"][0],
          construction["arrival"][1], args.orbit_turns))

    if args.show_animation:
        plotter.animate_target_circle_static(construction, show=True)
    elif args.animate:
        plotter.animate_target_circle_static(construction, save_path=args.animate)
        print("  animation saved %s" % args.animate)
    else:
        plotter.plot_target_circle_static(
            construction, save_path=args.save_plot, show=args.save_plot is None)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
