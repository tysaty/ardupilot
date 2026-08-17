"""Static, plan-once Dubins geometry in the harness (``TASK-023``).

The standalone ``py_plots/dubins_path.py`` draws the **whole** Dubins geometry in
one shot: the ring about the target, the two tangent lines to it, the two
orbit-entry options, and all six Dubins families routed to a chosen entry. That
is the **plan-once** view — the entire path computed and shown at once — as
opposed to the receding-horizon ``dubins`` / ``dubins_orbit`` algorithms, which
walk one guidance point along a path re-solved every 10 Hz tick. There is **no
look-ahead interval and no replanning here** (``TASK-023`` scope).

This module brings that static construction into the harness. It holds **no
state** and no ``numpy``: :func:`build_construction` assembles a plain-dict
description by **reusing the already-ported geometry** —
:func:`py_harness.geometry.orbit.tangent_points` for the tangent entries and
:func:`py_harness.geometry.dubins.generate_all` for the six families — and hands
that dict to the **read-only** plotter (``DEC-2026-07-22-01``). Nothing here
drives an algorithm, mutates harness state, or edits ``geometry`` / ``algorithms``.

Curvature and turn-radius state are taken from the **harness** config, not the
toy constants hard-coded in ``py_plots/dubins_path.py`` (``rho = 5``,
``ring = 3``): the CLI defaults ``rho``, the ring radius and the sampling from
:class:`~py_harness.config.HarnessConfig`, and the construction reports the arc
curvature ``1/rho`` the harness cares about.
"""

import argparse
import math

from .geometry import dubins as dubins_geom
from .geometry import orbit as orbit_geom


def _bearing(sx, sy, tx, ty):
    """Heading from ``S`` to ``T`` in the harness convention (0 = North, +CW)."""
    return math.atan2(tx - sx, ty - sy)


def build_construction(sx, sy, psi_i, tx, ty, ring_radius_m, rho,
                       delta_psi, delta_d, entry_index=0):
    """Assemble the static Dubins construction as a plain, drawable dict.

    Two modes, selected by ``ring_radius_m``:

    * **Ring approach** (``ring_radius_m > 0``): the two tangent lines to the ring
      about the target and their entry poses (``tangent_points``); the six Dubins
      families are routed from the start pose to the chosen tangent entry
      (``entry_index``, the CW/CCW option). This mirrors ``dubins_path.py``.
    * **Direct to point** (``ring_radius_m`` falsy): the families are routed
      straight to the target, arriving on the bearing to it — the bare ``dubins``
      terminal condition, with no standoff ring.

    All coordinates are the geometry frame ``x = East, y = North`` and headings
    are ``0 = North, +clockwise`` (``IR-008``), unchanged from the source geometry.

    Args:
        sx, sy: Start position (East, North), m.
        psi_i: Start heading, rad (0 = North, +CW).
        tx, ty: Target position (East, North), m.
        ring_radius_m: Standoff ring radius, m; falsy for direct-to-point.
        rho: Minimum turn radius, m (> 0). Its curvature ``1/rho`` is reported.
        delta_psi: Arc-point spacing, rad.
        delta_d: Straight-point spacing, m.
        entry_index: Which tangent entry the families fly to, ``0`` or ``1``
            (ignored in direct-to-point mode).

    Returns:
        A dict with keys:
        ``start`` ``(sx, sy, psi_i)``; ``target`` ``(tx, ty)``;
        ``ring_radius_m`` (float, ``0.0`` for direct); ``rho_m``;
        ``curvature_1_per_m`` (``1/rho`` or ``None``);
        ``entries`` list of ``(px, py, psi_f, sign)`` (empty when the start is
        inside the ring, the no-solution case); ``entry_index`` used or ``None``;
        ``goal`` ``(gx, gy, psi_f)`` the terminal pose the families fly to, or
        ``None`` when there is no reachable goal; ``families`` mapping each of the
        six names to ``{"points": [(x, y), ...], "length": float or None,
        "solved": bool}``; and ``shortest`` the shortest solvable family name or
        ``None``.

    Raises:
        ValueError: If ``entry_index`` is not 0 or 1, or ``rho`` is not positive.
    """
    if entry_index not in (0, 1):
        raise ValueError("entry_index must be 0 or 1, got %r" % (entry_index,))
    if rho <= 0.0:
        raise ValueError("rho (turn radius) must be positive, got %r" % (rho,))

    ring = float(ring_radius_m) if ring_radius_m else 0.0
    result = {
        "start": (sx, sy, psi_i),
        "target": (tx, ty),
        "ring_radius_m": ring,
        "rho_m": rho,
        "curvature_1_per_m": 1.0 / rho,
        "entries": [],
        "entry_index": None,
        "goal": None,
        "families": {name: {"points": [], "length": None, "solved": False}
                     for name, _ in dubins_geom.FAMILIES},
        "shortest": None,
    }

    if ring > 0.0:
        entries = orbit_geom.tangent_points(sx, sy, tx, ty, ring)
        result["entries"] = entries
        if not entries:
            return result  # start inside the ring: no tangent, no path
        idx = entry_index if entry_index < len(entries) else 0
        gx, gy, psi_f, _sign = entries[idx]
        result["entry_index"] = idx
    else:
        # Direct to the target point, arriving on the bearing to it.
        if math.hypot(tx - sx, ty - sy) < 1e-9:
            return result  # start is on the target: no path defined
        gx, gy, psi_f = tx, ty, _bearing(sx, sy, tx, ty)

    result["goal"] = (gx, gy, psi_f)

    paths = dubins_geom.generate_all(sx, sy, psi_i, gx, gy, psi_f,
                                     rho, delta_psi, delta_d)
    best_name, best_len = None, None
    for name, _ in dubins_geom.FAMILIES:
        pts, length = paths[name]
        solved = bool(pts)
        result["families"][name] = {
            "points": [(p[0], p[1]) for p in pts] if solved else [],
            "length": length if solved else None,
            "solved": solved,
        }
        if solved and (best_len is None or length < best_len):
            best_name, best_len = name, length
    result["shortest"] = best_name
    return result


def main(argv=None):
    """Entry point: ``python3 -m py_harness.dubins_static [options]``.

    Places the start at the origin and the target due North at ``--start-range-m``
    unless ``--target-e-m`` / ``--target-n-m`` override it, mirroring
    ``run_harness``. Defaults for the turn radius, ring radius and sampling come
    from :class:`~py_harness.config.HarnessConfig` (the harness values, not the
    ``py_plots`` toy constants).
    """
    from .config import HarnessConfig
    from . import plotter

    defaults = HarnessConfig()
    parser = argparse.ArgumentParser(
        description="Static plan-once Dubins geometry (TASK-023): the ring, its "
        "two tangent entries and all six Dubins families in one figure.")
    parser.add_argument("--start-range-m", type=float, default=250.0,
                        help="Start-to-target range, m (target due North).")
    parser.add_argument("--target-n-m", type=float, default=None,
                        help="Target North, m (overrides --start-range-m).")
    parser.add_argument("--target-e-m", type=float, default=0.0,
                        help="Target East, m.")
    parser.add_argument("--plane-heading-deg", type=float, default=0.0,
                        help="Start heading, deg (0 = North, +CW).")
    parser.add_argument("--turn-radius-m", type=float,
                        default=defaults.turn_radius_m,
                        help="Minimum turn radius rho, m.")
    parser.add_argument("--orbit-radius-m", type=float,
                        default=defaults.orbit_radius_m,
                        help="Standoff ring radius, m. Use 0 for direct-to-point.")
    parser.add_argument("--delta-psi-deg", type=float,
                        default=math.degrees(defaults.delta_psi_rad),
                        help="Arc-point spacing, deg.")
    parser.add_argument("--delta-d-m", type=float, default=defaults.delta_d_m,
                        help="Straight-point spacing, m.")
    parser.add_argument("--entry", type=int, choices=(0, 1), default=0,
                        help="Which tangent entry the families fly to (0 or 1).")
    parser.add_argument("--save-plot", default=None,
                        help="Write the figure to PNG instead of showing it.")
    args = parser.parse_args(argv)

    ty = args.target_n_m if args.target_n_m is not None else args.start_range_m
    construction = build_construction(
        0.0, 0.0, math.radians(args.plane_heading_deg),
        args.target_e_m, ty, args.orbit_radius_m, args.turn_radius_m,
        math.radians(args.delta_psi_deg), args.delta_d_m, entry_index=args.entry,
    )

    shortest = construction["shortest"]
    if shortest is None:
        if construction["ring_radius_m"] > 0.0 and not construction["entries"]:
            print("start is inside the ring: no tangent approach")
        else:
            print("no Dubins family solves this configuration")
    else:
        length = construction["families"][shortest]["length"]
        print("shortest family: %s (%.1f m)" % (shortest, length))

    plotter.plot_dubins_construction(
        construction, save_path=args.save_plot, show=args.save_plot is None)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
