"""Dubins-approach-to-orbit geometry with a ramped handoff (``TASK-002``).

This is the combined geometry behind :class:`algorithms.DubinsOrbitAlgorithm`:
fly the shortest Dubins path to a tangent entry on the ring of radius
``orbit_radius_m`` about the target, then circle. It was previously built inline
in the adapter; ``TASK-002`` factored it here so the adapter holds no geometry.

The one behavioural addition over the inline version is the **ramp**. The old
handoff was a hard distance test: outside a look-ahead of the ring the guidance
point came from the approach path, inside it came from the orbit, and the point
could step at the boundary. :func:`ramp_weight` blends the two over a transition
band so the guidance point is continuous across the handoff — the "simple ramp
function" the objective asks for.

Stateless and Lua-transliterable (``DEC-2026-06-25-04``): plain ``math``, plain
floats and dicts, no ``numpy``, no closures over module state. Frame throughout
is ``x = East``, ``y = North``, ``psi`` from North increasing clockwise, as in
:mod:`py_harness.geometry.dubins` and :mod:`py_harness.geometry.orbit`.
"""

import math

from . import dubins
from . import orbit


def ramp_weight(range_m, orbit_radius_m, look_ahead_m):
    """Blend weight for the Dubins-to-orbit handoff, in ``[0, 1]``.

    ``0`` outside the transition band (``range >= R + look_ahead``): pure
    approach. ``1`` on or inside the ring (``range <= R``): pure orbit. Linear in
    between. The band width is one look-ahead, so the ramp starts exactly where
    the old hard switch used to fire and finishes at the ring.
    """
    band = look_ahead_m
    outer = orbit_radius_m + band
    if band <= 0.0:
        return 0.0 if range_m > orbit_radius_m else 1.0
    if range_m >= outer:
        return 0.0
    if range_m <= orbit_radius_m:
        return 1.0
    return (outer - range_m) / band


def ring_entry(px, py, tx, ty, orbit_radius_m):
    """The tangent entry pose on the ring, or ``None`` if the start is inside it.

    Returns ``(ex, ey, psi_f)`` for the first of the two tangent options — the
    same choice the inline adapter made. The entry lies on the ring by
    construction (:func:`orbit.tangent_points`), so the Dubins path is flown to
    the ring, not to the target point.
    """
    entries = orbit.tangent_points(px, py, tx, ty, orbit_radius_m)
    if not entries:
        return None
    ex, ey, psi_f, _sign = entries[0]
    return ex, ey, psi_f


def approach_guidance(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                      look_ahead_m, delta_psi_rad, delta_d_m):
    """Guidance point on the shortest Dubins path to the ring's tangent entry.

    Returns ``(gx, gy, length, family)``, or ``None`` when the start is inside
    the ring or no Dubins family solves. ``family`` is the shortest solvable
    family by arc length (:func:`dubins.shortest`).
    """
    entry = ring_entry(px, py, tx, ty, orbit_radius_m)
    if entry is None:
        return None
    ex, ey, psi_f = entry
    best = dubins.shortest(
        px, py, psi_i, ex, ey, psi_f, turn_radius_m, delta_psi_rad, delta_d_m
    )
    if best is None:
        return None
    family, pts, length = best
    gx, gy = orbit.point_at_arc_length(pts, look_ahead_m)
    return gx, gy, length, family


def orbit_guidance(px, py, psi_i, tx, ty, orbit_radius_m, look_ahead_m):
    """Guidance point a look-ahead arc length around the ring.

    Returns ``(gx, gy, ring_angle_rad)``, or ``None`` when the aircraft is at the
    ring centre and no orbit angle is defined.
    """
    if math.hypot(px - tx, py - ty) < 1e-6:
        return None
    psi0 = orbit.entry_angle(px, py, tx, ty)
    direction = orbit.orbit_direction(psi0, psi_i)
    gx, gy, psi = orbit.orbit_point_at_arc_length(
        tx, ty, orbit_radius_m, psi0, direction, look_ahead_m
    )
    return gx, gy, psi


def planned_path(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                 delta_psi_rad, delta_d_m):
    """The exact Dubins curve from the pose to the ring's tangent entry.

    This is the geometry ``py_plots`` draws — the selected L/S/R path itself,
    not a flown track. It is returned as a list of ``(East, North)`` points so a
    read-only plotter can draw it as data without importing any harness module
    (``DEC-2026-07-22-01``). The path ends on the ring by construction, so it
    lands "onto the projected circle" (``TASK-002``).

    Returns an empty list when the start is inside the ring or no Dubins family
    solves.
    """
    entry = ring_entry(px, py, tx, ty, orbit_radius_m)
    if entry is None:
        return []
    ex, ey, psi_f = entry
    best = dubins.shortest(
        px, py, psi_i, ex, ey, psi_f, turn_radius_m, delta_psi_rad, delta_d_m
    )
    if best is None:
        return []
    _family, pts, _length = best
    return [(p[0], p[1]) for p in pts]


def guidance(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
             look_ahead_m, delta_psi_rad, delta_d_m):
    """One guidance point: the Dubins approach ramped into the orbit.

    Returns a dict ``{"gx", "gy", "phase", "family", "ring_angle_rad"}``, where
    ``phase`` is the ramp weight in ``[0, 1]`` (0 pure approach, 1 pure orbit),
    ``family`` is the Dubins family used or ``None``, and ``ring_angle_rad`` is
    the orbit angle or ``None``.

    Within the band both the approach and the orbit points are computed and
    blended, so the result is continuous where the old hard switch stepped. At
    the band edges the blend collapses to the pure case.

    Raises:
        ValueError: When neither an approach nor an orbit point exists — the
            aircraft is at the ring centre, or it is outside the ring and no
            Dubins family reaches the tangent entry.
    """
    w = ramp_weight(math.hypot(px - tx, py - ty), orbit_radius_m, look_ahead_m)

    approach = None
    if w < 1.0:
        approach = approach_guidance(
            px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
            look_ahead_m, delta_psi_rad, delta_d_m,
        )
    orb = None
    if w > 0.0:
        orb = orbit_guidance(px, py, psi_i, tx, ty, orbit_radius_m, look_ahead_m)

    if approach is None and orb is None:
        raise ValueError(
            "no Dubins approach to the ring and no orbit point (at the centre?)"
        )
    if orb is None:
        gx, gy, _length, family = approach
        return {"gx": gx, "gy": gy, "phase": 0.0,
                "family": family, "ring_angle_rad": None}
    if approach is None:
        gx, gy, psi = orb
        return {"gx": gx, "gy": gy, "phase": 1.0,
                "family": None, "ring_angle_rad": psi}

    ax, ay, _length, family = approach
    ox, oy, psi = orb
    return {
        "gx": (1.0 - w) * ax + w * ox,
        "gy": (1.0 - w) * ay + w * oy,
        "phase": w,
        "family": family,
        "ring_angle_rad": psi,
    }
