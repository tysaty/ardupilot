"""Heading-alignment guidance — the simple baseline (``TASK-010``).

A straight fly-to-the-target law: the guidance point is placed a look-ahead along
the bearing to the (estimated) kangaroo, so the aircraft aligns its heading with
the direction to the target. ``heading_a`` is the pure fly-to; ``heading_a_orbit``
ramps it into the orbit, reusing the ``dubins_orbit`` handoff.

The bearing convention (``IR-002``) is heading clockwise from North,
``atan2(dEast, dNorth)`` — the objective's ``atan(y, x)`` note, corrected. Here it
is implicit: the point is placed along the unit vector to the target, which is the
same thing without forming the angle. Distance is the Euclidean
``hypot(dNorth, dEast)``.

Stateless and Lua-transliterable: plain ``math``, plain floats. The estimated
target (from the state estimator, ``TASK-012``) enters through the snapshot, not
this module. Frame ``x = East``, ``y = North``.
"""

import math

from . import dubins_orbit
from . import orbit as orbit_geom  # noqa: F401  (kept parallel to other adapters)


def guidance(px, py, tx, ty, look_ahead_m):
    """Guidance point one look-ahead along the bearing to the target.

    Returns ``(gx, gy)`` in ``(East, North)``.

    Raises:
        ValueError: When the aircraft is on the target (no bearing).
    """
    dx, dy = tx - px, ty - py
    distance_m = math.hypot(dx, dy)
    if distance_m < 1e-6:
        raise ValueError("aircraft is on the target; no heading to align to")
    u = min(look_ahead_m, distance_m)
    return px + u * dx / distance_m, py + u * dy / distance_m


def guidance_orbit(px, py, psi_i, tx, ty, orbit_radius_m, look_ahead_m,
                   precompensate=True):
    """Heading fly-to ramped into the orbit about the target (``TASK-010``).

    Outside a look-ahead of the ring the guidance points at the target; within it
    the orbit takes over, blended by the same ramp as ``dubins_orbit`` so the
    point is continuous. Returns ``(gx, gy, phase)``.

    Raises:
        ValueError: When the aircraft is on the target and neither is defined.
    """
    range_m = math.hypot(tx - px, ty - py)
    w = dubins_orbit.ramp_weight(range_m, orbit_radius_m, look_ahead_m)

    head = None
    if w < 1.0:
        try:
            head = guidance(px, py, tx, ty, look_ahead_m)
        except ValueError:
            head = None
    orb = None
    if w > 0.0:
        orb = dubins_orbit.orbit_guidance(px, py, psi_i, tx, ty, orbit_radius_m,
                                          look_ahead_m, precompensate)

    if head is None and orb is None:
        raise ValueError("no heading approach and no orbit point (on the target?)")
    if orb is None:
        return head[0], head[1], 0.0
    if head is None:
        gx, gy, _psi = orb
        return gx, gy, 1.0
    ox, oy, _psi = orb
    return (1.0 - w) * head[0] + w * ox, (1.0 - w) * head[1] + w * oy, w
