"""Continue the orbit around the target, ramp-free, on the TASK-024 circle (``TASK-025``).

``TASK-024`` (:mod:`py_harness.geometry.dubins_target_circle`) flies a Dubins path
whose final turn circle is the target-centred ring and **arrives tangent** to it —
but only reaches the ring. This module adds the missing half: once on the ring, keep
**orbiting** around the target.

The hand-off uses **no ramp** (unlike :mod:`dubins_orbit`, which blends approach and
orbit with a linear weight over a look-ahead band). Because the ``TASK-024`` approach
arrives *tangent* to the ring, the switch is continuous **by geometry**:

* **Outside the ring** (``d = |plane - T| > R``): approach on the ``TASK-024``
  target-centred final circle.
* **On / inside the ring** (``d <= R``): continue around the ring in closed form —
  advance the guidance point a look-ahead of arc about the target, in the sense the
  current heading implies (:func:`orbit.orbit_direction`), which matches the sense
  the tangent approach arrived with.

Stateless: the sense is re-derived from the heading each tick, not stored, so no
mode flag or ramp weight is carried between ticks. The orbit curvature is
``1/R <= 1/R_min`` (guarded by ``R >= R_min``). Frame ``x = East, y = North``,
``psi`` from North clockwise (``IR-008``).
"""

import math

from . import dubins_target_circle as dtc
from . import orbit as orbit_geom


def guidance(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
             look_ahead_m, delta_psi, delta_d):
    """One guidance point: approach outside the ring, orbit continuation on it.

    Returns a dict ``{"gx", "gy", "phase", "direction", "curvature",
    "ring_angle_rad"?}``. ``phase`` is ``"approach"`` (outside) or ``"orbit"``
    (on/inside) — a discrete geometric switch, never a blended ramp weight.
    ``direction`` is ``"cw"``/``"ccw"``; ``ring_angle_rad`` is present in the orbit
    phase only.

    Raises:
        ValueError: If ``orbit_radius_m < turn_radius_m`` (curvature bound) or the
            aircraft is at the ring centre; or propagated from the approach.
    """
    R = orbit_radius_m
    if R < turn_radius_m - 1e-9:
        raise ValueError(
            "target circle radius %.3f m < minimum turn radius %.3f m: the orbit "
            "would exceed the curvature bound" % (R, turn_radius_m))

    d = math.hypot(px - tx, py - ty)
    if d < 1e-6:
        raise ValueError("aircraft is at the ring centre; no orbit angle")

    if d > R:
        # Outside the ring: approach on the TASK-024 target-centred final circle.
        g = dtc.guidance(px, py, psi_i, tx, ty, R, turn_radius_m,
                         look_ahead_m, delta_psi, delta_d)
        return {
            "gx": g["gx"],
            "gy": g["gy"],
            "phase": "approach",
            "direction": g["direction"],
            "curvature": g["curvature"],
        }

    # On / inside the ring: continue around it. No ramp — the tangent arrival makes
    # this continuous. Closed-form advance, so no sampling error enters guidance.
    psi0 = orbit_geom.entry_angle(px, py, tx, ty)
    direction = orbit_geom.orbit_direction(psi0, psi_i)
    gx, gy, psi = orbit_geom.orbit_point_at_arc_length(
        tx, ty, R, psi0, direction, look_ahead_m)
    return {
        "gx": gx,
        "gy": gy,
        "phase": "orbit",
        "direction": "cw" if direction > 0 else "ccw",
        "curvature": 1.0 / R,
        "ring_angle_rad": psi,
    }
