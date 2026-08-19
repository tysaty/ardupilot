"""Variable-amplitude weave combined with the orbit, via the ramp (``TASK-014``).

The variable-amplitude weave (:mod:`py_harness.geometry.var_amplitude_weave`) is
the approach; once the aircraft is within a look-ahead of the ring about the
target it hands off to the orbit, blended by the same **ramp** used for
``amplitude_orbit`` (``TASK-004``) and ``dubins_orbit`` (``TASK-002``), so the
guidance point is continuous across the handoff.

The ring is retained about the (possibly moving) target, keeping the on-orbit
behaviour in its final state. Stateless and Lua-transliterable, like the modules
it composes.
"""

import math

from . import dubins_orbit
from . import var_amplitude_weave as vaw_geom


def guidance(px, py, psi_i, tx, ty, s_m, v_rel_ms, orbit_radius_m, look_ahead_m,
             lambda_m, r_min_m, a_cap_m, d_full_m, lead_s, eta, phase_rad=0.0,
             v_floor_ms=vaw_geom.V_FLOOR_MS, precompensate=True):
    """One guidance point: the variable-amplitude weave ramped into the orbit.

    Returns a dict ``{gx, gy, phase, amplitude}`` where ``phase`` is the ramp
    weight in ``[0, 1]`` (0 pure weave approach, 1 pure orbit) and ``amplitude``
    is the weave amplitude in the approach region (``None`` on the ring).

    Raises:
        ValueError: When the aircraft is on the target and neither the weave nor
            the orbit is defined.
    """
    range_m = math.hypot(px - tx, py - ty)
    w = dubins_orbit.ramp_weight(range_m, orbit_radius_m, look_ahead_m)

    weave = None
    if w < 1.0:
        try:
            weave = vaw_geom.guidance(
                px, py, psi_i, tx, ty, s_m, v_rel_ms, lambda_m, r_min_m,
                a_cap_m, d_full_m, lead_s, eta, look_ahead_m, phase_rad,
                v_floor_ms,
            )
        except ValueError:
            weave = None
    orb = None
    if w > 0.0:
        orb = dubins_orbit.orbit_guidance(
            px, py, psi_i, tx, ty, orbit_radius_m, look_ahead_m,
            precompensate
        )

    if weave is None and orb is None:
        raise ValueError("no weave approach and no orbit point (on the target?)")
    if orb is None:
        return {"gx": weave["gx"], "gy": weave["gy"], "phase": 0.0,
                "amplitude": weave["amplitude"]}
    if weave is None:
        gx, gy, _psi = orb
        return {"gx": gx, "gy": gy, "phase": 1.0, "amplitude": None}

    ox, oy, _psi = orb
    return {
        "gx": (1.0 - w) * weave["gx"] + w * ox,
        "gy": (1.0 - w) * weave["gy"] + w * oy,
        "phase": w,
        "amplitude": weave["amplitude"],
    }
