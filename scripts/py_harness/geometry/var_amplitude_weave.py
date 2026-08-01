"""Variable-amplitude weave geometry (``TASK-014``).

A weave whose amplitude **adapts to the closing geometry**: it is minimised (a
straight line) when the aircraft is far from the target and grows as the aircraft
approaches, with the onset scaled by the **relative speed** between the aircraft
and the target. It is a sibling of :mod:`py_harness.geometry.amplitude`
(``TASK-004``) and reuses that module's wave, curvature cap and helpers — the
only new part is how the *desired* amplitude is shaped.

Provisional amplitude law (``DEC-2026-07-28-04``; **not approved** — no numeric
relation was agreed at ``MTG-2026-07-28-01``, so this is a documented default to
be confirmed or replaced):

    d_start_eff = d_full + lead_s * max(v_rel, V_FLOOR)      # onset scales with speed
    q           = clamp((d_start_eff - distance) / (d_start_eff - d_full), 0, 1)
    A_desired   = a_cap * smoothstep(q)
    A           = min(A_desired, curvature_limited_amplitude(lambda, R_min, eta))

so far from the target ``q = 0`` and the path is straight; near the target the
amplitude fills to the cap; and at a fixed distance a higher relative speed moves
the onset outward, giving a larger amplitude. The amplitude is **always** bounded
by the curvature limit ``1/R_min`` (``ADR-002``, `SR-004` — a modelling limit).

Like the fixed-amplitude weave this is **plane-anchored** (``u = 0`` is the
aircraft), reproducing ``A-DEC-009`` rather than correcting it, per the direction
to keep anchoring for a later comparison of approaches.

Stateless and Lua-transliterable (``DEC-2026-06-25-04``): plain ``math``, plain
floats and dicts, no ``numpy``, no module state. Frame is ``x = East``,
``y = North``.
"""

import math

from .amplitude import (
    clamp,
    curvature,
    curvature_limited_amplitude,
    smoothstep,
    wave_derivatives,
)

#: Relative-speed floor, m/s. Keeps the onset window strictly positive when the
#: aircraft and target move together (``v_rel -> 0``), so ``q`` never divides by
#: zero. Small so it barely shifts the onset in the normal (closing) case.
V_FLOOR_MS = 0.1


def onset_distance_m(d_full_m, v_rel_ms, lead_s, v_floor_ms=V_FLOOR_MS):
    """Distance at which the weave begins, scaled by relative speed.

    ``d_start_eff = d_full + lead_s * max(v_rel, v_floor)``. A ``lead_s`` in
    seconds turns closing speed into an onset lead ahead of ``d_full``.

    Raises:
        ValueError: For a non-positive ``lead_s`` (the onset window would vanish).
    """
    if lead_s <= 0.0:
        raise ValueError("lead_s must be positive, got %r" % lead_s)
    v_eff = v_rel_ms if v_rel_ms > v_floor_ms else v_floor_ms
    return d_full_m + lead_s * v_eff


def speed_scaled_amplitude(distance_m, v_rel_ms, lambda_m, r_min_m, a_cap_m,
                           d_full_m, lead_s, eta, v_floor_ms=V_FLOOR_MS):
    """The commanded amplitude: the speed-scaled envelope, capped by curvature.

    Ramps 0 -> ``a_cap`` (via ``smoothstep``) as the distance falls from the
    speed-dependent onset to ``d_full``, then clamps to the curvature limit.

    Raises:
        ValueError: For a negative ``a_cap_m`` or ``d_full_m``, or a non-positive
            ``lead_s`` (propagated from :func:`onset_distance_m`).
    """
    if a_cap_m < 0.0:
        raise ValueError("a_cap_m must be non-negative, got %r" % a_cap_m)
    if d_full_m < 0.0:
        raise ValueError("d_full_m must be non-negative, got %r" % d_full_m)
    d_start_eff = onset_distance_m(d_full_m, v_rel_ms, lead_s, v_floor_ms)
    window = d_start_eff - d_full_m  # = lead_s * v_eff > 0
    q = clamp((d_start_eff - distance_m) / window, 0.0, 1.0)
    a_desired = a_cap_m * smoothstep(q)
    return min(a_desired, curvature_limited_amplitude(lambda_m, r_min_m, eta))


def guidance(px, py, psi_i, tx, ty, s_m, v_rel_ms, lambda_m, r_min_m, a_cap_m,
             d_full_m, lead_s, eta, look_ahead_m, phase_rad=0.0,
             v_floor_ms=V_FLOOR_MS):
    """One guidance point for the variable-amplitude weave, plane-anchored.

    The straight reference runs from the aircraft toward the target; a point one
    look-ahead along it is offset laterally by the speed-scaled, curvature-limited
    wave. Frame ``x = East``, ``y = North``.

    Returns a dict ``{gx, gy, amplitude, curvature, curvature_limit, onset_m}``.

    Raises:
        ValueError: When the aircraft is on the target (no weave direction).
    """
    dx, dy = tx - px, ty - py
    distance_m = math.hypot(dx, dy)
    if distance_m < 1e-6:
        raise ValueError("aircraft is on the target; no weave direction")

    # Unit vector toward the target and the left normal, in (East, North).
    ux, uy = dx / distance_m, dy / distance_m
    nx, ny = -uy, ux

    # Plane-anchored straight reference: u = 0 is the aircraft (A-DEC-009).
    u = min(look_ahead_m, distance_m)
    line_x = px + u * ux
    line_y = py + u * uy

    amp = speed_scaled_amplitude(distance_m, v_rel_ms, lambda_m, r_min_m,
                                 a_cap_m, d_full_m, lead_s, eta, v_floor_ms)
    y, y_prime, y_double_prime = wave_derivatives(s_m, amp, lambda_m, phase_rad)
    return {
        "gx": line_x + nx * y,
        "gy": line_y + ny * y,
        "amplitude": amp,
        "curvature": curvature(y_prime, y_double_prime),
        "curvature_limit": curvature_limited_amplitude(lambda_m, r_min_m, eta),
        "onset_m": onset_distance_m(d_full_m, v_rel_ms, lead_s, v_floor_ms),
    }
