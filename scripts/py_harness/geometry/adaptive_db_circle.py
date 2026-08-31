"""Adaptive CS-orbit: the target-centred Dubins about a *predicted* target (``TASK-033``).

Extends ``TASK-024``/``TASK-025`` (:mod:`py_harness.geometry.dubins_target_circle`,
:mod:`py_harness.geometry.dubins_target_orbit`) in two ways, and changes neither
of them:

1. **The ring is centred on a prediction.** The standoff ring is centred on where
   the state estimator says the target will be ``k_horizon`` control ticks ahead, not on
   where it is now. The prediction itself is state-side and already exists — the
   adapter reads ``snapshot["target_est"]``, which ``state.py`` has already
   projected ``lookahead_steps`` intervals ahead (``TASK-017``). Nothing here
   estimates anything.
2. **The curve is regenerated on an interval, not every tick.** The CS path is
   committed at a replan instant and steered against for ``n_replan`` ticks. It is then
   replaced, unconditionally.

No cost-based selection (``TASK-033`` D4)
------------------------------------------
At a replan instant the new plan is taken. Always. There is no candidate set, no
cost function, no scoring of the new plan against the active one, no hysteresis
and no cooldown. The only thing deciding when the path changes is the tick
counter reaching ``n_replan``.

This is deliberate and is what separates this module from the architecture
``ADR-001`` removed. What is reinstated is a *commitment interval*; what is not
reinstated is a *replanning decision*.

(The word "cost" does appear one level down: ``dubins_target_circle.shortest_path``
ranks its four ``(s1, s2)`` sense pairs by turn-in cost and returns the least. That
is geometric sense selection **inside** one plan — it is how CW/CCW is fixed — and
it is untouched ``TASK-024`` behaviour. It never compares one plan with another.)

Hold policies (``TASK-033`` D5)
--------------------------------
``HOLD_PLAN`` (default)
    The predicted centre **and** the committed CS curve are both held for ``n_replan``
    ticks. Between replans the aircraft's progress along that committed curve is
    found by nearest-point projection and the carrot is placed ``look_ahead_m``
    further along it. The path the aircraft is flying is the one that was
    generated at the last replan instant.

``HOLD_CENTRE_ONLY``
    Only the predicted centre is held. The CS curve is re-solved from the live
    pose every tick against that frozen centre. This isolates the prediction
    variable from the commitment variable: with ``n_replan`` large it is "lead the target
    but never commit", where ``HOLD_PLAN`` is "lead the target and commit".

At ``k_horizon = 0``, ``n_replan = 1``, ``HOLD_CENTRE_ONLY`` the construction reduces term for
term to :mod:`dubins_target_orbit` reading ``target_est``, which is asserted as a
test rather than assumed.

The committed plan is rebuilt each tick from the **commit pose and held centre**
rather than stored as a sampled point list: ``shortest_path`` is deterministic, so
re-running it on the stored inputs returns the identical path (``PR-004``), and
``algorithm_state`` stays five floats instead of a few hundred points. A Lua port
would cache the sampled path instead; the harness re-derives it because the
harness is measuring *geometry*, not planning cost. The replan interval therefore
changes **which** plan is flown here, not how much computation happens.

Frame is ``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).
Stateless: everything carried between ticks travels through ``algorithm_state``
(``VR-015``, ``A-VAL-003``).
"""

import math

from . import dubins_target_circle as dtc
from . import orbit as orbit_geom

#: Hold the predicted centre and the committed CS curve for the whole interval.
HOLD_PLAN = "plan"

#: Hold only the predicted centre; re-solve the curve from the live pose each tick.
HOLD_CENTRE_ONLY = "centre_only"

#: Every accepted hold policy. Anything else is refused at construction.
HOLD_POLICIES = (HOLD_PLAN, HOLD_CENTRE_ONLY)


def progress_along(pts, px, py):
    """Arc length of the point on ``pts`` nearest to ``(px, py)``, metres.

    The aircraft does not fly the committed path exactly — it steers toward a
    carrot under a turn-rate limit — so "how far along am I" is a projection, not
    an accumulated distance. Accumulating ``speed * dt`` since the commit would
    drift from the path whenever the aircraft cut a corner.

    Plain loop over segments with a clamped point-to-segment projection: no
    ``numpy``, and it transliterates to Lua unchanged.

    Returns:
        Arc length from the start of ``pts`` to the nearest point, metres. ``0.0``
        for a single-point path.

    Raises:
        ValueError: If ``pts`` is empty.
    """
    if not pts:
        raise ValueError("progress_along needs a non-empty path")
    if len(pts) == 1:
        return 0.0

    best_d2 = None
    best_s = 0.0
    s = 0.0
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1][0], pts[i - 1][1]
        x1, y1 = pts[i][0], pts[i][1]
        vx, vy = x1 - x0, y1 - y0
        seg = math.hypot(vx, vy)
        if seg <= 1e-12:
            continue
        # Clamped projection of (px, py) onto the segment.
        t = ((px - x0) * vx + (py - y0) * vy) / (seg * seg)
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        cx, cy = x0 + t * vx, y0 + t * vy
        d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy)
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best_s = s + t * seg
        s += seg
    return best_s


def _orbit_hold(px, py, cx, cy, psi_i, R, look_ahead_m, precompensate):
    """Closed-form orbit continuation about a held centre. Shared by both policies.

    Identical in form to :mod:`dubins_target_orbit`'s orbit branch — the sense is
    re-derived from the heading each tick rather than stored — except that the
    centre is the **held predicted** centre, not the target's present position.
    """
    psi0 = orbit_geom.entry_angle(px, py, cx, cy)
    direction = orbit_geom.orbit_direction(psi0, psi_i)
    gx, gy, psi = orbit_geom.orbit_guidance_point(
        cx, cy, R, psi0, direction, look_ahead_m, precompensate)
    return {
        "gx": gx,
        "gy": gy,
        "phase": "orbit",
        "direction": "cw" if direction > 0 else "ccw",
        "curvature": 1.0 / R,
        "ring_angle_rad": psi,
    }


def guidance(px, py, psi_i, cx, cy, plan, orbit_radius_m, turn_radius_m,
             look_ahead_m, delta_psi, delta_d, hold_policy=HOLD_PLAN,
             precompensate=True):
    """One guidance point about the held predicted ring centre ``(cx, cy)``.

    The caller (the adapter) owns the replan clock and decides whether this tick
    is a replan instant; this function is told the answer through ``plan`` and
    holds no clock of its own.

    Args:
        px: Aircraft East position, metres.
        py: Aircraft North position, metres.
        psi_i: Aircraft heading, radians clockwise from North.
        cx: Held predicted ring centre, East, metres.
        cy: Held predicted ring centre, North, metres.
        plan: The committed plan as ``{"px", "py", "psi"}`` — the aircraft pose at
            the last replan instant — or ``None`` to commit the live pose now.
            Ignored entirely when ``hold_policy`` is ``HOLD_CENTRE_ONLY``.
        orbit_radius_m: Standoff ring radius ``R``, metres.
        turn_radius_m: Minimum turn radius ``rho``, metres. ``R >= rho`` required.
        look_ahead_m: Carrot arc advance, metres.
        delta_psi: Dubins arc sampling, radians.
        delta_d: Dubins straight sampling, metres.
        hold_policy: :data:`HOLD_PLAN` or :data:`HOLD_CENTRE_ONLY`.
        precompensate: ``TASK-027`` guidance-ring pre-compensation on the orbit
            branch. Derived for a *stationary* centre; with a stepping centre the
            residual is reported by the task, not corrected here (``TASK-033`` D10).

    Returns:
        ``{"gx", "gy", "phase", "direction", "curvature", "ring_angle_rad"?,
        "plan"?}``. ``phase`` is ``"approach"`` or ``"orbit"`` — a discrete
        geometric switch on ``d = |plane - centre|`` against ``R``, never a blended
        ramp weight. ``plan`` is the pose the returned approach was built from,
        present in the approach phase only, for the caller to carry forward.

    Raises:
        ValueError: If ``orbit_radius_m < turn_radius_m``, if the aircraft is at
            the ring centre, if ``hold_policy`` is unknown, or propagated from
            :func:`dubins_target_circle.shortest_path`.
    """
    if hold_policy not in HOLD_POLICIES:
        raise ValueError(
            "unknown hold policy %r; expected one of %s"
            % (hold_policy, ", ".join(HOLD_POLICIES)))

    R = orbit_radius_m
    if R < turn_radius_m - 1e-9:
        raise ValueError(
            "target circle radius %.3f m < minimum turn radius %.3f m: the orbit "
            "would exceed the curvature bound" % (R, turn_radius_m))

    d = math.hypot(px - cx, py - cy)
    if d < 1e-6:
        raise ValueError("aircraft is at the ring centre; no orbit angle")

    if d <= R:
        # On / inside the ring: continue around the held centre. No ramp — the
        # tangent arrival of the TASK-024 approach makes the switch continuous.
        return _orbit_hold(px, py, cx, cy, psi_i, R, look_ahead_m, precompensate)

    if hold_policy == HOLD_CENTRE_ONLY:
        # Re-solve the CS path from the live pose against the frozen centre.
        g = dtc.guidance(px, py, psi_i, cx, cy, R, turn_radius_m,
                         look_ahead_m, delta_psi, delta_d)
        return {
            "gx": g["gx"],
            "gy": g["gy"],
            "phase": "approach",
            "direction": g["direction"],
            "curvature": g["curvature"],
        }

    # HOLD_PLAN: fly the curve committed at the last replan instant.
    if plan is None:
        plan = {"px": px, "py": py, "psi": psi_i}

    pts, _reach, direction, _arrival = dtc.shortest_path(
        plan["px"], plan["py"], plan["psi"], cx, cy, R, turn_radius_m,
        delta_psi, delta_d)

    # Where the aircraft actually is along that committed curve, then a look-ahead
    # further on. point_at_arc_length clamps at the path end, which is the tangency
    # point on the ring — so a carrot that runs off the end sits at the tangency
    # until the phase test above switches to the orbit.
    s_now = progress_along(pts, px, py)
    gx, gy = orbit_geom.point_at_arc_length(pts, s_now + look_ahead_m)
    return {
        "gx": gx,
        "gy": gy,
        "phase": "approach",
        "direction": direction,
        "curvature": 1.0 / R,
        "plan": {"px": plan["px"], "py": plan["py"], "psi": plan["psi"]},
    }
