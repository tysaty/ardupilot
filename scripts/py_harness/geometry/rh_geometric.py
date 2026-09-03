"""Receding-horizon geometric planner (``TASK-039`` arm C).

Arm C of the three-approach comparison, in the form the draft itself recommends
as the first step: *"Before implementing a full numerical MPC solver, implement a
simpler receding-horizon geometric planner."* This is that planner. It is **not**
a numerical MPC and does not claim to be one — there is no solver, no gradient and
no convergence criterion. It is an exhaustive search over a small, feasible,
finite candidate set, which is what makes it transliterable to Lua (`VR-015`) and
what makes its planning cost a fixed, measurable number (`PR-002`, `A-SW-002`).

What it does each replan
------------------------
1. Predict the target over ``N = horizon_steps`` control ticks on constant
   velocity, from the **un-projected** estimate.
2. Roll the aircraft forward over the same ``N`` ticks for every candidate in a
   two-segment curvature grid: ``kappa_1`` held for ``segment_steps`` ticks, then
   ``kappa_2`` for the remainder. Both drawn from ``[-1/rho, +1/rho]``, so
   **every candidate satisfies the curvature bound by construction** — nothing is
   generated and then rejected, which is the cheap way to enforce ``FR-005`` and
   ``SR-002`` in an optimiser.
3. Score each rollout against the *predicted* standoff tube.
4. Take the best, and emit **one guidance point**: the winner's own pose
   ``command_steps`` ticks ahead — one tick by default. Only the first command is
   ever flown; the rest is re-derived at the next replan. That is the
   receding-horizon principle, and it is also the only emission rule that makes
   the aircraft fly the trajectory that was optimised — see :func:`guidance`.

Why two segments, and not one or ten
------------------------------------
One curvature can only fly circles: it can hold a ring about a *stationary*
centre and nothing else. Two can turn onto a translating ring and then hold it,
which is the manoeuvre the standoff problem actually needs. Ten would be a
trajectory optimiser and would not fit a 10 Hz Lua loop. The planning cost is
exactly ``n1 * n2 * N`` aircraft steps per replan and is reported, not assumed.

Why this arm attacks the orbit deformation differently
------------------------------------------------------
Arms A and B choose *where to centre a ring* and then place a carrot on it by a
fixed geometric rule. The deformation measured on 2026-09-02 — a 70 m commanded
ring flown at about 50 m in front of a 12.5 m/s target and over 110 m behind it —
comes from that carrot rule against a *translating* ring, and a standalone
kinematic model with no estimator, no prediction and no algorithm reproduces it.

This arm has no carrot rule to be wrong. It scores the **achieved range to the
predicted target** at every step of the rollout, so a path that would cut inside
the ring in front is charged for it before it is flown, and the emitted point is
its own next pose rather than a chord across the plan.

What that is worth is measured, not claimed: `py_harness.benchmark` reports the
deformation spread per arm, and it is not uniformly in this arm's favour. It is
the only arm that holds a round ring at low target speed (1.8 m spread at
5 m/s against 11 to 14 m for the ring arms) and it is not the best at
12.5 m/s.

Frame is ``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).
Stateless: the previous curvature and the replan clock travel through the
caller's ``algorithm_state`` (``VR-015``, ``A-VAL-003``). Plain ``math``, floats
and dicts; no ``numpy``.
"""

import math

from . import orbit as orbit_geom


def curvature_grid(turn_radius_m, count):
    """``count`` curvatures spanning ``[-1/rho, +1/rho]`` inclusive, 1/m.

    An **odd** ``count`` puts straight flight (``kappa = 0``) exactly on the grid,
    which matters: straight flight must always be a candidate, or the planner
    cannot choose to stop turning. An even count is accepted and produces a grid
    without it — the caller is told, not corrected, because silently changing a
    configured candidate count would make the reported planning cost wrong.

    Raises:
        ValueError: For a non-positive radius or ``count < 2``.
    """
    if turn_radius_m <= 0.0:
        raise ValueError("turn_radius_m must be positive, got %r" % turn_radius_m)
    if count < 2:
        raise ValueError("count must be >= 2; one candidate is not a search, "
                         "got %r" % count)
    k_max = 1.0 / turn_radius_m
    span = 2.0 * k_max
    return [-k_max + span * i / float(count - 1) for i in range(count)]


def rollout(px, py, psi_i, kappa1, kappa2, hold_steps, horizon_steps,
            airspeed_ms, dt_s):
    """Integrate the aircraft forward over the horizon under a two-segment command.

    Constant speed, curvature-commanded heading: ``dpsi = kappa * V * dt`` (the
    curvature is ``dpsi/ds`` and ``ds = V*dt``), then a straight step on the new
    heading. The same forward-Euler integration :class:`~py_harness.state.Harness`
    uses, so the planner's model of the aircraft and the aircraft agree — an
    optimiser scored against a different model than the one it flies would be
    measuring the mismatch.

    Args:
        px, py: Aircraft position, ``(East, North)`` metres.
        psi_i: Aircraft heading, radians clockwise from North.
        kappa1: Curvature held for the first ``hold_steps`` ticks, 1/m.
        kappa2: Curvature for the remainder of the horizon, 1/m.
        hold_steps: Ticks the first curvature is held. Clamped into
            ``[0, horizon_steps]``.
        horizon_steps: Horizon ``N`` in whole control ticks, ``>= 1``.
        airspeed_ms: Constant aircraft speed ``V``, m/s.
        dt_s: Control interval, seconds.

    Returns:
        ``horizon_steps + 1`` points ``[(x, y), ...]`` starting at the aircraft's
        present position, spaced ``V * dt_s`` metres apart.

    Raises:
        ValueError: For ``horizon_steps < 1``.
    """
    if horizon_steps < 1:
        raise ValueError("horizon_steps must be >= 1, got %r" % horizon_steps)
    hold = int(hold_steps)
    if hold < 0:
        hold = 0
    elif hold > horizon_steps:
        hold = horizon_steps
    d = airspeed_ms * dt_s
    x, y, psi = px, py, psi_i
    pts = [(x, y)]
    for i in range(int(horizon_steps)):
        kappa = kappa1 if i < hold else kappa2
        psi = psi + kappa * d
        x = x + d * math.sin(psi)
        y = y + d * math.cos(psi)
        pts.append((x, y))
    return pts


def score(pts, est_x, est_y, vx, vy, dt_s, orbit_radius_m, kappa1, kappa2,
          hold_steps, kappa_prev, weights):
    """Cost of one rollout against the predicted standoff tube.

    ``J = w_standoff * mean_i (r_i - R)^2``
    ``  + w_terminal * (r_N - R)^2``
    ``  + w_effort   * mean_i kappa_i^2``
    ``  + w_smooth   * (kappa_1 - kappa_prev)^2``

    ``r_i`` is the range from the aircraft's rolled-out position at step ``i`` to
    the target **predicted at that same step**, so the cost is target-relative
    throughout and never compares a future aircraft with a present target — which
    is the error the fixed-horizon arms are structurally exposed to.

    The step-0 point is excluded from the running mean: it is the present pose,
    identical for every candidate, so scoring it adds a constant that shifts every
    cost equally and buys nothing.

    Args:
        pts: Rollout from :func:`rollout`.
        est_x, est_y: **Un-projected** target estimate, ``(East, North)`` metres.
        vx, vy: Estimated target velocity, ``(East, North)`` m/s.
        dt_s: Control interval, seconds.
        orbit_radius_m: Standoff radius ``R``, metres.
        kappa1, kappa2: The candidate's two curvatures, 1/m.
        hold_steps: Ticks ``kappa1`` is held, for the effort mean.
        kappa_prev: Curvature commanded at the previous replan, or ``None`` on
            the first, in which case the smoothness term is zero.
        weights: ``{"standoff", "terminal", "effort", "smooth"}`` plain floats.

    Returns:
        ``{"cost", "mean_error_m", "terminal_error_m", "j_standoff",
        "j_terminal", "j_effort", "j_smooth"}``. ``mean_error_m`` is the RMS of
        ``r_i - R`` over the horizon, in metres, reported so the cost can be read
        back as a distance rather than only compared.
    """
    n = len(pts) - 1
    total = 0.0
    for i in range(1, n + 1):
        t_i = i * dt_s
        tx = est_x + vx * t_i
        ty = est_y + vy * t_i
        dev = math.hypot(pts[i][0] - tx, pts[i][1] - ty) - orbit_radius_m
        total += dev * dev
    j_standoff = total / float(n)

    t_n = n * dt_s
    dev_n = math.hypot(pts[n][0] - (est_x + vx * t_n),
                       pts[n][1] - (est_y + vy * t_n)) - orbit_radius_m
    j_terminal = dev_n * dev_n

    hold = min(max(int(hold_steps), 0), n)
    j_effort = (hold * kappa1 * kappa1 + (n - hold) * kappa2 * kappa2) / float(n)

    if kappa_prev is None:
        j_smooth = 0.0
    else:
        dk = kappa1 - kappa_prev
        j_smooth = dk * dk

    cost = (weights["standoff"] * j_standoff
            + weights["terminal"] * j_terminal
            + weights["effort"] * j_effort
            + weights["smooth"] * j_smooth)
    return {
        "cost": cost,
        "mean_error_m": math.sqrt(j_standoff),
        "terminal_error_m": dev_n,
        "j_standoff": j_standoff,
        "j_terminal": j_terminal,
        "j_effort": j_effort,
        "j_smooth": j_smooth,
    }


def plan(px, py, psi_i, est_x, est_y, vx, vy, dt_s, airspeed_ms,
         orbit_radius_m, turn_radius_m, horizon_steps, segment_steps,
         n_candidates, n_candidates_2, weights, kappa_prev):
    """Search the curvature grid and return the least-cost candidate.

    Ties are broken by the **first** candidate in grid order, which runs from the
    most negative curvature upward; the grid is symmetric, so this is a stable
    rule rather than a preference for one turn sense in any meaningful sense.

    Returns:
        ``{"kappa1", "kappa2", "pts", "evaluated", "rollout_steps", ...}`` merged
        with the winning :func:`score` dict. ``evaluated`` is the candidate count
        and ``rollout_steps`` the aircraft steps integrated — the planning-cost
        figures ``PR-002`` needs.
    """
    grid1 = curvature_grid(turn_radius_m, n_candidates)
    grid2 = curvature_grid(turn_radius_m, n_candidates_2)
    best = None
    for kappa1 in grid1:
        for kappa2 in grid2:
            pts = rollout(px, py, psi_i, kappa1, kappa2, segment_steps,
                          horizon_steps, airspeed_ms, dt_s)
            s = score(pts, est_x, est_y, vx, vy, dt_s, orbit_radius_m,
                      kappa1, kappa2, segment_steps, kappa_prev, weights)
            if best is None or s["cost"] < best["cost"]:
                s["kappa1"] = kappa1
                s["kappa2"] = kappa2
                s["pts"] = pts
                best = s
    best["evaluated"] = len(grid1) * len(grid2)
    best["rollout_steps"] = best["evaluated"] * int(horizon_steps)
    return best


def guidance(px, py, psi_i, est_x, est_y, vx, vy, dt_s, airspeed_ms,
             orbit_radius_m, turn_radius_m, command_steps, horizon_steps,
             segment_steps, n_candidates, n_candidates_2, weights,
             kappa_prev, held=None):
    """One guidance point from a fresh plan, or from a held command sequence.

    The point emitted is the rolled-out pose ``command_steps`` ticks ahead — the
    receding-horizon "apply the first command" rule at ``command_steps = 1``.
    **The configured ``look_ahead_m`` is deliberately not used here**: a carrot
    placed a look-ahead along a curving rollout is a chord across it, and the
    harness steers on the bearing to the carrot under a turn-rate limit, so a
    long carrot saturates the limiter and the aircraft flies its minimum radius
    instead of the plan. Measured: a 70 m ring about a stationary target held at
    45.0 m (exactly the minimum turn radius) under a 50 m carrot. The geometric
    arms are unaffected because their plans curve at ``1/R``, well inside the
    limit.

    Args:
        command_steps: Rollout steps ahead the guidance point is taken, ``>= 1``.
        held: ``None`` to re-optimise now. Otherwise
            ``{"kappa1", "kappa2", "remaining_hold_steps"}`` — the command
            sequence committed at the last replan, re-integrated **from the live
            pose**. Holding the *command* rather than the *path* is what keeps
            this drift-free: there is no stored trajectory for the aircraft to
            fall behind, so a commitment interval longer than one tick costs
            fidelity only through the staleness of the target prediction, which
            is the thing being studied.
        (others): as :func:`plan`, plus ``look_ahead_m``, the carrot arc advance
            in metres.

    Returns:
        ``{"gx", "gy", "phase", "curvature", "kappa1", "kappa2", ...}``.
        ``phase`` is ``"approach"`` while the aircraft is outside the ring about
        the predicted target and ``"orbit"`` once inside it — reported for
        comparability with the geometric arms, which switch construction there.
        **This arm does not switch construction**: one optimiser runs throughout,
        which is the point of it.
    """
    if held is None:
        best = plan(px, py, psi_i, est_x, est_y, vx, vy, dt_s, airspeed_ms,
                    orbit_radius_m, turn_radius_m, horizon_steps,
                    segment_steps, n_candidates, n_candidates_2, weights,
                    kappa_prev)
        replanned = True
    else:
        pts = rollout(px, py, psi_i, held["kappa1"], held["kappa2"],
                      held["remaining_hold_steps"], horizon_steps,
                      airspeed_ms, dt_s)
        best = score(pts, est_x, est_y, vx, vy, dt_s, orbit_radius_m,
                     held["kappa1"], held["kappa2"],
                     held["remaining_hold_steps"], kappa_prev, weights)
        best["kappa1"] = held["kappa1"]
        best["kappa2"] = held["kappa2"]
        best["pts"] = pts
        best["evaluated"] = 0
        best["rollout_steps"] = int(horizon_steps)
        replanned = False

    step = int(command_steps)
    if step < 1:
        step = 1
    elif step > len(best["pts"]) - 1:
        step = len(best["pts"]) - 1
    gx, gy = best["pts"][step]
    d = math.hypot(px - est_x, py - est_y)
    out = {
        "gx": gx,
        "gy": gy,
        "phase": "orbit" if d <= orbit_radius_m else "approach",
        # The curvature actually commanded now is the first segment's. Reported
        # as the FR-005 / SR-002 quantity; it is <= 1/rho by grid construction.
        "curvature": abs(best["kappa1"]),
        "kappa1": best["kappa1"],
        "kappa2": best["kappa2"],
        "replanned": replanned,
        "cost": best["cost"],
        "mean_error_m": best["mean_error_m"],
        "terminal_error_m": best["terminal_error_m"],
        "evaluated": best["evaluated"],
        "rollout_steps": best["rollout_steps"],
        "command_steps": step,
    }
    return out
