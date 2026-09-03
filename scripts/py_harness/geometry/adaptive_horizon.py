"""Adaptive prediction horizon: choose ``k_horizon`` per replan (``TASK-039`` arm B).

Arm B of the three-approach comparison. Where
:mod:`py_harness.geometry.adaptive_db_circle` (arm A) centres its standoff ring on
the target predicted at **one configured horizon**, this module evaluates a set of
candidate horizons at each replan instant, scores each with an explicit objective,
and selects the best. The horizon becomes a decision variable rather than a
setting.

The construction it selects between is unchanged: for each candidate ``k`` the
predicted centre is

    c_k = p_est + v_est * k * dt_s

and the plan is the ordinary CS target-circle path onto the ring about ``c_k``
(:mod:`dubins_target_circle`, ``TASK-024``). Nothing here is a new *path* family —
only a new *selection* over the existing one.

The objective (``TASK-039`` appendix, arm B)
--------------------------------------------
``J(k) = w_T*J_T + w_R*J_R + w_kappa*J_kappa + w_S*J_S``, all four terms
non-negative and in different units, so the weights are what make them
commensurable and are configuration, not constants (`SR-004`).

``J_T`` — **tangent registration**, the ``TASK-038`` signal evaluated
    predictively. The plan ends at a tangent point ``A`` on the ring about
    ``c_k``; the aircraft reaches it ``n_a = reach / V / dt_s`` ticks later; the
    target is predicted to be at ``p_est + v_est * n_a * dt_s`` then. The error is
    ``e_tan = ||A - p_K(n_a)|| - R``, **signed**, and ``J_T = e_tan^2``.
    This is the term that carries the timing: it is zero exactly when the horizon
    equals the aircraft's own transit time, which is the ``k ~ n_a`` relationship
    ``TASK-038`` measured.

``J_R`` — **standoff tube**, mean squared deviation from ``R`` sampled along the
    final ``tube_len_m`` metres of the candidate curve, each sample compared with
    the target predicted at that sample's own arrival time. Scores where the plan
    *goes*, not only where it *ends*. Restricted to the final stretch on purpose:
    farther out every candidate departs from the same aircraft pose, so the
    deviation there measures the transit rather than the plan, and including it
    would bias the selector toward whichever candidate happens to be shortest.

``J_kappa`` — **curvature excess**, ``max(0, kappa_req - 1/rho)^2``, where
    ``kappa_req`` is the curvature needed to hold radius ``R`` about a centre
    translating at the estimated target speed, evaluated at the ring bearing the
    candidate arrives on (:func:`ring_track_curvature`). Depends on ``k`` because
    the horizon changes *where on the ring* the aircraft arrives, and arriving
    abeam of the target's motion is the expensive place. Zero below about
    15 m/s of target speed at ``R = 70 m``; that is the term reporting the
    feasibility boundary rather than a defect.

``J_S`` — **switching**, ``(k - k_prev)^2`` in ticks squared, damping oscillation
    of the selected horizon between adjacent candidates on successive replans.
    Zero on the first replan, where there is no previous selection.

Relationship to ``ADR-001``
---------------------------
``ADR-001`` superseded cost-based selection among candidate **paths**, and
``ADR-004`` records that ``adaptive_db_circle`` was built with none so those
requirements stayed superseded. This module **does** score candidates and select
among them, and therefore reopens ``FR-004``/``FR-008``..``FR-011`` in substance.
That is deliberate and is the reason ``TASK-039`` requires a decision record
before any arm is adopted. It must not be read as a quiet reinstatement: nothing
here changes the status of any requirement.

Scope limit — approach phase only
---------------------------------
There is a tangent point only while there is a CS component. Once the aircraft is
on the ring, ``shortest_path`` has no solution and there is no handover point to
score, so **the horizon is not re-selected on the ring**: the last selection is
held and the orbit continues about the centre predicted at it. This is
``TASK-038``'s recorded structural finding, not an implementation shortcut, and it
bounds what arm B can achieve — the orbit deformation ``TASK-027`` reopened is
outside the phase in which this arm makes any decision at all.

What this module is, and is not
-------------------------------
It is **the selector only**. It scores candidate horizons and returns the winner;
it builds no flown path. The adapter hands the winning centre to
:mod:`adaptive_db_circle` — arm A's construction, unmodified — so the two arms
differ in exactly one thing: whether ``k_horizon`` is configured or chosen. That
makes the degeneracy exact and testable: collapse the candidate set to a single
horizon ``k`` and arm B must reproduce arm A at ``lookahead_steps = k``, field for
field. Any other difference between the arms would confound the comparison
``TASK-039`` exists to make.

Frame is ``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).
Stateless: the previous selection travels through the caller's
``algorithm_state`` (``VR-015``, ``A-VAL-003``). Plain ``math``, floats and dicts;
no ``numpy``.
"""

import math

from . import dubins_target_circle as dtc
from . import orbit as orbit_geom


def predicted_centre(est_x, est_y, vx, vy, k_steps, dt_s):
    """Ring centre for candidate horizon ``k_steps``, in ``(x = East, y = North)``.

    The same constant-velocity projection :mod:`py_harness.lookahead` applies
    state-side, written out here because arm B must project the **un-projected**
    estimate itself over each candidate: the state-side horizon is what this
    algorithm is replacing, so consuming it as well would lead the ring twice.

    Args:
        est_x: Estimated target East position, metres.
        est_y: Estimated target North position, metres.
        vx: Estimated target East velocity, m/s.
        vy: Estimated target North velocity, m/s.
        k_steps: Candidate horizon, whole control ticks, ``>= 0``.
        dt_s: Control interval, seconds.

    Returns:
        ``(x, y)`` metres.
    """
    horizon_s = k_steps * dt_s
    return est_x + vx * horizon_s, est_y + vy * horizon_s


def ring_track_curvature(orbit_radius_m, target_speed_ms, airspeed_ms, theta_rad):
    """Curvature needed to hold ``R`` about a centre translating at ``v_K``.

    Holding a constant radius ``R`` about a centre moving at ``v_K`` requires a
    **bearing-dependent** turn rate. With ``theta`` the aircraft's bearing on the
    ring measured from the centre's direction of travel,

        R * dtheta/dt = v_K * sin(theta) + sqrt(V^2 - v_K^2 * cos^2(theta))

    and the aircraft's curvature is ``kappa = (dtheta/dt) / V``. At ``v_K = 0``
    this reduces to ``1/R`` as it must.

    The sign of the ``v_K*sin(theta)`` term depends on the orbit sense. This
    returns the **conservative** branch, ``+|v_K*sin(theta)|``, because the term
    exists to report a curvature bound and the cheaper sense is the one that must
    not be assumed.

    Args:
        orbit_radius_m: Standoff radius ``R``, metres.
        target_speed_ms: Centre speed ``v_K``, m/s.
        airspeed_ms: Aircraft speed ``V``, m/s.
        theta_rad: Ring bearing from the centre's velocity direction, radians.

    Returns:
        Required curvature in 1/m, or ``None`` when the geometry is
        **unreachable** — ``V^2 < v_K^2 * cos^2(theta)``, i.e. the target
        out-runs the aircraft's along-track component. ``None`` is the honest
        answer: no finite curvature holds the ring, so returning a large number
        would misreport an impossibility as an expensive option.

    Raises:
        ValueError: For a non-positive radius or airspeed.
    """
    if orbit_radius_m <= 0.0:
        raise ValueError("orbit_radius_m must be positive, got %r"
                         % orbit_radius_m)
    if airspeed_ms <= 0.0:
        raise ValueError("airspeed_ms must be positive, got %r" % airspeed_ms)
    c = math.cos(theta_rad)
    disc = airspeed_ms * airspeed_ms - target_speed_ms * target_speed_ms * c * c
    if disc <= 0.0:
        return None
    theta_dot = (abs(target_speed_ms * math.sin(theta_rad))
                 + math.sqrt(disc)) / orbit_radius_m
    return theta_dot / airspeed_ms


def _bearing_from_velocity(px, py, cx, cy, vx, vy):
    """Bearing of ``(px, py)`` about ``(cx, cy)``, measured from ``(vx, vy)``.

    Radians in ``[-pi, pi)``. When the centre is stationary the velocity gives no
    direction, so ``0.0`` is returned and the caller's curvature term degenerates
    to the stationary case ``1/R``, which is correct.
    """
    if abs(vx) < 1e-9 and abs(vy) < 1e-9:
        return 0.0
    a = math.atan2(py - cy, px - cx) - math.atan2(vy, vx)
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def score_candidate(px, py, psi_i, est_x, est_y, vx, vy, k_steps, k_prev_steps,
                    dt_s, airspeed_ms, orbit_radius_m, turn_radius_m,
                    delta_psi, delta_d, weights, path_samples, tube_len_m):
    """Score one candidate horizon. Returns a dict, or ``None`` if it has no plan.

    ``None`` means the CS construction does not solve for this candidate — most
    often because the predicted centre has been led so far that the aircraft is
    inside the ring about it. That is a legitimate exclusion, not an error, and
    the caller counts it.

    Args:
        px, py: Aircraft position, ``(East, North)`` metres.
        psi_i: Aircraft heading, radians clockwise from North.
        est_x, est_y: **Un-projected** target estimate, ``(East, North)`` metres.
        vx, vy: Estimated target velocity, ``(East, North)`` m/s.
        k_steps: Candidate horizon, whole control ticks.
        k_prev_steps: Horizon selected at the previous replan, or ``None`` on the
            first, in which case the switching term is zero.
        dt_s: Control interval, seconds.
        airspeed_ms: Aircraft speed ``V``, m/s.
        orbit_radius_m: Standoff radius ``R``, metres.
        turn_radius_m: Minimum turn radius ``rho``, metres.
        delta_psi, delta_d: Dubins sampling, radians and metres.
        weights: ``{"tangent", "radial", "curvature", "switch"}`` plain floats.
        path_samples: Points sampled along the scored stretch of the curve.
        tube_len_m: Length of the final stretch scored by the standoff term,
            metres.

    Returns:
        ``{"k_steps", "cost", "e_tan_m", "n_a_steps", "reach_m", "centre_x",
        "centre_y", "arrival_x", "arrival_y", "curvature_req", "feasible",
        "j_tangent", "j_radial", "j_curvature", "j_switch"}``.
        ``curvature_req`` is ``None`` and ``feasible`` False when the ring is
        unreachable at the arrival bearing.
    """
    cx, cy = predicted_centre(est_x, est_y, vx, vy, k_steps, dt_s)
    try:
        pts, reach, _direction, arrival = dtc.shortest_path(
            px, py, psi_i, cx, cy, orbit_radius_m, turn_radius_m,
            delta_psi, delta_d)
    except ValueError:
        return None

    # Transit: how long the aircraft takes to fly the planned curve, in ticks.
    n_a = reach / airspeed_ms / dt_s

    # J_T -- the TASK-038 tangent registration error, evaluated at ARRIVAL and
    # signed. Zero when the planned handover point lands on the true standoff
    # ring at the moment the aircraft gets there.
    ax, ay = arrival[0], arrival[1]
    tx_at_arrival = est_x + vx * n_a * dt_s
    ty_at_arrival = est_y + vy * n_a * dt_s
    e_tan = math.hypot(ax - tx_at_arrival, ay - ty_at_arrival) - orbit_radius_m

    # J_R -- standoff tube over the final tube_len_m of the curve. Each sample is
    # compared with the target predicted at THAT sample's own arrival time, which
    # is what makes it a rendezvous measure rather than a snapshot.
    s_end = reach
    s_start = reach - tube_len_m
    if s_start < 0.0:
        s_start = 0.0
    n_samples = int(path_samples)
    total = 0.0
    for i in range(n_samples):
        if n_samples == 1:
            s = s_end
        else:
            s = s_start + (s_end - s_start) * i / float(n_samples - 1)
        qx, qy = orbit_geom.point_at_arc_length(pts, s)
        t_i = s / airspeed_ms
        rx = est_x + vx * t_i
        ry = est_y + vy * t_i
        dev = math.hypot(qx - rx, qy - ry) - orbit_radius_m
        total += dev * dev
    j_radial = total / float(n_samples)

    # J_kappa -- curvature needed to hold the ring at the bearing this candidate
    # arrives on, against the curvature bound. Zero unless the target is fast.
    v_k = math.hypot(vx, vy)
    theta = _bearing_from_velocity(ax, ay, cx, cy, vx, vy)
    kappa_req = ring_track_curvature(orbit_radius_m, v_k, airspeed_ms, theta)
    if kappa_req is None:
        feasible = False
        # Unreachable at this bearing. Charge the excess that a curvature equal
        # to the bound would still leave, so the candidate is ranked last among
        # those that solve rather than silently dropped: an unreachable ring is a
        # RESULT (TASK-039), and dropping it would hide the boundary.
        excess = 1.0 / turn_radius_m
    else:
        feasible = True
        excess = kappa_req - 1.0 / turn_radius_m
        if excess < 0.0:
            excess = 0.0
    j_curvature = excess * excess

    # J_S -- switching. Zero when there is no previous selection to move from.
    if k_prev_steps is None:
        j_switch = 0.0
    else:
        dk = float(k_steps - k_prev_steps)
        j_switch = dk * dk

    cost = (weights["tangent"] * e_tan * e_tan
            + weights["radial"] * j_radial
            + weights["curvature"] * j_curvature
            + weights["switch"] * j_switch)

    return {
        "k_steps": int(k_steps),
        "cost": cost,
        "e_tan_m": e_tan,
        "n_a_steps": n_a,
        "reach_m": reach,
        "centre_x": cx,
        "centre_y": cy,
        "arrival_x": ax,
        "arrival_y": ay,
        "curvature_req": kappa_req,
        "feasible": feasible,
        "j_tangent": e_tan * e_tan,
        "j_radial": j_radial,
        "j_curvature": j_curvature,
        "j_switch": j_switch,
    }


def select_horizon(px, py, psi_i, est_x, est_y, vx, vy, k_prev_steps, dt_s,
                   airspeed_ms, orbit_radius_m, turn_radius_m, delta_psi,
                   delta_d, horizons, weights, path_samples, tube_len_m):
    """Score every candidate in ``horizons`` and return the least-cost one.

    Args:
        horizons: Candidate horizons, whole control ticks. Ordered; ties are
            broken by the **first** candidate, so a shorter horizon wins a tie
            and the selector never prefers prediction it cannot justify.
        (others): as :func:`score_candidate`.

    Returns:
        The winning candidate dict with ``"evaluated"`` (candidates offered) and
        ``"solved"`` (candidates that produced a plan) added, or ``None`` when no
        candidate solved.
    """
    best = None
    solved = 0
    for k in horizons:
        cand = score_candidate(px, py, psi_i, est_x, est_y, vx, vy, k,
                               k_prev_steps, dt_s, airspeed_ms, orbit_radius_m,
                               turn_radius_m, delta_psi, delta_d, weights,
                               path_samples, tube_len_m)
        if cand is None:
            continue
        solved += 1
        if best is None or cand["cost"] < best["cost"]:
            best = cand
    if best is None:
        return None
    best["evaluated"] = len(horizons)
    best["solved"] = solved
    return best
