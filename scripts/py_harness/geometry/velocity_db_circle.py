"""Velocity-aligned stepwise CS-orbit: the ring one control tick ahead (``TASK-043``, arm D).

Arm D of the standoff comparison, proposed against arms A--C on the argument
that projecting the target far forward is unreliable: the extrapolation is only
as good as the constant-velocity model behind it, and under elastic motion that
model is wrong by construction (``A-TGT-002``). Arm D keeps the state estimator
and discards the horizon optimisation, stepping the ring centre exactly one
control tick on the estimated velocity and rebuilding the path every tick.

The construction
----------------
Each tick, from the **raw** (un-projected) estimate::

    target_step_n_m = target_est_raw_n_m + target_est_raw_vn_ms * dt_s * vd_step_ticks
    target_step_e_m = target_est_raw_e_m + target_est_raw_ve_ms * dt_s * vd_step_ticks

then the ordinary target-centred CS-onto-orbit construction about that stepped
centre. ``vd_step_ticks`` is 1 by default, which is the arm as specified.

Why this module is thin
-----------------------
The ring construction is **not reimplemented here**. Once the centre is chosen,
arm D builds the same geometry arm A builds, so :func:`guidance` delegates to
:mod:`py_harness.geometry.adaptive_db_circle` with ``HOLD_CENTRE_ONLY`` and no
committed plan — which, at a replan every tick, is term for term the same
construction. Duplicating it would fork two copies of tested geometry and give
the comparison a second uncontrolled difference (``VR-009``): arm D must differ
from arm A in *where the centre goes*, and in nothing else.

What arm D is not
-----------------
**Not arm A with a small horizon.** Arm A consumes ``target_est``, which
``state.py`` has already projected ``lookahead_steps`` ahead; the horizon is a
configured quantity held for the whole run. Arm D declares ``owns_horizon`` and
projects from ``target_est_raw`` itself, so the two never double-count and a
stray ``--lookahead-steps`` is refused rather than silently applied twice.

**Not exempt from chord-cutting.** This is a ring construction followed by a
carrot at ``look_ahead_m``, so the settled radius is ``r = R*cos(L/R)``
(``A-VAL-005``) exactly as for arms 0, A and B. Only arm C, which builds no ring
and uses no carrot, escapes it.

Frame is ``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).
Stateless: nothing is carried between ticks inside this module (``VR-015``,
``A-VAL-003``).
"""

from . import adaptive_db_circle as adb


def stepped_centre(est_raw, dt_s, step_ticks=1):
    """Ring centre one (or ``step_ticks``) control tick ahead of the raw estimate.

    The constant-velocity step of ``TASK-043``. Deliberately the whole of arm
    D's prediction: there is no horizon to select and no candidate set.

    Args:
        est_raw: The un-projected estimate as ``{"n_m", "e_m", "vn_ms",
            "ve_ms"}`` — ``snapshot["target_est_raw"]``.
        dt_s: Control interval, seconds.
        step_ticks: Whole control ticks to step. 1 is the arm as specified.

    Returns:
        ``(cx, cy)`` as ``(East, North)`` metres, matching the geometry frame.

    Raises:
        ValueError: If ``est_raw`` is None, if ``dt_s <= 0``, or if
            ``step_ticks < 1``.
    """
    if est_raw is None:
        raise ValueError(
            "velocity_db_circle needs the raw estimate; there is no "
            "present-position fallback")
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive, got %r" % dt_s)
    if step_ticks < 1:
        raise ValueError(
            "step_ticks must be >= 1; 0 removes the projection that defines "
            "this arm. Got %r" % step_ticks)
    lead_s = dt_s * step_ticks
    cx = est_raw["e_m"] + est_raw["ve_ms"] * lead_s
    cy = est_raw["n_m"] + est_raw["vn_ms"] * lead_s
    return cx, cy


def guidance(px, py, psi_i, cx, cy, orbit_radius_m, turn_radius_m,
             look_ahead_m, delta_psi, delta_d, precompensate=True,
             preferred_direction=None, sense_margin_m=0.0):
    """One guidance point about the stepped ring centre ``(cx, cy)``.

    Delegates to :func:`adaptive_db_circle.guidance` with
    :data:`~py_harness.geometry.adaptive_db_circle.HOLD_CENTRE_ONLY` and
    ``plan=None``: arm D commits nothing between ticks, so the curve is re-solved
    from the live pose every tick against the freshly stepped centre.

    Args:
        px: Aircraft East position, metres.
        py: Aircraft North position, metres.
        psi_i: Aircraft heading, radians clockwise from North.
        cx: Stepped ring centre, East, metres.
        cy: Stepped ring centre, North, metres.
        orbit_radius_m: Standoff ring radius ``R``, metres.
        turn_radius_m: Minimum turn radius ``rho``, metres. ``R >= rho`` required.
        look_ahead_m: Carrot arc advance, metres.
        delta_psi: Dubins arc sampling, radians.
        delta_d: Dubins straight sampling, metres.
        precompensate: ``TASK-027`` guidance-ring pre-compensation on the orbit
            branch.

    Returns:
        ``{"gx", "gy", "phase", "direction", "curvature", "ring_angle_rad"?}``.
        ``phase`` is ``"approach"`` or ``"orbit"``.

    Raises:
        ValueError: Propagated from the delegate — ``orbit_radius_m <
            turn_radius_m``, or the aircraft at the ring centre.
    """
    g = adb.guidance(
        px, py, psi_i, cx, cy, None,
        orbit_radius_m, turn_radius_m, look_ahead_m, delta_psi, delta_d,
        adb.HOLD_CENTRE_ONLY, precompensate, preferred_direction,
        sense_margin_m,
    )
    # HOLD_CENTRE_ONLY never commits a plan, so no "plan" key comes back; strip
    # defensively so arm D's contract cannot drift if the delegate changes.
    return {k: v for k, v in g.items() if k != "plan"}
