"""Substitutable geometric algorithms and the registry that selects them.

This is the algorithm module of the two-module harness
(``DEC-2026-06-25-01``). Every algorithm here implements the single interface
defined in :mod:`interface` (``DEC-2026-06-25-02``), so that the state module
never changes when an algorithm is swapped.

The registry exists so that ``VR-014``'s acceptance criterion — "two algorithms
run through the harness unchanged apart from a selection argument" — can be
tested rather than asserted.

Adapters only
-------------
No geometry lives in this file. It lives in :mod:`py_harness.geometry`, which
holds no state. What an adapter does is:

1. convert the harness snapshot's ``(north, east)`` into the geometry modules'
   ``x = East, y = North`` frame, explicitly, per ``IR-008``;
2. call the geometry;
3. pick **one guidance point** off the generated path at ``look_ahead_m`` of
   arc length — the carrot rule under "Guidance point" in ``GLOSSARY.md``,
   required because the interface returns a point and the geometry returns a
   path; and
4. convert back to ``(north, east)``.

Frame conversion is one line each way and is written out rather than hidden in
a helper, because a silent transpose here would be indistinguishable from a
geometry bug.
"""

import math

from .geometry import adaptive_db_circle as adaptive_db_circle_geom
from .geometry import adaptive_horizon as adaptive_horizon_geom
from .geometry import amplitude as amplitude_geom
from .geometry import amplitude_orbit as amplitude_orbit_geom
from .geometry import dubins as dubins_geom
from .geometry import dubins_orbit as dubins_orbit_geom
from .geometry import dubins_target_circle as dubins_target_circle_geom
from .geometry import dubins_target_orbit as dubins_target_orbit_geom
from .geometry import heading as heading_geom
from .geometry import orbit as orbit_geom
from .geometry import rh_geometric as rh_geometric_geom
from .geometry import var_amplitude_weave as var_amplitude_weave_geom
from .geometry import vaw_orbit as vaw_orbit_geom
from .interface import GeometricAlgorithm, NoSolution


def _target_ea(snapshot):
    """Target ``(east, north)`` for guidance: the estimate if present, else true.

    An algorithm reads the estimated target (`target_est`, from the state
    estimator) when one is running, otherwise the true target — the same call
    site works with or without estimation (``TASK-010``/``TASK-012``).
    """
    est = snapshot.get("target_est")
    if est is not None:
        return est["e_m"], est["n_m"]
    return snapshot["target_e_m"], snapshot["target_n_m"]


def _raw_estimate(snapshot, who):
    """The **un-projected** target estimate, for an algorithm that owns its horizon.

    ``TASK-039`` arms B and C choose their own prediction horizon, so they must
    project from an estimate that has not already had ``lookahead_steps`` applied
    to it state-side — otherwise a run with ``--lookahead-steps 25`` would lead
    the ring twice and the second lead would be invisible.

    Falls back to ``target_est`` only when ``target_est_raw`` is absent from the
    snapshot, which happens for a hand-built snapshot in a test; when an estimator
    is running the two are equal at ``lookahead_steps = 0`` anyway.

    Raises:
        NoSolution: When no estimator is running. Named after ``who`` and naming
            ``--estimate``, because such an algorithm has no present-position
            fallback: reading the truth would silently turn it into a different
            algorithm and the run would be mistaken for evidence.
    """
    est = snapshot.get("target_est_raw")
    if est is None:
        est = snapshot.get("target_est")
    if est is None:
        raise NoSolution(
            "%s requires the state estimator: it selects its own prediction "
            "horizon from the estimated target velocity and has no "
            "present-position fallback. Re-run with --estimate." % who)
    return est


def _weave_phase_m(snapshot):
    """Weave arc-length phase ``s`` from the snapshot: speed x elapsed time.

    Derived from ``t_s`` and ``plane_speed_ms`` rather than accumulated in the
    algorithm, so the adapter stays stateless (``A-VAL-003``). Constant speed
    makes along-track distance ``= speed * t``.
    """
    return snapshot["plane_speed_ms"] * snapshot["t_s"]


def _relative_speed(snapshot):
    """Relative speed magnitude between the aircraft and the target, m/s.

    ``|v_UAV - v_target|`` in the (East, North) plane, driving the variable-
    amplitude weave's onset (``TASK-014``). The aircraft velocity is its ground
    speed on its heading; the target velocity is the true ``target_v*_ms``. The
    estimate (``target_est``) carries a velocity too and could be substituted
    here, but the true value is used so this compares fairly against the
    fixed-amplitude ``amplitude``, which also reads the true target.
    """
    v = snapshot["plane_speed_ms"]
    hdg = snapshot["plane_hdg_rad"]
    uav_ve = v * math.sin(hdg)
    uav_vn = v * math.cos(hdg)
    return math.hypot(uav_ve - snapshot["target_ve_ms"],
                      uav_vn - snapshot["target_vn_ms"])


def _heading_to_geometry(hdg_rad):
    """Harness heading -> geometry ``psi``.

    Both measure from North increasing clockwise, so this is identity. It
    exists as a named function so the convention is asserted at the boundary
    rather than assumed.
    """
    return hdg_rad


class AmplitudeAlgorithm(GeometricAlgorithm):
    """Amplitude weave, ported from ``continuous_weave.lua`` (``TASK-004``).

    Discharges ``ACT-2026-06-25-04``: the shipping weave law as a harness
    algorithm. The geometry lives in :mod:`py_harness.geometry.amplitude`; this
    adapter only converts frames, derives the phase ``s`` from the snapshot and
    returns one guidance point, so it holds no geometry and no state.

    The weave is **plane-anchored** — the reference line is recomputed from the
    aircraft's position each cycle — reproducing the achieved-amplitude behaviour
    of ``A-DEC-009`` rather than correcting it. The amplitude is curvature-limited
    by the safety factor ``eta``; ``eta = 1`` sits at the ``1/R_min`` limit.

    Also registered under the alias ``continuous_weave`` (``ACT-2026-06-25-04``).

    Configuration used: ``turn_radius_m`` (as ``R_min``), ``look_ahead_m``,
    ``weave_lambda_m``, ``weave_a_cap_m``, ``weave_d_start_m``,
    ``weave_d_full_m``, ``weave_eta``.
    """

    name = "amplitude"

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = snapshot["target_e_m"], snapshot["target_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        try:
            g = amplitude_geom.guidance(
                px, py, psi_i, tx, ty, _weave_phase_m(snapshot),
                cfg["weave_lambda_m"], cfg["turn_radius_m"], cfg["weave_a_cap_m"],
                cfg["weave_d_start_m"], cfg["weave_d_full_m"], cfg["weave_eta"],
                cfg["look_ahead_m"], cfg.get("weave_envelope", "smoothstep"),
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": {
                "amplitude_m": g["amplitude"],
                "curvature": g["curvature"],
            },
        }


class AmplitudeOrbitAlgorithm(GeometricAlgorithm):
    """Amplitude weave handed off to the orbit via the ramp (``TASK-004``).

    The amplitude weave is the approach; within a look-ahead of the ring it ramps
    into the orbit about the target, keeping the on-orbit behaviour for a fixed or
    moving target. Geometry in :mod:`py_harness.geometry.amplitude_orbit`; this
    adapter stays a thin, stateless translator.

    Configuration used: as :class:`AmplitudeAlgorithm` plus ``orbit_radius_m``.
    """

    name = "amplitude_orbit"
    holds_orbit = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = snapshot["target_e_m"], snapshot["target_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        try:
            g = amplitude_orbit_geom.guidance(
                px, py, psi_i, tx, ty, _weave_phase_m(snapshot),
                cfg["orbit_radius_m"], cfg["look_ahead_m"],
                cfg["weave_lambda_m"], cfg["turn_radius_m"], cfg["weave_a_cap_m"],
                cfg["weave_d_start_m"], cfg["weave_d_full_m"], cfg["weave_eta"],
                cfg.get("weave_envelope", "smoothstep"),
                precompensate=cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        state = {"phase": g["phase"]}
        if g["amplitude"] is not None:
            state["amplitude_m"] = g["amplitude"]
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": state,
        }


class VarAmplitudeAlgorithm(GeometricAlgorithm):
    """Variable-amplitude weave (``TASK-014``); alias ``vaw``.

    Sibling of :class:`AmplitudeAlgorithm`. The geometry lives in
    :mod:`py_harness.geometry.var_amplitude_weave`; this adapter converts frames,
    derives the phase ``s`` and the relative speed ``v_rel`` from the snapshot,
    and returns one guidance point, holding no geometry and no state.

    The amplitude is minimised (straight) far from the target and grows as the
    aircraft closes, with the onset scaled by ``v_rel`` — a **provisional**,
    unapproved law (``DEC-2026-07-28-04``). It stays **plane-anchored**
    (``A-DEC-009``) and curvature-limited by ``eta`` (``|kappa| <= 1/R_min``).

    Configuration used: as :class:`AmplitudeAlgorithm` but ``weave_vaw_lead_s``
    replaces ``weave_d_start_m`` (the onset is derived from ``v_rel``).
    """

    name = "var_amplitude"

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = snapshot["target_e_m"], snapshot["target_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        v_rel = _relative_speed(snapshot)
        try:
            g = var_amplitude_weave_geom.guidance(
                px, py, psi_i, tx, ty, _weave_phase_m(snapshot), v_rel,
                cfg["weave_lambda_m"], cfg["turn_radius_m"], cfg["weave_a_cap_m"],
                cfg["weave_d_full_m"], cfg["weave_vaw_lead_s"], cfg["weave_eta"],
                cfg["look_ahead_m"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": {
                "amplitude_m": g["amplitude"],
                "curvature": g["curvature"],
                "relative_speed_ms": v_rel,
            },
        }


class VarAmplitudeOrbitAlgorithm(GeometricAlgorithm):
    """Variable-amplitude weave handed off to the orbit via the ramp (``TASK-014``).

    Alias ``vaw_orbit``. As :class:`VarAmplitudeAlgorithm` for the approach, then
    ramps into the orbit about the (possibly moving) target. Geometry in
    :mod:`py_harness.geometry.vaw_orbit`; a thin, stateless translator.

    Configuration used: as :class:`VarAmplitudeAlgorithm` plus ``orbit_radius_m``.
    """

    name = "var_amplitude_orbit"
    holds_orbit = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = snapshot["target_e_m"], snapshot["target_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        v_rel = _relative_speed(snapshot)
        try:
            g = vaw_orbit_geom.guidance(
                px, py, psi_i, tx, ty, _weave_phase_m(snapshot), v_rel,
                cfg["orbit_radius_m"], cfg["look_ahead_m"],
                cfg["weave_lambda_m"], cfg["turn_radius_m"], cfg["weave_a_cap_m"],
                cfg["weave_d_full_m"], cfg["weave_vaw_lead_s"], cfg["weave_eta"],
                precompensate=cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        state = {"phase": g["phase"], "relative_speed_ms": v_rel}
        if g["amplitude"] is not None:
            state["amplitude_m"] = g["amplitude"]
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": state,
        }


class DubinsAlgorithm(GeometricAlgorithm):
    """Shortest Dubins path from the aircraft pose to the target.

    Uses all six families from :mod:`py_harness.geometry.dubins`, the port of
    ``py_plots/dubins_path.py`` which is itself the port of ``SRC-GEOM``. The
    terminal heading is taken as the current bearing to the target, so the
    aircraft is asked to arrive pointing at it.

    Configuration used: ``turn_radius_m``, ``look_ahead_m``, ``delta_psi_rad``,
    ``delta_d_m``.

    Raises ``NoSolution`` when no family solves, which for the CSC families
    happens when the circle centres are closer than ``2*rho``.
    """

    name = "dubins"

    def guidance_point(self, snapshot):
        cfg = self.config
        # (north, east) -> geometry (x = East, y = North). IR-008.
        xi, yi = snapshot["plane_e_m"], snapshot["plane_n_m"]
        # Estimate/prediction when an estimator runs (TASK-017 look-ahead flows
        # through target_est); the true target otherwise. Unchanged without --estimate.
        xf, yf = _target_ea(snapshot)
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])

        dx, dy = xf - xi, yf - yi
        if math.hypot(dx, dy) < 1e-6:
            raise NoSolution("aircraft is on top of the target")
        psi_f = math.atan2(dx, dy)  # bearing to target, from North, clockwise

        best = dubins_geom.shortest(
            xi,
            yi,
            psi_i,
            xf,
            yf,
            psi_f,
            cfg["turn_radius_m"],
            cfg["delta_psi_rad"],
            cfg["delta_d_m"],
        )
        if best is None:
            raise NoSolution("no Dubins family solves this configuration")

        family, pts, length = best
        gx, gy = orbit_geom.point_at_arc_length(pts, cfg["look_ahead_m"])
        # geometry (x = East, y = North) -> (north, east). IR-008.
        return {
            "guidance_n_m": gy,
            "guidance_e_m": gx,
            "algorithm_state": {"path_length_m": length, "family": family},
        }


class DubinsTargetCircleAlgorithm(GeometricAlgorithm):
    """Dubins path whose final turn circle is the target-centred ring (``TASK-024``).

    Unlike :class:`DubinsAlgorithm`, which aims a terminal *pose* at the target and
    offsets the final turn circle to its left or right, this flies a Dubins path
    whose **final circle is centred on the target** (radius ``orbit_radius_m``), so
    the aircraft arrives **tangent to the ring**. The geometry
    (:mod:`py_harness.geometry.dubins_target_circle`) tries both initial turn senses
    and both ring senses and returns the least-cost approach; the clockwise /
    counter-clockwise sense is therefore fixed by geometry. Curvature is bounded by
    construction: ``orbit_radius_m >= turn_radius_m`` (else ``NoSolution``).

    **Reaching** the ring is this adapter's job; **continuing** around it is
    ``TASK-025``. Stateless, like the other adapters.

    Configuration used: ``turn_radius_m``, ``orbit_radius_m``, ``look_ahead_m``,
    ``delta_psi_rad``, ``delta_d_m``.
    """

    name = "dubins_target_circle"

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = _target_ea(snapshot)  # estimate/prediction if --estimate, else true
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        try:
            g = dubins_target_circle_geom.guidance(
                px, py, psi_i, tx, ty,
                cfg["orbit_radius_m"], cfg["turn_radius_m"], cfg["look_ahead_m"],
                cfg["delta_psi_rad"], cfg["delta_d_m"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": {
                "direction": g["direction"],
                "reach_length_m": g["reach_length_m"],
                "curvature": g["curvature"],
            },
        }


class DubinsTargetOrbitAlgorithm(GeometricAlgorithm):
    """Approach on the target-centred circle, then orbit — ramp-free (``TASK-025``).

    The station-keeping successor to :class:`DubinsTargetCircleAlgorithm`: it flies
    the same target-centred final-circle Dubins approach while **outside** the ring,
    then **continues to orbit** the target once **on** it. Unlike
    :class:`DubinsOrbitAlgorithm`, there is **no ramp** — the tangent arrival of the
    ``TASK-024`` circle makes the switch continuous by geometry
    (:mod:`py_harness.geometry.dubins_target_orbit`). Stateless: the orbit sense is
    re-derived from the heading each tick, not stored.

    Configuration used: ``turn_radius_m``, ``orbit_radius_m``, ``look_ahead_m``,
    ``delta_psi_rad``, ``delta_d_m``.
    """

    name = "dubins_target_orbit"
    holds_orbit = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = _target_ea(snapshot)  # estimate/prediction if --estimate, else true
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        try:
            g = dubins_target_orbit_geom.guidance(
                px, py, psi_i, tx, ty,
                cfg["orbit_radius_m"], cfg["turn_radius_m"], cfg["look_ahead_m"],
                cfg["delta_psi_rad"], cfg["delta_d_m"],
                cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        # The pose the returned approach was actually built from. Present in the
        # approach phase only; the orbit phase commits no curve.
        committed = g.get("plan")
        plan_valid = committed is not None
        if not plan_valid:
            committed = {"px": px, "py": py, "psi": psi_i}

        state = {
            "phase": g["phase"],
            "direction": g["direction"],
            "curvature": g["curvature"],
        }
        if "ring_angle_rad" in g:
            state["ring_angle_rad"] = g["ring_angle_rad"]
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": state,
        }


class OrbitAlgorithm(GeometricAlgorithm):
    """Circle the target on a ring of radius ``orbit_radius_m``.

    The aircraft's current angular position about the target sets the ring
    angle; the guidance point is placed ``look_ahead_m`` of arc further around
    the ring, in the direction that best continues the current heading.

    Closed form — no path sampling — so the guidance point carries no
    discretisation error.

    Configuration used: ``orbit_radius_m``, ``look_ahead_m``.
    """

    name = "orbit"
    holds_orbit = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = _target_ea(snapshot)  # estimate/prediction if --estimate, else true
        R = cfg["orbit_radius_m"]

        if math.hypot(px - tx, py - ty) < 1e-6:
            raise NoSolution("aircraft is at the ring centre; no orbit angle")

        psi0 = orbit_geom.entry_angle(px, py, tx, ty)
        direction = orbit_geom.orbit_direction(
            psi0, _heading_to_geometry(snapshot["plane_hdg_rad"])
        )
        try:
            gx, gy, psi = orbit_geom.orbit_guidance_point(
                tx, ty, R, psi0, direction, cfg["look_ahead_m"],
                cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        return {
            "guidance_n_m": gy,
            "guidance_e_m": gx,
            "algorithm_state": {"ring_angle_rad": psi, "direction": float(direction)},
        }


class DubinsOrbitAlgorithm(GeometricAlgorithm):
    """Dubins approach tangent to the ring, handed off to the orbit.

    The behaviour of ``py_plots/combined.py``: fly a Dubins path to a tangent
    entry point on the ring of radius ``orbit_radius_m`` about the target, then
    circle. Outside the ring the guidance point comes from the approach path;
    once within one look-ahead of the ring a **ramp** blends it into the orbit
    (``TASK-002``), so the guidance point is continuous across the handoff rather
    than stepping at a hard distance test.

    All the geometry — the tangent entry, the shortest Dubins family, the orbit
    point and the ramp — lives in :mod:`py_harness.geometry.dubins_orbit`. This
    adapter only converts frames and wraps the result, so it stays stateless: the
    ``phase`` it reports is the ramp weight, computed each call, not stored.

    Configuration used: ``turn_radius_m``, ``orbit_radius_m``, ``look_ahead_m``,
    ``delta_psi_rad``, ``delta_d_m``.
    """

    name = "dubins_orbit"
    holds_orbit = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = _target_ea(snapshot)  # estimate/prediction if --estimate, else true
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])

        try:
            g = dubins_orbit_geom.guidance(
                px, py, psi_i, tx, ty,
                cfg["orbit_radius_m"], cfg["turn_radius_m"], cfg["look_ahead_m"],
                cfg["delta_psi_rad"], cfg["delta_d_m"],
                cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))

        state = {"phase": g["phase"]}
        if g["family"] is not None:
            state["family"] = g["family"]
        if g["ring_angle_rad"] is not None:
            state["ring_angle_rad"] = g["ring_angle_rad"]
        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": state,
        }


class AdaptiveDbCircleAlgorithm(GeometricAlgorithm):
    """CS-onto-orbit Dubins about a **predicted** target, replanned on an interval (``TASK-033``).

    The extension of :class:`DubinsTargetOrbitAlgorithm`, which is left untouched
    and still selectable. Two differences, and nothing else:

    1. **The ring is centred on the prediction.** It reads ``target_est`` — which
       ``state.py`` has already projected ``lookahead_steps`` (``k_horizon``) control
       intervals ahead on constant velocity (``TASK-017``) — and centres the whole
       construction on it: the target-centred final circle, the tangent, the
       tangency point and the orbit continuation. So the aircraft plans against
       where the kangaroo will be, not a position it can only ever arrive behind.
    2. **The plan is committed for ``replan_every`` (``n_replan``) ticks.** At a replan
       instant a new plan is generated and **taken unconditionally**; between
       replan instants the held plan is flown.

    **No cost-based selection.** There is no candidate set, no cost function, no
    scoring of the new plan against the active one, no hysteresis and no cooldown.
    The tick counter reaching ``n_replan`` is the only thing that changes the path. This
    is a *commitment interval*, not the replanning *decision* ``ADR-001`` removed.

    **The estimator is required.** Unlike the other adapters this does **not** use
    :func:`_target_ea`, which silently falls back to the true target. Without an
    estimator there is no prediction, so the run would quietly become plain
    ``dubins_target_orbit`` under a different name and be mistaken for evidence.
    It raises instead, naming ``--estimate``.

    All state travels through ``algorithm_state`` (``VR-015``, ``A-VAL-003``): the
    held centre, the commit pose, the tick counter and the previous guidance point.
    The adapter owns the replan clock; the geometry holds none.

    Reported per tick: ``phase``, ``direction``, ``curvature``, ``ring_angle_rad``
    (orbit phase only), ``replanned``, ``ticks_since_replan``, the held centre, the
    prediction lead, and ``replan_step_m`` on replan ticks where it is measurable.

    Configuration used: ``turn_radius_m``, ``orbit_radius_m``, ``look_ahead_m``,
    ``delta_psi_rad``, ``delta_d_m``, ``orbit_precompensate``, ``replan_every``,
    ``hold_policy``.
    """

    name = "adaptive_db_circle"
    holds_orbit = True
    requires_estimate = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])

        est = snapshot.get("target_est")
        if est is None:
            raise NoSolution(
                "adaptive_db_circle requires the state estimator: it centres the "
                "ring on the predicted target and has no present-position "
                "fallback. Re-run with --estimate (and --lookahead-steps k for a "
                "non-zero horizon).")

        n_replan = int(cfg["replan_every"])
        policy = cfg["hold_policy"]
        st = snapshot.get("algorithm_state") or {}

        # Replan clock. ticks_since_replan is 0 on the tick the plan was committed.
        # No plan yet (first tick) is a replan; otherwise the counter reaching m is.
        prev_ticks = st.get("ticks_since_replan")
        replanned = prev_ticks is None or (prev_ticks + 1) >= n_replan

        if replanned:
            cx, cy = est["e_m"], est["n_m"]
            # None tells the geometry to commit the live pose. Unconditionally:
            # there is no comparison with the plan being replaced (D4).
            plan = None
            ticks = 0
        else:
            cx, cy = st["centre_e_m"], st["centre_n_m"]
            # Carry the committed plan forward only while it is still a valid
            # approach commit. A replan instant that falls in the ORBIT phase
            # commits no approach curve (there is none to commit), so its stored
            # pose can be inside the ring; re-entering the approach phase with it
            # would ask for a tangent from inside the ring, which has no solution.
            # Passing None re-commits from the live pose, which the phase test
            # guarantees is outside the ring. The aircraft crosses the boundary
            # repeatedly while settling, so this is the ordinary case, not a rare
            # one.
            if st.get("plan_valid"):
                plan = {"px": st["plan_e_m"], "py": st["plan_n_m"],
                        "psi": st["plan_psi_rad"]}
            else:
                plan = None
            ticks = prev_ticks + 1

        try:
            g = adaptive_db_circle_geom.guidance(
                px, py, psi_i, cx, cy, plan,
                cfg["orbit_radius_m"], cfg["turn_radius_m"], cfg["look_ahead_m"],
                cfg["delta_psi_rad"], cfg["delta_d_m"],
                policy, cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))

        # The pose the returned approach was actually built from. Present in the
        # approach phase only; the orbit phase commits no curve.
        committed = g.get("plan")
        plan_valid = committed is not None
        if not plan_valid:
            committed = {"px": px, "py": py, "psi": psi_i}

        state = {
            "phase": g["phase"],
            "direction": g["direction"],
            "curvature": g["curvature"],
            "replanned": replanned,
            "ticks_since_replan": ticks,
            "centre_n_m": cy,
            "centre_e_m": cx,
            # How far ahead of the true kangaroo the orbited centre sits. A known
            # designed offset, not an error - reported so it is never read as one.
            "prediction_lead_m": math.hypot(cx - snapshot["target_e_m"],
                                            cy - snapshot["target_n_m"]),
            # The committed pose, carried forward so the held plan is rebuilt
            # identically next tick. Parameters, never a sampled point list.
            "plan_n_m": committed["py"],
            "plan_e_m": committed["px"],
            "plan_psi_rad": committed["psi"],
            "plan_valid": plan_valid,
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
        }
        if "ring_angle_rad" in g:
            state["ring_angle_rad"] = g["ring_angle_rad"]

        # The FR-011 / PR-008 quantity: the guidance-point jump caused by the plan
        # changing, isolated from the aircraft's own motion by evaluating the OLD
        # plan at the SAME pose. Omitted (rather than faked as 0.0) when there is
        # no old plan or it no longer solves from here.
        if replanned and prev_ticks is not None:
            old_plan = None
            if st.get("plan_valid"):
                old_plan = {"px": st["plan_e_m"], "py": st["plan_n_m"],
                            "psi": st["plan_psi_rad"]}
            try:
                old = adaptive_db_circle_geom.guidance(
                    px, py, psi_i, st["centre_e_m"], st["centre_n_m"], old_plan,
                    cfg["orbit_radius_m"], cfg["turn_radius_m"],
                    cfg["look_ahead_m"], cfg["delta_psi_rad"], cfg["delta_d_m"],
                    policy, cfg["orbit_precompensate"],
                )
            except ValueError:
                pass
            else:
                state["replan_step_m"] = math.hypot(g["gx"] - old["gx"],
                                                    g["gy"] - old["gy"])
        elif not replanned:
            state["replan_step_m"] = 0.0

        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": state,
        }


class AdaptiveHorizonCsAlgorithm(GeometricAlgorithm):
    """CS-orbit whose prediction horizon is **selected** each replan (``TASK-039`` arm B).

    Arm B of the three-approach comparison. Structurally
    :class:`AdaptiveDbCircleAlgorithm` with one substitution: instead of consuming
    the configured ``lookahead_steps``, it evaluates the candidate horizons in
    ``cfg["ah_candidate_horizons"]`` at each replan instant and keeps the one that
    minimises the objective in
    :mod:`py_harness.geometry.adaptive_horizon`.

    **Exactly one thing differs from arm A.** The winning centre is handed to
    arm A's own construction (:mod:`adaptive_db_circle`), unmodified, so the
    committed CS curve, the hold policy, the replan clock, the orbit continuation
    and the reported state are all arm A's. Collapse the candidate set to one
    horizon and this reproduces arm A at that horizon field for field, which is
    asserted as a test rather than claimed.

    **This arm reinstates cost-based selection.** ``ADR-001`` superseded it and
    ``ADR-004`` records that arm A was built without it. Selecting among scored
    candidates reopens ``FR-004`` and ``FR-008``..``FR-011`` in substance. That is
    the point of the arm and the reason ``TASK-039`` requires a decision record
    before any arm is adopted; nothing here changes a requirement's status, and
    running this algorithm is not an adoption.

    **The horizon is selected in the approach phase only.** On the ring there is
    no tangent point, so there is nothing to score (``TASK-038``'s structural
    finding). The last selection is held and the orbit continues about the centre
    predicted at it. The consequence is recorded rather than hidden: this arm
    makes no decision during the phase in which the orbit deformation
    (``TASK-027``, reopened) occurs, so it cannot be expected to remove it.

    **The estimator is required**, and the estimate must be **un-projected**: this
    reads ``target_est_raw``, not ``target_est``, because the state-side horizon
    is the thing being replaced. A non-zero ``lookahead_steps`` is therefore
    refused by the runner and the driver (``owns_horizon``) rather than silently
    double-counted.

    Reported per tick, additionally to arm A's fields: ``k_horizon_steps`` (the
    selection), ``selection_cost``, ``e_tan_pred_m`` (the signed ``TASK-038``
    registration error the winner predicts), ``n_a_steps`` (the transit the
    winner implies), ``candidates_scored`` and ``candidates_solved``.

    Configuration used: everything :class:`AdaptiveDbCircleAlgorithm` uses, plus
    ``dt_s``, ``airspeed_ms``, ``ah_candidate_horizons``, ``ah_w_tangent``,
    ``ah_w_radial``, ``ah_w_curvature``, ``ah_w_switch``, ``ah_path_samples``.
    """

    name = "adaptive_horizon_cs"
    holds_orbit = True
    requires_estimate = True
    owns_horizon = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        est = _raw_estimate(snapshot, "adaptive_horizon_cs")

        n_replan = int(cfg["replan_every"])
        policy = cfg["hold_policy"]
        st = snapshot.get("algorithm_state") or {}

        prev_ticks = st.get("ticks_since_replan")
        replanned = prev_ticks is None or (prev_ticks + 1) >= n_replan
        k_prev = st.get("k_horizon_steps")

        weights = {
            "tangent": cfg["ah_w_tangent"],
            "radial": cfg["ah_w_radial"],
            "curvature": cfg["ah_w_curvature"],
            "switch": cfg["ah_w_switch"],
        }
        # The standoff term scores the final two ring radii of the approach; see
        # the geometry module for why the whole curve is the wrong window.
        tube_len_m = 2.0 * cfg["orbit_radius_m"]

        # Phase gate (TASK-038's structural finding, enforced rather than hoped
        # for). The horizon is re-selected only while the aircraft is OUTSIDE the
        # ring about the centre it is currently holding — the approach phase,
        # where a tangent point exists to score. Without this gate the selector
        # runs on the ring too, where the only candidates that still produce a CS
        # solution are the short ones; it then picks a near-zero horizon, the ring
        # stops leading, the aircraft is pushed off it, the transit grows and the
        # horizon jumps back. Measured as a limit cycle spanning the whole
        # candidate set (k = 0 to 60 within one tail), which is worse than either
        # end of it held fixed.
        k_hold = (k_prev if k_prev is not None
                  else cfg["ah_candidate_horizons"][0])
        hold_x, hold_y = adaptive_horizon_geom.predicted_centre(
            est["e_m"], est["n_m"], est["ve_ms"], est["vn_ms"], k_hold,
            cfg["dt_s"])
        in_orbit_phase = math.hypot(px - hold_x, py - hold_y) <= cfg["orbit_radius_m"]

        chosen = None
        if replanned and not in_orbit_phase:
            chosen = adaptive_horizon_geom.select_horizon(
                px, py, psi_i, est["e_m"], est["n_m"], est["ve_ms"],
                est["vn_ms"], k_prev, cfg["dt_s"], cfg["airspeed_ms"],
                cfg["orbit_radius_m"], cfg["turn_radius_m"],
                cfg["delta_psi_rad"], cfg["delta_d_m"],
                cfg["ah_candidate_horizons"], weights,
                int(cfg["ah_path_samples"]), tube_len_m)

        if chosen is not None:
            k_sel = chosen["k_steps"]
            cx, cy = chosen["centre_x"], chosen["centre_y"]
            plan = None
            ticks = 0
        elif replanned:
            # Either the phase gate suppressed the selection, or no candidate
            # solved (every one of them puts the aircraft inside its own ring).
            # Both are the orbit phase: hold the last selection and re-centre the
            # ring on the target predicted at it, exactly as arm A re-centres on
            # its fixed horizon. This is a phase, not a failure, so it must not
            # raise, and the horizon must not be silently reset to zero.
            k_sel = k_hold
            cx, cy = hold_x, hold_y
            plan = None
            ticks = 0
        else:
            k_sel = k_prev
            cx, cy = st["centre_e_m"], st["centre_n_m"]
            if st.get("plan_valid"):
                plan = {"px": st["plan_e_m"], "py": st["plan_n_m"],
                        "psi": st["plan_psi_rad"]}
            else:
                plan = None
            ticks = prev_ticks + 1

        try:
            g = adaptive_db_circle_geom.guidance(
                px, py, psi_i, cx, cy, plan,
                cfg["orbit_radius_m"], cfg["turn_radius_m"], cfg["look_ahead_m"],
                cfg["delta_psi_rad"], cfg["delta_d_m"],
                policy, cfg["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))

        committed = g.get("plan")
        plan_valid = committed is not None
        if not plan_valid:
            committed = {"px": px, "py": py, "psi": psi_i}

        state = {
            "phase": g["phase"],
            "direction": g["direction"],
            "curvature": g["curvature"],
            "replanned": replanned,
            "ticks_since_replan": ticks,
            "centre_n_m": cy,
            "centre_e_m": cx,
            "prediction_lead_m": math.hypot(cx - snapshot["target_e_m"],
                                            cy - snapshot["target_n_m"]),
            "plan_n_m": committed["py"],
            "plan_e_m": committed["px"],
            "plan_psi_rad": committed["psi"],
            "plan_valid": plan_valid,
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            # The selection itself. Carried every tick, not only on replans, so a
            # plot of the horizon against time has no holes in it.
            "k_horizon_steps": k_sel,
            # False on the ticks the phase gate suppressed re-selection, so a
            # plot of the horizon shows where it was CHOSEN and where it was
            # merely HELD. Withholding that distinction would read as a stable
            # selector when it is a suppressed one.
            "horizon_selected": chosen is not None,
        }
        if "ring_angle_rad" in g:
            state["ring_angle_rad"] = g["ring_angle_rad"]
        if chosen is not None:
            state["selection_cost"] = chosen["cost"]
            # Signed, and PREDICTED: what the winner expects the TASK-038
            # registration error to be. The achieved value is measured offline
            # from the recorded history by py_harness.tangent_error.
            state["e_tan_pred_m"] = chosen["e_tan_m"]
            state["n_a_steps"] = chosen["n_a_steps"]
            state["candidates_scored"] = chosen["evaluated"]
            state["candidates_solved"] = chosen["solved"]
            state["ring_feasible"] = chosen["feasible"]

        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": state,
        }


class RhGeometricAlgorithm(GeometricAlgorithm):
    """Receding-horizon geometric planner (``TASK-039`` arm C).

    Arm C of the three-approach comparison, and the only one of the three that is
    not a ring construction. Each replan it rolls a two-segment curvature grid
    forward over ``rh_horizon_steps`` ticks against the *predicted* target,
    scores each rollout on its achieved standoff range, and emits one guidance
    point a look-ahead along the winner. See
    :mod:`py_harness.geometry.rh_geometric` for the model and the objective.

    **Not an MPC.** No solver, no gradient, no convergence criterion: an
    exhaustive search over ``rh_candidates * rh_candidates_2`` feasible
    candidates. That is what keeps it inside ``VR-015`` and what makes its
    planning cost a fixed number — ``cfg.rh_rollout_steps`` aircraft steps per
    replan — rather than something to be measured after the fact. ``PR-002`` and
    ``A-SW-002`` are the binding constraints on this arm and the reason the cost
    is reported per tick.

    **Curvature is bounded by construction**, not by rejection: the grid spans
    ``[-1/rho, +1/rho]``, so ``FR-005`` and ``SR-002`` hold for every candidate
    including the ones not chosen.

    **Between replans the command is held, not the path.** The committed
    curvature pair is re-integrated from the live pose each tick, so there is no
    stored trajectory for the aircraft to drift away from. A commitment interval
    longer than one tick costs only the staleness of the target prediction, which
    is the variable under study.

    **The estimator is required** and the estimate must be un-projected
    (``owns_horizon``): this arm predicts the target over its own horizon.

    Reported per tick: ``phase``, ``curvature`` (the commanded first segment),
    ``kappa1_1pm``, ``kappa2_1pm``, ``plan_cost``, ``rollout_rms_error_m``,
    ``terminal_error_m``, ``replanned``, ``ticks_since_replan``,
    ``candidates_scored`` and ``rollout_steps``.

    Configuration used: ``airspeed_ms``, ``turn_radius_m``, ``orbit_radius_m``,
    ``dt_s``, ``replan_every``, ``rh_horizon_steps``, ``rh_command_steps``,
    ``rh_segment_steps``, ``rh_candidates``, ``rh_candidates_2``,
    ``rh_w_standoff``, ``rh_w_terminal``, ``rh_w_effort``, ``rh_w_smooth``.
    """

    name = "rh_geometric"
    holds_orbit = True
    requires_estimate = True
    owns_horizon = True

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        est = _raw_estimate(snapshot, "rh_geometric")

        n_replan = int(cfg["replan_every"])
        st = snapshot.get("algorithm_state") or {}
        prev_ticks = st.get("ticks_since_replan")
        replanned = prev_ticks is None or (prev_ticks + 1) >= n_replan

        segment_steps = int(cfg["rh_segment_steps"])
        kappa_prev = st.get("kappa1_1pm")

        if replanned:
            held = None
            ticks = 0
        else:
            ticks = prev_ticks + 1
            remaining = segment_steps - ticks
            if remaining < 0:
                remaining = 0
            held = {"kappa1": st["kappa1_1pm"], "kappa2": st["kappa2_1pm"],
                    "remaining_hold_steps": remaining}

        weights = {
            "standoff": cfg["rh_w_standoff"],
            "terminal": cfg["rh_w_terminal"],
            "effort": cfg["rh_w_effort"],
            "smooth": cfg["rh_w_smooth"],
        }

        try:
            g = rh_geometric_geom.guidance(
                px, py, psi_i, est["e_m"], est["n_m"], est["ve_ms"],
                est["vn_ms"], cfg["dt_s"], cfg["airspeed_ms"],
                cfg["orbit_radius_m"], cfg["turn_radius_m"],
                int(cfg["rh_command_steps"]),
                int(cfg["rh_horizon_steps"]), segment_steps,
                int(cfg["rh_candidates"]), int(cfg["rh_candidates_2"]),
                weights, kappa_prev, held)
        except ValueError as exc:
            raise NoSolution(str(exc))

        return {
            "guidance_n_m": g["gy"],
            "guidance_e_m": g["gx"],
            "algorithm_state": {
                "phase": g["phase"],
                # No ring is constructed, so there is no CW/CCW sense to report.
                # Named from the sign of the commanded curvature instead, which
                # is the comparable quantity.
                "direction": "cw" if g["kappa1"] >= 0.0 else "ccw",
                "curvature": g["curvature"],
                "kappa1_1pm": g["kappa1"],
                "kappa2_1pm": g["kappa2"],
                "replanned": replanned,
                "ticks_since_replan": ticks,
                "plan_cost": g["cost"],
                "rollout_rms_error_m": g["mean_error_m"],
                "terminal_error_m": g["terminal_error_m"],
                "candidates_scored": g["evaluated"],
                "rollout_steps": g["rollout_steps"],
                "command_steps": g["command_steps"],
                # The centre this arm holds its standoff about is the target it
                # predicted from, so metrics.centre_of("ring") reports something
                # meaningful for it rather than falling through to the truth.
                "centre_n_m": est["n_m"],
                "centre_e_m": est["e_m"],
                "prediction_lead_m": math.hypot(
                    est["e_m"] - snapshot["target_e_m"],
                    est["n_m"] - snapshot["target_n_m"]),
            },
        }


class HeadingAAlgorithm(GeometricAlgorithm):
    """Heading-alignment fly-to (``TASK-010``): aim at the estimated kangaroo.

    The simple baseline to compare against the Dubins and weave families. It reads
    the **estimated** target (``target_est``, from the state estimator, `TASK-012`)
    when one is running, else the true target, and places the guidance point a
    look-ahead along the bearing to it. Geometry in
    :mod:`py_harness.geometry.heading`; this adapter only translates.

    Configuration used: ``look_ahead_m``.
    """

    name = "heading_a"

    def guidance_point(self, snapshot):
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = _target_ea(snapshot)
        try:
            gx, gy = heading_geom.guidance(px, py, tx, ty,
                                           self.config["look_ahead_m"])
        except ValueError as exc:
            raise NoSolution(str(exc))
        return {"guidance_n_m": gy, "guidance_e_m": gx, "algorithm_state": {}}


class HeadingAOrbitAlgorithm(GeometricAlgorithm):
    """Heading fly-to ramped into the orbit about the estimated kangaroo.

    ``heading_a`` on approach, then the orbit via the shared ramp (`TASK-010`).

    Configuration used: ``orbit_radius_m``, ``look_ahead_m``.
    """

    name = "heading_a_orbit"
    holds_orbit = True

    def guidance_point(self, snapshot):
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        tx, ty = _target_ea(snapshot)
        try:
            gx, gy, phase = heading_geom.guidance_orbit(
                px, py, psi_i, tx, ty,
                self.config["orbit_radius_m"], self.config["look_ahead_m"],
                self.config["orbit_precompensate"],
            )
        except ValueError as exc:
            raise NoSolution(str(exc))
        return {"guidance_n_m": gy, "guidance_e_m": gx,
                "algorithm_state": {"phase": phase}}


#: Name-to-class registry. Selecting an algorithm is a lookup here and nothing
#: else; no other part of the harness may branch on algorithm identity.
REGISTRY = {
    AmplitudeAlgorithm.name: AmplitudeAlgorithm,
    AmplitudeOrbitAlgorithm.name: AmplitudeOrbitAlgorithm,
    VarAmplitudeAlgorithm.name: VarAmplitudeAlgorithm,
    VarAmplitudeOrbitAlgorithm.name: VarAmplitudeOrbitAlgorithm,
    DubinsAlgorithm.name: DubinsAlgorithm,
    DubinsTargetCircleAlgorithm.name: DubinsTargetCircleAlgorithm,
    DubinsTargetOrbitAlgorithm.name: DubinsTargetOrbitAlgorithm,
    OrbitAlgorithm.name: OrbitAlgorithm,
    DubinsOrbitAlgorithm.name: DubinsOrbitAlgorithm,
    AdaptiveDbCircleAlgorithm.name: AdaptiveDbCircleAlgorithm,
    AdaptiveHorizonCsAlgorithm.name: AdaptiveHorizonCsAlgorithm,
    RhGeometricAlgorithm.name: RhGeometricAlgorithm,
    HeadingAAlgorithm.name: HeadingAAlgorithm,
    HeadingAOrbitAlgorithm.name: HeadingAOrbitAlgorithm,
    # Alias: continuous_weave is the amplitude weave (ACT-2026-06-25-04).
    "continuous_weave": AmplitudeAlgorithm,
    # Aliases: vaw / vaw_orbit are the variable-amplitude weave (TASK-014).
    "vaw": VarAmplitudeAlgorithm,
    "vaw_orbit": VarAmplitudeOrbitAlgorithm,
}

#: Algorithms with geometry implemented; ``continuous_weave`` is an alias of
#: ``amplitude``.
IMPLEMENTED = ("amplitude", "amplitude_orbit", "var_amplitude",
               "var_amplitude_orbit", "dubins", "dubins_target_circle",
               "dubins_target_orbit", "orbit", "dubins_orbit", "heading_a",
               "heading_a_orbit", "adaptive_db_circle", "adaptive_horizon_cs",
               "rh_geometric")


def build(name, config):
    """Construct the algorithm registered under ``name``.

    Args:
        name: Registry key, one of :data:`REGISTRY`.
        config: Plain dict of float configuration. Build it from a
            :class:`~py_harness.config.HarnessConfig` with
            :func:`config_dict`.

    Returns:
        A :class:`~interface.GeometricAlgorithm` instance.

    Raises:
        KeyError: If ``name`` is not registered. The message lists the
            available names, because a typo here is otherwise indistinguishable
            from an unimplemented algorithm.
    """
    if name not in REGISTRY:
        raise KeyError(
            "unknown algorithm %r; available: %s"
            % (name, ", ".join(sorted(REGISTRY)))
        )
    return REGISTRY[name](config)


def config_dict(cfg):
    """Flatten a :class:`~py_harness.config.HarnessConfig` for an algorithm.

    Plain floats in a plain dict, per ``DEC-2026-06-25-04``. Derived values are
    included so an algorithm never recomputes the coordinated-turn relation
    itself.
    """
    return {
        "airspeed_ms": cfg.airspeed_ms,
        "turn_radius_m": cfg.turn_radius_m,
        "orbit_radius_m": cfg.orbit_radius_m,
        "bank_limit_deg": cfg.bank_limit_deg,
        "gravity_ms2": cfg.gravity_ms2,
        "look_ahead_m": cfg.look_ahead_m,
        "delta_psi_rad": cfg.delta_psi_rad,
        "delta_d_m": cfg.delta_d_m,
        "turn_radius_bank_deg": cfg.turn_radius_bank_deg,
        "orbit_bank_deg": cfg.orbit_bank_deg,
        "bank_limited_turn_radius_m": cfg.bank_limited_turn_radius_m,
        "weave_lambda_m": cfg.weave_lambda_m,
        "weave_a_cap_m": cfg.weave_a_cap_m,
        "weave_d_start_m": cfg.weave_d_start_m,
        "weave_d_full_m": cfg.weave_d_full_m,
        "weave_eta": cfg.weave_eta,
        "weave_vaw_lead_s": cfg.weave_vaw_lead_s,
        "orbit_precompensate": cfg.orbit_precompensate,
        # TASK-033. replan_every is a whole tick count and hold_policy an
        # enumerated string; both map onto Lua scalars, so the plain-dict
        # contract (DEC-2026-06-25-04) still holds.
        "replan_every": cfg.replan_every,
        "hold_policy": cfg.hold_policy,
        # TASK-039. An algorithm that owns its prediction horizon (arms B and C)
        # projects the target itself, so it needs the control interval; every
        # other algorithm reads a horizon that state.py has already applied and
        # ignores this. Still a plain float, so DEC-2026-06-25-04 holds.
        "dt_s": cfg.dt_s,
        # Arm B: the candidate horizons are derived from the three bounds so the
        # bounds and the list they generate cannot disagree. A plain list of
        # whole tick counts maps onto a Lua array.
        "ah_candidate_horizons": cfg.ah_candidate_horizons,
        "ah_w_tangent": cfg.ah_w_tangent,
        "ah_w_radial": cfg.ah_w_radial,
        "ah_w_curvature": cfg.ah_w_curvature,
        "ah_w_switch": cfg.ah_w_switch,
        "ah_path_samples": cfg.ah_path_samples,
        # Arm C.
        "rh_horizon_steps": cfg.rh_horizon_steps,
        "rh_candidates": cfg.rh_candidates,
        "rh_candidates_2": cfg.rh_candidates_2,
        "rh_segment_steps": cfg.rh_segment_steps,
        "rh_command_steps": cfg.rh_command_steps,
        "rh_w_standoff": cfg.rh_w_standoff,
        "rh_w_terminal": cfg.rh_w_terminal,
        "rh_w_effort": cfg.rh_w_effort,
        "rh_w_smooth": cfg.rh_w_smooth,
    }
