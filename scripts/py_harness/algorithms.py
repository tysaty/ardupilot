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

from .geometry import amplitude as amplitude_geom
from .geometry import amplitude_orbit as amplitude_orbit_geom
from .geometry import dubins as dubins_geom
from .geometry import dubins_orbit as dubins_orbit_geom
from .geometry import dubins_target_circle as dubins_target_circle_geom
from .geometry import dubins_target_orbit as dubins_target_orbit_geom
from .geometry import heading as heading_geom
from .geometry import orbit as orbit_geom
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
               "heading_a_orbit")


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
    }
