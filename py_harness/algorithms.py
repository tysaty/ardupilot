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

from .geometry import dubins as dubins_geom
from .geometry import orbit as orbit_geom
from .interface import GeometricAlgorithm, NoSolution


def _heading_to_geometry(hdg_rad):
    """Harness heading -> geometry ``psi``.

    Both measure from North increasing clockwise, so this is identity. It
    exists as a named function so the convention is asserted at the boundary
    rather than assumed.
    """
    return hdg_rad


class ContinuousWeaveAlgorithm(GeometricAlgorithm):
    """Port of the shipping continuous weave law.

    To be transliterated from ``modules/continuous_weave.lua``, whose
    ``straight_weave`` (line 37) is the shipping guidance law and the only one
    the controller currently calls. Porting it in first gives the harness a
    known-behaviour reference against which a second algorithm can be judged
    (``ACT-2026-06-25-04``).

    Two properties of the Lua source must survive the port, or the harness is
    validating something the aircraft does not fly:

    * the weave is **plane-anchored** — the reference line is recomputed from
      the aircraft's current position every cycle, which is the source of the
      achieved-amplitude discrepancy recorded in ``ADR-001`` and
      ``A-DEC-009``. The port must reproduce the defect, not silently fix it.
    * the sinusoid's phase argument is the **arc-length accumulator**, not
      elapsed time. It is accumulated state, so it travels through the
      snapshot's ``algorithm_state`` field rather than a module global
      (``DEC-2026-06-25-04``, ``A-VAL-003``).

    Status: SKELETON. Still open as ``ACT-2026-06-25-04``.
    """

    name = "continuous_weave"

    def guidance_point(self, snapshot):
        raise NotImplementedError(
            "Port of continuous_weave.lua straight_weave is ACT-2026-06-25-04."
        )


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
        xf, yf = snapshot["target_e_m"], snapshot["target_n_m"]
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

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = snapshot["target_e_m"], snapshot["target_n_m"]
        R = cfg["orbit_radius_m"]

        if math.hypot(px - tx, py - ty) < 1e-6:
            raise NoSolution("aircraft is at the ring centre; no orbit angle")

        psi0 = orbit_geom.entry_angle(px, py, tx, ty)
        direction = orbit_geom.orbit_direction(
            psi0, _heading_to_geometry(snapshot["plane_hdg_rad"])
        )
        gx, gy, psi = orbit_geom.orbit_point_at_arc_length(
            tx, ty, R, psi0, direction, cfg["look_ahead_m"]
        )
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
    once within one look-ahead of the ring it comes from the orbit.

    The handoff is a distance test rather than a mode flag, so the algorithm
    stays stateless and the harness keeps no behaviour state — the property
    ``ADR-001`` removed from the controller and which the supervisor asked to
    keep simple here.

    Configuration used: ``turn_radius_m``, ``orbit_radius_m``, ``look_ahead_m``,
    ``delta_psi_rad``, ``delta_d_m``.
    """

    name = "dubins_orbit"

    def guidance_point(self, snapshot):
        cfg = self.config
        px, py = snapshot["plane_e_m"], snapshot["plane_n_m"]
        tx, ty = snapshot["target_e_m"], snapshot["target_n_m"]
        psi_i = _heading_to_geometry(snapshot["plane_hdg_rad"])
        R = cfg["orbit_radius_m"]

        range_m = math.hypot(px - tx, py - ty)
        entries = orbit_geom.tangent_points(px, py, tx, ty, R)

        # On or inside the ring, or within a look-ahead of it: orbit.
        if not entries or range_m <= R + cfg["look_ahead_m"]:
            if range_m < 1e-6:
                raise NoSolution("aircraft is at the ring centre; no orbit angle")
            psi0 = orbit_geom.entry_angle(px, py, tx, ty)
            direction = orbit_geom.orbit_direction(psi0, psi_i)
            gx, gy, psi = orbit_geom.orbit_point_at_arc_length(
                tx, ty, R, psi0, direction, cfg["look_ahead_m"]
            )
            return {
                "guidance_n_m": gy,
                "guidance_e_m": gx,
                "algorithm_state": {"phase": 1.0, "ring_angle_rad": psi},
            }

        # Outside: Dubins to the first tangent entry, arriving along the tangent.
        ex, ey, psi_f, _sign = entries[0]
        best = dubins_geom.shortest(
            px,
            py,
            psi_i,
            ex,
            ey,
            psi_f,
            cfg["turn_radius_m"],
            cfg["delta_psi_rad"],
            cfg["delta_d_m"],
        )
        if best is None:
            raise NoSolution("no Dubins family reaches the ring entry point")
        family, pts, length = best
        gx, gy = orbit_geom.point_at_arc_length(pts, cfg["look_ahead_m"])
        return {
            "guidance_n_m": gy,
            "guidance_e_m": gx,
            "algorithm_state": {
                "phase": 0.0,
                "approach_length_m": length,
                "family": family,
            },
        }


#: Name-to-class registry. Selecting an algorithm is a lookup here and nothing
#: else; no other part of the harness may branch on algorithm identity.
REGISTRY = {
    ContinuousWeaveAlgorithm.name: ContinuousWeaveAlgorithm,
    DubinsAlgorithm.name: DubinsAlgorithm,
    OrbitAlgorithm.name: OrbitAlgorithm,
    DubinsOrbitAlgorithm.name: DubinsOrbitAlgorithm,
}

#: Algorithms with geometry implemented. ``continuous_weave`` is still a stub.
IMPLEMENTED = ("dubins", "orbit", "dubins_orbit")


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
    }
