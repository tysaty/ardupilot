"""Plane and target state, time stepping, recorded history and the plotter.

This is the state module of the two-module harness required by
``DEC-2026-06-25-01``. It owns:

1. the state of the plane and the target at a given time;
2. advancing that state by one fixed time step; and
3. the plotter (``DEC-2026-06-25-03``).

It owns no geometry. It calls a :class:`~interface.GeometricAlgorithm` to
obtain each guidance point and knows nothing about how that point is derived.

Recorded tension (``DEC-2026-06-25-03``)
----------------------------------------
``ARCHITECTURE.md`` requires component responsibilities and data ownership to
stay explicit and independently testable. Co-locating the plotter with state
weakens that. The plotter was placed here by request, and the mitigation is
part of the decision rather than an implementation detail: the plotter reads a
recorded state history and shall not mutate state or call the algorithm.

If the plotter later needs to drive the algorithm — to sweep a parameter
interactively, as the existing ``py_plots/`` sliders do — this separation is
under pressure and must be revisited rather than quietly relaxed.

Modelling limitation (``A-VAL-001``)
------------------------------------
This is a **kinematic** model. The aircraft flies at constant speed and turns
at up to ``v / R``; there is no L1, TECS, EKF or airframe response. It can
validate geometry. It cannot validate tracking, and it must not be cited as
evidence for ``SR-001`` to ``SR-003``.
"""

import math

from .interface import NoSolution


def wrap_pi(a):
    """Wrap an angle to ``[-pi, pi)``. Angles are wrapped before comparison.

    Half-open at ``+pi``: an input of exactly ``-pi`` or ``+pi`` returns
    ``-pi``. ``IR-003`` requires the interval to be documented, and this is it.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PlaneState:
    """Aircraft state at one instant, in local North/East metres.

    Attributes are plain floats. Headings are radians clockwise from North
    (``IR-002``); distances metres, speeds m/s (``IR-005``).
    """

    __slots__ = ("n_m", "e_m", "hdg_rad", "speed_ms")

    def __init__(self, n_m=0.0, e_m=0.0, hdg_rad=0.0, speed_ms=0.0):
        self.n_m = float(n_m)
        self.e_m = float(e_m)
        self.hdg_rad = float(hdg_rad)
        self.speed_ms = float(speed_ms)

    def as_dict(self):
        """Return the plain-float dict form used in an algorithm snapshot."""
        return {
            "plane_n_m": self.n_m,
            "plane_e_m": self.e_m,
            "plane_hdg_rad": self.hdg_rad,
            "plane_speed_ms": self.speed_ms,
        }


class TargetState:
    """Target state at one instant, in local North/East metres."""

    __slots__ = ("n_m", "e_m", "vn_ms", "ve_ms")

    def __init__(self, n_m=0.0, e_m=0.0, vn_ms=0.0, ve_ms=0.0):
        self.n_m = float(n_m)
        self.e_m = float(e_m)
        self.vn_ms = float(vn_ms)
        self.ve_ms = float(ve_ms)

    def as_dict(self):
        """Return the plain-float dict form used in an algorithm snapshot."""
        return {
            "target_n_m": self.n_m,
            "target_e_m": self.e_m,
            "target_vn_ms": self.vn_ms,
            "target_ve_ms": self.ve_ms,
        }


class Harness:
    """Owns plane and target state and advances them by fixed time steps.

    The harness holds the only mutable state in the system. An algorithm
    receives an immutable snapshot and returns a guidance point plus its own
    accumulated state; it never reaches into the harness.
    """

    def __init__(
        self,
        plane,
        target,
        algorithm,
        dt_s=0.1,
        turn_radius_m=None,
        infeasible=False,
    ):
        """
        Args:
            plane: :class:`PlaneState` initial aircraft state.
            target: :class:`TargetState` initial target state.
            algorithm: A :class:`~interface.GeometricAlgorithm` instance.
            dt_s: Fixed time step, seconds. Defaults to 0.1 s to match the
                shipping controller's 100 ms loop (``control_cont.lua``).
            turn_radius_m: Radius used for the kinematic turn-rate limit,
                ``omega_max = speed / radius``. ``None`` leaves the aircraft
                able to turn instantly, which is not flyable and is only useful
                for isolating geometry from steering.
            infeasible: True when the configuration violated the bank limit and
                the run proceeded anyway. Stamped on every history sample so no
                output can be mistaken for a flyable trajectory.
        """
        self.plane = plane
        self.target = target
        self.algorithm = algorithm
        self.dt_s = float(dt_s)
        self.turn_radius_m = None if turn_radius_m is None else float(turn_radius_m)
        self.infeasible = bool(infeasible)
        self.t_s = 0.0
        self.history = []
        self.algorithm_state = {}
        #: Set when a run stopped early because the algorithm had no solution.
        self.stopped_reason = None

    def snapshot(self):
        """Build the read-only algorithm snapshot for the current instant.

        Returns:
            Plain dict with the keys in ``interface.SNAPSHOT_FIELDS``.
        """
        snap = {"t_s": self.t_s, "algorithm_state": dict(self.algorithm_state)}
        snap.update(self.plane.as_dict())
        snap.update(self.target.as_dict())
        return snap

    def max_turn_rate(self):
        """Turn-rate limit in rad/s, or ``None`` when unconstrained."""
        if self.turn_radius_m is None or self.turn_radius_m <= 0.0:
            return None
        return self.plane.speed_ms / self.turn_radius_m

    def step(self):
        """Advance one time step.

        Builds a snapshot, asks the algorithm for a guidance point, steers the
        aircraft toward it under its turn-rate limit, advances the target on
        its constant-velocity model, appends the instant to :attr:`history` and
        increments :attr:`t_s`.

        Raises:
            interface.NoSolution: Propagated unchanged. The harness records the
                failure and stops; it does not substitute a fallback point,
                because choosing a safe fallback is a controller decision
                governed by ``SR-007``.
        """
        snap = self.snapshot()
        result = self.algorithm.guidance_point(snap)

        gn = result["guidance_n_m"]
        ge = result["guidance_e_m"]
        self.algorithm_state = dict(result.get("algorithm_state", {}))

        # Steer toward the guidance point, limited by the turn rate.
        desired_hdg = math.atan2(ge - self.plane.e_m, gn - self.plane.n_m)
        error = wrap_pi(desired_hdg - self.plane.hdg_rad)
        rate_limit = self.max_turn_rate()
        if rate_limit is not None:
            max_step = rate_limit * self.dt_s
            error = max(-max_step, min(max_step, error))
        self.plane.hdg_rad = wrap_pi(self.plane.hdg_rad + error)

        # Advance the aircraft along its new heading at constant speed.
        d = self.plane.speed_ms * self.dt_s
        self.plane.n_m += d * math.cos(self.plane.hdg_rad)
        self.plane.e_m += d * math.sin(self.plane.hdg_rad)

        # Advance the target on its constant-velocity model.
        self.target.n_m += self.target.vn_ms * self.dt_s
        self.target.e_m += self.target.ve_ms * self.dt_s

        self.t_s += self.dt_s

        self.history.append(
            {
                "t_s": self.t_s,
                "plane_n_m": self.plane.n_m,
                "plane_e_m": self.plane.e_m,
                "plane_hdg_rad": self.plane.hdg_rad,
                "target_n_m": self.target.n_m,
                "target_e_m": self.target.e_m,
                "guidance_n_m": gn,
                "guidance_e_m": ge,
                "infeasible": self.infeasible,
            }
        )

    def run(self, duration_s):
        """Step until ``duration_s`` elapses or the algorithm has no solution.

        A :class:`~interface.NoSolution` stops the run and is recorded in
        :attr:`stopped_reason` rather than propagating. This is the harness
        "records the failure and stops" behaviour in the interface contract: it
        does not substitute a fallback point, and it does not pretend the run
        completed. A caller wanting the exception can drive :meth:`step`
        directly.
        """
        if self.dt_s <= 0.0:
            raise ValueError("dt_s must be positive, got %r" % self.dt_s)
        n_steps = int(round(float(duration_s) / self.dt_s))
        for _ in range(n_steps):
            try:
                self.step()
            except NoSolution as exc:
                self.stopped_reason = str(exc)
                break
        return self.history

    def achieved_orbit_radius_m(self, fraction=0.25):
        """Mean aircraft-to-target range over the last ``fraction`` of the run.

        The harness's own measurement of what the aircraft actually flew, as
        opposed to what was commanded. Reported because a guidance point placed
        ahead on a ring is chased from inside it, so achieved radius is smaller
        than commanded radius — the same commanded-versus-achieved gap that
        ``A-DEC-009`` records for the weave's amplitude.

        Returns ``None`` for an empty history.
        """
        if not self.history:
            return None
        n = max(1, int(len(self.history) * fraction))
        tail = self.history[-n:]
        total = 0.0
        for s in tail:
            total += math.hypot(
                s["plane_n_m"] - s["target_n_m"], s["plane_e_m"] - s["target_e_m"]
            )
        return total / len(tail)


def plot_history(history, axes=None, title=None, orbit_radius_m=None):
    """Draw a recorded state history.

    This function is **read-only by contract** (``DEC-2026-06-25-03``). It does
    not mutate ``history``, does not construct or call a
    :class:`~interface.GeometricAlgorithm`, and does not advance a
    :class:`Harness`. It takes a completed history and draws it.

    ``matplotlib`` is imported here rather than at module scope so the harness
    and its tests import cleanly in a headless environment.

    Args:
        history: The list produced by :attr:`Harness.history`.
        axes: Optional matplotlib axes to draw into. A new figure is created
            when omitted.
        title: Optional title. An infeasible run overrides it with a warning.
        orbit_radius_m: When given, draw the ring about the final target
            position for reference.

    Returns:
        The axes drawn into.

    Raises:
        ValueError: If ``history`` is empty.
    """
    if not history:
        raise ValueError("plot_history needs a non-empty history")

    import matplotlib.pyplot as plt

    if axes is None:
        _fig, axes = plt.subplots(figsize=(7, 7))

    plane_e = [s["plane_e_m"] for s in history]
    plane_n = [s["plane_n_m"] for s in history]
    target_e = [s["target_e_m"] for s in history]
    target_n = [s["target_n_m"] for s in history]
    guide_e = [s["guidance_e_m"] for s in history]
    guide_n = [s["guidance_n_m"] for s in history]

    axes.plot(plane_e, plane_n, color="tab:blue", linewidth=2, label="aircraft")
    axes.plot(
        guide_e, guide_n, color="tab:orange", linewidth=0.8, alpha=0.6,
        label="guidance point",
    )
    axes.plot(target_e, target_n, color="tab:green", linewidth=2, label="target")
    axes.plot(plane_e[0], plane_n[0], "o", color="tab:blue", label="start")
    axes.plot(target_e[-1], target_n[-1], "x", color="tab:green", markersize=10)

    if orbit_radius_m:
        ring_e, ring_n = [], []
        for i in range(101):
            a = 2.0 * math.pi * i / 100.0
            ring_e.append(target_e[-1] + orbit_radius_m * math.sin(a))
            ring_n.append(target_n[-1] + orbit_radius_m * math.cos(a))
        axes.plot(ring_e, ring_n, "--", color="grey", linewidth=1, label="orbit ring")

    if history[0].get("infeasible"):
        axes.set_title(
            "INFEASIBLE CONFIGURATION — violates the bank limit; not a flyable path"
        )
    elif title:
        axes.set_title(title)

    axes.set_xlabel("East (m)")
    axes.set_ylabel("North (m)")
    axes.set_aspect("equal")
    axes.grid(True)
    axes.legend(loc="best", fontsize="small")
    return axes
