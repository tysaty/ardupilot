"""Plane and target state, time stepping and recorded history.

This is the state module of the two-module harness required by
``DEC-2026-06-25-01``. It owns:

1. the state of the plane and the target at a given time; and
2. advancing that state by one fixed time step, recording each instant.

It owns no geometry. It calls a :class:`~interface.GeometricAlgorithm` to
obtain each guidance point and knows nothing about how that point is derived.

Plotter split out, 2026-07-22 (``DEC-2026-07-22-01``)
-----------------------------------------------------
``DEC-2026-06-25-03`` placed the plotter here, and was recorded as "Agreed,
with a recorded tension" against ``ARCHITECTURE.md``'s requirement that
component responsibilities stay explicit and independently testable. The
meeting note said to revisit that rather than quietly relax it. It was
revisited by direction on 2026-07-22 and the plotter now lives in
:mod:`py_harness.plotter`.

The read-only mitigation is **strengthened, not relaxed**: that module does not
import this one, so it cannot mutate harness state or call an algorithm. It
consumes recorded histories, in memory or from JSON. Producing a run and
drawing it are now separate steps.

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
        kangaroo=None,
        estimator=None,
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
            kangaroo: Optional ``kangaroo(t) -> (n, e, vn, ve)`` target-motion
                model (``TASK-009``). When given, the target follows the mode at
                each step; when ``None``, it advances on its constant velocity.
                The algorithm is unaffected either way — it reads the target from
                the snapshot.
            estimator: Optional state estimator with ``update(meas_x, meas_y, dt)``
                (``TASK-012``). When given, the harness feeds it the target
                position each step and exposes its estimate as the snapshot's
                ``target_est``; this is a state-side concern, so the estimator is
                never held by the algorithm.
        """
        self.plane = plane
        self.target = target
        self.algorithm = algorithm
        self.dt_s = float(dt_s)
        self.turn_radius_m = None if turn_radius_m is None else float(turn_radius_m)
        self.infeasible = bool(infeasible)
        self.kangaroo = kangaroo
        self.estimator = estimator
        #: Latest {n_m, e_m, vn_ms, ve_ms} estimate, or None.
        self.target_est = None
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
        snap = {"t_s": self.t_s, "algorithm_state": dict(self.algorithm_state),
                "target_est": dict(self.target_est) if self.target_est else None}
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
        # State-side estimation (TASK-012): feed the estimator the current target
        # position and expose its estimate in the snapshot. The algorithm consumes
        # it via target_est; it never holds the estimator.
        if self.estimator is not None:
            out = self.estimator.update(self.target.n_m, self.target.e_m, self.dt_s)
            if out is not None:
                self.target_est = {"n_m": out["x"], "e_m": out["y"],
                                   "vn_ms": out["vx"], "ve_ms": out["vy"]}

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

        # Advance the target: a kangaroo mode model if given (TASK-009),
        # otherwise its constant-velocity model.
        if self.kangaroo is not None:
            n, e, vn, ve = self.kangaroo(self.t_s + self.dt_s)
            self.target.n_m, self.target.e_m = n, e
            self.target.vn_ms, self.target.ve_ms = vn, ve
        else:
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
