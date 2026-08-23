"""Scenario session and the one configurable view (``TASK-029``).

A **scenario** is a run in which the kangaroo's mode, heading and speed change
during flight — declared up front as a schedule, or changed by hand through the
interactive driver (:mod:`py_harness.scenario_driver`). Either way the run
**continues**: it is never restarted, and the recorded history is one unbroken
record.

Two layers, deliberately separated:

* :class:`ScenarioSession` — the logic. Owns a live :class:`~state.Harness`, edits
  the remaining schedule, logs every change and exports it. **No matplotlib**, so
  it is unit-testable headlessly and usable from a script.
* :mod:`py_harness.scenario_driver` — the view. Matplotlib widgets over a session.

The continuity invariants (`TASK-029`), all enforced here rather than in the view:

1. The run is never restarted; the aircraft carries its state through a change.
2. Target position is continuous; only its velocity steps.
3. History is one unbroken record, ``t_s`` strictly increasing.
4. A change edits only the **remaining** schedule, anchored at the current time
   and the target's current position. Elapsed legs are immutable.
5. Estimator and weave phase carry through — nothing is reset.
6. Metrics are computed the same way for scripted and interactive changes.

Reproducibility: an interactive session exports an ordinary leg list, which
replayed headlessly reproduces the same history. A hand-driven exploration and a
scripted run therefore produce identical artefacts.

Absorbs the intent of ``TASK-008`` (graphical target relocation — see
:meth:`ScenarioSession.relocate_target`) and the deferred graphical front door of
``TASK-013``: this module and its driver are that front door, rather than a third
interactive entry point.
"""

import json
import math

from . import algorithms
from . import kangaroo as kang
from . import metrics
from .config import HarnessConfig
from .interface import NoSolution
from .state import Harness, PlaneState, TargetState


#: Default schedule for the reference scenario — one leg per change type, with a
#: long first leg so the aircraft reaches the ring before anything changes.
#: ``(duration_s, mode, heading_deg, speed_ms)``.
DEFAULT_LEGS = [
    (60.0, "straight", 0.0, 5.0),     # settle onto the ring
    (60.0, "straight", 90.0, 12.0),   # heading AND speed change
    (60.0, "circle", 90.0, 8.0),      # mode change
    (60.0, "point", 0.0, 0.0),        # stops dead
]

#: Default algorithm for a scenario. Any registry name works.
DEFAULT_ALGORITHM = "dubins_target_orbit"


class ScenarioSession:
    """A live run whose kangaroo schedule can be edited while it flies.

    The harness and the algorithm are untouched by this class: it only supplies
    the ``kangaroo(t)`` callable the harness already accepts, and replaces it when
    the schedule changes (``VR-014``).
    """

    def __init__(self, legs=None, algorithm=DEFAULT_ALGORITHM, config=None,
                 start_range_m=300.0, plane_heading_deg=0.0,
                 target_n_m=None, target_e_m=None, radius_m=150.0,
                 length_m=300.0, width_m=150.0):
        """
        Args:
            legs: Initial schedule, ``[(duration_s, mode, heading_deg, speed_ms)]``.
                Defaults to :data:`DEFAULT_LEGS`.
            algorithm: Registry name. Not modified by anything here.
            config: :class:`~config.HarnessConfig`; a default is built if omitted.
            start_range_m: Initial aircraft-to-target range, m (target due North).
            plane_heading_deg: Initial aircraft heading, degrees from North.
            target_n_m, target_e_m: Explicit target placement, overriding
                ``start_range_m``.
            radius_m, length_m, width_m: Circle/rectangle mode parameters.
        """
        self.config = config or HarnessConfig()
        self.algorithm_name = algorithm
        self.radius_m = float(radius_m)
        self.length_m = float(length_m)
        self.width_m = float(width_m)

        tn = float(start_range_m) if target_n_m is None else float(target_n_m)
        te = 0.0 if target_e_m is None else float(target_e_m)

        self.legs = [tuple(leg) for leg in (legs if legs is not None
                                            else DEFAULT_LEGS)]
        #: Every applied change, as ``{t_s, duration_s, mode, heading_deg,
        #: speed_ms, source}``. This is what makes an interactive run reproducible.
        self.change_log = []
        #: Absolute times at which a change was applied, for the plot markers.
        self.markers = []

        self.harness = Harness(
            plane=PlaneState(n_m=0.0, e_m=0.0,
                             hdg_rad=math.radians(plane_heading_deg),
                             speed_ms=self.config.airspeed_ms),
            target=TargetState(n_m=tn, e_m=te),
            algorithm=algorithms.build(algorithm,
                                       algorithms.config_dict(self.config)),
            dt_s=self.config.dt_s,
            turn_radius_m=self.config.turn_radius_m,
            kangaroo=kang.build_scripted(
                self.legs, start_n=tn, start_e=te, radius_m=self.radius_m,
                length_m=self.length_m, width_m=self.width_m),
        )
        self.stopped_reason = None

    # ------------------------------------------------------------------
    # Running
    # ------------------------------------------------------------------

    @property
    def t_s(self):
        """Elapsed simulated seconds."""
        return self.harness.t_s

    @property
    def history(self):
        """The single unbroken history. Never reset by a change."""
        return self.harness.history

    def step(self, n=1):
        """Advance ``n`` ticks. Returns True while the run is still solvable.

        A :class:`NoSolution` is recorded and stops the session, matching
        ``Harness.run``'s behaviour rather than propagating into a GUI callback.
        """
        for _ in range(int(n)):
            if self.stopped_reason:
                return False
            try:
                self.harness.step()
            except NoSolution as exc:
                self.stopped_reason = str(exc)
                return False
        return True

    def run(self, duration_s):
        """Advance ``duration_s`` seconds, or until the run stops."""
        self.step(int(round(float(duration_s) / self.config.dt_s)))
        return self.history

    # ------------------------------------------------------------------
    # Changing the schedule mid-run — the continuity-critical part
    # ------------------------------------------------------------------

    def target_position(self):
        """The target's current ``(n_m, e_m)``."""
        return self.harness.target.n_m, self.harness.target.e_m

    def apply_change(self, mode=None, heading_deg=None, speed_ms=None,
                     duration_s=3600.0, source="gui"):
        """Re-command the target from **now**, continuing from where it is.

        Any of ``mode``/``heading_deg``/``speed_ms`` may be given; those omitted
        keep the value currently in force. The new leg is anchored at the current
        time and the target's **current position**, so the target turns or changes
        pace from where it actually is and never jumps (invariants 2 and 4).

        Elapsed legs are not altered — only the future is rewritten. The harness,
        the algorithm, the history, the estimator and the weave phase are all
        untouched, so the run continues (invariants 1, 3 and 5).

        Returns the applied ``(duration_s, mode, heading_deg, speed_ms)`` leg.

        Raises:
            ValueError: From :func:`kangaroo.make_segments` for an unknown mode,
                a non-positive duration or a negative speed.
        """
        current = self.current_leg()
        mode = current[1] if mode is None else str(mode).lower()
        heading_deg = current[2] if heading_deg is None else float(heading_deg)
        speed_ms = current[3] if speed_ms is None else float(speed_ms)
        leg = (float(duration_s), mode, heading_deg, speed_ms)

        tn, te = self.target_position()
        t_now = self.t_s
        # Anchor at the next tick: the current tick's position is already recorded,
        # and the new leg governs from the following step onward.
        self.harness.kangaroo = kang.build_scripted(
            [leg], start_n=tn, start_e=te, t0=t_now,
            radius_m=self.radius_m, length_m=self.length_m, width_m=self.width_m)

        self.change_log.append({
            "t_s": t_now, "duration_s": leg[0], "mode": leg[1],
            "heading_deg": leg[2], "speed_ms": leg[3], "source": source,
        })
        self.markers.append(t_now)
        return leg

    def relocate_target(self, n_m, e_m, source="gui"):
        """Move the target to ``(n_m, e_m)``, keeping its current motion.

        This is ``TASK-008``'s graphical relocation, absorbed here. It is the one
        change that **does** move the target discontinuously — that is its purpose
        — so it is logged distinctly from a manoeuvre and the continuity invariant
        on position is deliberately not claimed for it.
        """
        self.harness.target.n_m = float(n_m)
        self.harness.target.e_m = float(e_m)
        leg = self.current_leg()
        self.harness.kangaroo = kang.build_scripted(
            [leg], start_n=float(n_m), start_e=float(e_m), t0=self.t_s,
            radius_m=self.radius_m, length_m=self.length_m, width_m=self.width_m)
        self.change_log.append({
            "t_s": self.t_s, "duration_s": leg[0], "mode": leg[1],
            "heading_deg": leg[2], "speed_ms": leg[3], "source": source,
            "relocate_n_m": float(n_m), "relocate_e_m": float(e_m),
        })
        self.markers.append(self.t_s)
        return n_m, e_m

    def current_leg(self):
        """The leg in force now: the last applied change, else the schedule's."""
        if self.change_log:
            c = self.change_log[-1]
            return (c["duration_s"], c["mode"], c["heading_deg"], c["speed_ms"])
        t, acc = self.t_s, 0.0
        for leg in self.legs:
            acc += leg[0]
            if t < acc:
                return tuple(leg)
        return tuple(self.legs[-1])

    # ------------------------------------------------------------------
    # Export and replay
    # ------------------------------------------------------------------

    def export_legs(self):
        """The schedule that reproduces this session, as an ordinary leg list.

        Interactive changes are converted back into durations, so a hand-driven
        session becomes a script. Replaying it headlessly reproduces the history —
        which is what makes an interactive result evidence rather than a
        demonstration.
        """
        if not self.change_log:
            return [tuple(leg) for leg in self.legs]

        legs, prev_t = [], 0.0
        # Legs from the original schedule that elapsed before the first change.
        first_change = self.change_log[0]["t_s"]
        acc = 0.0
        for leg in self.legs:
            if acc >= first_change:
                break
            dur = min(leg[0], first_change - acc)
            if dur > 0:
                legs.append((dur, leg[1], leg[2], leg[3]))
            acc += leg[0]
        prev_t = first_change

        for i, c in enumerate(self.change_log):
            nxt = (self.change_log[i + 1]["t_s"]
                   if i + 1 < len(self.change_log) else self.t_s)
            dur = nxt - c["t_s"]
            if dur <= 0.0:
                continue    # superseded before it ran; drop it
            legs.append((dur, c["mode"], c["heading_deg"], c["speed_ms"]))
            prev_t = nxt
        return legs

    def export(self, path):
        """Write the session's schedule and setup as JSON, for headless replay."""
        data = {
            "algorithm": self.algorithm_name,
            "legs": [list(leg) for leg in self.export_legs()],
            "change_log": self.change_log,
            "markers": self.markers,
            "config": {"dt_s": self.config.dt_s,
                       "orbit_radius_m": self.config.orbit_radius_m,
                       "turn_radius_m": self.config.turn_radius_m,
                       "airspeed_ms": self.config.airspeed_ms,
                       "look_ahead_m": self.config.look_ahead_m},
            "duration_s": self.t_s,
        }
        with open(path, "w") as handle:
            json.dump(data, handle, indent=2)
        return path

    # ------------------------------------------------------------------
    # Measurement
    # ------------------------------------------------------------------

    def reconvergence(self, tolerance_m=2.0, settle_s=None):
        """Per-change response: peak ring error and time to return within tolerance.

        For each applied change, scans forward from it and reports the worst ring
        error reached and how long until the error falls back below
        ``tolerance_m`` and stays there for ``settle_s``.

        Returns a list of ``{t_s, mode, heading_deg, speed_ms, peak_error_m,
        settle_s}``; ``settle_s`` is ``None`` when it never re-converged before the
        run ended, which is itself a result.
        """
        R = self.config.orbit_radius_m
        dt = self.config.dt_s
        settle_s = 5.0 if settle_s is None else float(settle_s)
        need = max(1, int(round(settle_s / dt)))
        hist = self.history
        if not hist:
            return []

        def err(sample):
            return abs(math.hypot(sample["plane_n_m"] - sample["target_n_m"],
                                  sample["plane_e_m"] - sample["target_e_m"]) - R)

        out = []
        for c in self.change_log:
            start = None
            for i, s in enumerate(hist):
                if s["t_s"] >= c["t_s"]:
                    start = i
                    break
            if start is None:
                continue
            peak, settled_at, run_len = 0.0, None, 0
            for i in range(start, len(hist)):
                e = err(hist[i])
                peak = max(peak, e)
                if e <= tolerance_m:
                    run_len += 1
                    if run_len >= need and settled_at is None:
                        settled_at = hist[i]["t_s"] - (need - 1) * dt
                else:
                    run_len = 0
            out.append({
                "t_s": c["t_s"], "mode": c["mode"],
                "heading_deg": c["heading_deg"], "speed_ms": c["speed_ms"],
                "peak_error_m": peak,
                # Clamp: the settle window is backdated to its start, which can
                # land marginally before the change itself when the error never
                # left tolerance (a change with no visible effect).
                "settle_s": max(0.0, settled_at - c["t_s"])
                            if settled_at is not None else None,
            })
        return out

    def steady_state(self):
        """Steady-state ring statistics for the run (``TASK-027``)."""
        return metrics.steady_state_stats(self.history,
                                          self.config.orbit_radius_m)


def replay(path_or_data, duration_s=None):
    """Rebuild and re-run a session from an exported schedule.

    The reproducibility check: an interactive session's export, replayed, must
    give the same history it produced live.
    """
    if isinstance(path_or_data, str):
        with open(path_or_data) as handle:
            data = json.load(handle)
    else:
        data = path_or_data
    cfg = HarnessConfig(**data.get("config", {}))
    session = ScenarioSession(legs=[tuple(leg) for leg in data["legs"]],
                              algorithm=data.get("algorithm", DEFAULT_ALGORITHM),
                              config=cfg)
    session.run(duration_s if duration_s is not None else data["duration_s"])
    return session
