"""Interactive scenario driver — the harness's graphical front door (``TASK-029``).

A live animation of a running :class:`~scenario.ScenarioSession` with controls for
the kangaroo's **mode, heading and speed**, changeable *while it flies*. The run
continues through every change: it is never restarted, and the recorded history
stays one unbroken record.

This module is **the** interactive entry point, absorbing:

* ``TASK-029`` — mode/heading/speed changed on the fly, animated;
* ``TASK-008`` — graphical target relocation (click on the plot to move the
  kangaroo), which was deferred pending exactly this driver;
* ``TASK-013``'s deferred **graphical front door** — algorithm and run parameters
  are chosen here rather than in a separate GUI, so there is one interactive
  surface rather than three.

Architecture (``DEC-2026-07-22-01``)
------------------------------------
The plotter stays **read-only**: it imports no harness module and only draws data
handed to it (:func:`plotter.draw_scene`). The controls live *here*, in the driver,
which owns the live harness. That decision was taken twice — once when the
``py_plots`` slider pattern was rejected, and again in ``TASK-008`` — and this
module upholds it rather than reopening it.

The same :func:`plotter.draw_scene` renders the live view and the offline
animation, so what is seen interactively and what an exported schedule reproduces
cannot drift apart.

Sim time is decoupled from wall-clock: ``--speed`` sets ticks per frame, and
pausing stops sim time entirely. Results therefore do not depend on machine speed
and match the headless replay.

Usage::

    python3 -m py_harness.scenario_driver
    python3 -m py_harness.scenario_driver --algorithm dubins_orbit --dt-s 0.1
    python3 -m py_harness.scenario_driver --export session.json
"""

import argparse
import math

from . import algorithms
from . import metrics
from . import plotter
from . import scenario
from . import series as series_mod
from .config import HarnessConfig, HOLD_POLICIES
from . import zone as zone_mod
from .geodetic import DEFAULT_ORIGIN
from .kangaroo import LEG_MODES


#: Sim ticks advanced per rendered frame. Decouples sim time from wall-clock.
DEFAULT_TICKS_PER_FRAME = 2

#: Milliseconds between frames.
DEFAULT_INTERVAL_MS = 50

#: How many manoeuvre markers the LIVE view draws, most recent first.
#:
#: Each marker is a diamond plus a labelled annotation on the target's track, so
#: a session driven by hand — where every heading nudge and speed change adds
#: one — buries the scene under overlapping orange labels within a minute. The
#: live view is for watching; one marker answers "what did I just change?" and
#: the rest is clutter.
#:
#: The OFFLINE renderer and the experiment bundle are deliberately unaffected and
#: keep every marker: an archived plot is read once, carefully, and the full
#: change history is what makes a step in the error attributable.
LIVE_MARKERS_SHOWN = 1


class ScenarioView:
    """Matplotlib view and controls over a :class:`~scenario.ScenarioSession`.

    Holds no simulation state of its own — every change is delegated to the
    session, which enforces the continuity invariants. The view only reads
    ``session.history`` to draw.
    """

    def __init__(self, session, ticks_per_frame=DEFAULT_TICKS_PER_FRAME,
                 interval_ms=DEFAULT_INTERVAL_MS, export_path=None,
                 extent_m=None, origin=None, session_factory=None):
        """
        Args:
            session_factory: Optional ``callable(algorithm_name) ->
                ScenarioSession`` that rebuilds the run under a different
                guidance law. When given, the algorithm selector appears and
                choosing a name **restarts** the scenario with it. When omitted
                the selector is hidden — a caller that constructed a session by
                hand has not told the view how to construct another, and
                guessing would silently drop its configuration.
            (others): as before.
        """
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, RadioButtons, Slider

        self.session = session
        self.session_factory = session_factory
        self.ticks_per_frame = int(ticks_per_frame)
        self.interval_ms = int(interval_ms)
        self.export_path = export_path
        self.origin = origin
        self.paused = False
        # Fixed square view (TASK-029 U2). Sized from the inclusion zone when one
        # is set, so the bound and the frame agree; the axes never rescale during
        # a run, so distances stay comparable between frames.
        if extent_m is not None:
            self.extent_m = float(extent_m)
        elif session.zone is not None:
            self.extent_m = max(session.zone.half_e, session.zone.half_n) * 1.08
        else:
            self.extent_m = 1200.0
        zc = session.zone
        self.centre = ((zc.centre_e_m, zc.centre_n_m) if zc is not None
                       else (0.0, 0.0))
        self._plt = plt

        leg = session.current_leg()
        self._mode, self._heading, self._speed = leg[1], leg[2], leg[3]

        self.fig = plt.figure(figsize=(13.0, 8.6))
        self.fig.canvas.manager.set_window_title(
            "py_harness scenario driver — %s" % session.algorithm_name)
        # TASK-040 D3: the scene shrinks to make room for a live error graph,
        # which is drawn from the SAME `series` functions the post-run graphs
        # use, so the live curve and the archived one cannot disagree.
        self.ax = self.fig.add_axes([0.30, 0.33, 0.66, 0.60])
        self.ax.set_aspect("equal")
        self.ax.grid(color="#E3E9ED", linewidth=0.8)
        self.ax.set_axisbelow(True)
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")

        #: Live error graph. Ring error only: it is a hypot per sample and stays
        #: cheap to recompute every frame. The tangent-registration error is
        #: deliberately NOT live — it costs a Dubins solve per sample, so a live
        #: version would slow the animation in proportion to the run's length.
        #: `TASK-036` owns the fuller live graph (a horizon selector, withheld
        #: invalid samples); this is the shared-series half of it.
        self.ax_error = self.fig.add_axes([0.36, 0.07, 0.60, 0.19])
        self.ax_error.grid(color="#E3E9ED", linewidth=0.7)
        self.ax_error.set_axisbelow(True)

        # ---- algorithm selector -------------------------------------------
        # matplotlib has no combo box, so this is one: a button showing the
        # current algorithm, which toggles an overlaid list. The list sits on top
        # of the other controls while open and hides again on selection, so it
        # costs no permanent panel space -- fourteen algorithms laid out
        # permanently would fill the column.
        self._algo_open = False
        self.ax_algo_btn = plt.axes([0.03, 0.928, 0.20, 0.040])
        self.ax_algo_btn.set_title("guidance algorithm", fontsize=8.5,
                                   color="#6B7A85", pad=5)
        self.btn_algo = Button(self.ax_algo_btn, self._algo_button_label())
        self.btn_algo.label.set_fontsize(8.5)
        self.btn_algo.on_clicked(self._on_algo_toggle)

        self._algo_names = sorted(algorithms.IMPLEMENTED)
        # The menu floats over the SCENE, not over the left control column.
        # Opening it in the column put fourteen radio circles exactly where the
        # kangaroo-mode radio's five had been, which reads as the mode control
        # breaking rather than as a menu opening. Over the plot it is
        # unmistakably a menu, and the mode radio stays visible beside it.
        self.ax_algo_list = plt.axes([0.325, 0.33, 0.215, 0.53], zorder=20,
                                     facecolor="#FFFFFF")
        for spine in self.ax_algo_list.spines.values():
            spine.set(edgecolor="#16222C", linewidth=1.2)
        # Kept clear of the footer, which occupies the top ~6% of the figure.
        self.ax_algo_list.set_title("click a name — restarts the run",
                                    fontsize=8.5, color="#16222C", pad=5)
        active = (self._algo_names.index(session.algorithm_name)
                  if session.algorithm_name in self._algo_names else 0)
        self.radio_algo = RadioButtons(self.ax_algo_list, self._algo_names,
                                       active=active)
        for label in self.radio_algo.labels:
            label.set_fontsize(8)
        self.radio_algo.on_clicked(self._on_algo_selected)
        self._set_algo_list_visible(False)
        if session_factory is None:
            # No way to rebuild, so no selector. Hidden rather than shown-and-
            # broken: a control that does nothing is worse than none.
            self.ax_algo_btn.set_visible(False)

        # ---- controls, all live -------------------------------------------
        self.ax_mode = plt.axes([0.03, 0.66, 0.20, 0.24])
        self.ax_mode.set_title("kangaroo mode", fontsize=10)
        # LEG_MODES, not MODES: `elastic` (TASK-030) is a scriptable leg mode and
        # TASK-039 compares every arm against a varying target speed, so it has to
        # be reachable from the live controls and not only from a script.
        self.radio_mode = RadioButtons(self.ax_mode, list(LEG_MODES),
                                       active=list(LEG_MODES).index(self._mode)
                                       if self._mode in LEG_MODES else 0)
        self.radio_mode.on_clicked(self._on_mode)

        self.ax_hdg = plt.axes([0.06, 0.58, 0.17, 0.03])
        self.slider_hdg = Slider(self.ax_hdg, "heading", 0.0, 360.0,
                                 valinit=self._heading % 360.0, valstep=1.0)
        self.slider_hdg.on_changed(self._on_heading)

        self.ax_spd = plt.axes([0.06, 0.52, 0.17, 0.03])
        self.slider_spd = Slider(self.ax_spd, "speed", 0.0,
                                 2.0 * session.config.airspeed_ms,
                                 valinit=self._speed, valstep=0.5)
        self.slider_spd.on_changed(self._on_speed)

        self.ax_pause = plt.axes([0.03, 0.43, 0.09, 0.045])
        self.btn_pause = Button(self.ax_pause, "Pause")
        self.btn_pause.on_clicked(self._on_pause)

        self.ax_step = plt.axes([0.14, 0.43, 0.09, 0.045])
        self.btn_step = Button(self.ax_step, "Step")
        self.btn_step.on_clicked(self._on_step)

        self.ax_export = plt.axes([0.03, 0.37, 0.20, 0.045])
        self.btn_export = Button(self.ax_export, "Export schedule")
        self.btn_export.on_clicked(self._on_export)

        # TASK-029 U1: tracks and SITL waypoints.
        self.ax_tracks = plt.axes([0.03, 0.31, 0.20, 0.045])
        self.btn_tracks = Button(self.ax_tracks, "Export tracks + waypoints")
        self.btn_tracks.on_clicked(self._on_export_all)

        self.footer = self.fig.text(
            0.30, 0.995, "", fontsize=8, family="monospace", color="#16222C",
            va="top", linespacing=1.5)
        self.status = self.fig.text(
            0.03, 0.25, "", fontsize=9, va="top", family="monospace",
            color="#16222C")
        self.fig.text(0.03, 0.06,
                      "Click the plot to move the kangaroo (TASK-008).\n"
                      "Changes take effect at the next tick; the run continues.",
                      fontsize=8, color="#6B7A85", va="bottom")

        # TASK-008: graphical relocation.
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

    # ------------------------------------------------------------------
    # Control callbacks — each delegates to the session, never mutates state
    # ------------------------------------------------------------------

    # -- the algorithm selector ----------------------------------------

    def _algo_button_label(self):
        return "%s  \u25be" % self.session.algorithm_name

    def _set_algo_list_visible(self, visible):
        self._algo_open = bool(visible)
        self.ax_algo_list.set_visible(self._algo_open)
        # `RadioButtons.circles` was replaced by a scatter collection in
        # matplotlib 3.7; hiding the axes covers both, and the labels are hidden
        # explicitly so nothing survives a redraw of the scene beneath.
        for label in self.radio_algo.labels:
            label.set_visible(self._algo_open)

    def _on_algo_toggle(self, _event):
        if self.session_factory is None:
            return
        self._set_algo_list_visible(not self._algo_open)
        self.fig.canvas.draw_idle()

    def _on_algo_selected(self, name):
        """Rebuild the run under `name` and restart it.

        A **restart**, not a hot-swap: the aircraft's pose, the estimator's
        covariance and the algorithm's accumulated state are all mid-run
        quantities, and carrying them across would compare the new law from a
        starting condition the old one produced. Restarting is what makes the
        two windows comparable, which is the whole reason to switch.
        """
        if self.session_factory is None or not self._algo_open:
            # RadioButtons still receive clicks while their axes are hidden, so
            # the open check is load-bearing, not defensive.
            return
        self._set_algo_list_visible(False)
        if name == self.session.algorithm_name:
            self.fig.canvas.draw_idle()
            return
        try:
            self.session = self.session_factory(name)
        except Exception as exc:                     # noqa: BLE001
            # A refused combination (a horizon an algorithm owns, an infeasible
            # radius) must not take the window down mid-demonstration.
            self.status.set_text("could not start %s:\n%s" % (name, exc))
            self.fig.canvas.draw_idle()
            return

        leg = self.session.current_leg()
        self._mode, self._heading, self._speed = leg[1], leg[2], leg[3]
        self.btn_algo.label.set_text(self._algo_button_label())
        self.fig.canvas.manager.set_window_title(
            "py_harness scenario driver — %s" % self.session.algorithm_name)
        self._draw()
        self.fig.canvas.draw_idle()

    # -- kangaroo controls ---------------------------------------------

    def _on_mode(self, label):
        self._mode = label
        self._apply()

    def _on_heading(self, value):
        self._heading = float(value)
        self._apply()

    def _on_speed(self, value):
        self._speed = float(value)
        self._apply()

    def _apply(self):
        self.session.apply_change(mode=self._mode, heading_deg=self._heading,
                                  speed_ms=self._speed, source="gui")

    def _on_pause(self, _event):
        self.paused = not self.paused
        self.btn_pause.label.set_text("Resume" if self.paused else "Pause")

    def _on_step(self, _event):
        self.paused = True
        self.btn_pause.label.set_text("Resume")
        self.session.step(1)
        self._draw()

    def _on_export(self, _event):
        path = self.export_path or "scenario-session.json"
        self.session.export(path)
        print("exported schedule -> %s" % path)

    def _on_export_all(self, _event):
        """Tracks (CSV/JSON) and the kangaroo's change points for SITL."""
        from . import export as export_mod
        stem = (self.export_path or "scenario-session.json").rsplit(".", 1)[0]
        written = export_mod.export_all(self.session, stem, origin=self.origin)
        for kind, path in sorted(written.items()):
            print("exported %-14s -> %s" % (kind, path))

    def _on_click(self, event):
        """Relocate the kangaroo to the clicked point (``TASK-008``)."""
        if event.inaxes is not self.ax or event.xdata is None:
            return
        self.session.relocate_target(n_m=event.ydata, e_m=event.xdata,
                                     source="gui-click")

    # ------------------------------------------------------------------
    # Frame loop
    # ------------------------------------------------------------------

    def _draw(self):
        self.ax.clear()
        self.ax.grid(color="#E3E9ED", linewidth=0.8)
        self.ax.set_axisbelow(True)
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")
        zone = self.session.zone
        plotter.draw_scene(self.ax, self.session.history,
                           orbit_radius_m=self.session.config.orbit_radius_m,
                           markers=self._live_markers(),
                           zone_corners=zone.corners() if zone else None)
        self.ax.set_aspect("equal")
        # Fixed square window — set every frame so nothing can rescale it.
        cx, cy = self.centre
        self.ax.set_xlim(cx - self.extent_m, cx + self.extent_m)
        self.ax.set_ylim(cy - self.extent_m, cy + self.extent_m)
        if self.session.history:
            # `draw_scene` returns early on an empty history, so there are no
            # labelled artists and matplotlib warns. A restart from the
            # algorithm selector hits exactly that frame.
            self.ax.legend(loc="upper right", fontsize=8, frameon=False)
        self.status.set_text(self._status_text())
        self.footer.set_text(self._footer_text())
        self._draw_error()

    def _live_markers(self):
        """The manoeuvre markers the live view draws — the most recent only.

        See :data:`LIVE_MARKERS_SHOWN`. The offline renderer and the experiment
        bundle keep all of them.
        """
        return self.session.markers[-LIVE_MARKERS_SHOWN:]

    def _draw_error(self):
        """The live error graph, from the shared `series` extraction (TASK-040).

        Two lines when a prediction lead exists — error about the true kangaroo
        and about the centre the algorithm actually held — because quoting either
        alone misleads, in opposite directions (`TASK-033` D2).
        """
        self.ax_error.clear()
        self.ax_error.grid(color="#E3E9ED", linewidth=0.7)
        self.ax_error.set_axisbelow(True)
        history = self.session.history
        if not history:
            self.ax_error.set_ylabel("ring error (m)", fontsize=8)
            return
        radius = self.session.config.orbit_radius_m
        panels = [(series_mod.ring_error(history, radius, "target"), "#1F6FEB")]
        if self.session.estimate:
            panels.append(
                (series_mod.ring_error(history, radius, "ring"), "#C2571C"))
        for entry, colour in panels:
            data = series_mod.to_plot_data({entry.name: entry},
                                           self._live_markers())
            plotter.draw_series_panel(self.ax_error, data["panels"][0], colour,
                                      data["markers"])
        self.ax_error.set_ylabel("ring error (m)", fontsize=8)
        self.ax_error.set_xlabel("simulated time (s)", fontsize=8)
        if len(panels) == 2:
            # Explicit proxy handles: the panel also draws a zero rule and the
            # manoeuvre markers, and an automatic legend picks those up instead
            # of the two data lines.
            from matplotlib.lines import Line2D
            self.ax_error.legend(
                handles=[Line2D([], [], color=colour, linewidth=1.4)
                         for _entry, colour in panels],
                labels=["vs true target", "vs held centre"],
                fontsize=7, frameon=False, loc="upper right")

    def _status_text(self):
        """Live readout.

        ``TASK-037``: when the algorithm centres its ring on a *predicted*
        target, the error about the true kangaroo includes the designed
        prediction lead and is therefore not a tracking failure. Both are shown,
        labelled, for the same reason ``run_harness`` reports dual-centre
        figures (``TASK-033``) — quoting either alone misleads, in opposite
        directions.
        """
        hist = self.session.history
        r = self.session.config.orbit_radius_m
        err = float("nan")
        pred_line = ""
        if hist:
            cur = hist[-1]
            err = abs(math.hypot(cur["plane_n_m"] - cur["target_n_m"],
                                 cur["plane_e_m"] - cur["target_e_m"]) - r)
            # The centre the algorithm ACTUALLY HELD, via the same selector the
            # archived metrics and the error graph use (`metrics.centre_of`).
            # Reading `target_est_*` directly instead — the LIVE projected
            # estimate — is only equal to it when `n_replan == 1`; with a
            # commitment interval the held centre is frozen between replans
            # while the estimate keeps moving, and the two diverged by up to
            # 50 m on arm B at `--replan-every 5`. The label below says "held
            # centre", so it has to be the held centre.
            cn, ce = metrics.centre_of(cur, "ring")
            if cur.get("target_est_n_m") is not None:
                lead = math.hypot(cn - cur["target_n_m"], ce - cur["target_e_m"])
                if lead > 0.5:
                    pred_line = (
                        "  vs true   %8.2f m  (incl. %.0f m lead)\n"
                        "  vs ring   %8.2f m  (held centre)\n"
                        % (err,
                           lead,
                           abs(math.hypot(cur["plane_n_m"] - cn,
                                          cur["plane_e_m"] - ce) - r)))
        return (
            "t          %8.1f s\n"
            "mode       %8s\n"
            "heading    %8.0f deg\n"
            "speed      %8.1f m/s\n"
            "%s"
            "changes    %8d\n"
            "%s"
            % (self.session.t_s, self._mode, self._heading % 360.0, self._speed,
               pred_line or "ring error %8.2f m\n" % err,
               len(self.session.change_log),
               self.session.stopped_reason or "")
        )

    def _footer_text(self):
        """What is flying, and how (``TASK-029`` U3).

        ``dt_s`` is the **algorithm refresh rate** and has nothing to do with the
        animation's frame rate; conflating them is the likeliest misreading of
        this window, so both are named.
        """
        cfg = self.session.config
        hist = self.session.history
        phase = "-"
        if hist:
            st = hist[-1].get("algorithm_state") or {}
            phase = str(st.get("phase", st.get("direction", "-")))
        zone = self.session.zone
        zone_txt = ("zone %.0f m square" % zone.side_m) if zone else "unbounded"
        # TASK-037: with the prediction controls reachable here, the horizon and
        # commitment interval must be visible — an orbit about a predicted
        # centre is unreadable without knowing how far ahead it is aimed.
        pred_txt = ""
        if self.session.estimate:
            pred_txt = ("\nk_horizon %d ticks (%.2g s)   n_replan %d ticks "
                        "(%.3g Hz)   hold %s"
                        % (self.session.lookahead_steps,
                           self.session.lookahead_steps * cfg.dt_s,
                           cfg.replan_every, cfg.replan_rate_hz,
                           cfg.hold_policy))
        # TASK-039. An arm that DECIDES something has to show the decision, or
        # the window shows a trajectory with no account of why it was chosen.
        # Read from the algorithm's own reported state, so nothing here branches
        # on the algorithm's name (VR-014) — an arm that reports none of these
        # keys simply adds no line.
        st = (hist[-1].get("algorithm_state") or {}) if hist else {}
        if "k_horizon_steps" in st:
            candidates = cfg.ah_candidate_horizons
            pred_txt += (
                "\nselected k_horizon %d ticks (%.2g s)   %s   candidates "
                "%d..%d step %d (%d)   e_tan(pred) %s"
                % (st["k_horizon_steps"], st["k_horizon_steps"] * cfg.dt_s,
                   "CHOSEN" if st.get("horizon_selected") else "held (on ring)",
                   cfg.ah_k_min_steps, cfg.ah_k_max_steps, cfg.ah_k_step,
                   len(candidates),
                   "-" if st.get("e_tan_pred_m") is None
                   else "%+.1f m" % st["e_tan_pred_m"]))
        if "kappa1_1pm" in st:
            pred_txt += (
                "\nN %d ticks (%.2g s)   segment %d   grid %dx%d = %d "
                "candidates, %d rollout steps   command +%d tick(s), "
                "look-ahead UNUSED   kappa1 %+.5f 1/m (bound %.5f)   "
                "rollout RMS %.1f m"
                % (cfg.rh_horizon_steps, cfg.rh_horizon_s,
                   cfg.rh_segment_steps, cfg.rh_candidates,
                   cfg.rh_candidates_2,
                   cfg.rh_candidates * cfg.rh_candidates_2,
                   cfg.rh_rollout_steps, cfg.rh_command_steps,
                   st["kappa1_1pm"],
                   1.0 / cfg.turn_radius_m, st.get("rollout_rms_error_m", 0.0)))
        return (
            "algorithm  %s   |   phase %s   |   refresh %.3g Hz  (dt %.2f s)\n"
            "R %.0f m   Rmin %.0f m   look-ahead %.0f m   precomp %s   |   %s"
            "   |   view %.0f m square   |   animation %.0f fps (%gx sim, not "
            "the refresh rate)%s"
            % (self.session.algorithm_name, phase, 1.0 / cfg.dt_s, cfg.dt_s,
               cfg.orbit_radius_m, cfg.turn_radius_m, cfg.look_ahead_m,
               "on" if cfg.orbit_precompensate else "OFF", zone_txt,
               2 * self.extent_m, 1000.0 / self.interval_ms,
               self.ticks_per_frame * cfg.dt_s / (self.interval_ms / 1000.0),
               pred_txt)
        )

    def _tick(self, _frame):
        if not self.paused and not self.session.stopped_reason:
            self.session.step(self.ticks_per_frame)
        self._draw()
        return []

    def show(self):
        """Run the interactive view until the window is closed."""
        from matplotlib.animation import FuncAnimation
        self._anim = FuncAnimation(self.fig, self._tick,
                                   interval=self.interval_ms,
                                   blit=False, cache_frame_data=False)
        self._plt.show()
        return self.session


def render_session(session, save_path=None, animate_path=None, frames=120,
                   fps=20, figsize=(9.0, 8.0), extent_m=None):
    """Render a finished session offline, using the **same** drawing primitive.

    The live view and this share :func:`plotter.draw_scene`, so an exported
    schedule replayed here reproduces what was seen interactively rather than a
    second, subtly different picture (``TASK-029``).
    """
    import matplotlib
    if save_path or animate_path:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    hist = session.history
    R = session.config.orbit_radius_m

    def frame_axes(ax, upto):
        ax.clear()
        ax.grid(color="#E3E9ED", linewidth=0.8)
        ax.set_axisbelow(True)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        plotter.draw_scene(ax, hist, orbit_radius_m=R,
                           markers=session.markers, upto=upto,
                           zone_corners=zone.corners() if zone else None)
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)

    # Fixed square extent, matching the live view (TASK-029 U2): from the zone
    # when one is set, else fitted once to the whole run — never rescaling.
    zone = session.zone
    if extent_m is not None:
        half = float(extent_m)
        cx, cy = ((zone.centre_e_m, zone.centre_n_m) if zone else (0.0, 0.0))
    elif zone is not None:
        half = max(zone.half_e, zone.half_n) * 1.08
        cx, cy = zone.centre_e_m, zone.centre_n_m
    else:
        es = [s["plane_e_m"] for s in hist] + [s["target_e_m"] for s in hist]
        ns = [s["plane_n_m"] for s in hist] + [s["target_n_m"] for s in hist]
        cx, cy = (min(es) + max(es)) / 2.0, (min(ns) + max(ns)) / 2.0
        half = max(max(es) - min(es), max(ns) - min(ns)) / 2.0 * 1.15 + R
    xlim, ylim = (cx - half, cx + half), (cy - half, cy + half)

    out = {}
    if save_path:
        fig, ax = plt.subplots(figsize=figsize)
        frame_axes(ax, None)
        ax.legend(loc="upper right", fontsize=8, frameon=False)
        fig.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)
        out["png"] = save_path
    if animate_path:
        from matplotlib.animation import FuncAnimation, PillowWriter
        fig, ax = plt.subplots(figsize=figsize)
        step = max(1, len(hist) // int(frames))
        idx = list(range(1, len(hist) + 1, step))
        anim = FuncAnimation(fig, lambda i: frame_axes(ax, i), frames=idx,
                             blit=False, cache_frame_data=False)
        anim.save(animate_path, writer=PillowWriter(fps=int(fps)))
        plt.close(fig)
        out["gif"] = animate_path
    return out


def build_session(args, algorithm=None):
    """A :class:`~scenario.ScenarioSession` from parsed CLI arguments.

    This is ``TASK-013``'s front-door role: run parameters chosen here rather than
    in a separate configuration GUI.

    Args:
        args: Parsed CLI arguments.
        algorithm: Override ``args.algorithm``. Used by the view's algorithm
            selector to rebuild the same run under a different guidance law,
            so every other setting is carried across unchanged and only the
            algorithm differs — which is what makes the two runs comparable.
    """
    overrides = {}
    for name in ("airspeed_ms", "turn_radius_m", "orbit_radius_m",
                 "look_ahead_m", "dt_s", "replan_every", "hold_policy",
                 # TASK-039 arm B and arm C, so each approach is configurable
                 # from the same front door as the ones that came before it.
                 "ah_k_min_steps", "ah_k_max_steps", "ah_k_step",
                 "rh_horizon_steps", "rh_segment_steps", "rh_candidates",
                 "rh_candidates_2", "rh_command_steps"):
        value = getattr(args, name, None)
        if value is not None:
            overrides[name] = value
    cfg = HarnessConfig(**overrides)
    name = algorithm or args.algorithm
    # TASK-039: an algorithm that owns its own horizon projects the target from
    # the un-projected estimate, so a state-side --lookahead-steps would lead the
    # ring twice and the second lead would not appear anywhere on screen. Refuse
    # rather than silently pick one.
    lookahead_steps = getattr(args, "lookahead_steps", None) or 0
    owns_horizon = getattr(algorithms.REGISTRY[name], "owns_horizon", False)
    if owns_horizon and lookahead_steps:
        if algorithm is None:
            # Asked for on the command line: refuse, because the operator stated
            # two horizons and only one can be honoured.
            raise SystemExit(
                "%s selects its own prediction horizon, so --lookahead-steps "
                "must be 0 (it is the state-side horizon this algorithm "
                "replaces; setting both would lead the ring twice). Set the "
                "candidate range with --ah-k-min-steps / --ah-k-max-steps / "
                "--ah-k-step instead." % name)
        # Switched to from the selector: the horizon came from the PREVIOUS
        # algorithm's flags, not from a choice about this one, so drop it rather
        # than refusing a selection the operator plainly meant.
        lookahead_steps = 0
    legs = [(3600.0, args.mode, args.heading_deg, args.speed_ms)]
    if getattr(args, "no_zone", False):
        zone = False
    elif getattr(args, "zone_side_m", None) is not None:
        zone = zone_mod.InclusionZone(side_m=args.zone_side_m)
    else:
        zone = None                      # the 2 km square default
    return scenario.ScenarioSession(
        legs=legs, algorithm=name, config=cfg,
        start_range_m=args.start_range_m,
        plane_heading_deg=args.plane_heading_deg,
        zone=zone,
        containment_margin_m=getattr(args, "containment_margin_m", None),
        estimate=getattr(args, "estimate", False),
        lookahead_steps=lookahead_steps)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Interactive scenario driver: change the kangaroo's mode, "
        "heading and speed while the run continues (TASK-029). Also the "
        "harness's graphical front door (TASK-008, TASK-013).")
    parser.add_argument("--algorithm", default=scenario.DEFAULT_ALGORITHM,
                        choices=sorted(algorithms.REGISTRY),
                        help="Guidance algorithm (default: %(default)s).")
    parser.add_argument("--mode", default="straight", choices=list(LEG_MODES),
                        help="Initial kangaroo mode (default: %(default)s). "
                             "'elastic' surges and eases about the commanded "
                             "speed (TASK-030), which is the varying-speed "
                             "profile TASK-039 compares the arms against.")
    parser.add_argument("--heading-deg", type=float, default=0.0,
                        help="Initial kangaroo heading, deg from North.")
    parser.add_argument("--speed-ms", type=float, default=5.0,
                        help="Initial kangaroo speed, m/s.")
    parser.add_argument("--start-range-m", type=float, default=300.0,
                        help="Initial aircraft-to-target range, m.")
    parser.add_argument("--plane-heading-deg", type=float, default=0.0,
                        help="Initial aircraft heading, deg from North.")
    parser.add_argument("--airspeed-ms", type=float, default=None)
    parser.add_argument("--turn-radius-m", type=float, default=None)
    parser.add_argument("--orbit-radius-m", type=float, default=None)
    parser.add_argument("--look-ahead-m", type=float, default=None)
    parser.add_argument("--dt-s", type=float, default=None,
                        help="Algorithm refresh interval, s (default 0.1).")
    # TASK-037: the prediction-planning controls, so the TASK-033 algorithm is
    # reachable here and not only headless. Same names and units as run_harness.
    parser.add_argument("--estimate", action="store_true",
                        help="Run the state estimator (TASK-012) and feed the "
                             "algorithm its estimate. Forced on for an "
                             "algorithm that declares requires_estimate.")
    parser.add_argument("--lookahead-steps", type=int, default=None,
                        help="Prediction horizon k_horizon, whole control "
                             "ticks (TASK-017). Horizon in seconds is "
                             "k_horizon * dt_s. 0 (default) is the identity.")
    parser.add_argument("--replan-every", type=int, default=None,
                        help="Commitment interval n_replan, whole control "
                             "ticks (TASK-033). The committed plan is held "
                             "between replans. 1 (default) replans every tick.")
    parser.add_argument("--hold-policy", choices=HOLD_POLICIES, default=None,
                        help="What is held between replans (TASK-033): 'plan' "
                             "freezes the predicted centre and the committed "
                             "curve; 'centre_only' freezes the centre and "
                             "re-solves the curve each tick.")
    # TASK-039 arm B: the candidate horizon set the selector chooses from.
    parser.add_argument("--ah-k-min-steps", type=int, default=None,
                        help="adaptive_horizon_cs: smallest candidate horizon, "
                             "whole control ticks (default 0 — 'do not "
                             "predict' stays in the set).")
    parser.add_argument("--ah-k-max-steps", type=int, default=None,
                        help="adaptive_horizon_cs: largest candidate horizon, "
                             "whole control ticks (default 40 = 4.0 s).")
    parser.add_argument("--ah-k-step", type=int, default=None,
                        help="adaptive_horizon_cs: spacing between candidate "
                             "horizons, whole control ticks (default 5).")
    # TASK-039 arm C: the receding-horizon planner's own horizon and grid.
    parser.add_argument("--rh-horizon-steps", type=int, default=None,
                        help="rh_geometric: planning horizon N, whole control "
                             "ticks (default 45 = 4.5 s). Not the prediction "
                             "horizon and not the replan interval.")
    parser.add_argument("--rh-segment-steps", type=int, default=None,
                        help="rh_geometric: ticks the first curvature is held "
                             "before the second (default 20 = 2.0 s).")
    parser.add_argument("--rh-candidates", type=int, default=None,
                        help="rh_geometric: first-segment curvature samples "
                             "over [-1/Rmin, +1/Rmin] (default 15; odd keeps "
                             "straight flight on the grid).")
    parser.add_argument("--rh-candidates-2", type=int, default=None,
                        help="rh_geometric: second-segment curvature samples "
                             "(default 9).")
    parser.add_argument("--rh-command-steps", type=int, default=None,
                        help="rh_geometric: rollout steps ahead the emitted "
                             "guidance point is taken (default 1 — apply the "
                             "first command). Larger values are a chord across "
                             "the plan and saturate the turn-rate limit.")
    parser.add_argument("--ticks-per-frame", type=int,
                        default=DEFAULT_TICKS_PER_FRAME,
                        help="Sim ticks advanced per rendered frame.")
    parser.add_argument("--interval-ms", type=int, default=DEFAULT_INTERVAL_MS,
                        help="Milliseconds between frames.")
    parser.add_argument("--export", dest="export_path", default=None,
                        help="Path stem the Export buttons write to.")
    parser.add_argument("--extent-m", type=float, default=None,
                        help="Half-width of the fixed square view, m. Defaults "
                        "to the inclusion zone, so the frame matches the bound.")
    parser.add_argument("--zone-side-m", type=float, default=None,
                        help="Inclusion-zone side, m (TASK-032; default 2000 = a "
                        "2 km square). The kangaroo is turned back at it; the "
                        "aircraft is measured, never steered.")
    parser.add_argument("--no-zone", action="store_true",
                        help="Run unbounded, with no inclusion zone.")
    parser.add_argument("--containment-margin-m", type=float, default=None,
                        help="How far inside the boundary the kangaroo turns. "
                        "Defaults to the orbit radius, which keeps the aircraft "
                        "inside too.")
    parser.add_argument("--origin-lat", type=float, default=None,
                        help="Origin latitude for waypoint export. Default is a "
                        "deliberately FICTIONAL origin (AGENTS.md privacy).")
    parser.add_argument("--origin-lon", type=float, default=None,
                        help="Origin longitude for waypoint export.")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    session = build_session(args)
    origin = None
    if args.origin_lat is not None or args.origin_lon is not None:
        lat0, lon0, alt0 = DEFAULT_ORIGIN
        origin = (args.origin_lat if args.origin_lat is not None else lat0,
                  args.origin_lon if args.origin_lon is not None else lon0,
                  alt0)
    view = ScenarioView(session, ticks_per_frame=args.ticks_per_frame,
                        interval_ms=args.interval_ms,
                        export_path=args.export_path,
                        extent_m=args.extent_m, origin=origin,
                        session_factory=lambda name: build_session(args, name))
    view.show()
    if args.export_path:
        session.export(args.export_path)
        print("exported schedule -> %s" % args.export_path)
    return session


if __name__ == "__main__":
    main()
