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
from . import plotter
from . import scenario
from .config import HarnessConfig
from .kangaroo import MODES


#: Sim ticks advanced per rendered frame. Decouples sim time from wall-clock.
DEFAULT_TICKS_PER_FRAME = 2

#: Milliseconds between frames.
DEFAULT_INTERVAL_MS = 50


class ScenarioView:
    """Matplotlib view and controls over a :class:`~scenario.ScenarioSession`.

    Holds no simulation state of its own — every change is delegated to the
    session, which enforces the continuity invariants. The view only reads
    ``session.history`` to draw.
    """

    def __init__(self, session, ticks_per_frame=DEFAULT_TICKS_PER_FRAME,
                 interval_ms=DEFAULT_INTERVAL_MS, export_path=None):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button, RadioButtons, Slider

        self.session = session
        self.ticks_per_frame = int(ticks_per_frame)
        self.interval_ms = int(interval_ms)
        self.export_path = export_path
        self.paused = False
        self._plt = plt

        leg = session.current_leg()
        self._mode, self._heading, self._speed = leg[1], leg[2], leg[3]

        self.fig = plt.figure(figsize=(13.0, 8.0))
        self.fig.canvas.manager.set_window_title(
            "py_harness scenario driver — %s" % session.algorithm_name)
        self.ax = self.fig.add_axes([0.30, 0.08, 0.66, 0.84])
        self.ax.set_aspect("equal")
        self.ax.grid(color="#E3E9ED", linewidth=0.8)
        self.ax.set_axisbelow(True)
        self.ax.set_xlabel("East (m)")
        self.ax.set_ylabel("North (m)")

        # ---- controls, all live -------------------------------------------
        self.ax_mode = plt.axes([0.03, 0.66, 0.20, 0.24])
        self.ax_mode.set_title("kangaroo mode", fontsize=10)
        self.radio_mode = RadioButtons(self.ax_mode, list(MODES),
                                       active=list(MODES).index(self._mode)
                                       if self._mode in MODES else 0)
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

        self.status = self.fig.text(
            0.03, 0.30, "", fontsize=9, va="top", family="monospace",
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
        plotter.draw_scene(self.ax, self.session.history,
                           orbit_radius_m=self.session.config.orbit_radius_m,
                           markers=self.session.markers)
        self.ax.set_aspect("equal")
        self.ax.legend(loc="upper right", fontsize=8, frameon=False)
        self.status.set_text(self._status_text())

    def _status_text(self):
        hist = self.session.history
        err = float("nan")
        if hist:
            cur = hist[-1]
            err = abs(math.hypot(cur["plane_n_m"] - cur["target_n_m"],
                                 cur["plane_e_m"] - cur["target_e_m"])
                      - self.session.config.orbit_radius_m)
        return (
            "t          %8.1f s\n"
            "mode       %8s\n"
            "heading    %8.0f deg\n"
            "speed      %8.1f m/s\n"
            "ring error %8.2f m\n"
            "changes    %8d\n"
            "%s"
            % (self.session.t_s, self._mode, self._heading % 360.0, self._speed,
               err, len(self.session.change_log),
               self.session.stopped_reason or "")
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
                   fps=20, figsize=(9.0, 8.0)):
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
                           markers=session.markers, upto=upto)
        ax.set_aspect("equal")
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)

    es = [s["plane_e_m"] for s in hist] + [s["target_e_m"] for s in hist]
    ns = [s["plane_n_m"] for s in hist] + [s["target_n_m"] for s in hist]
    pad = 1.15
    cx, cy = (min(es) + max(es)) / 2.0, (min(ns) + max(ns)) / 2.0
    half = max(max(es) - min(es), max(ns) - min(ns)) / 2.0 * pad + R
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


def build_session(args):
    """A :class:`~scenario.ScenarioSession` from parsed CLI arguments.

    This is ``TASK-013``'s front-door role: run parameters chosen here rather than
    in a separate configuration GUI.
    """
    overrides = {}
    for name in ("airspeed_ms", "turn_radius_m", "orbit_radius_m",
                 "look_ahead_m", "dt_s"):
        value = getattr(args, name, None)
        if value is not None:
            overrides[name] = value
    cfg = HarnessConfig(**overrides)
    legs = [(3600.0, args.mode, args.heading_deg, args.speed_ms)]
    return scenario.ScenarioSession(
        legs=legs, algorithm=args.algorithm, config=cfg,
        start_range_m=args.start_range_m,
        plane_heading_deg=args.plane_heading_deg)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Interactive scenario driver: change the kangaroo's mode, "
        "heading and speed while the run continues (TASK-029). Also the "
        "harness's graphical front door (TASK-008, TASK-013).")
    parser.add_argument("--algorithm", default=scenario.DEFAULT_ALGORITHM,
                        choices=sorted(algorithms.REGISTRY),
                        help="Guidance algorithm (default: %(default)s).")
    parser.add_argument("--mode", default="straight", choices=list(MODES),
                        help="Initial kangaroo mode (default: %(default)s).")
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
    parser.add_argument("--ticks-per-frame", type=int,
                        default=DEFAULT_TICKS_PER_FRAME,
                        help="Sim ticks advanced per rendered frame.")
    parser.add_argument("--interval-ms", type=int, default=DEFAULT_INTERVAL_MS,
                        help="Milliseconds between frames.")
    parser.add_argument("--export", dest="export_path", default=None,
                        help="Path the Export button writes the schedule to.")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    session = build_session(args)
    view = ScenarioView(session, ticks_per_frame=args.ticks_per_frame,
                        interval_ms=args.interval_ms,
                        export_path=args.export_path)
    view.show()
    if args.export_path:
        session.export(args.export_path)
        print("exported schedule -> %s" % args.export_path)
    return session


if __name__ == "__main__":
    main()
