"""Read-only plotter for recorded harness runs.

Split out of ``state.py`` on 2026-07-22 by direction. This **supersedes the
co-location half of** ``DEC-2026-06-25-03``, which placed the plotter in the
state module. That decision was recorded as "Agreed, with a recorded tension"
against ``ARCHITECTURE.md``'s requirement that component responsibilities stay
explicit and independently testable; the meeting note said to revisit it rather
than quietly relax it. This is the revisit. The **read-only mitigation is not
relaxed — it is strengthened.**

Structural guarantee
--------------------
This module does not import :mod:`py_harness.algorithms`, :mod:`py_harness.state`
or :mod:`py_harness.config`. It consumes recorded histories — in memory or from
JSON on disk — and draws them. It therefore *cannot* mutate harness state or
call an algorithm, which previously rested on a docstring and a source-grep
test. A run is produced by ``run_harness.py``; plotting it is a separate step.

Drawing conventions, inherited from ``../py_plots/``
---------------------------------------------------
* **``z`` is time** in the 3D view, with axes "East (x)", "North (y)",
  "time (z)" and ``box_aspect (1, 1, 0.6)`` — from ``combined.py``.
* A **2D top-down companion** figure with equal aspect and a grid — from
  ``combined.py``, ``cylinder_curve.py`` and ``constrained_curve.py``.
* **Fixed limits** computed once from a ``reach`` value so the view does not
  breathe between runs — from ``cylinder_curve.py``.
* **``CheckButtons`` toggles** in a left-hand panel titled "show", mapping each
  label to its artists — from ``dubins_path.py``. The slider pattern from those
  same scripts is deliberately *not* carried over: sliders there recomputed
  geometry, which would require this module to call an algorithm.
* Marker vocabulary: ``ks`` start, ``k*`` target, dashed grey ring, and the
  ``tab:`` colour cycle used for the six Dubins families.
* **Metrics in the legend label**, as ``dubins_path.py`` does with
  ``f"{name} ({length:.1f} m)"``.

Overlaying several runs on shared axes is the point of this module: it is the
side-by-side comparison ``MTG-2026-06-25-01`` asked for and that a single
``--algorithm`` invocation cannot give.

matplotlib
----------
Imported lazily inside :func:`plot_runs` only. Everything else here is pure and
importable without it. **matplotlib is not installed in this environment, so
:func:`plot_runs` has never executed**; the pure helpers below are tested, the
drawing is not.
"""

import json
import math

#: Colour cycle, matching ``py_plots/dubins_path.py``'s ``FAMILIES``.
RUN_COLOURS = (
    "tab:blue",
    "tab:orange",
    "tab:green",
    "tab:red",
    "tab:purple",
    "tab:brown",
)


# --------------------------------------------------------------------------
# History I/O — how a run reaches the plotter without the plotter running it
# --------------------------------------------------------------------------

def save_run(path, name, history, meta=None):
    """Write one recorded run to JSON.

    Args:
        path: Destination file.
        name: Label for the run, used in the legend.
        history: The list produced by ``Harness.history``.
        meta: Optional plain dict of run metadata, e.g. configuration values.
    """
    payload = {"name": name, "meta": dict(meta or {}), "history": list(history)}
    with open(path, "w") as handle:
        json.dump(payload, handle)
    return path


def load_run(path):
    """Read a run written by :func:`save_run`.

    Returns:
        ``(name, history, meta)``.

    Raises:
        ValueError: If the file is not a run record.
    """
    with open(path) as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict) or "history" not in payload:
        raise ValueError("%s is not a harness run record" % path)
    return payload.get("name", path), payload["history"], payload.get("meta", {})


# --------------------------------------------------------------------------
# Pure preparation — testable without matplotlib
# --------------------------------------------------------------------------

def series(history):
    """Split a history into plain lists ready for drawing.

    Returns a dict with ``plane_e``, ``plane_n``, ``target_e``, ``target_n``,
    ``guide_e``, ``guide_n`` and ``t``.

    Raises:
        ValueError: If ``history`` is empty.
    """
    if not history:
        raise ValueError("series() needs a non-empty history")
    return {
        "plane_e": [s["plane_e_m"] for s in history],
        "plane_n": [s["plane_n_m"] for s in history],
        "target_e": [s["target_e_m"] for s in history],
        "target_n": [s["target_n_m"] for s in history],
        "guide_e": [s["guidance_e_m"] for s in history],
        "guide_n": [s["guidance_n_m"] for s in history],
        "t": [s["t_s"] for s in history],
    }


def frame_at(data, i):
    """One animation frame's data at sample index ``i`` (``TASK-011``).

    Pure: given a :func:`series` dict, returns the plane path **already
    travelled** (samples ``0..i``), the current plane and target positions, the
    target path so far, and the time. No matplotlib — the animation driver draws
    it, so the plotter stays data-only (``DEC-2026-07-22-01``).

    Raises:
        ValueError: If ``data`` has no samples.
    """
    n = len(data["t"])
    if n == 0:
        raise ValueError("frame_at() needs a non-empty series")
    i = 0 if i < 0 else (n - 1 if i >= n else i)
    return {
        "t": data["t"][i],
        "plane_e": data["plane_e"][: i + 1],
        "plane_n": data["plane_n"][: i + 1],
        "plane_pos": (data["plane_e"][i], data["plane_n"][i]),
        "target_e": data["target_e"][: i + 1],
        "target_n": data["target_n"][: i + 1],
        "target_pos": (data["target_e"][i], data["target_n"][i]),
    }


def ring_points(cx, cy, radius_m, n=100):
    """Circle about ``(cx, cy)`` as ``(east, north)`` lists.

    The orbit ring, drawn dashed grey. ``py_plots`` built this with
    ``numpy.linspace``; here it is a loop, matching the rest of the harness.
    """
    east, north = [], []
    for i in range(n + 1):
        a = 2.0 * math.pi * i / n
        east.append(cx + radius_m * math.sin(a))
        north.append(cy + radius_m * math.cos(a))
    return east, north


def cylinder_surface(cx, cy, radius_m, z0, z1, n_th=40, n_z=20):
    """Vertical cylinder about the target spanning a time band.

    Ported from ``py_plots/combined.py:87``, with the ``numpy.meshgrid``
    replaced by nested lists. Returns ``(X, Y, Z)`` as lists of rows, which is
    what ``plot_surface`` accepts.
    """
    if n_th < 2 or n_z < 2:
        raise ValueError("cylinder_surface needs at least 2x2 samples")
    X, Y, Z = [], [], []
    for j in range(n_z):
        z = z0 + (z1 - z0) * j / (n_z - 1)
        row_x, row_y, row_z = [], [], []
        for i in range(n_th):
            a = 2.0 * math.pi * i / (n_th - 1)
            row_x.append(cx + radius_m * math.sin(a))
            row_y.append(cy + radius_m * math.cos(a))
            row_z.append(z)
        X.append(row_x)
        Y.append(row_y)
        Z.append(row_z)
    return X, Y, Z


def compute_reach(runs, orbit_radius_m=None, margin=1.1):
    """Half-extent for fixed, symmetric axis limits.

    ``py_plots`` sized its axes once from the widest case so the view does not
    "breathe" as data changes. Same idea, computed from the recorded runs.

    Args:
        runs: Sequence of ``(name, history)`` pairs.
        orbit_radius_m: Included so the ring stays in frame.
        margin: Multiplier applied to the widest extent.

    Returns:
        A positive half-extent, never zero.
    """
    widest = 0.0
    for _name, history in runs:
        for s in history:
            for e, n in (
                (s["plane_e_m"], s["plane_n_m"]),
                (s["target_e_m"], s["target_n_m"]),
                (s["guidance_e_m"], s["guidance_n_m"]),
            ):
                widest = max(widest, abs(e), abs(n))
    if orbit_radius_m:
        widest += abs(orbit_radius_m)
    return max(1.0, widest * margin)


def achieved_radius(history, fraction=0.25):
    """Mean aircraft-to-target range over the last ``fraction`` of a run.

    Duplicated deliberately from ``Harness.achieved_orbit_radius_m`` so this
    module needs no import from ``state``. Used in legend labels, following
    ``dubins_path.py``'s habit of putting the metric in the label.
    """
    if not history:
        return None
    n = max(1, int(len(history) * fraction))
    tail = history[-n:]
    total = 0.0
    for s in tail:
        total += math.hypot(
            s["plane_n_m"] - s["target_n_m"], s["plane_e_m"] - s["target_e_m"]
        )
    return total / len(tail)


def legend_label(name, history, orbit_radius_m=None):
    """Series label carrying its own metric, per the ``py_plots`` pattern."""
    achieved = achieved_radius(history)
    if achieved is None:
        return name
    if orbit_radius_m:
        return "%s (%.0f m of %.0f m commanded)" % (name, achieved, orbit_radius_m)
    return "%s (final range %.0f m)" % (name, achieved)


def any_infeasible(runs):
    """True when any run was produced under an overridden bank limit."""
    return any(
        s.get("infeasible") for _name, history in runs for s in history
    )


# --------------------------------------------------------------------------
# Drawing — matplotlib only lives below this line
# --------------------------------------------------------------------------

def plot_runs(runs, orbit_radius_m=None, show_3d=True, planned=None,
              save_path=None, show=False):
    """Overlay recorded runs on shared axes.

    Read-only: this draws ``runs`` and touches nothing else. There is no code
    path from here to an algorithm, because this module does not import one.

    Args:
        runs: Sequence of ``(name, history)`` pairs. Several runs overlay.
        orbit_radius_m: Draws the ring, and the cylinder in the 3D view.
        show_3d: Also build the ``z = time`` 3D figure.
        planned: Optional sequence aligned with ``runs``. Each element is a list
            of ``(East, North)`` points — the exact planned Dubins geometry — or
            ``None``. Drawn solid over the flown track (``TASK-002``). Points
            only: the plotter still imports no harness module.
        save_path: Base path for PNG output. The 3D figure gets a ``-3d``
            suffix. Omit to skip saving.
        show: Call ``plt.show()``. Off by default so the function is usable
            headless.

    Returns:
        A dict of the figures created, keyed ``"topdown"`` and ``"3d"``.

    Raises:
        ValueError: If ``runs`` is empty.

    Status: **never executed** — matplotlib is not installed here.
    """
    if not runs:
        raise ValueError("plot_runs needs at least one run")

    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.widgets import CheckButtons

    reach = compute_reach(runs, orbit_radius_m)
    infeasible = any_infeasible(runs)
    figures = {}

    # ---- 2D top-down companion -------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(8, 8))
    plt.subplots_adjust(left=0.24)
    ax2.set_title(
        "INFEASIBLE — violates the bank limit; not a flyable path"
        if infeasible
        else "Top-down (X-Y)"
    )
    ax2.set_xlabel("East (x)")
    ax2.set_ylabel("North (y)")
    ax2.set_aspect("equal")
    ax2.grid(True)
    ax2.set_xlim(-reach, reach)
    ax2.set_ylim(-reach, reach)

    toggle_map = {}
    first = series(runs[0][1])
    ax2.plot([first["plane_e"][0]], [first["plane_n"][0]], "ks", markersize=8,
             label="start")
    # Target: a star at its final position, plus its trail if it moved (TASK-003).
    tgt_moved = (
        abs(first["target_e"][-1] - first["target_e"][0]) > 1e-6
        or abs(first["target_n"][-1] - first["target_n"][0]) > 1e-6
    )
    if tgt_moved:
        trail = ax2.plot(first["target_e"], first["target_n"], color="k",
                         linestyle="--", linewidth=1, alpha=0.6,
                         label="target path")[0]
        ax2.plot([first["target_e"][0]], [first["target_n"][0]], "k^",
                 markersize=8)
        toggle_map["target path"] = [trail]
    ax2.plot([first["target_e"][-1]], [first["target_n"][-1]], "k*", markersize=14,
             label="target")

    if orbit_radius_m:
        te, tn = first["target_e"], first["target_n"]
        ring_artists = []
        # A moving target carries its orbit ring with it (TASK-003): draw the ring
        # lightly at the target's position at successive instants, so the orbit is
        # seen travelling with the target. The centres come from the recorded
        # target history, so this stays data-only (DEC-2026-07-22-01).
        if tgt_moved:
            n_rings = 6
            last = len(te) - 1
            for k in range(n_rings):
                i = int(round(last * k / (n_rings - 1))) if last else 0
                re, rn = ring_points(te[i], tn[i], orbit_radius_m)
                ring_artists.append(
                    ax2.plot(re, rn, color="grey", linewidth=0.8, alpha=0.25)[0]
                )
        # The ring at the final target position, shaded and dashed on top.
        ring_e, ring_n = ring_points(te[-1], tn[-1], orbit_radius_m)
        disc = ax2.fill(ring_e, ring_n, color="deepskyblue", alpha=0.12,
                        label="orbit region (%.0f m)" % orbit_radius_m)[0]
        ring = ax2.plot(ring_e, ring_n, color="grey", linestyle="--",
                        linewidth=1)[0]
        toggle_map["ring"] = ring_artists + [disc, ring]

    for index, (name, history) in enumerate(runs):
        colour = RUN_COLOURS[index % len(RUN_COLOURS)]
        data = series(history)
        track = ax2.plot(data["plane_e"], data["plane_n"], color=colour, linewidth=2,
                         label=legend_label(name, history, orbit_radius_m))[0]
        carrot = ax2.plot(data["guide_e"], data["guide_n"], color=colour,
                          linewidth=0.7, alpha=0.5, linestyle=":")[0]
        toggle_map[name] = [track, carrot]

        # Exact planned Dubins geometry (TASK-002), overlaid on the flown track.
        if planned and index < len(planned) and planned[index]:
            pe = [p[0] for p in planned[index]]
            pn = [p[1] for p in planned[index]]
            plan = ax2.plot(pe, pn, color="black", linewidth=1.4, alpha=0.85,
                            label="%s planned Dubins" % name)[0]
            toggle_map[name].append(plan)

    ax2.legend(loc="lower left", fontsize=8)

    ax_check = plt.axes([0.01, 0.4, 0.18, 0.4])
    ax_check.set_title("show", fontsize=9)
    labels = list(toggle_map)
    check = CheckButtons(ax_check, labels, [True] * len(labels))

    def on_check(label):
        for artist in toggle_map[label]:
            artist.set_visible(not artist.get_visible())
        fig2.canvas.draw_idle()

    check.on_clicked(on_check)
    fig2._harness_check = check  # keep the widget alive
    figures["topdown"] = fig2

    # ---- 3D, z = time ----------------------------------------------------
    if show_3d:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("East (x)")
        ax.set_ylabel("North (y)")
        ax.set_zlabel("time (z)")
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)
        ax.set_box_aspect((1, 1, 0.6))

        for index, (name, history) in enumerate(runs):
            colour = RUN_COLOURS[index % len(RUN_COLOURS)]
            data = series(history)
            ax.plot(data["plane_e"], data["plane_n"], data["t"], color=colour,
                    linewidth=2, label=legend_label(name, history, orbit_radius_m))
            ax.plot(data["target_e"], data["target_n"], data["t"], color="k",
                    linestyle=":", linewidth=1)

        if orbit_radius_m:
            span = max(s["t"][-1] for s in (series(h) for _n, h in runs))
            X, Y, Z = cylinder_surface(
                first["target_e"][-1], first["target_n"][-1], orbit_radius_m, 0.0, span
            )
            # cylinder_surface returns nested lists (kept transliterable and
            # tested as such); plot_surface needs arrays with an ndim.
            ax.plot_surface(
                np.asarray(X), np.asarray(Y), np.asarray(Z),
                color="deepskyblue", alpha=0.15,
            )

        ax.legend(loc="lower left", fontsize=8)
        figures["3d"] = fig

    if save_path:
        figures["topdown"].savefig(save_path, dpi=120)
        if "3d" in figures:
            base = save_path.rsplit(".", 1)
            stem = base[0]
            ext = base[1] if len(base) > 1 else "png"
            figures["3d"].savefig("%s-3d.%s" % (stem, ext), dpi=120)
    if show:
        plt.show()
    return figures


def animate_runs(runs, orbit_radius_m=None, save_path=None, frames=120, fps=20):
    """2D animation of the plane and kangaroo over time (``TASK-011``).

    The plot evolves with ``t``: the plane's already-travelled path grows each
    frame, the kangaroo moves along its path, and the orbit ring is projected
    around the kangaroo at its current position. The animation spans the whole
    recorded window (``--duration-s``), decimated to ``frames`` frames.

    Read-only and data-only: each frame is built from :func:`frame_at`, so this
    imports no harness module (``DEC-2026-07-22-01``).

    Args:
        runs: ``(name, history)`` pairs, as for :func:`plot_runs`.
        orbit_radius_m: Draws the ring travelling with the kangaroo when set.
        save_path: File to write (``.gif`` via Pillow). Omit to only return the
            animation object.
        frames: Target frame count; the history is decimated to this.
        fps: Frames per second for the writer.

    Returns:
        The ``FuncAnimation`` object.
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    series_list = [series(h) for _n, h in runs]
    n_samples = max(len(s["t"]) for s in series_list)
    stride = max(1, n_samples // max(1, frames))
    frame_indices = list(range(0, n_samples, stride))
    if frame_indices[-1] != n_samples - 1:
        frame_indices.append(n_samples - 1)

    reach = compute_reach(runs, orbit_radius_m)
    infeasible = any_infeasible(runs)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_xlabel("East (x)")
    ax.set_ylabel("North (y)")

    # One target trail + ring (the kangaroo is shared across runs); one path and
    # marker per run.
    target_trail, = ax.plot([], [], color="k", linestyle="--", linewidth=1,
                            alpha=0.6, label="kangaroo path")
    target_dot, = ax.plot([], [], "k*", markersize=14)
    ring_line, = ax.plot([], [], color="grey", linestyle="--", linewidth=1) \
        if orbit_radius_m else (ax.plot([], [])[0],)
    plane_lines, plane_dots = [], []
    for index, (name, _h) in enumerate(runs):
        colour = RUN_COLOURS[index % len(RUN_COLOURS)]
        line, = ax.plot([], [], color=colour, linewidth=2, label=name)
        dot, = ax.plot([], [], "o", color=colour, markersize=6)
        plane_lines.append(line)
        plane_dots.append(dot)
    title = ax.set_title("t = 0.0 s" + (" — INFEASIBLE" if infeasible else ""))
    ax.legend(loc="upper right", fontsize=8)

    def update(k):
        i = frame_indices[k]
        for run_i, data in enumerate(series_list):
            fr = frame_at(data, i)
            plane_lines[run_i].set_data(fr["plane_e"], fr["plane_n"])
            plane_dots[run_i].set_data([fr["plane_pos"][0]], [fr["plane_pos"][1]])
        fr0 = frame_at(series_list[0], i)
        target_trail.set_data(fr0["target_e"], fr0["target_n"])
        target_dot.set_data([fr0["target_pos"][0]], [fr0["target_pos"][1]])
        if orbit_radius_m:
            re, rn = ring_points(fr0["target_pos"][0], fr0["target_pos"][1],
                                 orbit_radius_m)
            ring_line.set_data(re, rn)
        title.set_text("t = %.1f s" % fr0["t"] + (" — INFEASIBLE" if infeasible else ""))
        return plane_lines + plane_dots + [target_trail, target_dot, ring_line]

    anim = FuncAnimation(fig, update, frames=len(frame_indices),
                         interval=1000.0 / fps, blit=False)
    if save_path:
        anim.save(save_path, writer=PillowWriter(fps=fps))
    return anim


def main(argv=None):
    """Plot one or more saved runs.

    Takes JSON files written by ``run_harness.py --save-run``. Deliberately
    takes files rather than algorithm names: this script cannot run a geometry,
    and that is the point.
    """
    import argparse

    parser = argparse.ArgumentParser(description=main.__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", help="Run JSON files to overlay.")
    parser.add_argument("--orbit-radius-m", type=float, default=None)
    parser.add_argument("--save", default=None, help="Write PNG instead of showing.")
    parser.add_argument("--no-3d", action="store_true")
    args = parser.parse_args(argv)

    loaded = []
    for path in args.runs:
        name, history, _meta = load_run(path)
        loaded.append((name, history))

    plot_runs(
        loaded,
        orbit_radius_m=args.orbit_radius_m,
        show_3d=not args.no_3d,
        save_path=args.save,
        show=args.save is None,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
