"""Thesis figures F1–F13 for the `TASK-045` campaign, from recorded outputs only.

Created 2026-09-13.

Every figure here is produced from ``master.csv``, ``MANIFEST.json`` and the
per-cell bundle files (``series.json``, ``history.json``, ``ticks.csv``,
``spec.json``) — **never from a live run** — so each is regenerable without
re-running the grid (`TASK-045` D6). Read-only, like :mod:`plotter`: this
module imports nothing that can drive an algorithm (`DEC-2026-07-22-01`).

Conventions that hold across every figure (`TASK-045`, "Figures"):

* **One colour per arm, fixed for the whole thesis**, in ladder order
  ``0, D, A, B, C`` (:data:`ARM_COLOURS`).
* **Missing, not zero.** A cell whose metric is ``None`` is an absent bar or a
  gap in the line, never a bar at 0.
* **Feasibility band on every speed axis.** The holdable region shaded, the
  curvature-infeasible and unreachable regions labelled. The boundary is
  computed from the manifest's fixed parameters, not typed in.
* **Coverage beside every ``e_tan`` figure.**
* **``elastic`` axes labelled "fast-phase ratio"**, not "speed".

F11–F13 are the Chapter 3 illustrative figures; F1–F10 are Chapter 4.
"""

import csv
import json
import math
import os

from . import plotter


#: Arm colours in ladder order. Okabe–Ito, colour-blind safe. Keyed by the
#: arm's base letter, so a `TASK-048` hysteresis arm (``0H``, ``DH``, ``AH``,
#: ``BH``) is drawn in its base arm's colour; the manifest gives the order.
ARM_ORDER = ("0", "D", "A", "B", "C")
ARM_COLOURS = {
    "0": "#4D4D4D",     # baseline: neutral
    "D": "#E69F00",     # one-tick step
    "A": "#0072B2",     # fixed horizon
    "B": "#009E73",     # adaptive horizon
    "C": "#CC79A7",     # receding horizon
}
ARM_LABELS = {
    "0": "arm 0 — baseline",
    "D": "arm D — 1-tick step",
    "A": "arm A — fixed horizon",
    "B": "arm B — adaptive horizon",
    "C": "arm C — receding horizon",
}


def arm_colour(arm):
    return ARM_COLOURS[str(arm)[0]]


def arm_label(arm):
    """The arm's legend label by its base letter. A hysteresis counterpart
    (``AH``) is labelled as its parent (author's direction, 2026-09-15): the
    hysteresis is described once in the thesis and inherited by the arms, so
    figures name the arm only."""
    return ARM_LABELS[str(arm)[0]]


def arm_order(manifest):
    """The arms of this campaign, in the manifest's (arm set's) order."""
    return list(manifest.get("arms") or ARM_ORDER)


def arm_for(manifest, letter):
    """The campaign's arm id whose base letter is ``letter`` (``"A"`` ->
    ``"A"`` in `CAMP-002`, ``"AH"`` in `CAMP-003`)."""
    for arm in arm_order(manifest):
        if str(arm)[0] == letter:
            return arm
    raise ValueError("no arm with base letter %r in this campaign" % letter)


def _cid(manifest, letter, suffix):
    """Cell id for this campaign's arm with base ``letter``; ``suffix`` None is
    the stationary-target cell (``<arm>-point``)."""
    arm = arm_for(manifest, letter)
    return "%s-%s" % (arm, "point" if suffix is None else suffix)

#: Mode permutations in reporting order and their axis labels.
MODE_ORDER = (("point", "constant"), ("straight", "constant"),
              ("straight", "elastic"), ("circle", "constant"),
              ("circle", "elastic"), ("rectangle", "constant"),
              ("rectangle", "elastic"), ("rand", "constant"))
MODE_LABELS = {
    ("point", "constant"): "point",
    ("straight", "constant"): "straight",
    ("straight", "elastic"): "straight\n(elastic)",
    ("circle", "constant"): "circle",
    ("circle", "elastic"): "circle\n(elastic)",
    ("rectangle", "constant"): "rect.",
    ("rectangle", "elastic"): "rect.\n(elastic)",
    ("rand", "constant"): "rand\n(5 seeds)",
}

#: Holdable ratios — below the feasibility boundary at the fixed parameters.
HOLDABLE_RATIOS = (0.25, 0.5)

#: The F11 single-cell renders: Chapter 3 marker -> (base letter, cell suffix).
F11_CELLS = (
    ("base-point", "0", None),
    ("armA-straight", "A", "straight-constant-half"),
    ("armB-straight-elastic", "B", "straight-elastic-half"),
    ("armB-circle", "B", "circle-constant-half"),
    ("armC-circle", "C", "circle-constant-half"),
    ("armD-straight", "D", "straight-constant-half"),
)

#: F4 representative cells (arm prefix added per arm).
F4_CELLS = ("straight-constant-half", "circle-elastic-half",
            "rand-constant-half-s01")

FIGURE_NAMES = ("F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10",
                "F11", "F12", "F13", "F20")


# --------------------------------------------------------------------------
# Loading
# --------------------------------------------------------------------------

def _parse(value):
    if value == "":
        return None
    if value == "True":
        return True
    if value == "False":
        return False
    try:
        return float(value)
    except ValueError:
        return value


#: Identity and label columns kept as text: arm ``"0"`` is a name, not a number.
STRING_COLUMNS = ("cell_id", "sub", "arm", "algorithm", "mode_base",
                  "mode_pace", "ratio_name", "status", "stopped_reason",
                  "s1_target", "s1_sampling")


def load_master(out_dir):
    """``master.csv`` as a list of dicts with typed values, ``None`` for empty."""
    with open(os.path.join(out_dir, "master.csv"), newline="") as handle:
        return [dict((k, (v if k in STRING_COLUMNS else _parse(v)))
                     for k, v in row.items())
                for row in csv.DictReader(handle)]


def load_manifest(out_dir):
    with open(os.path.join(out_dir, "MANIFEST.json")) as handle:
        return json.load(handle)


def _bundle(out_dir, manifest, cell_id):
    entry = manifest["cells"].get(cell_id)
    if not entry or not entry.get("bundle"):
        return None
    return os.path.join(out_dir, entry["bundle"])


def _load_series(directory):
    with open(os.path.join(directory, "series.json")) as handle:
        return json.load(handle)["series"]


def _load_history(directory):
    with open(os.path.join(directory, "history.json")) as handle:
        return json.load(handle)["history"]


def _load_json(directory, name):
    with open(os.path.join(directory, name)) as handle:
        return json.load(handle)


def feasibility_boundary(manifest):
    """Speed ratio above which the ring is not holdable at the fixed
    parameters: ``v_K <= V (R/rho − 1)`` from ``(V + v_K)/(R V) <= 1/rho``."""
    fixed = manifest["fixed"]
    return fixed["orbit_radius_m"] / fixed["turn_radius_m"] - 1.0


def _mean(values):
    present = [v for v in values if v is not None]
    return (sum(present) / len(present)) if present else None


def _std(values):
    present = [v for v in values if v is not None]
    if len(present) < 2:
        return None
    m = sum(present) / len(present)
    return math.sqrt(sum((v - m) ** 2 for v in present) / (len(present) - 1))


def _main_rows(rows):
    return [r for r in rows if r["sub"] == "main"]


def _cells(rows, arm=None, base=None, pace=None, ratio=None):
    out = []
    for r in rows:
        if arm is not None and r["arm"] != arm:
            continue
        if base is not None and r["mode_base"] != base:
            continue
        if pace is not None and r["mode_pace"] != pace:
            continue
        if ratio is not None and r["speed_ratio"] != ratio:
            continue
        out.append(r)
    return out


# --------------------------------------------------------------------------
# Shared drawing helpers
# --------------------------------------------------------------------------

def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 8, "axes.titlesize": 9,
                         "axes.labelsize": 8, "legend.fontsize": 7,
                         "xtick.labelsize": 7, "ytick.labelsize": 7})
    return plt


def _feasibility_band(ax, boundary, ratios, unreachable_from=1.0, label=True):
    """Shade the holdable region and label the two failure regions."""
    lo = min(ratios) if ratios else 0.0
    hi = max(ratios) if ratios else 1.5
    pad = 0.05 * (hi - lo) if hi > lo else 0.1
    ax.axvspan(lo - pad, boundary, color="#DCEFD9", alpha=0.6, zorder=0)
    ax.axvline(boundary, color="#6B7A85", linewidth=0.8, linestyle=":")
    ax.axvline(unreachable_from, color="#6B7A85", linewidth=0.8, linestyle=":")
    if label:
        ymax = ax.get_ylim()[1]
        ax.text((lo - pad + boundary) / 2.0, ymax, "holdable", ha="center",
                va="top", fontsize=6, color="#3A6B35")
        ax.text((boundary + unreachable_from) / 2.0, ymax, "curvature-\ninfeasible",
                ha="center", va="top", fontsize=6, color="#6B7A85")
        ax.text((unreachable_from + hi + pad) / 2.0, ymax, "unreachable",
                ha="center", va="top", fontsize=6, color="#6B7A85")
    ax.set_xlim(lo - pad, hi + pad)


def _save(fig, out_dir, name, stem):
    directory = os.path.join(out_dir, "figures")
    os.makedirs(directory, exist_ok=True)
    base = os.path.join(directory, "%s-%s" % (name, stem))
    fig.savefig(base + ".png", dpi=200, bbox_inches="tight")
    fig.savefig(base + ".pdf", bbox_inches="tight")
    import matplotlib.pyplot as plt
    plt.close(fig)
    return base + ".png"


def _grouped_bars(ax, rows, metric, arms, groups=MODE_ORDER,
                  ratios=HOLDABLE_RATIOS):
    """Grouped bars: one group per mode permutation, one bar per arm; height =
    mean of ``metric`` over the holdable ratios (and seeds); error bar = its
    sample standard deviation. A `None` metric draws **no bar**."""
    width = 0.8 / len(arms)
    for g_index, (base, pace) in enumerate(groups):
        for a_index, arm in enumerate(arms):
            cells = [r for r in _cells(rows, arm=arm, base=base, pace=pace)
                     if base == "point" or r["speed_ratio"] in ratios]
            values = [r[metric] for r in cells]
            mean = _mean(values)
            if mean is None:
                continue
            x = g_index + (a_index - (len(arms) - 1) / 2.0) * width
            err = _std(values)
            ax.bar(x, mean, width=width * 0.95, color=arm_colour(arm),
                   yerr=None if err is None else err, capsize=1.5,
                   error_kw={"linewidth": 0.6}, label=arm_label(arm)
                   if g_index == 0 else None)
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([MODE_LABELS[g] for g in groups], fontsize=6)
    ax.grid(axis="y", color="#E3E9ED", linewidth=0.6)
    ax.set_axisbelow(True)


# --------------------------------------------------------------------------
# F1 — error by algorithm
# --------------------------------------------------------------------------

def figure_f1(out_dir, rows, manifest):
    plt = _plt()
    rows = _main_rows(rows)
    made = []
    for metric, stem, ylabel in (
            ("rms_ring_error_m", "error-by-algorithm",
             "RMS ring error vs true target (m)"),
            ("rms_ring_error_held_m", "error-by-algorithm-held",
             "RMS ring error vs held centre (m)")):
        fig, axes = plt.subplots(1, 2, figsize=(8.5, 3.2), sharey=True)
        for ax, ratio in zip(axes, HOLDABLE_RATIOS):
            _grouped_bars(ax, rows, metric, arm_order(manifest), ratios=(ratio,))
            ax.set_title("speed ratio %.2f (elastic: fast-phase ratio)" % ratio)
        axes[0].set_ylabel(ylabel)
        axes[0].legend(loc="upper left", frameon=False)
        fig.suptitle("Holdable ratios only; kangaroo_rand bars average five "
                     "seeds (error bar = sample s.d.); no bar = undefined",
                     fontsize=7, y=1.02)
        made.append(("F1", _save(fig, out_dir, "F1", stem)))
    return made


# --------------------------------------------------------------------------
# F2 — arm x mode heat map, one panel per ratio
# --------------------------------------------------------------------------

def figure_f2(out_dir, rows, manifest):
    plt = _plt()
    rows = _main_rows(rows)
    ratios = [r[1] for r in manifest["subs"]["main"]["speed_ratios"]]
    metric = "rms_ring_error_m"
    order = arm_order(manifest)
    grid = {}
    vmax = 0.0
    for ratio in ratios:
        for a_i, arm in enumerate(order):
            for m_i, (base, pace) in enumerate(MODE_ORDER):
                cells = _cells(rows, arm=arm, base=base, pace=pace,
                               ratio=None if base == "point" else ratio)
                value = _mean([c[metric] for c in cells])
                grid[(ratio, a_i, m_i)] = value
                if value is not None:
                    vmax = max(vmax, value)
    fig, axes = plt.subplots(1, len(ratios), figsize=(1.9 * len(ratios) + 1.2, 2.9),
                             sharey=True, squeeze=False)
    axes = list(axes.flat)
    import numpy as np
    cmap = plt.get_cmap("viridis").with_extremes(bad="#D9D9D9")
    image = None
    for ax, ratio in zip(axes, ratios):
        data = np.full((len(order), len(MODE_ORDER)), np.nan)
        for a_i in range(len(order)):
            for m_i in range(len(MODE_ORDER)):
                v = grid[(ratio, a_i, m_i)]
                if v is not None:
                    data[a_i, m_i] = v
        image = ax.imshow(np.ma.masked_invalid(data), cmap=cmap, vmin=0.0,
                          vmax=vmax, aspect="auto")
        for a_i in range(len(order)):
            for m_i in range(len(MODE_ORDER)):
                v = grid[(ratio, a_i, m_i)]
                ax.text(m_i, a_i, "–" if v is None else "%.0f" % v,
                        ha="center", va="center", fontsize=5,
                        color="white" if (v is not None and v > 0.55 * vmax) else "black")
        ax.set_xticks(range(len(MODE_ORDER)))
        ax.set_xticklabels([MODE_LABELS[g].replace("\n", " ").replace(
            " (5 seeds)", "") for g in MODE_ORDER],
            rotation=60, ha="right", fontsize=5)
        ax.set_yticks(range(len(order)))
        ax.set_yticklabels(["arm %s" % a for a in order])
        boundary = feasibility_boundary(manifest)
        tag = ("holdable" if ratio <= boundary
               else ("unreachable" if ratio >= 1.0 else "curvature-infeasible"))
        ax.set_title("ratio %.2f\n%s" % (ratio, tag), fontsize=7)
    fig.colorbar(image, ax=axes, shrink=0.8, label="RMS ring error vs target (m)")
    fig.suptitle("Grey = undefined (no contact, stopped run); point column is "
                 "ratio-independent", fontsize=7, y=1.04)
    return [("F2", _save(fig, out_dir, "F2", "arm-mode-heatmap"))]


# --------------------------------------------------------------------------
# F3 — error against speed ratio
# --------------------------------------------------------------------------

def figure_f3(out_dir, rows, manifest):
    plt = _plt()
    rows = _main_rows(rows)
    boundary = feasibility_boundary(manifest)
    ratios = [r[1] for r in manifest["subs"]["main"]["speed_ratios"]]
    groups = [g for g in MODE_ORDER if g[0] != "point"]
    fig, axes = plt.subplots(2, 4, figsize=(8.0, 4.4), sharey=True)
    axes = list(axes.flat)
    for ax, (base, pace) in zip(axes, groups):
        for arm in arm_order(manifest):
            xs, ys = [], []
            for ratio in ratios:
                cells = _cells(rows, arm=arm, base=base, pace=pace, ratio=ratio)
                v = _mean([c["rms_ring_error_m"] for c in cells])
                xs.append(ratio)
                ys.append(v)
            # Break the line at None: the arm stopped there.
            segment_x, segment_y = [], []
            for x, y in zip(xs, ys):
                if y is None:
                    if segment_x:
                        ax.plot(segment_x, segment_y, marker="o", markersize=3,
                                color=arm_colour(arm), linewidth=1.2)
                    segment_x, segment_y = [], []
                else:
                    segment_x.append(x)
                    segment_y.append(y)
            if segment_x:
                ax.plot(segment_x, segment_y, marker="o", markersize=3,
                        color=arm_colour(arm), linewidth=1.2,
                        label=arm_label(arm))
        ax.set_title(MODE_LABELS[(base, pace)].replace("\n", " "), fontsize=8)
        ax.set_xlabel("fast-phase ratio" if pace == "elastic" else "speed ratio")
        ax.grid(color="#E3E9ED", linewidth=0.6)
        _feasibility_band(ax, boundary, ratios)
    for ax in axes[len(groups):]:
        ax.axis("off")
    axes[0].set_ylabel("RMS ring error vs target (m)")
    axes[4].set_ylabel("RMS ring error vs target (m)")
    handles, labels = axes[0].get_legend_handles_labels()
    axes[-1].legend(handles, labels, loc="center", frameon=False)
    fig.tight_layout()
    return [("F3", _save(fig, out_dir, "F3", "error-vs-speed-ratio"))]


# --------------------------------------------------------------------------
# F4 / F12 — error versus time, five arms on one axis
# --------------------------------------------------------------------------

def _marker_times(spec, record):
    times, elapsed = [], 0.0
    for leg in spec["kangaroo"]["legs"][:-1]:
        elapsed += leg["duration_s"]
        if 0.0 < elapsed < spec["run"]["duration_s"]:
            times.append(elapsed)
    for change in (record.get("kangaroo") or {}).get("changes") or []:
        times.append(change["t_s"])
    return sorted(set(round(t, 6) for t in times))


def _overlay_ring_error(ax, out_dir, manifest, suffix, mark_contact=True,
                        cut_at_containment=False):
    """Draw ring_error_target_m of every arm's ``<arm>-<suffix>`` cell.

    ``cut_at_containment`` stops every trace at the cell's first zone
    containment turn and draws no marker for it (the straight kangaroo at
    ratio 0.5 is turned back at 50.4 s, `ISSUE-M9`): what follows is the
    zone's manoeuvre, not the arm's response to the schedule.
    """
    drawn = 0
    markers = None
    for arm in arm_order(manifest):
        directory = _bundle(out_dir, manifest, "%s-%s" % (arm, suffix))
        if directory is None or not os.path.isfile(
                os.path.join(directory, "series.json")):
            continue
        series = _load_series(directory)
        s = series.get("ring_error_target_m")
        if s is None:
            continue
        t, v = s["t_s"], s["values"]
        record = _load_json(directory, "record.json")
        cut_s = (first_containment_turn_s(record) if cut_at_containment
                 else None)
        if cut_s is not None:
            keep = [i for i, ti in enumerate(t) if ti <= cut_s + 1e-9]
            t = [t[i] for i in keep]
            v = [v[i] for i in keep]
        # Draw defined spans only.
        xs, ys = [], []
        for ti, vi in zip(t, v):
            if vi is None:
                if xs:
                    ax.plot(xs, ys, color=arm_colour(arm), linewidth=1.0)
                xs, ys = [], []
            else:
                xs.append(ti)
                ys.append(vi)
        ax.plot(xs, ys, color=arm_colour(arm), linewidth=1.0,
                label=arm_label(arm))
        if markers is None:
            markers = _marker_times(_load_json(directory, "spec.json"), record)
            if cut_s is not None:
                markers = [m for m in markers if m < cut_s - 1e-9]
        if mark_contact:
            contact = ((record["metrics"].get("post_contact") or {})
                       .get("target") or {}).get("t_contact_s")
            if contact is not None:
                ax.plot([contact], [0.0], marker="v", markersize=5,
                        color=arm_colour(arm), linestyle="none", zorder=5)
        drawn += 1
    for t_mark in markers or []:
        ax.axvline(t_mark, color="#C2571C", linewidth=0.7, linestyle=(0, (4, 3)))
    ax.axhline(0.0, color="#6B7A85", linewidth=0.6)
    ax.grid(color="#E3E9ED", linewidth=0.6)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("ring error vs target (m)")
    return drawn


def figure_f4(out_dir, rows, manifest):
    plt = _plt()
    fig, axes = plt.subplots(len(F4_CELLS), 1, figsize=(7.0, 2.2 * len(F4_CELLS)),
                             sharex=True)
    for ax, suffix in zip(axes, F4_CELLS):
        _overlay_ring_error(ax, out_dir, manifest, suffix)
        ax.set_title(suffix + "  (▼ = first contact; dashed = kangaroo "
                     "manoeuvre)", fontsize=8)
    axes[0].legend(loc="upper right", frameon=False, ncol=3)
    fig.tight_layout()
    return [("F4", _save(fig, out_dir, "F4", "error-vs-time"))]


def figure_f12(out_dir, rows, manifest):
    """F4 applied to the five ``*-straight-constant-half`` cells. Refuses to
    draw if the five specs' initial conditions differ."""
    plt = _plt()
    conditions = {}
    for arm in arm_order(manifest):
        directory = _bundle(out_dir, manifest, "%s-straight-constant-half" % arm)
        if directory is None:
            raise ValueError("F12: cell %s-straight-constant-half has no bundle"
                             % arm)
        spec = _load_json(directory, "spec.json")
        conditions[arm] = json.dumps(spec["initial_conditions"], sort_keys=True)
    if len(set(conditions.values())) != 1:
        raise ValueError("F12 refused: the five cells' initial conditions "
                         "differ, so the overlay would not be like-for-like: %r"
                         % conditions)
    fig, ax = plt.subplots(1, 1, figsize=(7.0, 3.0))
    _overlay_ring_error(ax, out_dir, manifest, "straight-constant-half",
                        cut_at_containment=True)
    ax.set_title("Straight kangaroo at ratio 0.5, same start for every arm "
                 "(▼ = first contact; drawn to the zone's containment turn)",
                 fontsize=8)
    ax.legend(loc="upper right", frameon=False, ncol=3)
    fig.tight_layout()
    return [("F12", _save(fig, out_dir, "F12", "radial-error-five-arms"))]


# --------------------------------------------------------------------------
# F5 — post-contact hold quality
# --------------------------------------------------------------------------

def figure_f5(out_dir, rows, manifest):
    plt = _plt()
    rows = _main_rows(rows)
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.2))
    _grouped_bars(axes[0], rows, "post_contact_mean_radial_m", arm_order(manifest))
    axes[0].set_ylabel("post-contact mean |radial error| (m)")
    axes[0].set_title("Hold quality once the ring is reached", fontsize=8)
    _grouped_bars(axes[1], rows, "t_contact_s", arm_order(manifest))
    axes[1].set_ylabel("time to first contact (s)")
    axes[1].set_title("Time to reach the ring", fontsize=8)
    axes[0].legend(loc="upper left", frameon=False)
    fig.suptitle("Holdable ratios (0.25, 0.5) averaged; no bar = contact never "
                 "made", fontsize=7, y=1.02)
    fig.tight_layout()
    return [("F5", _save(fig, out_dir, "F5", "post-contact-hold"))]


# --------------------------------------------------------------------------
# F6 — per-tick error distribution, post-contact ticks, holdable cells
# --------------------------------------------------------------------------

def figure_f6(out_dir, rows, manifest):
    plt = _plt()
    order = arm_order(manifest)
    pooled = dict((arm, []) for arm in order)
    for r in _main_rows(rows):
        if r["mode_base"] != "point" and r["speed_ratio"] not in HOLDABLE_RATIOS:
            continue
        directory = _bundle(out_dir, manifest, r["cell_id"])
        if directory is None:
            continue
        path = os.path.join(directory, "ticks.csv")
        if not os.path.isfile(path):
            continue
        with open(path, newline="") as handle:
            for tick in csv.DictReader(handle):
                if tick["post_contact"] == "True" and tick["ring_error_target_m"]:
                    pooled[r["arm"]].append(float(tick["ring_error_target_m"]))
    fig, ax = plt.subplots(1, 1, figsize=(6.0, 3.0))
    data = [pooled[a] for a in order if pooled[a]]
    labels = ["arm %s\n(n=%d)" % (a, len(pooled[a])) for a in order if pooled[a]]
    colours = [arm_colour(a) for a in order if pooled[a]]
    parts = ax.violinplot(data, showmedians=True, showextrema=True)
    for body, colour in zip(parts["bodies"], colours):
        body.set_facecolor(colour)
        body.set_alpha(0.6)
    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.axhline(0.0, color="#6B7A85", linewidth=0.6)
    ax.set_ylabel("ring error vs target (m), post-contact ticks")
    ax.set_title("Pooled over holdable cells (ratios 0.25, 0.5 and point); "
                 "post-contact ticks only", fontsize=8)
    ax.grid(axis="y", color="#E3E9ED", linewidth=0.6)
    fig.tight_layout()
    return [("F6", _save(fig, out_dir, "F6", "per-tick-error-distribution"))]


# --------------------------------------------------------------------------
# F7 — tangent registration error with its coverage
# --------------------------------------------------------------------------

def figure_f7(out_dir, rows, manifest):
    plt = _plt()
    fig, ax = plt.subplots(1, 1, figsize=(6.0, 3.2))
    for arm in arm_order(manifest):
        cells = [r for r in _cells(_main_rows(rows), arm=arm)
                 if r["rms_e_tan_m"] is not None and r["e_tan_coverage"] is not None]
        ax.scatter([c["e_tan_coverage"] for c in cells],
                   [c["rms_e_tan_m"] for c in cells], s=12,
                   color=arm_colour(arm), alpha=0.7, label=arm_label(arm),
                   edgecolors="none")
    ax.set_xlabel("e_tan coverage (fraction of ticks where defined)")
    ax.set_ylabel("RMS e_tan (m)")
    ax.set_title("One point per cell; two arms at equal RMS but different "
                 "coverage are not equal", fontsize=8)
    ax.grid(color="#E3E9ED", linewidth=0.6)
    ax.legend(frameon=False)
    fig.tight_layout()
    return [("F7", _save(fig, out_dir, "F7", "e-tan-vs-coverage"))]


# --------------------------------------------------------------------------
# F8 — selected horizon and velocity-estimate error under elastic motion
# --------------------------------------------------------------------------

def figure_f8(out_dir, rows, manifest, cell_id=None):
    plt = _plt()
    cell_id = cell_id or _cid(manifest, "B", "straight-elastic-half")
    directory = _bundle(out_dir, manifest, cell_id)
    if directory is None:
        raise ValueError("F8: no bundle for %s" % cell_id)
    series = _load_series(directory)
    panels = (("target_speed_ms", "target speed (m/s)"),
              ("target_velocity_estimate_error_ms",
               "velocity-estimate\nerror (m/s)"),
              ("selected_horizon_s", "selected horizon (s)"))
    fig, axes = plt.subplots(len(panels), 1, figsize=(7.0, 5.0), sharex=True)
    for ax, (name, label) in zip(axes, panels):
        s = series.get(name)
        if s is None:
            ax.text(0.5, 0.5, "%s: not produced by this cell" % name,
                    transform=ax.transAxes, ha="center")
            continue
        xs, ys = [], []
        for t, v in zip(s["t_s"], s["values"]):
            if v is None:
                if xs:
                    ax.plot(xs, ys, color="#1F6FEB", linewidth=1.0)
                xs, ys = [], []
            else:
                xs.append(t)
                ys.append(v)
        ax.plot(xs, ys, color="#1F6FEB", linewidth=1.0)
        ax.set_ylabel(label)
        ax.grid(color="#E3E9ED", linewidth=0.6)
        ax.set_title("%s  (coverage %.2f)" % (name, s["coverage"]), fontsize=7,
                     loc="right")
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("%s: the estimator lags each ramp and the selected horizon "
                 "moves with it" % cell_id, fontsize=8)
    fig.tight_layout()
    return [("F8", _save(fig, out_dir, "F8", "horizon-under-elastic"))]


# --------------------------------------------------------------------------
# F9 — cell status map
# --------------------------------------------------------------------------

STATUS_CATEGORIES = (
    ("settled", "#2E7D32"),
    ("contact, unsettled", "#F9A825"),
    ("no contact", "#B0BEC5"),
    ("stopped early", "#C62828"),
    ("not run", "#FFFFFF"),
)


def _status_category(row):
    if row["status"] != "complete" and row["status"] != "curvature_breach":
        return "not run"
    if row["partial"]:
        return "stopped early"
    if row["t_contact_s"] is None:
        return "no contact"
    if row["settled"]:
        return "settled"
    return "contact, unsettled"


def figure_f9(out_dir, rows, manifest):
    plt = _plt()
    rows = _main_rows(rows)
    ratios = [r[1] for r in manifest["subs"]["main"]["speed_ratios"]]
    seeds = manifest["subs"]["main"]["seeds"]
    # Columns: point, then for each moving permutation x ratio (rand x seeds).
    columns = [("point", "constant", None, None)]
    for base, pace in MODE_ORDER[1:]:
        for ratio in ratios:
            if base == "rand":
                for seed in seeds:
                    columns.append((base, pace, ratio, seed))
            else:
                columns.append((base, pace, ratio, None))
    index = {}
    for r in rows:
        index[(r["mode_base"], r["mode_pace"], r["speed_ratio"],
               None if r["seed"] is None else int(r["seed"]), r["arm"])] = r
    colours = dict(STATUS_CATEGORIES)
    fig, ax = plt.subplots(1, 1, figsize=(10.0, 2.2))
    breaches = 0
    order = arm_order(manifest)
    for c_i, (base, pace, ratio, seed) in enumerate(columns):
        for a_i, arm in enumerate(order):
            row = index.get((base, pace, ratio, seed, arm))
            cat = _status_category(row) if row else "not run"
            ax.add_patch(plt.Rectangle((c_i, a_i), 1, 1, facecolor=colours[cat],
                                       edgecolor="white", linewidth=0.3))
            if row and (row.get("zone_plane_breaches") or 0) > 0:
                ax.plot(c_i + 0.5, a_i + 0.5, marker="x", color="black",
                        markersize=3, linestyle="none")
                breaches += 1
    ax.set_xlim(0, len(columns))
    ax.set_ylim(0, len(order))
    ax.set_yticks([i + 0.5 for i in range(len(order))])
    ax.set_yticklabels(["arm %s" % a for a in order])
    # Group labels along x.
    ticks, labels = [], []
    start = 0
    for base, pace in MODE_ORDER:
        n = (1 if base == "point" else
             len(ratios) * (len(seeds) if base == "rand" else 1))
        ticks.append(start + n / 2.0)
        labels.append(MODE_LABELS[(base, pace)].replace("\n", " "))
        ax.axvline(start, color="black", linewidth=0.5)
        start += n
    ax.set_xticks(ticks)
    ax.set_xticklabels(labels, fontsize=6)
    ax.set_title("Cell status, %d cells; within each group ratios run %s "
                 "left to right (rand: five seeds per ratio); x = aircraft "
                 "zone breach (%d)" % (len(rows), ", ".join("%g" % r for r in ratios),
                                       breaches), fontsize=7)
    from matplotlib.patches import Patch
    ax.legend([Patch(facecolor=c, edgecolor="#999999") for _n, c in STATUS_CATEGORIES],
              [n for n, _c in STATUS_CATEGORIES], loc="upper center",
              bbox_to_anchor=(0.5, -0.25), ncol=5, frameon=False)
    ax.invert_yaxis()
    return [("F9", _save(fig, out_dir, "F9", "cell-status-map"))]


# --------------------------------------------------------------------------
# F10 — chord-cutting against resolution (S1)
# --------------------------------------------------------------------------

def figure_f10(out_dir, rows, manifest):
    plt = _plt()
    s1 = [r for r in rows if r["sub"] == "chord" and r["s1_flown_radius_m"] is not None]
    if not s1:
        raise ValueError("F10: no completed S1 cells in master.csv")
    R = manifest["fixed"]["orbit_radius_m"]
    rho = manifest["fixed"]["turn_radius_m"]
    rates = sorted(set(r["s1_rate_hz"] for r in s1))
    looks = sorted(set(r["s1_look_ahead_m"] for r in s1))
    rate_colours = dict(zip(rates, plt.get_cmap("viridis")(
        [i / max(1, len(rates) - 1) for i in range(len(rates))])))
    made = []
    for target in ("point", "straight"):
        fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.0))
        for ax, precomp in zip(axes[:2], (False, True)):
            for rate in rates:
                xs, ys = [], []
                for look in looks:
                    cells = [r for r in s1 if r["s1_target"] == target
                             and r["s1_rate_hz"] == rate and r["s1_look_ahead_m"] == look
                             and r["s1_precompensate"] is precomp
                             and r["s1_sampling"] == "dflt"]
                    v = _mean([c["s1_flown_radius_m"] for c in cells])
                    if v is not None:
                        xs.append(look)
                        ys.append(v)
                ax.plot(xs, ys, marker="o", markersize=3, linewidth=1.0,
                        color=rate_colours[rate], label="%g Hz" % rate)
            fine_x = [looks[0] + (looks[-1] - looks[0]) * i / 60.0 for i in range(61)]
            ax.plot(fine_x, [R * math.cos(L / R) for L in fine_x], color="black",
                    linestyle="--", linewidth=0.9, label="R cos(L/R)")
            ax.axhline(R, color="#6B7A85", linewidth=0.7, linestyle=":")
            ax.axhline(rho, color="#9E2F27", linewidth=0.7, linestyle=":",
                       label="turn radius rho")
            ax.set_xlabel("look-ahead L (m)")
            ax.set_ylabel("flown radius, post-contact mean (m)")
            ax.set_title("pre-compensation %s" % ("ON" if precomp else "OFF"),
                         fontsize=8)
            ax.grid(color="#E3E9ED", linewidth=0.6)
        axes[0].legend(frameon=False, fontsize=6)
        # Residual against rate, OFF, per look-ahead.
        ax = axes[2]
        look_colours = dict(zip(looks, plt.get_cmap("plasma")(
            [i / max(1, len(looks) - 1) for i in range(len(looks))])))
        for look in looks:
            xs, ys = [], []
            for rate in rates:
                cells = [r for r in s1 if r["s1_target"] == target
                         and r["s1_rate_hz"] == rate and r["s1_look_ahead_m"] == look
                         and r["s1_precompensate"] is False
                         and r["s1_sampling"] == "dflt"]
                v = _mean([c["s1_residual_m"] for c in cells])
                if v is not None:
                    xs.append(rate)
                    ys.append(v)
            ax.plot(xs, ys, marker="o", markersize=3, linewidth=1.0,
                    color=look_colours[look], label="L = %g m" % look)
        ax.set_xscale("log")
        ax.set_xlabel("control rate (Hz)")
        ax.set_ylabel("residual above R cos(L/R) (m), OFF")
        ax.set_title("what the rate buys", fontsize=8)
        ax.grid(color="#E3E9ED", linewidth=0.6, which="both")
        ax.legend(frameon=False, fontsize=6)
        fig.suptitle("S1, %s target, default path sampling (the 4x-finer axis "
                     "is in master.csv)" % target, fontsize=8)
        fig.tight_layout()
        made.append(("F10", _save(fig, out_dir, "F10",
                                  "chord-cutting-vs-resolution-%s" % target)))
    return made


# --------------------------------------------------------------------------
# F11 — single-cell path renders from history.json
# --------------------------------------------------------------------------

def _fit_extent(history, pad_m=40.0):
    ns = [s["plane_n_m"] for s in history] + [s["target_n_m"] for s in history]
    es = [s["plane_e_m"] for s in history] + [s["target_e_m"] for s in history]
    lo_n, hi_n, lo_e, hi_e = min(ns), max(ns), min(es), max(es)
    span = max(hi_n - lo_n, hi_e - lo_e) + 2 * pad_m
    cn, ce = (lo_n + hi_n) / 2.0, (lo_e + hi_e) / 2.0
    return (ce - span / 2.0, ce + span / 2.0), (cn - span / 2.0, cn + span / 2.0)


def first_containment_turn_s(record):
    """Time of the zone's first containment turn in a cell, or ``None``."""
    times = [c["t_s"] for c in (record.get("kangaroo") or {}).get("changes") or []
             if c.get("source") == "zone"]
    return min(times) if times else None


def _kangaroo_phrase(cell):
    """``"kangaroo travelling <mode>"`` for a render title."""
    base = cell.get("mode_base")
    pace = cell.get("mode_pace")
    if base == "point":
        return "a stationary kangaroo"
    if base == "composite":
        return "the composite kangaroo"
    mode = base if base != "rand" else "kangaroo_rand"
    if pace == "elastic":
        mode = "elastic " + mode
    return "kangaroo travelling %s" % mode


def render_title(cell):
    """``Arm <X> following kangaroo travelling <mode>``, by the arm's base
    letter: a hysteresis counterpart (``AH``) is titled as its parent (author's
    direction, 2026-09-15)."""
    arm = str(cell.get("arm") or "?")
    return "Arm %s following %s" % (arm[0], _kangaroo_phrase(cell))


def render_cell(out_dir, manifest, cell_id, stem, name="F11"):
    """One cell's scene from its ``history.json``: flown track, kangaroo
    track, the ring and, for a predicting arm, the held centre. Fitted extent
    (the campaign's ``view.png`` is drawn at the 2 km zone extent, which is
    unreadable at thesis width).

    Drawn **up to the zone's first containment turn** when there is one: the
    straight kangaroo at ratio 0.5 is turned back at 50.4 s in the 2 km zone
    (`ISSUE-M9`), and the bounce misconstrues the final geometry the figure
    is meant to show. The cut is recorded in the title's duration.
    """
    plt = _plt()
    directory = _bundle(out_dir, manifest, cell_id)
    if directory is None:
        raise ValueError("%s: no bundle for %s" % (name, cell_id))
    history = _load_history(directory)
    record = _load_json(directory, "record.json")
    spec = _load_json(directory, "spec.json")
    R = record["config"]["orbit_radius_m"]
    cell = record.get("cell") or {}
    arm = cell.get("arm")
    cut_s = first_containment_turn_s(record)
    if cut_s is not None:
        # The turn is applied at t and governs from the next step, so the
        # sample at t itself is still on the un-turned path.
        history = [h for h in history if h["t_s"] <= cut_s + 1e-9]
    markers = [t for t in _marker_times(spec, record)
               if cut_s is None or t < cut_s - 1e-9]
    fig, ax = plt.subplots(1, 1, figsize=(4.2, 4.2))
    ax.grid(color="#E3E9ED", linewidth=0.6)
    ax.set_axisbelow(True)
    plotter.draw_scene(ax, history, orbit_radius_m=R, markers=markers,
                       track_colour=arm_colour(arm) if arm else "#1F6FEB")
    # The held centre, where the arm reports one (arms A, B, C, D).
    cn = [(s.get("algorithm_state") or {}).get("centre_n_m") for s in history]
    ce = [(s.get("algorithm_state") or {}).get("centre_e_m") for s in history]
    pts = [(e, n) for n, e in zip(cn, ce) if n is not None and e is not None]
    if pts:
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#C2571C",
                linewidth=0.7, linestyle=(0, (2, 2)), label="held centre")
    ax.plot([history[0]["plane_e_m"]], [history[0]["plane_n_m"]], marker="s",
            markersize=4, color=arm_colour(arm) if arm else "#1F6FEB", linestyle="none")
    xlim, ylim = _fit_extent(history)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    shown_s = history[-1]["t_s"] if history else 0.0
    ax.set_title("%s\n%s, %.1f s%s" % (
        render_title(cell), cell_id, shown_s,
        " (to the zone's containment turn)" if cut_s is not None else ""),
        fontsize=8)
    ax.legend(loc="best", frameon=False, fontsize=6)
    return _save(fig, out_dir, name, stem)


def figure_f11(out_dir, rows, manifest):
    return [("F11", render_cell(out_dir, manifest, _cid(manifest, letter, suffix), stem))
            for stem, letter, suffix in F11_CELLS]


# --------------------------------------------------------------------------
# F13 — the five kangaroo modes on one figure
# --------------------------------------------------------------------------

F13_CELLS = (("point", None), ("straight", "straight-constant-half"),
             ("circle", "circle-constant-half"),
             ("rectangle", "rectangle-constant-half"),
             ("kangaroo_rand (seed 1)", "rand-constant-half-s01"))
F13_COLOURS = ("#4D4D4D", "#0072B2", "#009E73", "#E69F00", "#CC79A7")


def figure_f13(out_dir, rows, manifest):
    plt = _plt()
    fig, ax = plt.subplots(1, 1, figsize=(4.8, 4.8))
    for (label, suffix), colour in zip(F13_CELLS, F13_COLOURS):
        cell_id = _cid(manifest, "0", suffix)
        directory = _bundle(out_dir, manifest, cell_id)
        if directory is None:
            raise ValueError("F13: no bundle for %s" % cell_id)
        history = _load_history(directory)
        es = [s["target_e_m"] for s in history]
        ns = [s["target_n_m"] for s in history]
        if label == "point":
            ax.plot(es[:1], ns[:1], marker="*", markersize=12, color=colour,
                    linestyle="none", label=label)
        else:
            ax.plot(es, ns, color=colour, linewidth=1.2, label=label)
            ax.plot(es[-1:], ns[-1:], marker="o", markersize=4, color=colour,
                    linestyle="none")
    ic = manifest["initial_conditions"]
    ax.plot([ic["target_e_m"]], [ic["target_n_m"]], marker="+", markersize=10,
            color="black", linestyle="none", label="common start (%g, %g)"
            % (ic["target_n_m"], ic["target_e_m"]))
    ax.set_aspect("equal")
    ax.grid(color="#E3E9ED", linewidth=0.6)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Kangaroo tracks, 60 s at ratio 0.5, every mode from the same "
                 "start", fontsize=8)
    ax.legend(frameon=False, fontsize=6, loc="upper center",
              bbox_to_anchor=(0.5, -0.1), ncol=2)
    fig.tight_layout()
    return [("F13", _save(fig, out_dir, "F13", "kangaroo-modes"))]


# --------------------------------------------------------------------------
# F20 — the composite kangaroo in the box, every arm over it (TASK-050)
# --------------------------------------------------------------------------
# Drawn from a composite sub-experiment directory (`<campaign>/composite-box350`
# or `<campaign>/composite`), whose manifest holds one cell per arm per ratio.
# Colour follows the MODE FAMILY (five hues, fixed order, the F13 set); the
# elastic variant of a family is the same hue dashed, so the nine phases need
# no ninth hue and a colour-blind reader still separates pace by line style.

F20_RATIO = 0.5
F20_FAMILY_COLOURS = {"point": "#4D4D4D", "straight": "#0072B2",
                      "circle": "#009E73", "rectangle": "#E69F00",
                      "rand": "#CC79A7"}


def _phase_family(phase):
    if phase.startswith("point"):
        return "point"
    if phase == "rand":
        return "rand"
    return phase.replace("elastic-", "")


def _f20_phase_runs(ticks, spec):
    """``[(phase, [(e, n), ...]), ...]`` of the kangaroo track from
    ``ticks.csv``'s ``mode_leg`` and the spec's recorded phases."""
    phases = spec["kangaroo"]["composite_fit"]["phases"]
    runs = []
    for row in ticks:
        index = int(row["mode_leg"].split(":")[0])
        phase = phases[index] if index < len(phases) else phases[-1]
        point = (float(row["target_e_m"]), float(row["target_n_m"]))
        if runs and runs[-1][0] == phase:
            runs[-1][1].append(point)
        else:
            # Repeat the previous point so the runs join without a gap.
            start = [runs[-1][1][-1]] if runs else []
            runs.append((phase, start + [point]))
    return runs


def figure_f20(out_dir, rows, manifest, ratio=F20_RATIO):
    """Left: the composite kangaroo track coloured by mode family (elastic
    dashed), the zone, its containment inset and the fit's usable region.
    Right: the same track in grey with every arm's flown track over it."""
    plt = _plt()
    sub = [r for r in rows if r["mode_base"] == "composite"
           and r["speed_ratio"] == ratio and r["status"] == "complete"]
    if not sub:
        raise ValueError("F20: no complete composite cell at ratio %g in %s"
                         % (ratio, out_dir))
    by_arm = dict((r["arm"], r) for r in sub)
    arms = [a for a in arm_order(manifest) if a in by_arm]
    first = _bundle(out_dir, manifest, by_arm[arms[0]]["cell_id"])
    spec = _load_json(first, "spec.json")
    record = _load_json(first, "record.json")
    fit = spec["kangaroo"]["composite_fit"]
    side = spec["zone"]["side_m"]
    R = record["config"]["orbit_radius_m"]
    with open(os.path.join(first, "ticks.csv"), newline="") as handle:
        ticks = list(csv.DictReader(handle))

    fig, (ax_k, ax_a) = plt.subplots(1, 2, figsize=(8.4, 4.4))
    for ax in (ax_k, ax_a):
        for half, colour, style, label in (
                (side / 2.0, "#9E2F27", (0, (7, 4)), "zone %.0f m" % side),
                (side / 2.0 - R, "#9E2F27", (0, (2, 2)),
                 "containment inset (R = %.0f m)" % R),
                (fit["half_m"], "#6B7A85", (0, (1, 2)),
                 "usable region (margin %.0f m)" % fit["margin_m"])):
            xs = [-half, half, half, -half, -half]
            ys = [-half, -half, half, half, -half]
            # The boundary lines are labelled once, on the left panel.
            ax.plot(xs, ys, color=colour, linewidth=0.9, linestyle=style,
                    label=label if ax is ax_k else None)
        ax.set_aspect("equal")
        ax.grid(color="#E3E9ED", linewidth=0.6)
        ax.set_axisbelow(True)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        pad = side / 2.0 + 25.0
        ax.set_xlim(-pad, pad)
        ax.set_ylim(-pad, pad)

    seen = set()
    for phase, pts in _f20_phase_runs(ticks, spec):
        family = _phase_family(phase)
        colour = F20_FAMILY_COLOURS[family]
        dashed = phase.startswith("elastic")
        label = None
        key = (family, dashed)
        if key not in seen:
            seen.add(key)
            label = family + (" (elastic)" if dashed else "")
        if family == "point":
            ax_k.plot([pts[-1][0]], [pts[-1][1]], marker="*", markersize=9,
                      color=colour, linestyle="none", label=label)
        else:
            ax_k.plot([p[0] for p in pts], [p[1] for p in pts], color=colour,
                      linewidth=1.3, linestyle=(0, (3, 2)) if dashed else "-",
                      label=label)
    ax_k.plot([0.0], [0.0], marker="s", markersize=4, color="black",
              linestyle="none", label="aircraft start")
    ax_k.set_title("Composite kangaroo, ratio %g: start %.0f m, circle r %.1f m, "
                   "rectangle %.0f x %.1f m,\nrand seed %d, %.0f s"
                   % (ratio, fit["start_range_m"], fit["radius_m"],
                      fit["length_m"], fit["width_m"], fit["rand_seed"],
                      fit["duration_s"]), fontsize=8)
    ax_k.legend(frameon=False, fontsize=6, loc="upper center",
                bbox_to_anchor=(0.5, -0.12), ncol=3)

    es = [float(r["target_e_m"]) for r in ticks]
    ns = [float(r["target_n_m"]) for r in ticks]
    ax_a.plot(es, ns, color="#B0B7BC", linewidth=1.0, label="kangaroo")
    turns = []
    for arm in arms:
        directory = _bundle(out_dir, manifest, by_arm[arm]["cell_id"])
        history = _load_history(directory)
        ax_a.plot([h["plane_e_m"] for h in history],
                  [h["plane_n_m"] for h in history], color=arm_colour(arm),
                  linewidth=0.7, alpha=0.9, label=arm_label(arm))
        turns.append(by_arm[arm]["zone_containment_turns"])
    ax_a.set_title("Every arm's flown track over it (containment turns: %s)"
                   % ", ".join("%g" % (t or 0) for t in turns), fontsize=8)
    ax_a.legend(frameon=False, fontsize=6, loc="upper center",
                bbox_to_anchor=(0.5, -0.12), ncol=3)
    fig.tight_layout()
    return [("F20", _save(fig, out_dir, "F20", "composite-in-box"))]


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------

FIGURES = {
    "F1": figure_f1, "F2": figure_f2, "F3": figure_f3, "F4": figure_f4,
    "F5": figure_f5, "F6": figure_f6, "F7": figure_f7, "F8": figure_f8,
    "F9": figure_f9, "F10": figure_f10, "F11": figure_f11, "F12": figure_f12,
    "F13": figure_f13, "F20": figure_f20,
}


def make_figures(out_dir, names=None):
    """Produce the named figures (default: all) and return ``[(name, path)]``.

    A figure whose inputs are missing (an S1 figure before S1 has run) raises
    :class:`ValueError` naming what is missing; nothing is re-run.
    """
    rows = load_master(out_dir)
    manifest = load_manifest(out_dir)
    made = []
    if names is None:
        names = list(FIGURE_NAMES)
        # F10 is the S1 figure; a campaign without S1 (CAMP-003) has none to
        # draw, and "all figures" should mean all that exist for it.
        if not any(r["sub"] == "chord" for r in rows):
            names.remove("F10")
        # F20 is the composite figure (TASK-050) and lives in a composite
        # sub-experiment directory; a grid directory has no composite cells,
        # and a composite directory has only those.
        if any(r["mode_base"] == "composite" for r in rows):
            names = ["F20"]
        else:
            names.remove("F20")
    for name in names:
        if name not in FIGURES:
            raise ValueError("unknown figure %r; choose from %s"
                             % (name, ", ".join(FIGURE_NAMES)))
        made.extend(FIGURES[name](out_dir, rows, manifest))
    return made
