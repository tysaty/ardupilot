#!/usr/bin/env python3
"""
Plot DubinsParamSweep results from arduplane.py autotest.

Usage:
    python3 scripts/plot_sweep2.py                              # auto-picks latest CSV
    python3 scripts/plot_sweep2.py buildlogs/dubins_sweep_XYZ.csv
    python3 scripts/plot_sweep2.py --out /tmp/plots/

Requirements:
    pip install matplotlib numpy
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import matplotlib.ticker
except ImportError:
    sys.exit("matplotlib not installed: pip3 install matplotlib")

PARAMS = ["CTRL_REBUILD_MS", "CTRL_STREAK", "CTRL_DUB_DIST", "CTRL_DUB_VEL"]
PARAM_LABELS = {
    "CTRL_REBUILD_MS": "Rebuild interval (ms)",
    "CTRL_STREAK":     "Streak count",
    "CTRL_DUB_DIST":   "Dubins distance (m)",
    "CTRL_DUB_VEL":    "Dubins velocity (m/s)",
}
KANG_MODES = ["random", "straight", "circle", "rectangle"]
KANG_COLOURS = {m: c for m, c in zip(KANG_MODES, plt.cm.tab10.colors)}


# ── Data loading ──────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if "kang_mode" not in (reader.fieldnames or []):
            sys.exit(
                f"ERROR: '{path}' looks like an old-format sweep CSV (no kang_mode column).\n"
                "Use scripts/plot_sweep.py for old-format files."
            )
        for r in reader:
            if r.get("l1_error_m") in ("", "None", None) or r.get("l2_error_m") in ("", "None", None):
                continue
            try:
                rows.append({
                    "trial":           int(r["trial"]),
                    "name":            r["name"],
                    "kang_mode":       r["kang_mode"],
                    "CTRL_REBUILD_MS": int(float(r["CTRL_REBUILD_MS"])),
                    "CTRL_STREAK":     int(float(r["CTRL_STREAK"])),
                    "CTRL_DUB_DIST":   int(float(r["CTRL_DUB_DIST"])),
                    "CTRL_DUB_VEL":    int(float(r["CTRL_DUB_VEL"])),
                    "l2":              float(r["l2_error_m"]),
                })
            except (ValueError, KeyError):
                continue
    return rows


def find_latest_csv():
    candidates = sorted(glob.glob("buildlogs/dubins_sweep_*.csv"))
    if not candidates:
        candidates = sorted(glob.glob("sweep_results_*.csv"))
    if not candidates:
        sys.exit("No dubins_sweep_*.csv found. Run from the repo root or pass a CSV path.")
    return candidates[-1]


# ── Plot 1: kang-mode comparison — violin + strip ─────────────────────────────

def plot_kang_mode_comparison(rows, out_dir, prefix):
    fig, ax = plt.subplots(figsize=(9, 6))
    fig.suptitle("Kangaroo-mode comparison — L2 tracking error distribution", fontsize=13)

    data_by_mode = {m: [r["l2"] for r in rows if r["kang_mode"] == m] for m in KANG_MODES}
    present = [m for m in KANG_MODES if data_by_mode[m]]

    positions = list(range(len(present)))
    parts = ax.violinplot(
        [data_by_mode[m] for m in present],
        positions=positions,
        showmedians=True,
        showextrema=True,
    )
    for body, mode in zip(parts["bodies"], present):
        body.set_facecolor(KANG_COLOURS[mode])
        body.set_alpha(0.6)
    parts["cmedians"].set_color("black")
    parts["cmins"].set_color("grey")
    parts["cmaxes"].set_color("grey")
    parts["cbars"].set_color("grey")

    rng = np.random.default_rng(42)
    for i, mode in enumerate(present):
        vals = data_by_mode[mode]
        jitter = rng.uniform(-0.12, 0.12, len(vals))
        ax.scatter(np.full(len(vals), i) + jitter, vals,
                   color=KANG_COLOURS[mode], s=20, alpha=0.45, edgecolors="none", zorder=3)
        best = min(vals)
        ax.annotate(f"min={best:.1f}m", xy=(i, best),
                    xytext=(i + 0.18, best - 2), fontsize=7.5)

    ax.set_xticks(positions)
    ax.set_xticklabels([f"KANG_{m.upper()}" for m in present], fontsize=10)
    ax.set_ylabel("Mean L2 error (m)")
    ax.set_xlabel("Kangaroo mode")

    path = os.path.join(out_dir, f"{prefix}_plot1_kang_mode_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 2: sorted bars per kang_mode (2×2) ───────────────────────────────────

def _abbrev_name(name):
    return (name
            .replace("rebuild_ms", "r")
            .replace("_streak", " s")
            .replace("_dub_dist", " d")
            .replace("_dub_vel", " v"))


def plot_sorted_bars_per_mode(rows, out_dir, prefix):
    rebuild_vals = sorted({r["CTRL_REBUILD_MS"] for r in rows})
    cmap = matplotlib.colormaps.get_cmap("RdYlGn_r").resampled(len(rebuild_vals))
    colour_map = {v: cmap(i) for i, v in enumerate(rebuild_vals)}

    fig = plt.figure(figsize=(20, 14))
    fig.suptitle("Parameter combos sorted by L2 error — per kangaroo mode", fontsize=14, y=0.995)
    subfigs = fig.subfigures(2, 2, hspace=0.15, wspace=0.08)

    for subfig, mode in zip(subfigs.flat, KANG_MODES):
        subfig.suptitle(f"KANG_{mode.upper()}", fontsize=12, fontweight="bold",
                        ha="left", x=0.02, color="black")
        ax = subfig.subplots(1, 1)
        subset = sorted([r for r in rows if r["kang_mode"] == mode], key=lambda r: r["l2"])
        if not subset:
            ax.set_title("(no data)")
            continue
        l2s     = [r["l2"] for r in subset]
        colours = [colour_map[r["CTRL_REBUILD_MS"]] for r in subset]

        ax.bar(range(len(subset)), l2s, color=colours, edgecolor="white", linewidth=0.3)
        ax.set_xticks(range(len(subset)))
        ax.set_xticklabels([_abbrev_name(r["name"]) for r in subset],
                           rotation=75, ha="right", fontsize=5.5)
        ax.set_ylabel("L2 error (m)")
        ax.annotate(
            f"Best: {_abbrev_name(subset[0]['name'])}\nL2={l2s[0]:.1f}m",
            xy=(0, l2s[0]), xytext=(2, l2s[0] + 2),
            fontsize=7.5,
        )

    legend_patches = [
        mpatches.Patch(color=colour_map[v], label=f"REBUILD_MS={v}")
        for v in rebuild_vals
    ]
    fig.legend(handles=legend_patches, title="CTRL_REBUILD_MS",
               loc="lower center", ncol=len(rebuild_vals), fontsize=9, bbox_to_anchor=(0.5, 0.01))

    path = os.path.join(out_dir, f"{prefix}_plot2_sorted_bars_per_mode.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 3: marginal effects grid (4 modes × 4 params) ───────────────────────

def plot_marginal_effects_grid(rows, out_dir, prefix):
    fig = plt.figure(figsize=(20, 22))
    fig.suptitle("Marginal effect of each parameter on L2 error — per kangaroo mode\n(mean ± std)",
                 fontsize=13, y=0.995)
    subfigs = fig.subfigures(4, 1, hspace=0.12)

    for subfig, mode in zip(subfigs, KANG_MODES):
        subfig.suptitle(f"KANG_{mode.upper()}", fontsize=12, fontweight="bold",
                        ha="left", x=0.02, color="black")
        axes = subfig.subplots(1, 4)
        subfig.subplots_adjust(wspace=0.5)
        subset = [r for r in rows if r["kang_mode"] == mode]

        for col_i, (ax, param) in enumerate(zip(axes, PARAMS)):
            if not subset:
                ax.set_visible(False)
                continue
            levels = sorted({r[param] for r in subset})
            means  = [np.mean([r["l2"] for r in subset if r[param] == lv]) for lv in levels]
            stds   = [np.std( [r["l2"] for r in subset if r[param] == lv]) for lv in levels]
            colours = ["#4daf4a" if m == min(means) else "#e41a1c" if m == max(means) else "#377eb8"
                       for m in means]
            ax.bar([str(lv) for lv in levels], means, yerr=stds, capsize=4,
                   color=colours, edgecolor="white", linewidth=0.5)
            ax.set_title(PARAM_LABELS[param], fontsize=9)
            if col_i == 0:
                ax.set_ylabel("Mean L2 error (m)", fontsize=8)
            ax.tick_params(axis="x", labelsize=8)
            ax.tick_params(axis="y", labelsize=7)

    path = os.path.join(out_dir, f"{prefix}_plot3_marginal_effects_grid.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 4: heatmaps per kang_mode ────────────────────────────────────────────

def plot_heatmaps_per_mode(rows, out_dir, prefix):
    other_params = ["CTRL_STREAK", "CTRL_DUB_DIST", "CTRL_DUB_VEL"]
    rebuild_vals = sorted({r["CTRL_REBUILD_MS"] for r in rows})
    kang_label = {m: f"KANG_{m.upper()}" for m in KANG_MODES}

    fig = plt.figure(figsize=(20, 22))
    fig.suptitle("Interaction heatmaps — mean L2 error (m)\nCTRL_REBUILD_MS × each other param",
                 fontsize=13, y=0.995)
    subfigs = fig.subfigures(4, 1, hspace=0.12)

    for subfig, mode in zip(subfigs, KANG_MODES):
        subfig.suptitle(kang_label[mode], fontsize=12, fontweight="bold",
                        ha="left", x=0.02, color="black")
        axes = subfig.subplots(1, 3)
        subfig.subplots_adjust(wspace=0.5)
        subset = [r for r in rows if r["kang_mode"] == mode]

        for col_i, (ax, other) in enumerate(zip(axes, other_params)):
            if not subset:
                ax.set_visible(False)
                continue
            other_vals = sorted({r[other] for r in subset})
            grid = np.zeros((len(rebuild_vals), len(other_vals)))

            for i, rv in enumerate(rebuild_vals):
                for j, ov in enumerate(other_vals):
                    vals = [r["l2"] for r in subset if r["CTRL_REBUILD_MS"] == rv and r[other] == ov]
                    grid[i, j] = np.mean(vals) if vals else np.nan

            valid = grid[~np.isnan(grid)]
            im = ax.imshow(grid, cmap="RdYlGn_r", aspect="auto",
                           vmin=valid.min() if valid.size else 0,
                           vmax=valid.max() if valid.size else 1)
            subfig.colorbar(im, ax=ax, label="L2 (m)", fraction=0.046, pad=0.04)

            ax.set_xticks(range(len(other_vals)))
            ax.set_xticklabels([str(v) for v in other_vals], fontsize=8)
            ax.set_yticks(range(len(rebuild_vals)))
            ax.set_yticklabels([str(v) for v in rebuild_vals], fontsize=8)
            ax.set_title(f"REBUILD_MS × {other.replace('CTRL_', '')}", fontsize=9)
            ax.set_xlabel(PARAM_LABELS[other], fontsize=8)
            if col_i == 0:
                ax.set_ylabel("CTRL_REBUILD_MS", fontsize=8)

            for i in range(len(rebuild_vals)):
                for j in range(len(other_vals)):
                    if not np.isnan(grid[i, j]):
                        ax.text(j, i, f"{grid[i, j]:.3f}", ha="center", va="center",
                                fontsize=7, color="black")

    path = os.path.join(out_dir, f"{prefix}_plot4_heatmaps_per_mode.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 5: ranked L2 error across all trials, coloured by kang_mode ─────────

def plot_ranked_l2(rows, out_dir, prefix):
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.suptitle("All trials ranked by L2 error — coloured by kangaroo mode", fontsize=13)

    sorted_rows = sorted(rows, key=lambda r: r["l2"])
    for rank, r in enumerate(sorted_rows):
        ax.scatter(rank, r["l2"], color=KANG_COLOURS[r["kang_mode"]],
                   s=30, alpha=0.8, edgecolors="none", zorder=3)

    legend_patches = [
        mpatches.Patch(color=KANG_COLOURS[m], label=f"KANG_{m.upper()}")
        for m in KANG_MODES if any(r["kang_mode"] == m for r in rows)
    ]
    ax.legend(handles=legend_patches, fontsize=9, loc="upper left")
    ax.set_xlabel("Rank (best → worst)")
    ax.set_ylabel("Mean L2 error (m)")
    ax.yaxis.set_major_formatter(matplotlib.ticker.FormatStrFormatter("%.1f"))

    path = os.path.join(out_dir, f"{prefix}_plot5_ranked_l2.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Plot DubinsParamSweep results (kang_mode-aware)")
    ap.add_argument("csv", nargs="?", help="CSV file to plot (default: latest dubins_sweep_*.csv)")
    ap.add_argument("--out", default=None, help="Output directory for PNGs (default: same dir as CSV)")
    args = ap.parse_args()

    csv_path = args.csv or find_latest_csv()
    out_dir  = args.out or os.path.dirname(os.path.abspath(csv_path))
    prefix   = os.path.splitext(os.path.basename(csv_path))[0]

    print(f"Loading {csv_path}...")
    rows = load_csv(csv_path)
    print(f"  {len(rows)} valid trials loaded")
    for mode in KANG_MODES:
        n = sum(1 for r in rows if r["kang_mode"] == mode)
        print(f"    {mode}: {n} trials")

    print("Generating plots...")
    plot_kang_mode_comparison(rows, out_dir, prefix)
    plot_sorted_bars_per_mode(rows, out_dir, prefix)
    plot_marginal_effects_grid(rows, out_dir, prefix)
    plot_heatmaps_per_mode(rows, out_dir, prefix)
    plot_ranked_l2(rows, out_dir, prefix)

    print(f"\nDone. 5 PNGs saved to {out_dir}/")


if __name__ == "__main__":
    main()
