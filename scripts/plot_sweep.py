#!/usr/bin/env python3
"""
Plot parameter sweep results from run_sweep.py.

Usage:
    python3 scripts/plot_sweep.py                        # auto-picks latest CSV
    python3 scripts/plot_sweep.py sweep_results_XYZ.csv

Requirements:
    pip install matplotlib
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")   # no display needed — saves to file
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
except ImportError:
    sys.exit("matplotlib not installed: pip3 install matplotlib")

PARAMS = ["CTRL_REBUILD_MS", "CTRL_STREAK", "CTRL_DUB_DIST", "CTRL_DUB_VEL"]
PARAM_LABELS = {
    "CTRL_REBUILD_MS": "Rebuild interval (ms)",
    "CTRL_STREAK":     "Streak count",
    "CTRL_DUB_DIST":   "Dubins distance (m)",
    "CTRL_DUB_VEL":    "Dubins velocity (m/s)",
}


# ── Data loading ──────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r.get("l1_error_m") in ("", "None", None):
                continue
            rows.append({
                "name":           r["name"],
                "CTRL_REBUILD_MS": int(float(r["CTRL_REBUILD_MS"])),
                "CTRL_STREAK":    int(float(r["CTRL_STREAK"])),
                "CTRL_DUB_DIST":  int(float(r["CTRL_DUB_DIST"])),
                "CTRL_DUB_VEL":   int(float(r["CTRL_DUB_VEL"])),
                "l1":             float(r["l1_error_m"]),
                "l2":             float(r["l2_error_m"]),
            })
    return rows


def find_latest_csv():
    files = sorted(glob.glob("sweep_results_*.csv"))
    if not files:
        sys.exit("No sweep_results_*.csv found. Run from the repo root.")
    return files[-1]


# ── Plot 1: all combinations sorted by L1 ────────────────────────────────────

def plot_sorted_bars(rows, out_dir):
    rows_s = sorted(rows, key=lambda r: r["l1"])
    l1s    = [r["l1"] for r in rows_s]
    names  = [r["name"] for r in rows_s]

    rebuild_vals = sorted({r["CTRL_REBUILD_MS"] for r in rows})
    cmap   = cm.get_cmap("RdYlGn_r", len(rebuild_vals))
    colour_map = {v: cmap(i) for i, v in enumerate(rebuild_vals)}
    colours = [colour_map[r["CTRL_REBUILD_MS"]] for r in rows_s]

    fig, ax = plt.subplots(figsize=(14, 7))
    bars = ax.bar(range(len(rows_s)), [v / 1000 for v in l1s], color=colours, edgecolor="white", linewidth=0.4)

    ax.set_xticks(range(len(rows_s)))
    ax.set_xticklabels(
        [n.replace("rebuild_ms", "r").replace("_streak", " s").replace("_dub_dist", " d").replace("_dub_vel", " v")
         for n in names],
        rotation=75, ha="right", fontsize=6.5,
    )
    ax.set_ylabel("Cumulative L1 tracking error (km)")
    ax.set_title("All 36 grid combinations — sorted by L1 error")

    legend_patches = [
        matplotlib.patches.Patch(color=colour_map[v], label=f"REBUILD_MS={v}")
        for v in rebuild_vals
    ]
    ax.legend(handles=legend_patches, title="CTRL_REBUILD_MS", loc="upper left")

    ax.annotate(
        f"Best: {rows_s[0]['name']}\nL1={l1s[0]/1000:.1f} km",
        xy=(0, l1s[0] / 1000), xytext=(3, l1s[0] / 1000 + 2),
        fontsize=8, arrowprops=dict(arrowstyle="->", color="black"),
    )

    fig.tight_layout()
    path = os.path.join(out_dir, "plot1_sorted_bars.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 2: marginal effect of each parameter ─────────────────────────────────

def plot_marginal_effects(rows, out_dir):
    fig, axes = plt.subplots(1, 4, figsize=(16, 5))
    fig.suptitle("Marginal effect of each parameter on L1 error (mean ± std)", fontsize=13)

    for ax, param in zip(axes, PARAMS):
        levels = sorted({r[param] for r in rows})
        means  = [np.mean([r["l1"] for r in rows if r[param] == lv]) / 1000 for lv in levels]
        stds   = [np.std( [r["l1"] for r in rows if r[param] == lv]) / 1000 for lv in levels]

        colours = ["#4daf4a" if m == min(means) else "#e41a1c" if m == max(means) else "#377eb8"
                   for m in means]
        ax.bar([str(lv) for lv in levels], means, yerr=stds, capsize=5,
               color=colours, edgecolor="white", linewidth=0.5)
        ax.set_title(PARAM_LABELS[param], fontsize=10)
        ax.set_xlabel("Value")
        ax.set_ylabel("Mean L1 error (km)" if ax == axes[0] else "")
        ax.tick_params(axis="x", labelsize=9)

    fig.tight_layout()
    path = os.path.join(out_dir, "plot2_marginal_effects.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 3: heatmaps — REBUILD_MS × each other param ─────────────────────────

def plot_heatmaps(rows, out_dir):
    other_params = ["CTRL_STREAK", "CTRL_DUB_DIST", "CTRL_DUB_VEL"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.suptitle("Interaction heatmaps — mean L1 error (km)\n(CTRL_REBUILD_MS vs each other parameter)", fontsize=12)

    rebuild_vals = sorted({r["CTRL_REBUILD_MS"] for r in rows})

    for ax, other in zip(axes, other_params):
        other_vals = sorted({r[other] for r in rows})
        grid = np.zeros((len(rebuild_vals), len(other_vals)))

        for i, rv in enumerate(rebuild_vals):
            for j, ov in enumerate(other_vals):
                subset = [r["l1"] for r in rows if r["CTRL_REBUILD_MS"] == rv and r[other] == ov]
                grid[i, j] = np.mean(subset) / 1000 if subset else np.nan

        im = ax.imshow(grid, cmap="RdYlGn_r", aspect="auto",
                       vmin=grid[~np.isnan(grid)].min(),
                       vmax=grid[~np.isnan(grid)].max())
        fig.colorbar(im, ax=ax, label="L1 (km)")

        ax.set_xticks(range(len(other_vals)))
        ax.set_xticklabels([str(v) for v in other_vals])
        ax.set_yticks(range(len(rebuild_vals)))
        ax.set_yticklabels([str(v) for v in rebuild_vals])
        ax.set_xlabel(PARAM_LABELS[other], fontsize=9)
        ax.set_ylabel("CTRL_REBUILD_MS" if ax == axes[0] else "")
        ax.set_title(f"REBUILD_MS × {other.replace('CTRL_', '')}", fontsize=10)

        for i in range(len(rebuild_vals)):
            for j in range(len(other_vals)):
                ax.text(j, i, f"{grid[i, j]:.1f}", ha="center", va="center",
                        fontsize=8, color="black")

    fig.tight_layout()
    path = os.path.join(out_dir, "plot3_heatmaps.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 4: L1 vs L2 scatter ──────────────────────────────────────────────────

def plot_l1_l2_scatter(rows, out_dir):
    l1s = np.array([r["l1"] for r in rows]) / 1000
    l2s = np.array([r["l2"] for r in rows]) / 1000

    rebuild_vals = sorted({r["CTRL_REBUILD_MS"] for r in rows})
    cmap = cm.get_cmap("RdYlGn_r", len(rebuild_vals))
    colour_map = {v: cmap(i) for i, v in enumerate(rebuild_vals)}

    fig, ax = plt.subplots(figsize=(7, 6))
    for r in rows:
        ax.scatter(r["l1"] / 1000, r["l2"] / 1000,
                   color=colour_map[r["CTRL_REBUILD_MS"]], s=60, alpha=0.8, edgecolors="white", linewidths=0.5)

    # Fit line
    m, b = np.polyfit(l1s, l2s, 1)
    xs = np.linspace(l1s.min(), l1s.max(), 100)
    ax.plot(xs, m * xs + b, "k--", linewidth=1, label=f"fit: L2 = {m:.2f}·L1 + {b:.1f}")

    ax.set_xlabel("L1 error (km)")
    ax.set_ylabel("L2 error (km)")
    ax.set_title("L1 vs L2 cumulative tracking error")
    ax.legend(fontsize=8)

    legend_patches = [
        matplotlib.patches.Patch(color=colour_map[v], label=f"REBUILD_MS={v}")
        for v in rebuild_vals
    ]
    ax.legend(handles=legend_patches + [
        matplotlib.lines.Line2D([0], [0], color="k", linestyle="--", label=f"fit: L2={m:.2f}·L1+{b:.1f}")
    ], fontsize=8)

    fig.tight_layout()
    path = os.path.join(out_dir, "plot4_l1_vs_l2.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Plot run_sweep.py results")
    ap.add_argument("csv", nargs="?", help="CSV file to plot (default: latest sweep_results_*.csv)")
    ap.add_argument("--out", default=None, help="Output directory for PNGs (default: same dir as CSV)")
    args = ap.parse_args()

    csv_path = args.csv or find_latest_csv()
    out_dir  = args.out or os.path.dirname(os.path.abspath(csv_path))

    print(f"Loading {csv_path}...")
    rows = load_csv(csv_path)
    print(f"  {len(rows)} trials loaded")

    print("Generating plots...")
    plot_sorted_bars(rows, out_dir)
    plot_marginal_effects(rows, out_dir)
    plot_heatmaps(rows, out_dir)
    plot_l1_l2_scatter(rows, out_dir)

    print(f"\nDone. 4 PNGs saved to {out_dir}/")


if __name__ == "__main__":
    main()
