#!/usr/bin/env python3
"""
Plot DubinsParamSweep results — J-centric analysis.

Usage:
    python3 scripts/plot_sweep2.py                    # auto-picks latest CSV
    python3 scripts/plot_sweep2.py path/to/sweep.csv
    python3 scripts/plot_sweep2.py --out /tmp/plots/
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
    import matplotlib.lines
except ImportError:
    sys.exit("matplotlib not installed: pip3 install matplotlib")

# ── Constants ─────────────────────────────────────────────────────────────────

ALL_PARAMS = [
    "CTRL_REBUILD_MS", "CTRL_STREAK", "CTRL_DUB_DIST", "CTRL_DUB_VEL",
    "KF_PROC_NOISE", "KF_MEAS_NOISE",
]

PARAM_LABELS = {
    "CTRL_REBUILD_MS": "Rebuild (ms)",
    "CTRL_STREAK":     "Streak",
    "CTRL_DUB_DIST":   "Engage dist (m)",
    "CTRL_DUB_VEL":    "Engage vel (m/s)",
    "KF_PROC_NOISE":   "KF Q (process noise)",
    "KF_MEAS_NOISE":   "KF R (measurement noise)",
}

KANG_MODES   = ["random", "straight", "circle", "rectangle"]
KANG_COLOURS = {m: c for m, c in zip(KANG_MODES, plt.cm.tab10.colors)}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _float_or(d, key, fallback):
    v = d.get(key, "")
    if v in ("", "None", None):
        return fallback
    try:
        return float(v)
    except ValueError:
        return fallback


def _varied_params(rows, param_list):
    """Return only params that have more than one unique value in this dataset."""
    return [p for p in param_list if len({r[p] for r in rows}) > 1]


def _present_modes(rows):
    return [m for m in KANG_MODES if any(r["kang_mode"] == m for r in rows)]


def _subfigs(fig, n):
    """Return a list of n subfigures stacked vertically."""
    sf = fig.subfigures(n, 1, hspace=0.14)
    return [sf] if n == 1 else list(sf)


# ── Data loading ──────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if "kang_mode" not in (reader.fieldnames or []):
            sys.exit(
                f"ERROR: '{os.path.basename(path)}' is an old-format sweep CSV "
                "(no kang_mode column).\nUse scripts/plot_sweep.py for old-format files."
            )
        for r in reader:
            if r.get("l2_error_m") in ("", "None", None):
                continue
            try:
                j_raw = r.get("j_min", "")
                j_min = None
                if j_raw not in ("", "None", None):
                    j_val = float(j_raw)
                    if j_val >= 0:
                        j_min = j_val

                rows.append({
                    "trial":           int(r["trial"]),
                    "name":            r["name"],
                    "kang_mode":       r["kang_mode"],
                    "CTRL_REBUILD_MS": int(float(r["CTRL_REBUILD_MS"])),
                    "CTRL_STREAK":     int(float(r["CTRL_STREAK"])),
                    "CTRL_DUB_DIST":   int(float(r["CTRL_DUB_DIST"])),
                    "CTRL_DUB_VEL":    int(float(r["CTRL_DUB_VEL"])),
                    "CTRL_W_HDG_KANG": _float_or(r, "CTRL_W_HDG_KANG", 0.4),
                    "CTRL_W_HDG_CHG":  _float_or(r, "CTRL_W_HDG_CHG",  0.2),
                    "CTRL_W_DIST_PLN": _float_or(r, "CTRL_W_DIST_PLN", 0.2),
                    "CTRL_W_DIST_KNG": _float_or(r, "CTRL_W_DIST_KNG", 0.2),
                    "KF_PROC_NOISE":   _float_or(r, "KF_PROC_NOISE", 0.1),
                    "KF_MEAS_NOISE":   _float_or(r, "KF_MEAS_NOISE", 5.0),
                    "l2":              float(r["l2_error_m"]),
                    "j_min":           j_min,
                })
            except (ValueError, KeyError):
                continue
    return rows


def remove_outliers(rows, key, iqr_factor=3.0):
    """Drop rows where key > Q3 + iqr_factor*IQR or < Q1 - iqr_factor*IQR."""
    vals = [r[key] for r in rows if r[key] is not None]
    if len(vals) < 4:
        return rows
    q1, q3 = np.percentile(vals, 25), np.percentile(vals, 75)
    iqr = q3 - q1
    lo, hi = q1 - iqr_factor * iqr, q3 + iqr_factor * iqr
    kept = [r for r in rows if r[key] is None or lo <= r[key] <= hi]
    dropped = len(rows) - len(kept)
    if dropped:
        print(f"  Outlier filter ({key}, {iqr_factor}×IQR [{lo:.2f}, {hi:.2f}]): "
              f"removed {dropped} rows")
    return kept


def find_latest_csv():
    candidates = sorted(glob.glob(os.path.expanduser("~/src/buildlogs/dubins_sweep_*.csv")))
    if not candidates:
        candidates = sorted(glob.glob("buildlogs/dubins_sweep_*.csv"))
    if not candidates:
        sys.exit("No dubins_sweep_*.csv found in ~/src/buildlogs/.")
    return candidates[-1]


# ── Plot 1: Top configs by J_min ──────────────────────────────────────────────

def plot_top_configs(rows, out_dir, prefix, n=15):
    j_rows = [r for r in rows if r["j_min"] is not None]
    if not j_rows:
        print("  Skipping top-configs plot — no j_min data")
        return

    varied = _varied_params(rows, ALL_PARAMS)
    sorted_rows = sorted(j_rows, key=lambda r: r["j_min"])[:n]

    fig, ax = plt.subplots(figsize=(11, max(5, len(sorted_rows) * 0.55)))
    fig.suptitle(f"Top {len(sorted_rows)} configurations by minimum cost J", fontsize=13)

    ys     = list(range(len(sorted_rows)))
    js     = [r["j_min"] for r in sorted_rows]
    colors = [KANG_COLOURS[r["kang_mode"]] for r in sorted_rows]

    bars = ax.barh(ys, js, color=colors, edgecolor="white", linewidth=0.4)

    for i, (bar, r) in enumerate(zip(bars, sorted_rows)):
        label_parts = [f"{PARAM_LABELS.get(p, p)}={r[p]}" for p in varied]
        label = "  " + "   ".join(label_parts)
        ax.text(bar.get_width() * 1.01, i, label, va="center", fontsize=7.5)

    ax.set_yticks(ys)
    ax.set_yticklabels([f"#{i+1}  {r['kang_mode']}" for i, r in enumerate(sorted_rows)],
                       fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel("J_min (cost function value)")
    ax.xaxis.set_minor_locator(matplotlib.ticker.AutoMinorLocator())

    legend_patches = [
        mpatches.Patch(color=KANG_COLOURS[m], label=f"KANG_{m.upper()}")
        for m in _present_modes(j_rows)
    ]
    ax.legend(handles=legend_patches, fontsize=8, loc="lower right")
    fig.tight_layout()

    path = os.path.join(out_dir, f"{prefix}_plot1_top_configs_J.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 2: KF noise heatmap — J_min per mode ─────────────────────────────────

def plot_kf_heatmap(rows, out_dir, prefix):
    j_rows = [r for r in rows if r["j_min"] is not None]
    if not j_rows:
        print("  Skipping KF heatmap — no j_min data")
        return

    present = _present_modes(j_rows)
    n_modes = len(present)
    q_vals = sorted({r["KF_PROC_NOISE"] for r in j_rows})
    r_vals = sorted({r["KF_MEAS_NOISE"] for r in j_rows})

    if len(q_vals) < 2 and len(r_vals) < 2:
        print("  Skipping KF heatmap — only one KF value in data")
        return

    fig = plt.figure(figsize=(max(8, len(r_vals) * 3), max(4, n_modes * 3.5)))
    fig.suptitle("Grid Search: Cost Function Values by Filter Noise (Q, R)", fontsize=13, y=0.995)
    subfigs = _subfigs(fig, n_modes)

    for subfig, mode in zip(subfigs, present):
        ax = subfig.subplots(1, 1)
        subset = [r for r in j_rows if r["kang_mode"] == mode]
        if not subset:
            ax.set_visible(False)
            continue

        grid = np.full((len(q_vals), len(r_vals)), np.nan)
        for i, q in enumerate(q_vals):
            for j, rv in enumerate(r_vals):
                vals = [r["j_min"] for r in subset
                        if r["KF_PROC_NOISE"] == q and r["KF_MEAS_NOISE"] == rv]
                if vals:
                    grid[i, j] = np.mean(vals)

        valid = grid[~np.isnan(grid)]
        vmin, vmax = (valid.min(), valid.max()) if valid.size else (0, 1)
        im = ax.imshow(grid, cmap="RdYlGn_r", aspect="auto", vmin=vmin, vmax=vmax)
        subfig.colorbar(im, ax=ax, label="J (minimum)", fraction=0.046, pad=0.04)

        ax.set_xticks(range(len(r_vals)))
        ax.set_xticklabels([str(v) for v in r_vals], fontsize=9)
        ax.set_yticks(range(len(q_vals)))
        ax.set_yticklabels([str(v) for v in q_vals], fontsize=9)
        ax.set_xlabel("KF measurement noise R", fontsize=9)
        ax.set_ylabel("KF process noise Q", fontsize=9)

        for i in range(len(q_vals)):
            for j in range(len(r_vals)):
                if not np.isnan(grid[i, j]):
                    ax.text(j, i, f"{grid[i, j]:.1f}", ha="center", va="center",
                            fontsize=9, color="black", fontweight="bold")


    path = os.path.join(out_dir, f"{prefix}_plot2_kf_heatmap_J.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 3: Marginal effects on J_min (varied params only) ────────────────────

def plot_marginal_J(rows, out_dir, prefix):
    j_rows = [r for r in rows if r["j_min"] is not None]
    if not j_rows:
        print("  Skipping marginal J plot — no j_min data")
        return

    varied = _varied_params(j_rows, ALL_PARAMS)
    if not varied:
        print("  Skipping marginal J plot — no varied params")
        return

    present = _present_modes(j_rows)
    n_modes = len(present)
    n_cols  = len(varied)
    fig_w   = max(12, n_cols * 3.2)

    fig = plt.figure(figsize=(fig_w, max(5, n_modes * 4.5)))
    fig.suptitle("Marginal effect of each parameter on J_min\n(mean ± std, varied params only)",
                 fontsize=13, y=0.995)
    subfigs = _subfigs(fig, n_modes)

    for subfig, mode in zip(subfigs, present):
        subfig.suptitle(f"KANG_{mode.upper()}", fontsize=11, fontweight="bold",
                        ha="left", x=0.02)
        axes = subfig.subplots(1, n_cols)
        if n_cols == 1:
            axes = [axes]
        subfig.subplots_adjust(wspace=0.5)
        subset = [r for r in j_rows if r["kang_mode"] == mode]

        for col_i, (ax, param) in enumerate(zip(axes, varied)):
            if not subset:
                ax.set_visible(False)
                continue
            levels = sorted({r[param] for r in subset})
            means  = [np.mean([r["j_min"] for r in subset if r[param] == lv]) for lv in levels]
            stds   = [np.std( [r["j_min"] for r in subset if r[param] == lv]) for lv in levels]
            colours = ["#4daf4a" if m == min(means) else "#e41a1c" if m == max(means) else "#377eb8"
                       for m in means]
            ax.bar([str(lv) for lv in levels], means, yerr=stds, capsize=4,
                   color=colours, edgecolor="white", linewidth=0.5)
            ax.set_title(PARAM_LABELS.get(param, param), fontsize=9)
            if col_i == 0:
                ax.set_ylabel("Mean J_min", fontsize=8)
            ax.tick_params(axis="x", labelsize=8)
            ax.tick_params(axis="y", labelsize=7)

    path = os.path.join(out_dir, f"{prefix}_plot3_marginal_J.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 4: J_min vs L2 scatter ───────────────────────────────────────────────

def plot_j_vs_l2(rows, out_dir, prefix):
    j_rows = [r for r in rows if r["j_min"] is not None]
    if not j_rows:
        print("  Skipping J vs L2 — no j_min data")
        return

    MODE_DISPLAY = {
        "random":    "Kangaroo Mode Run: Random",
        "straight":  "Kangaroo Mode Run: Straight",
        "circle":    "Kangaroo Mode Run: Circle",
        "rectangle": "Kangaroo Mode Run: Rectangle",
    }

    present = _present_modes(j_rows)
    fig, ax = plt.subplots(figsize=(8, 6))
    fig.suptitle("Comparison of Cost Function to L2 Norm Error", fontsize=12)

    for mode in present:
        subset = [r for r in j_rows if r["kang_mode"] == mode]
        jv  = [r["j_min"] for r in subset]
        l2v = [r["l2"]    for r in subset]
        ax.scatter(jv, l2v, color=KANG_COLOURS[mode], s=50, alpha=0.8,
                   edgecolors="white", linewidths=0.4, zorder=3,
                   label=MODE_DISPLAY.get(mode, mode))

    jv_all  = np.array([r["j_min"] for r in j_rows])
    l2v_all = np.array([r["l2"]    for r in j_rows])
    if len(jv_all) >= 3:
        m, b = np.polyfit(jv_all, l2v_all, 1)
        xs = np.linspace(jv_all.min(), jv_all.max(), 100)
        ax.plot(xs, m * xs + b, "k--", lw=1.2,
                label=f"fit  L2 = {m:.3f}·J + {b:.1f}m")
        corr = np.corrcoef(jv_all, l2v_all)[0, 1]
        ax.text(0.97, 0.97, f"r = {corr:.3f}", transform=ax.transAxes,
                ha="right", va="top", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))

    ax.set_xlabel("J - Cost Function (unitless)", fontsize=11)
    ax.set_ylabel("Average L2 Tracking Error (m), over run", fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    path = os.path.join(out_dir, f"{prefix}_plot4_J_vs_L2.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Plot 5: J_min distribution by kang_mode ───────────────────────────────────

def plot_j_distribution(rows, out_dir, prefix):
    j_rows = [r for r in rows if r["j_min"] is not None]
    if not j_rows:
        print("  Skipping J distribution — no j_min data")
        return

    present = _present_modes(j_rows)
    data    = {m: [r["j_min"] for r in j_rows if r["kang_mode"] == m] for m in present}
    present = [m for m in present if data[m]]

    fig, ax = plt.subplots(figsize=(max(6, len(present) * 3), 6))
    fig.suptitle("J_min distribution — per kangaroo mode", fontsize=13)

    positions = list(range(len(present)))
    if len(present) > 1:
        parts = ax.violinplot([data[m] for m in present], positions=positions,
                              showmedians=True, showextrema=True)
        for body, mode in zip(parts["bodies"], present):
            body.set_facecolor(KANG_COLOURS[mode])
            body.set_alpha(0.55)
        for key in ("cmedians", "cmins", "cmaxes", "cbars"):
            parts[key].set_color("black")
            parts[key].set_linewidth(1.2)

    rng = np.random.default_rng(42)
    for i, mode in enumerate(present):
        vals = data[mode]
        jitter = rng.uniform(-0.1, 0.1, len(vals))
        ax.scatter(np.full(len(vals), i) + jitter, vals,
                   color=KANG_COLOURS[mode], s=25, alpha=0.6, edgecolors="none", zorder=4)
        best = min(vals)
        ax.annotate(f"min={best:.2f}", xy=(i, best),
                    xytext=(i + 0.15, best * 0.97), fontsize=8)

    ax.set_xticks(positions)
    ax.set_xticklabels([f"KANG_{m.upper()}" for m in present], fontsize=10)
    ax.set_ylabel("J_min (cost function value)", fontsize=11)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()

    path = os.path.join(out_dir, f"{prefix}_plot5_J_distribution.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Plot DubinsParamSweep results (J-centric)")
    ap.add_argument("csv", nargs="?", help="CSV file (default: latest dubins_sweep_*.csv)")
    ap.add_argument("--out", default=None, help="Output directory (default: same dir as CSV)")
    args = ap.parse_args()

    csv_path = args.csv or find_latest_csv()
    out_dir  = args.out or os.path.dirname(os.path.abspath(csv_path))
    prefix   = os.path.splitext(os.path.basename(csv_path))[0]

    print(f"Loading {os.path.basename(csv_path)}...")
    rows = load_csv(csv_path)
    print(f"  {len(rows)} valid trials loaded")
    for mode in _present_modes(rows):
        n = sum(1 for r in rows if r["kang_mode"] == mode)
        print(f"    {mode}: {n} trials")
    n_j = sum(1 for r in rows if r["j_min"] is not None)
    print(f"  {n_j} trials have j_min data")

    rows = remove_outliers(rows, "j_min")
    rows = remove_outliers(rows, "l2")
    print(f"  {len(rows)} trials after outlier removal")

    varied = _varied_params(rows, ALL_PARAMS)
    fixed  = [p for p in ALL_PARAMS if p not in varied]
    if fixed:
        print(f"  Fixed params (excluded from marginal plots): {', '.join(fixed)}")

    print("Generating plots...")
    plot_top_configs(rows, out_dir, prefix)
    plot_kf_heatmap(rows, out_dir, prefix)
    plot_marginal_J(rows, out_dir, prefix)
    plot_j_vs_l2(rows, out_dir, prefix)
    plot_j_distribution(rows, out_dir, prefix)

    print(f"\nDone. 5 PNGs saved to {out_dir}/")


if __name__ == "__main__":
    main()
