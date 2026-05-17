#!/usr/bin/env python3
"""
Plot J (cost function) over simulation time for a DubinsBestTrial run.

Usage:
    python3 scripts/plot_best_trial.py # auto-finds latest dubins_best_J_*.csv
    python3 scripts/plot_best_trial.py path/to/file.csv #explicit file
"""

import sys
import os
import csv
import glob

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

BUILDLOGS = os.path.expanduser("~/src/buildlogs")


def find_latest_best_J_csv(buildlogs=BUILDLOGS):
    csvs = sorted(glob.glob(os.path.join(buildlogs, "dubins_best_J_*.csv")))
    if not csvs:
        raise FileNotFoundError(f"No dubins_best_J_*.csv found in {buildlogs}")
    return csvs[-1]


def load_j_series(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["sim_time_s"]), float(r["J"])))
    return rows


def plot_j_over_time(path, out_dir=None):
    rows = load_j_series(path)
    if not rows:
        print(f"ERROR: no data in {path}")
        return

    times = np.array([r[0] for r in rows])
    js    = np.array([r[1] for r in rows])

    running_min = np.minimum.accumulate(js)

    fig, ax = plt.subplots(figsize=(11, 5))

    ax.plot(times, js, "o-", ms=4, lw=1.2, color="steelblue", label="Cost function at swap")
    ax.plot(times, running_min, "--", lw=1.8, color="crimson", label="Minimum value in sequence")

    # mark global best
    best_idx = int(np.argmin(js))
    ax.axhline(js[best_idx], color="crimson", lw=0.6, alpha=0.4)
    ax.annotate(
        f"J min = {js[best_idx]:.4f}\n@ t={times[best_idx]:.1f}s",
        xy=(times[best_idx], js[best_idx]),
        xytext=(10, 15), textcoords="offset points",
        fontsize=8, color="crimson",
        #arrowprops=dict(arrowstyle="->", color="crimson", lw=0.8),
    )

    ax.set_xlabel("Sim time (s)", fontsize=11)
    ax.set_ylabel("J (cost function)", fontsize=11)
    ax.set_title(
        #f"Lowest Configured Cost Function Value - Performance over time\n{os.path.basename(path)}",

        "Lowest Configured Cost Function Value - Performance over time",
        fontsize=11,
    )
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    fig.tight_layout()

    if out_dir is None:
        out_dir = os.path.dirname(path)
    out = os.path.join(out_dir, "plot_best_J_over_time.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved: {out}")

    # summary stats
    print(f"\n  Points collected : {len(rows)}")
    print(f"  Time span        : {times[0]:.1f}s – {times[-1]:.1f}s")
    print(f"  J initial        : {js[0]:.4f}")
    print(f"  J final          : {js[-1]:.4f}")
    print(f"  J min            : {js[best_idx]:.4f} @ t={times[best_idx]:.1f}s")
    print(f"  J mean           : {js.mean():.4f}")
    pct = 100.0 * (js[0] - js[best_idx]) / js[0] if js[0] != 0 else 0
    print(f"  Improvement      : {pct:.1f}% from first to best")


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = find_latest_best_J_csv()
    print(f"Using: {path}")
    plot_j_over_time(path)


if __name__ == "__main__":
    main()
