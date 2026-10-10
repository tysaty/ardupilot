"""Brute-force grid search for arm FH's prediction lead (``TASK-067``).

Arm F / FH (``TASK-060``) shifts the baseline's guidance point by the
estimated kangaroo velocity over ``af_step_ticks`` control ticks,
``g' = g + v_est * dt_s * af_step_ticks``. This module runs FH (and F, the
no-hysteresis control) at every candidate lead in every regime of the
``TASK-045`` grid and reports which lead gives the lowest error. No selector:
each cell flies one fixed lead, exactly the grid cell
:func:`Py_Sweep_Experiment.build_spec` builds with ``af_step_ticks``
overridden; the lead is the only thing that varies.

The answer is the lead, not the trajectory, so each cell keeps its
``spec.json`` and ``record.json`` (the record includes its replay
verification). History, series and ticks are not kept: every cell is
deterministic and its spec reproduces them (``experiment.run_spec``).

Usage, from ``src/ardupilot/scripts``::

    python3 -m py_harness.fh_lead_sweep --all        # plan, run, aggregate
    python3 -m py_harness.fh_lead_sweep --aggregate  # re-aggregate and plot

Pure harness: no guidance law changed (`VR-014`).
"""

import argparse
import csv
import datetime
import json
import math
import multiprocessing
import os
import shutil
import statistics
import subprocess
import tempfile
import time

from . import Py_Sweep_Experiment as sweep
from . import experiment

#: Candidate leads, whole control ticks (``D1``, proposed in the task file):
#: 0.1 s to 4.0 s at ``dt_s`` = 0.1, the upper end matching arm B's largest
#: candidate horizon (``AH_K_MAX_STEPS``). 0 is not a valid lead (``>= 1``).
LEADS = (1, 2, 3, 5, 8, 10, 15, 20, 25, 30, 40)

#: ``(arm id, arm set)``: FH and its no-hysteresis control F (``D4``).
ARMS = (("FH", "hyst"), ("F", "base"))

#: The ranking metric (``D1``): mean absolute radial error about the true
#: kangaroo from first contact to the end of the run (``metrics.post_contact_radial``).
PRIMARY = "post_contact_mean_radial_m"
#: Reported beside it.
SECONDARY = ("rms_ring_error_m", "rms_e_tan_m", "t_contact_s",
             "post_contact_max_abs_radial_m")

CAMPAIGN_ID = "CAMP-067-fh-lead-sweep"
DEFAULT_OUT_DIR = os.path.join(sweep.REPO_ROOT, "experiments", "campaigns", CAMPAIGN_ID)

RESULT_COLUMNS = (
    "cell_id", "arm", "algorithm", "af_step_ticks", "lead_s", "regime",
    "mode_base", "mode_pace", "ratio_name", "speed_ratio", "target_speed_ms",
    "seed", "status", "replay_ok", "partial", PRIMARY) + SECONDARY + (
    "mean_radius_m", "settled", "zone_plane_breaches", "zone_containment_turns")


def regime_of(base, pace, ratio_name):
    """``point``, or ``<base>-<pace>-<ratio>`` (rand seeds pooled)."""
    return "point" if base == "point" else "%s-%s-%s" % (base, pace, ratio_name)


def cell_id(arm_id, base, pace, ratio_name, seed, lead):
    return "%s-L%02d" % (sweep.cell_id(arm_id, base, pace, ratio_name, seed), lead)


def lead_spec(arm_id, arm_set, base, pace, ratio_name, speed_ratio, seed, lead):
    """The grid cell's spec with ``af_step_ticks`` set to ``lead``."""
    spec = sweep.build_spec(arm_id, base, pace, ratio_name, speed_ratio, seed,
                            arm_set=arm_set)
    spec = json.loads(json.dumps(spec))
    spec["algorithm"]["overrides"]["af_step_ticks"] = int(lead)
    spec["experiment_id"] = cell_id(arm_id, base, pace, ratio_name, seed, lead)
    spec["objective"] = (
        "TASK-067 lead sweep: arm %s (%s) at af_step_ticks %d against a %s "
        "kangaroo at %s pace, speed ratio %s"
        % (arm_id, spec["algorithm"]["name"], lead, base, pace,
           "n/a" if speed_ratio is None else speed_ratio))
    return experiment.validate_spec(spec)


def plan():
    """Every (arm, regime cell, lead) to run, in reporting order."""
    jobs = []
    for arm_id, arm_set in ARMS:
        for cell in sweep.expand_grid(arms=[arm_id], arm_set=arm_set):
            for lead in LEADS:
                jobs.append({
                    "arm": arm_id, "arm_set": arm_set,
                    "mode_base": cell["mode_base"], "mode_pace": cell["mode_pace"],
                    "ratio_name": cell["ratio_name"], "speed_ratio": cell["speed_ratio"],
                    "target_speed_ms": cell["target_speed_ms"], "seed": cell["seed"],
                    "algorithm": cell["algorithm"], "lead": lead,
                    "feasible": cell["feasibility"].get("feasible"),
                    "unreachable": cell["feasibility"].get("unreachable"),
                })
    return jobs


def run_job(job, out_dir):
    """Run one cell; keep its spec and record. Returns a result row."""
    cid = cell_id(job["arm"], job["mode_base"], job["mode_pace"],
                  job["ratio_name"], job["seed"], job["lead"])
    row = {"cell_id": cid, "arm": job["arm"], "algorithm": job["algorithm"],
           "af_step_ticks": job["lead"],
           "regime": regime_of(job["mode_base"], job["mode_pace"], job["ratio_name"]),
           "mode_base": job["mode_base"], "mode_pace": job["mode_pace"],
           "ratio_name": job["ratio_name"], "speed_ratio": job["speed_ratio"],
           "target_speed_ms": job["target_speed_ms"], "seed": job["seed"]}
    try:
        spec = lead_spec(job["arm"], job["arm_set"], job["mode_base"], job["mode_pace"],
                         job["ratio_name"], job["speed_ratio"], job["seed"], job["lead"])
        row["lead_s"] = spec["aircraft"]["dt_s"] * job["lead"]
        session = experiment.run_spec(spec)
        tmp = tempfile.mkdtemp(prefix="fhlead-")
        try:
            experiment.write_bundle(
                spec, session, verify=True, render=False, directory=tmp,
                cell={"arm": job["arm"], "mode_base": job["mode_base"],
                      "mode_pace": job["mode_pace"], "speed_ratio": job["speed_ratio"],
                      "seed": job["seed"], "af_step_ticks": job["lead"]},
                n_a_max_steps=int(round(sweep.TRANSIT_WINDOW_S / spec["aircraft"]["dt_s"])))
            dest = os.path.join(out_dir, cid)
            os.makedirs(dest, exist_ok=True)
            for name in ("spec.json", "record.json"):
                shutil.copyfile(os.path.join(tmp, name), os.path.join(dest, name))
            with open(os.path.join(tmp, "record.json")) as handle:
                record = json.load(handle)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        entry = dict(job, sub=sweep.SUB_MAIN, status=sweep.STATUS_COMPLETE,
                     replay_ok=(record.get("replay_verification") or {}).get(
                         "history_matches"))
        flat = sweep.master_row(cid, entry, record)
        row.update({k: flat.get(k) for k in RESULT_COLUMNS if k in flat and k not in row})
        row["status"] = "complete" if not flat.get("partial") else "partial"
        row["replay_ok"] = entry["replay_ok"]
    except Exception as exc:                          # recorded, never dropped (VR-012)
        row["status"] = "error"
        row["error"] = "%s: %s" % (type(exc).__name__, exc)
    return row


def _worker(args):
    return run_job(*args)


def _git(*cmd):
    try:
        return subprocess.check_output(("git",) + cmd, cwd=os.path.dirname(__file__),
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


def run_all(out_dir=DEFAULT_OUT_DIR, processes=None, progress=print):
    os.makedirs(out_dir, exist_ok=True)
    jobs = plan()
    manifest = {
        "campaign": CAMPAIGN_ID, "task": "TASK-067",
        "module": "py_harness.fh_lead_sweep",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "not_a_flight_configuration": (
            "Harness campaign (SR-004). No value here is a flight limit; results are "
            "evidence for A-VAL-001, not flight evidence."),
        "leads_ticks": list(LEADS), "arms": [a for a, _ in ARMS],
        "primary_metric": PRIMARY, "secondary_metrics": list(SECONDARY),
        "grid": {"mode_permutations": [list(p) for p in sweep.MODE_PERMUTATIONS],
                 "speed_ratios": [list(r) for r in sweep.SPEED_RATIOS],
                 "rand_seeds": list(sweep.RAND_SEEDS),
                 "duration_s": sweep.DURATION_S,
                 "initial_conditions": sweep.INITIAL_CONDITIONS,
                 "kangaroo_geometry": sweep.KANGAROO_GEOMETRY, "zone": sweep.ZONE},
        "decisions": {"D1": "provisional: the task file's proposed leads, regimes and metric",
                      "D4": "F included as the control"},
        "software": {"ardupilot_commit": _git("rev-parse", "HEAD"),
                     "ardupilot_dirty": bool(_git("status", "--porcelain", "--", "scripts"))},
        "cells": len(jobs),
    }
    with open(os.path.join(out_dir, "MANIFEST.json"), "w") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    t0 = time.time()
    rows = []
    processes = processes or max(1, (os.cpu_count() or 2) - 2)
    with multiprocessing.get_context("spawn").Pool(processes) as pool:
        work = [(j, out_dir) for j in jobs]
        for i, row in enumerate(pool.imap(_worker, work, chunksize=4), 1):
            rows.append(row)
            if i % 100 == 0 or i == len(jobs):
                progress("  %d / %d cells, %.0f s" % (i, len(jobs), time.time() - t0))
    write_results(rows, out_dir)
    return rows


def write_results(rows, out_dir):
    cols = list(RESULT_COLUMNS) + ["error"]
    with open(os.path.join(out_dir, "results.csv"), "w", newline="") as handle:
        w = csv.DictWriter(handle, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow(row)


def read_results(out_dir):
    def num(v):
        if v in ("", "None", None):
            return None
        try:
            return float(v)
        except ValueError:
            return v
    with open(os.path.join(out_dir, "results.csv")) as handle:
        return [dict((k, num(v)) if k not in ("cell_id", "arm", "algorithm", "regime",
                                              "mode_base", "mode_pace", "ratio_name",
                                              "status", "error") else (k, v)
                     for k, v in row.items()) for row in csv.DictReader(handle)]


# --------------------------------------------------------------------------
# Aggregation: per-regime optimum, overall optimum, sensitivity
# --------------------------------------------------------------------------

def regime_table(rows, arm, metric=PRIMARY):
    """``{regime: {lead: value}}``: the metric per regime and lead, rand seeds
    averaged. A lead with any seed that never made contact (metric ``None``)
    is ``None`` for that regime."""
    out = {}
    for r in rows:
        if r["arm"] != arm:
            continue
        by_lead = out.setdefault(r["regime"], {})
        by_lead.setdefault(int(r["af_step_ticks"]), []).append(r.get(metric))
    table = {}
    for regime, by_lead in out.items():
        table[regime] = {}
        for lead, vals in by_lead.items():
            table[regime][lead] = (None if any(v is None for v in vals)
                                   else statistics.mean(vals))
    return table


def summarise(rows, arm):
    table = regime_table(rows, arm)
    regimes = sorted(table, key=_regime_order)
    per_regime = []
    for regime in regimes:
        by_lead = table[regime]
        ok = {k: v for k, v in by_lead.items() if v is not None}
        if not ok:
            per_regime.append({"regime": regime, "best_lead": None, "best": None,
                               "at_lead_1": by_lead.get(1), "no_contact_leads": sorted(by_lead)})
            continue
        best = min(ok, key=lambda k: ok[k])
        per_regime.append({
            "regime": regime, "best_lead": best, "best": ok[best],
            "at_lead_1": by_lead.get(1),
            "gain_vs_lead_1": (None if by_lead.get(1) is None else by_lead[1] - ok[best]),
            "no_contact_leads": sorted(k for k, v in by_lead.items() if v is None)})
    # Overall: only regimes where every lead made contact, so each lead is
    # judged on the same set (otherwise a lead that loses contact looks good).
    full = [g for g in regimes if all(v is not None for v in table[g].values())]
    overall = {}
    for lead in LEADS:
        vals = [table[g][lead] for g in full if lead in table[g]]
        overall[lead] = {"mean": statistics.mean(vals) if vals else None,
                         "worst": max(vals) if vals else None,
                         "no_contact_regimes": sum(1 for g in regimes
                                                   if table[g].get(lead) is None)}

    def rank(field):
        return min(LEADS, key=lambda k: (math.inf if overall[k][field] is None
                                         else overall[k][field]))
    best_mean, best_worst = rank("mean"), rank("worst")
    return {"arm": arm, "per_regime": per_regime, "overall": overall,
            "regimes_compared": len(full), "regimes_total": len(regimes),
            "best_by_mean": best_mean, "best_by_worst": best_worst}


_MODE_ORDER = {(b, p): i for i, (b, p) in enumerate(sweep.MODE_PERMUTATIONS)}
_RATIO_ORDER = {n: i for i, (n, _r) in enumerate(sweep.SPEED_RATIOS)}


def _regime_order(regime):
    if regime == "point":
        return (-1, -1, "")
    parts = regime.split("-", 2)
    if len(parts) != 3:
        return (99, 99, regime)
    base, pace, ratio = parts
    return (_MODE_ORDER.get((base, pace), 99), _RATIO_ORDER.get(ratio, 99), "")


def aggregate(out_dir=DEFAULT_OUT_DIR, plots=True):
    rows = read_results(out_dir)
    status = {}
    for r in rows:
        status[r["status"]] = status.get(r["status"], 0) + 1
    summary = {"status_counts": status, "arms": {}}
    for arm, _set in ARMS:
        summary["arms"][arm] = summarise(rows, arm)
    with open(os.path.join(out_dir, "summary.json"), "w") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")
    _write_per_regime_csv(summary, out_dir)
    if plots:
        _plot(rows, summary, out_dir)
    return summary


def _write_per_regime_csv(summary, out_dir):
    with open(os.path.join(out_dir, "best_lead_by_regime.csv"), "w", newline="") as handle:
        w = csv.writer(handle)
        w.writerow(["arm", "regime", "best_lead_ticks", "best_" + PRIMARY,
                    PRIMARY + "_at_lead_1", "gain_vs_lead_1_m", "no_contact_leads"])
        for arm, s in summary["arms"].items():
            for p in s["per_regime"]:
                w.writerow([arm, p["regime"], p["best_lead"], _r(p["best"]),
                            _r(p["at_lead_1"]), _r(p.get("gain_vs_lead_1")),
                            " ".join(str(x) for x in p["no_contact_leads"])])


def _r(v):
    return None if v is None else round(v, 3)


def _plot(rows, summary, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    leads = list(LEADS)
    # 1. overall mean and worst against lead, FH and F
    fig, ax = plt.subplots(figsize=(7, 4))
    for arm, style in (("FH", "-"), ("F", "--")):
        o = summary["arms"][arm]["overall"]
        ax.plot([k * 0.1 for k in leads], [o[k]["mean"] for k in leads], style, marker="o",
                label="%s mean over regimes" % arm)
        ax.plot([k * 0.1 for k in leads], [o[k]["worst"] for k in leads], style, marker="x",
                alpha=0.6, label="%s worst regime" % arm)
    ax.set_xlabel("lead (s) = af_step_ticks x 0.1 s")
    ax.set_ylabel("post-contact mean |radial error| (m)")
    ax.set_yscale("log")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("TASK-067: error against lead, over %d regimes"
                 % summary["arms"]["FH"]["regimes_compared"])
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "error_vs_lead_overall.png"), dpi=150)
    plt.close(fig)
    # 2. FH per mode permutation, one line per ratio
    table = regime_table(rows, "FH")
    perms = [p for p in sweep.MODE_PERMUTATIONS if p[0] != "point"]
    fig, axes = plt.subplots(2, 4, figsize=(14, 6.5), sharex=True)
    for ax, (base, pace) in zip(axes.flat, perms):
        for name, _ratio in sweep.SPEED_RATIOS:
            g = regime_of(base, pace, name)
            if g in table:
                ax.plot([k * 0.1 for k in leads], [table[g].get(k) for k in leads],
                        marker=".", label=name)
        ax.set_title("%s %s" % (base, pace), fontsize=9)
        ax.set_yscale("log")
        ax.grid(True, which="both", alpha=0.3)
    ax = axes.flat[len(perms)]
    if "point" in table:
        ax.plot([k * 0.1 for k in leads], [table["point"].get(k) for k in leads], marker=".")
        ax.set_title("point", fontsize=9)
        ax.set_yscale("log")
        ax.grid(True, which="both", alpha=0.3)
    axes.flat[0].legend(fontsize=7, title="speed ratio", title_fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("lead (s)")
    for ax in axes[:, 0]:
        ax.set_ylabel("post-contact mean |e_r| (m)")
    fig.suptitle("TASK-067: arm FH error against lead, per regime")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "error_vs_lead_by_regime_FH.png"), dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--all", action="store_true", help="run every cell, then aggregate")
    action.add_argument("--aggregate", action="store_true")
    action.add_argument("--plan", action="store_true", help="print the cell count only")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR)
    parser.add_argument("--processes", type=int, default=None)
    args = parser.parse_args(argv)
    if args.plan:
        jobs = plan()
        print("%d cells: %d arms x %d leads x %d grid cells"
              % (len(jobs), len(ARMS), len(LEADS), len(jobs) // (len(ARMS) * len(LEADS))))
        return 0
    if args.all:
        run_all(args.out, args.processes)
    summary = aggregate(args.out)
    for arm, s in summary["arms"].items():
        print("%s: best lead by mean %d ticks, by worst %d ticks (%d of %d regimes compared)"
              % (arm, s["best_by_mean"], s["best_by_worst"], s["regimes_compared"],
                 s["regimes_total"]))
    print("status:", summary["status_counts"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
