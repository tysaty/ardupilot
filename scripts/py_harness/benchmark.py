"""Common moving-target benchmark for the three predictive approaches (``TASK-039``).

One harness, one set of scenarios, one set of metrics, shared by every approach —
so the comparison is a comparison (`VR-009`, `PR-004`). Each approach is an entry
in :data:`ARMS`: an algorithm registry name plus the configuration that arm is run
under. **The arm is the configuration, not just the algorithm**; arm A and the
unpredicted baseline are the same algorithm family at different horizons, and
saying so in one table is the point.

Arms
----
====== ================================ ==========================================
Arm    Algorithm                        What it decides
====== ================================ ==========================================
``0``  ``dubins_target_orbit``          Nothing. The unpredicted baseline the
                                        others must beat to have earned anything.
``A``  ``adaptive_db_circle``           Where to centre the ring, at **one
                                        configured** horizon (`TASK-033`).
``B``  ``adaptive_horizon_cs``          Where to centre the ring **and which
                                        horizon to use**, per replan (arm A plus
                                        a selector; `TASK-022`, `TASK-038`).
``C``  ``rh_geometric``                 The whole trajectory, over a receding
                                        horizon, against aircraft constraints.
====== ================================ ==========================================

Scenarios
---------
Two target-speed **profiles**, both swept over the same speed points:

``constant``
    ``straight`` mode at a fixed speed. The regime every recorded result to date
    was measured in.
``elastic``
    ``elastic`` mode over a straight base (`TASK-030`) — surge and ease, so the
    speed varies within the run. The commanded speed is the **fast** phase; the
    slow phase is a fixed fraction of it. Included because a whole-run mean
    against a varying speed is a fiction: the ring breathes, and the deformation
    must be reported per speed band (:func:`metrics.deformation_by_speed`) or it
    reads as a middling number describing no instant of the run.

The sweep reaches and passes the **dynamic feasibility boundary**. Holding a
constant radius ``R`` about a centre moving at ``v_K`` needs a bearing-dependent
turn rate, worst case ``kappa = (V + v_K) / (R*V)``; at ``R = 70 m`` and
``V = 25 m/s`` that meets the ``1/45 m`` curvature bound at about
``v_K = 15.6 m/s``, and no standoff ring is holdable at all once ``v_K >= V``.
Cells at or past the boundary are **reported as results, marked infeasible**, not
dropped: where the geometry stops being possible is one of the things the
benchmark is for.

What this module is not
-----------------------
It runs and measures. It draws nothing (no ``matplotlib`` import anywhere in the
call path, so it runs headless and under test) and it decides nothing — no arm is
adopted here. Adoption is `ADR-007`, on this evidence, and arms B and C both
reinstate cost-based selection, which `ADR-001` superseded; running them is not
adopting them.
"""

import argparse
import json
import math
import sys
import time

from . import algorithms
from . import kangaroo as kang
from . import metrics
from . import tangent_error
from .config import HarnessConfig
from .estimator import KalmanFilter
from .state import Harness, PlaneState, TargetState


#: Fixed aircraft and scenario parameters, identical for every arm and every
#: cell. Changing one of these invalidates every recorded row, which is why they
#: are one dict here rather than defaults scattered over the call sites.
FIXED = {
    "airspeed_ms": 25.0,
    "turn_radius_m": 45.0,
    "orbit_radius_m": 70.0,
    "look_ahead_m": 50.0,
    "dt_s": 0.1,
}

#: Initial aircraft-to-target range, metres. The target starts due North of the
#: aircraft, which starts heading North at ``FIXED["airspeed_ms"]``.
START_RANGE_M = 300.0

#: Run length, seconds. 180 s at 12.5 m/s carries the target 2.25 km, so the
#: aircraft settles and then holds for well over half the run.
DURATION_S = 180.0

#: Tail fraction treated as steady state, matching `metrics.steady_state_stats`.
STEADY_FRACTION = 0.25

#: Target speeds swept, m/s. Ends at 15 m/s (speed ratio 0.6), which straddles
#: the feasibility boundary described in the module docstring.
DEFAULT_SPEEDS_MS = (0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0)

#: Target-speed profiles. ``mode`` is the kangaroo mode; ``elastic_base`` applies
#: only to the elastic profile.
PROFILES = {
    "constant": {"mode": "straight", "label": "constant speed"},
    "elastic": {"mode": "elastic", "label": "elastic (surge and ease)"},
}

#: The approaches under comparison. Each is a **complete configuration**, not a
#: bare algorithm name: ``overrides`` are applied to :data:`FIXED` to build the
#: `HarnessConfig`, and ``lookahead_steps`` is the state-side horizon (which must
#: be 0 for an arm that owns its own horizon — asserted, not assumed).
ARMS = {
    "0": {
        "label": "baseline — no prediction",
        "algorithm": "dubins_target_orbit",
        "overrides": {},
        "lookahead_steps": 0,
        "task": "TASK-025",
        "note": "CS onto the ring about the target's PRESENT position. Every "
                "other arm has to beat this to have earned its complexity.",
    },
    "A": {
        "label": "predicted centre, fixed horizon",
        "algorithm": "adaptive_db_circle",
        # k_horizon = 25 ticks (2.5 s) and n_replan = 1 are the cell TASK-033
        # measured at 10.62 m RMS ring error, and the benchmark reproduces that
        # figure before it is trusted to judge anything (TASK-039 acceptance).
        "overrides": {"replan_every": 1, "hold_policy": "plan"},
        "lookahead_steps": 25,
        "task": "TASK-033",
        "note": "One configured horizon, held for the whole run.",
    },
    "B": {
        "label": "adaptive horizon",
        "algorithm": "adaptive_horizon_cs",
        "overrides": {"replan_every": 5, "hold_policy": "plan"},
        "lookahead_steps": 0,
        "task": "TASK-022 / TASK-038",
        "note": "Arm A with the horizon selected per replan from the candidate "
                "set. Selection is suppressed on the ring, where no tangent "
                "point exists to score.",
    },
    "C": {
        "label": "receding-horizon geometric",
        "algorithm": "rh_geometric",
        "overrides": {"replan_every": 1},
        "lookahead_steps": 0,
        "task": "TASK-039",
        "note": "No ring construction. Scores rolled-out trajectories against "
                "the predicted standoff tube and applies only the first command.",
    },
}

#: Order arms are reported in. Explicit, because dict order is an implementation
#: detail and a comparison table whose row order drifts is hard to read across runs.
ARM_ORDER = ("0", "A", "B", "C")


def feasibility(orbit_radius_m, turn_radius_m, airspeed_ms, target_speed_ms):
    """Whether a constant-radius standoff is dynamically holdable at this speed.

    Worst case over ring bearing is abeam the target's motion, where the required
    curvature is ``(V + v_K) / (R*V)``; the aircraft can supply at most
    ``1/rho``. Above ``v_K >= V`` nothing holds the ring at any curvature,
    because the aircraft cannot out-run the centre on the upwind side.

    Returns:
        ``{"required_curvature_1pm", "curvature_bound_1pm",
        "required_radius_m", "feasible", "unreachable"}``. ``unreachable`` is the
        ``v_K >= V`` case and is distinct from merely infeasible: one is a
        curvature the airframe cannot supply, the other is a rendezvous that does
        not exist.
    """
    bound = 1.0 / turn_radius_m
    unreachable = target_speed_ms >= airspeed_ms
    required = (airspeed_ms + target_speed_ms) / (orbit_radius_m * airspeed_ms)
    return {
        "required_curvature_1pm": required,
        "curvature_bound_1pm": bound,
        "required_radius_m": 1.0 / required,
        "feasible": (not unreachable) and required <= bound,
        "unreachable": unreachable,
    }


def build_config(arm):
    """The `HarnessConfig` for one arm: :data:`FIXED` overlaid with its overrides.

    Raises:
        ValueError: If the arm sets a non-zero ``lookahead_steps`` for an
            algorithm that declares ``owns_horizon``. Such an algorithm projects
            the target itself, so a state-side horizon would be applied twice and
            the second lead would be invisible in every recorded figure.
    """
    settings = dict(FIXED)
    settings.update(arm["overrides"])
    settings["lookahead_steps"] = int(arm["lookahead_steps"])
    cls = algorithms.REGISTRY[arm["algorithm"]]
    if getattr(cls, "owns_horizon", False) and settings["lookahead_steps"]:
        raise ValueError(
            "%s declares owns_horizon: it selects its own prediction horizon "
            "and reads the un-projected estimate, so lookahead_steps must be 0. "
            "Got %r — the ring would be led twice."
            % (arm["algorithm"], settings["lookahead_steps"]))
    return HarnessConfig(**settings)


def run_cell(arm_id, target_speed_ms, profile="constant", duration_s=DURATION_S,
             start_range_m=START_RANGE_M):
    """Run one arm at one target speed under one profile.

    Returns:
        ``(harness, config)``. The harness carries the history and any
        ``stopped_reason``; nothing is measured here, so a caller can apply its
        own metrics to the same run.
    """
    arm = ARMS[arm_id]
    cfg = build_config(arm)
    spec = PROFILES[profile]

    estimator = None
    cls = algorithms.REGISTRY[arm["algorithm"]]
    if getattr(cls, "requires_estimate", False) or arm["lookahead_steps"]:
        estimator = KalmanFilter()
        estimator.init(start_range_m, 0.0)

    harness = Harness(
        plane=PlaneState(n_m=0.0, e_m=0.0, hdg_rad=0.0,
                         speed_ms=cfg.airspeed_ms),
        target=TargetState(n_m=start_range_m, e_m=0.0),
        algorithm=algorithms.build(arm["algorithm"],
                                   algorithms.config_dict(cfg)),
        dt_s=cfg.dt_s,
        turn_radius_m=cfg.turn_radius_m,
        kangaroo=kang.build(spec["mode"], speed_ms=target_speed_ms,
                            fwd_m=start_range_m, elastic_base="straight"),
        estimator=estimator,
        lookahead_steps=cfg.lookahead_steps,
    )
    harness.run(duration_s)
    return harness, cfg


def planning_time_ms(arm_id, target_speed_ms=12.5, profile="constant",
                     settle_s=60.0, samples=200):
    """Mean wall-clock cost of **one** ``guidance_point`` call, milliseconds.

    ``PR-002`` and ``PR-009`` require this **measured, not estimated**, and the
    per-cell wall clock cannot supply it: that figure includes the target model,
    the estimator, the integration step and the history append, so it is an upper
    bound on planning and not planning.

    Method. Fly the arm for ``settle_s`` so the samples come from a converged run
    rather than the opening approach — an arm whose cost depends on the phase
    would otherwise be measured in the cheap one. Then replay the recorded
    snapshots through a **fresh** algorithm instance, feeding each the
    ``algorithm_state`` the run actually produced, and time the calls.

    Two things this figure is not:

    * **It is not a Lua figure.** ArduPilot's scripting engine is interpreted and
      has a per-cycle instruction budget; a CPython measurement does not
      extrapolate to it. See ``docs/TASK-039-ISSUES.md`` ISSUE-C3.
    * **It is not portable between machines.** It is meaningful as a **ratio
      between arms** rather than as an absolute.

    Returns:
        ``{"mean_ms", "max_ms", "samples", "budget_ms"}``. ``budget_ms`` is the
        control period ``dt_s * 1000``, so the reader does not have to work out
        what the figure is competing with.
    """
    arm = ARMS[arm_id]
    cfg = build_config(arm)
    empty = {"mean_ms": None, "max_ms": None, "samples": 0,
             "budget_ms": cfg.dt_s * 1000.0}
    harness, _cfg = run_cell(arm_id, target_speed_ms, profile,
                             duration_s=settle_s)
    history = harness.history[-int(samples):]
    if not history:
        return empty

    algorithm = algorithms.build(arm["algorithm"], algorithms.config_dict(cfg))
    timings = []
    for i, sample in enumerate(history):
        # Rebuild the snapshot the algorithm saw. The recorded sample is the
        # POST-step state, which is close enough for a COST measurement and is
        # explicitly not close enough for a geometric one — nothing here reads
        # the result.
        previous = history[i - 1] if i else {}
        snapshot = {
            "t_s": sample["t_s"],
            "plane_n_m": sample["plane_n_m"], "plane_e_m": sample["plane_e_m"],
            "plane_hdg_rad": sample["plane_hdg_rad"],
            "plane_speed_ms": cfg.airspeed_ms,
            "target_n_m": sample["target_n_m"], "target_e_m": sample["target_e_m"],
            "target_vn_ms": sample["target_vn_ms"],
            "target_ve_ms": sample["target_ve_ms"],
            "algorithm_state": dict(previous.get("algorithm_state") or {}),
            "target_est": _estimate_from(sample, "target_est"),
            "target_est_raw": _estimate_from(sample, "target_est_raw"),
        }
        started = time.perf_counter()
        try:
            algorithm.guidance_point(snapshot)
        except Exception:
            # A replayed snapshot can land in a state the live run never did (a
            # post-step pose against a pre-step plan). Skip it rather than let
            # one unsolvable sample stand in for the arm's cost.
            continue
        timings.append((time.perf_counter() - started) * 1000.0)

    if not timings:
        return empty
    return {
        "mean_ms": sum(timings) / len(timings),
        "max_ms": max(timings),
        "samples": len(timings),
        "budget_ms": cfg.dt_s * 1000.0,
    }


def _estimate_from(sample, prefix):
    """Rebuild a ``{n_m, e_m, vn_ms, ve_ms}`` estimate from a history sample.

    The history records an estimate's position but not its velocity, so the
    target's true velocity stands in. Acceptable **only** because the sole caller
    is :func:`planning_time_ms`, which discards every result and measures time.
    """
    n = sample.get(prefix + "_n_m")
    if n is None:
        return None
    return {"n_m": n, "e_m": sample[prefix + "_e_m"],
            "vn_ms": sample["target_vn_ms"], "ve_ms": sample["target_ve_ms"]}


def measure(harness, cfg, arm_id, target_speed_ms, profile, elapsed_s=None):
    """Every reported metric for one finished cell, as one flat-ish dict.

    The five metric families ``TASK-039`` asks for, plus the run's own bookkeeping:

    * **standoff** — dual-centre steady-state ring statistics, about the true
      target *and* about the centre the arm held. Both, always: for a
      prediction-planning arm the true-target figure includes a *designed* lead
      and the held-centre figure excludes the thing the operator cares about, so
      either quoted alone misleads, in opposite directions (`TASK-033` D2).
    * **deformation** — achieved radius by bearing relative to the target's
      heading, and the same split by instantaneous speed band, which is the only
      honest way to report the elastic profile.
    * **prediction / geometry** — the `TASK-038` tangent-registration summary,
      including its coverage, restricted to transits inside the candidate range.
    * **curvature** — the largest curvature any arm commanded, against the
      ``1/rho`` bound (`FR-005`, `SR-002`).
    * **planning cost** — candidates scored and rollout steps per replan, for
      `PR-002` and `A-SW-002`.
    """
    hist = harness.history
    R = cfg.orbit_radius_m
    row = {
        "arm": arm_id,
        "arm_label": ARMS[arm_id]["label"],
        "algorithm": ARMS[arm_id]["algorithm"],
        "profile": profile,
        "target_speed_ms": target_speed_ms,
        "speed_ratio": target_speed_ms / cfg.airspeed_ms,
        "lookahead_steps": cfg.lookahead_steps,
        "replan_every": cfg.replan_every,
        "steps": len(hist),
        "stopped_reason": harness.stopped_reason,
        "wall_clock_s": elapsed_s,
    }
    row.update(feasibility(R, cfg.turn_radius_m, cfg.airspeed_ms,
                           target_speed_ms))

    if not hist:
        # A cell that produced nothing is a row of Nones, not an absent row.
        # Dropping it would make a table of failures look like a table of
        # successes with fewer columns (VR-012).
        row.update({"rms_ring_error_m": None, "mean_radius_m": None,
                    "rms_ring_error_held_m": None, "prediction_lead_m": None,
                    "settled": None, "deformation_spread_m": None,
                    "rms_e_tan_m": None, "mean_e_tan_m": None,
                    "e_tan_coverage": None, "max_curvature_1pm": None,
                    "curvature_ok": None, "deformation_bins_m": None,
                    "deformation_by_speed_m": None,
                    "candidates_scored": None, "rollout_steps": None,
                    "tick_wall_clock_ms": None})
        return row

    dual = metrics.dual_centre_stats(hist, R, fraction=STEADY_FRACTION)
    row["rms_ring_error_m"] = dual["target"]["rms_ring_error_m"]
    row["max_ring_error_m"] = dual["target"]["max_ring_error_m"]
    row["mean_radius_m"] = dual["target"]["mean_radius_m"]
    row["rms_ring_error_held_m"] = dual["ring"]["rms_ring_error_m"]
    row["prediction_lead_m"] = dual["mean_prediction_lead_m"]
    row["settled"] = dual["target"]["settled"]

    deform = metrics.orbit_deformation(hist, R, n_bins=4,
                                       fraction=STEADY_FRACTION)
    row["deformation_spread_m"] = deform["spread_m"]
    row["deformation_bins_m"] = [b["mean_radius_m"] for b in deform["bins"]]
    row["deformation_samples"] = deform["samples"]
    row["deformation_excluded_stationary"] = deform["excluded_stationary"]

    by_speed = metrics.deformation_by_speed(hist, R, n_bins=4, fraction=1.0)
    row["deformation_by_speed_m"] = dict(
        (label, {"spread_m": d["spread_m"], "samples": d["samples"]})
        for label, d in by_speed.items())

    tan = tangent_error.run_summary(hist, cfg)
    row["rms_e_tan_m"] = tan["rms_e_tan_m"]
    row["mean_e_tan_m"] = tan["mean_e_tan_m"]
    row["rms_cross_track_m"] = tan["rms_cross_track_m"]
    row["median_n_a_steps"] = tan["median_n_a_steps"]
    row["e_tan_coverage"] = tan["coverage"]

    curvatures = [(s.get("algorithm_state") or {}).get("curvature")
                  for s in hist]
    curvatures = [c for c in curvatures if c is not None]
    row["max_curvature_1pm"] = max(curvatures) if curvatures else None
    row["curvature_ok"] = (row["max_curvature_1pm"] is None
                           or row["max_curvature_1pm"] <= 1.0 / cfg.turn_radius_m
                           + 1e-9)

    last = hist[-1].get("algorithm_state") or {}
    row["candidates_scored"] = last.get("candidates_scored")
    row["rollout_steps"] = last.get("rollout_steps")
    # Whole-tick cost: the run's wall clock over its steps. An UPPER BOUND on
    # planning, because it includes the target model, the estimator, the
    # integration step and the history append; `planning_time_ms` isolates the
    # guidance call itself.
    if elapsed_s is not None and row["steps"]:
        row["tick_wall_clock_ms"] = elapsed_s / row["steps"] * 1000.0
    else:
        row["tick_wall_clock_ms"] = None
    return row


def sweep(arm_ids=None, speeds_ms=None, profiles=None, duration_s=DURATION_S,
          progress=None):
    """Run the full cross product and return one row per cell.

    Args:
        arm_ids: Arms to run, in :data:`ARM_ORDER` order. Default: all.
        speeds_ms: Target speeds, m/s. Default: :data:`DEFAULT_SPEEDS_MS`.
        profiles: Profile names. Default: both.
        duration_s: Run length, seconds.
        progress: Optional ``callable(row)`` invoked as each cell finishes, so a
            long sweep is not silent.

    Returns:
        A list of :func:`measure` dicts.
    """
    arm_ids = [a for a in ARM_ORDER if a in (arm_ids or ARM_ORDER)]
    speeds_ms = list(speeds_ms if speeds_ms is not None else DEFAULT_SPEEDS_MS)
    profiles = list(profiles or PROFILES)
    rows = []
    for profile in profiles:
        for speed in speeds_ms:
            for arm_id in arm_ids:
                started = time.time()
                harness, cfg = run_cell(arm_id, speed, profile, duration_s)
                row = measure(harness, cfg, arm_id, speed, profile,
                              elapsed_s=time.time() - started)
                rows.append(row)
                if progress is not None:
                    progress(row)
    return rows


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------

def _fmt(value, spec="%8.2f", width=8):
    if value is None:
        return " " * (width - 1) + "-"
    return spec % value


def format_table(rows, profile=None):
    """The comparison table as fixed-width text, one row per cell.

    ``ring`` is RMS ring error about the **true** target; ``held`` about the
    centre the arm held. ``spread`` is the orbit deformation. ``e_tan`` is the
    `TASK-038` registration RMS and ``cov`` its coverage — quoted together,
    because a small RMS over 5% of a run is not a small error.
    """
    out = []
    header = ("prof     v_K  beta  arm  %-28s %8s %8s %8s %8s %8s %5s %5s"
              % ("configuration", "ring", "held", "meanR", "spread", "e_tan",
                 "cov", "feas"))
    out.append(header)
    out.append("-" * len(header))
    for row in rows:
        if profile is not None and row["profile"] != profile:
            continue
        flag = "yes" if row["feasible"] else (
            "UNRE" if row["unreachable"] else "NO")
        out.append(
            "%-8s %4.1f %5.2f  %-3s  %-28s %s %s %s %s %s %5s %5s"
            % (row["profile"], row["target_speed_ms"], row["speed_ratio"],
               row["arm"], row["arm_label"][:28],
               _fmt(row["rms_ring_error_m"]),
               _fmt(row["rms_ring_error_held_m"]),
               _fmt(row["mean_radius_m"]),
               _fmt(row["deformation_spread_m"]),
               _fmt(row["rms_e_tan_m"]),
               "-" if row["e_tan_coverage"] is None
               else "%.2f" % row["e_tan_coverage"],
               flag))
    return "\n".join(out)


def write_csv(rows, path):
    """Write the scalar columns as CSV. Nested columns are omitted, not flattened.

    Flattening the per-bin and per-band dicts into a wide CSV would produce
    columns whose meaning depends on the bin count used at the time; the JSON
    output carries them intact instead.
    """
    columns = ("profile", "target_speed_ms", "speed_ratio", "arm", "algorithm",
               "lookahead_steps", "replan_every", "rms_ring_error_m",
               "rms_ring_error_held_m", "mean_radius_m", "max_ring_error_m",
               "prediction_lead_m", "settled", "deformation_spread_m",
               "rms_e_tan_m", "mean_e_tan_m", "rms_cross_track_m",
               "e_tan_coverage", "median_n_a_steps", "max_curvature_1pm",
               "curvature_ok", "required_curvature_1pm", "curvature_bound_1pm",
               "feasible", "unreachable", "steps", "stopped_reason",
               "candidates_scored", "rollout_steps", "wall_clock_s",
               "tick_wall_clock_ms")
    with open(path, "w") as handle:
        handle.write(",".join(columns) + "\n")
        for row in rows:
            values = []
            for name in columns:
                value = row.get(name)
                if value is None:
                    values.append("")
                elif isinstance(value, str):
                    values.append('"%s"' % value.replace('"', "'"))
                else:
                    values.append(str(value))
            handle.write(",".join(values) + "\n")
    return path


def build_parser():
    parser = argparse.ArgumentParser(
        description="Common moving-target benchmark for the three predictive "
                    "standoff approaches (TASK-039). Runs every arm over the "
                    "same speed sweep and reports one row per cell.")
    parser.add_argument("--arms", default=",".join(ARM_ORDER),
                        help="Comma-separated arm ids from %s (default: all)."
                             % ", ".join(ARM_ORDER))
    parser.add_argument("--speeds-ms", default=None,
                        help="Comma-separated target speeds, m/s (default: %s)."
                             % ", ".join(str(s) for s in DEFAULT_SPEEDS_MS))
    parser.add_argument("--profiles", default=",".join(sorted(PROFILES)),
                        help="Comma-separated profiles: constant, elastic.")
    parser.add_argument("--duration-s", type=float, default=DURATION_S,
                        help="Run length per cell, s (default: %(default)s).")
    parser.add_argument("--json", dest="json_path", default=None,
                        help="Write the full result set, nested columns "
                             "included, to this path.")
    parser.add_argument("--csv", dest="csv_path", default=None,
                        help="Write the scalar columns to this path.")
    parser.add_argument("--save-runs", dest="save_runs", default=None,
                        help="Directory to write each cell's history to as a "
                             "plotter run file, so the read-only plotter and "
                             "the offline renderer can draw them without "
                             "re-running the sweep.")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress the per-cell progress lines.")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    arm_ids = [a.strip() for a in args.arms.split(",") if a.strip()]
    unknown = [a for a in arm_ids if a not in ARMS]
    if unknown:
        raise SystemExit("unknown arm(s) %s; available: %s"
                         % (", ".join(unknown), ", ".join(ARM_ORDER)))
    speeds = (DEFAULT_SPEEDS_MS if args.speeds_ms is None
              else tuple(float(s) for s in args.speeds_ms.split(",")))
    profiles = [p.strip() for p in args.profiles.split(",") if p.strip()]
    unknown = [p for p in profiles if p not in PROFILES]
    if unknown:
        raise SystemExit("unknown profile(s) %s; available: %s"
                         % (", ".join(unknown), ", ".join(sorted(PROFILES))))

    def report(row):
        sys.stderr.write(
            "  %-8s v_K=%4.1f arm %-2s  ring=%s spread=%s  %4.1fs\n"
            % (row["profile"], row["target_speed_ms"], row["arm"],
               _fmt(row["rms_ring_error_m"]), _fmt(row["deformation_spread_m"]),
               row["wall_clock_s"] or 0.0))

    rows = sweep(arm_ids, speeds, profiles, args.duration_s,
                 progress=None if args.quiet else report)

    for profile in profiles:
        print()
        print("%s — %s" % (profile, PROFILES[profile]["label"]))
        print(format_table(rows, profile=profile))

    print()
    print("Planning cost per guidance call, over a converged run (PR-002, "
          "PR-009).")
    print("CPython on this machine: NOT a Lua figure and not portable between "
          "machines.")
    print("Read the ratio between arms, not the absolute.")
    print("%-4s %-24s %9s %9s %9s" % ("arm", "algorithm", "mean ms", "max ms",
                                      "budget ms"))
    print("-" * 60)
    timings = {}
    for arm_id in arm_ids:
        timing = planning_time_ms(arm_id)
        timings[arm_id] = timing
        print("%-4s %-24s %9s %9s %9.1f"
              % (arm_id, ARMS[arm_id]["algorithm"],
                 "-" if timing["mean_ms"] is None else "%.3f" % timing["mean_ms"],
                 "-" if timing["max_ms"] is None else "%.3f" % timing["max_ms"],
                 timing["budget_ms"]))

    breaches = [r for r in rows if r["curvature_ok"] is False]
    if breaches:
        print()
        print("CURVATURE BREACH in %d cell(s) — FR-005 / SR-002:" % len(breaches))
        for row in breaches:
            print("  arm %s %s v_K=%.1f: %.6f 1/m against a %.6f 1/m bound"
                  % (row["arm"], row["profile"], row["target_speed_ms"],
                     row["max_curvature_1pm"], row["curvature_bound_1pm"]))

    if args.json_path:
        with open(args.json_path, "w") as handle:
            json.dump({"fixed": FIXED, "start_range_m": START_RANGE_M,
                       "duration_s": args.duration_s,
                       "arms": ARMS, "planning_time_ms": timings,
                       "rows": rows}, handle, indent=2)
        print("\nwrote %s" % args.json_path)
    if args.csv_path:
        write_csv(rows, args.csv_path)
        print("wrote %s" % args.csv_path)
    if args.save_runs:
        import os
        from . import plotter
        os.makedirs(args.save_runs, exist_ok=True)
        for profile in profiles:
            for speed in speeds:
                for arm_id in arm_ids:
                    harness, cfg = run_cell(arm_id, speed, profile,
                                            args.duration_s)
                    name = "arm%s-%s-v%04.1f" % (arm_id, profile, speed)
                    path = os.path.join(args.save_runs, name + ".json")
                    plotter.save_run(path, name, harness.history,
                                     meta={"arm": arm_id, "profile": profile,
                                           "target_speed_ms": speed,
                                           "orbit_radius_m": cfg.orbit_radius_m,
                                           "algorithm": ARMS[arm_id]["algorithm"]})
        print("wrote run files -> %s" % args.save_runs)

    # A curvature breach is a requirement violation, not a bad number.
    return 1 if breaches else 0


if __name__ == "__main__":
    raise SystemExit(main())
