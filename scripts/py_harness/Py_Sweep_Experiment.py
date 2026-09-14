"""Sweep every guidance arm across every kangaroo mode permutation and five
target-speed ratios, one bundle per cell (``TASK-045``).

Created 2026-09-13. Module name per `TASK-045` decision `D7`.

The grid
--------
::

    5 guidance arms  x  8 kangaroo mode permutations  x  5 target-speed ratios

at 60 s per run, viewer off, every cell reproducible from ``seed + config``:

* **arms** come from :data:`benchmark.ARMS` in :data:`benchmark.ARM_ORDER`, and
  every cell's algorithm configuration is produced by
  :func:`benchmark.build_config`, so the parity guard
  (``DECLARED_ARM_DIFFERENCES``) governs here as it does there;
* **mode permutations** are ``base x pace`` (`TASK-030`: ``elastic`` is a
  modifier over the base modes): ``point``; ``straight``, ``circle`` and
  ``rectangle`` at constant and elastic pace; ``kangaroo_rand`` across
  :data:`RAND_SEEDS`. ``point`` has no speed axis and ``kangaroo_rand`` no
  elastic form (it is already a schedule);
* **speed ratios** are kangaroo speed as a fraction of the aircraft's
  ``airspeed_ms``, **tabulated** in :data:`SPEED_RATIOS` (`D1`) in the shape of
  :data:`cs_orbit_sweep.SPEED_CASES` and resolved by
  :func:`cs_orbit_sweep.speed_for`.

Three of the five ratios are past the feasibility boundary (`D3`): at
``R = 70 m``, ``rho = 45 m`` the ring is holdable only below about 0.62; 0.75
is curvature-infeasible and 1.0 and 1.5 are unreachable. Those cells are
**kept as controlled failures** — :func:`benchmark.feasibility` is recorded on
every row so a reader does not mistake "unreachable" for "the arm failed".

No new run loop
---------------
The sweep is a grid expander over the existing pieces. Every cell is one
experiment spec (`TASK-040`) run through :func:`experiment.session_from_spec`
and archived by :func:`experiment.write_bundle`, so a cell run here is
byte-for-byte the bundle ``python3 -m py_harness.experiment --spec`` would
write for the same spec. ``kangaroo_rand`` is expressed as its seeded leg list
(:func:`kangaroo.rand_legs`) with the seed recorded beside it.

Like-for-like starts
--------------------
:data:`INITIAL_CONDITIONS` is **one dict applied to every cell** — aircraft at
the origin on 140 deg (`D9`), kangaroo at (300, 0) — written into every
``spec.json`` and echoed once in ``MANIFEST.json``. Changing any value in it
invalidates every recorded row, exactly as :data:`benchmark.FIXED` does. Note
that ``benchmark.py`` keeps its own 0 deg start, so this grid's numbers are not
expected to reproduce the ``ISSUE-D1``..``D4`` figures exactly.

Sub-experiment S1 (``--sub chord``)
-----------------------------------
The baseline alone across control rate, carrot distance, pre-compensation and
Dubins path sampling, to settle whether resolution has any meaningful effect on
orbit chord-cutting. Its falsifiable prediction, written before the run, is in
:data:`S1_PREDICTION`; the transit cap ``n_a_max_steps`` is scaled to hold a
4.0 s window at every rate (``ISSUE-M5``).

S2 (the 350 m flight-area repeat) is **deliberately not implemented** here
(`D10`): it waits on the first write-up of the main grid.

Not a configuration of record
-----------------------------
No value in any spec, manifest or bundle written here is an approved
flight-safety limit (`SR-004`); every ``record.json`` carries that statement.
"""

import argparse
import csv
import datetime
import json
import math
import os
import sys
import time

from . import benchmark
from . import experiment
from . import kangaroo as kang
from .config import HarnessConfig
from .cs_orbit_sweep import speed_for


#: Campaign id and its default location: ``experiments/campaigns/<id>`` at the
#: **repository root** (four levels above this file), not under the vendored
#: ArduPilot tree the module happens to live in. Resolved from ``__file__`` so
#: the default is the same wherever the command is run from.
CAMPAIGN_ID = "CAMP-002-arm-mode-ratio"
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         "..", "..", "..", ".."))
DEFAULT_OUT_DIR = os.path.join(REPO_ROOT, "experiments", "campaigns",
                               CAMPAIGN_ID)

#: One campaign per arm table (`TASK-048`): the `TASK-045` grid over
#: :data:`benchmark.ARMS` is `CAMP-002`; the same grid over
#: :data:`benchmark.ARMS_HYST` (orbit-sense hysteresis on arms 0, D, A and B,
#: arm C unchanged) is `CAMP-003`. Separate directories, so neither overwrites
#: the other and every figure can be produced for both.
DEFAULT_ARM_SET = "base"
CAMPAIGN_IDS = {
    "base": CAMPAIGN_ID,
    "hyst": "CAMP-003-arm-mode-ratio-hyst",
}


def default_out_dir(arm_set=DEFAULT_ARM_SET):
    return os.path.join(REPO_ROOT, "experiments", "campaigns",
                        CAMPAIGN_IDS[arm_set])


def arm_table(arm_set=DEFAULT_ARM_SET):
    """``(ARMS, ARM_ORDER)`` for a named arm set (`benchmark.ARM_SETS`)."""
    if arm_set not in benchmark.ARM_SETS:
        raise ValueError("unknown arm set %r; use one of %s"
                         % (arm_set, ", ".join(sorted(benchmark.ARM_SETS))))
    return benchmark.ARM_SETS[arm_set]

#: Run length per cell, seconds. Shorter than the benchmark's 180 s (`D5`):
#: the steady tail is the final 15 s and ``settled`` is carried per cell.
DURATION_S = 60.0

#: The ``e_tan`` transit window, seconds, held constant across control rates
#: by passing ``n_a_max_steps = round(TRANSIT_WINDOW_S / dt_s)`` explicitly
#: (``ISSUE-M5``: the default cap is tick-denominated). 4.0 s at the main
#: grid's 10 Hz is the 40-tick default.
TRANSIT_WINDOW_S = 4.0

#: Target speed as a fraction of ``airspeed_ms`` (`D1`): ``(name, ratio)`` in
#: the shape of :data:`cs_orbit_sweep.SPEED_CASES`. Overridable with
#: ``--speed-ratios``; whichever table was used is written to ``MANIFEST.json``.
SPEED_RATIOS = (
    ("quarter", 0.25),
    ("half", 0.50),
    ("three-quarter", 0.75),
    ("equal", 1.00),
    ("one-and-half", 1.50),
)

#: Fixed, recorded and asserted for every cell. The aircraft always starts at
#: the origin (:data:`experiment.PLANE_START_NE`); the kangaroo starts at the
#: same point whatever its mode, because :func:`kangaroo.make_segments`
#: offsets every mode to the placed start. Units: metres and degrees.
INITIAL_CONDITIONS = {
    "plane_n_m": 0.0,
    "plane_e_m": 0.0,
    "plane_heading_deg": 140.0,          # D9: the Chapter 3 figures' heading
    "start_range_m": 300.0,
    "target_n_m": 300.0,
    "target_e_m": 0.0,
    "kangaroo_heading_deg": 0.0,         # straight / elastic-straight go North
}

#: Circle and rectangle geometry, metres — the experiment spec defaults.
KANGAROO_GEOMETRY = {"radius_m": 150.0, "length_m": 300.0, "width_m": 150.0}

#: The harness default inclusion zone. The 350 m flight-area repeat (S2) is
#: not implemented (`D10`).
ZONE = {"side_m": 2000.0, "contain_target": True, "containment_margin_m": None}

#: ``kangaroo_rand`` leg-length bounds, seconds — :func:`kangaroo.build`'s
#: ``rand_min_s`` / ``rand_max_s`` defaults, stated here because the spec path
#: expands the seeded schedule into explicit legs.
RAND_MIN_S = 5.0
RAND_MAX_S = 20.0

#: The seeds every arm is run against. Identical across arms, asserted.
RAND_SEEDS = (1, 2, 3, 4, 5)

#: ``(base, pace)`` permutations, in reporting order. ``elastic`` is the slow
#: phase at ``ELASTIC_SLOW_FACTOR`` of the tabulated speed, which is the
#: **fast** phase — a ratio label on an elastic cell means the fast phase.
MODE_PERMUTATIONS = (
    ("point", "constant"),
    ("straight", "constant"),
    ("straight", "elastic"),
    ("circle", "constant"),
    ("circle", "elastic"),
    ("rectangle", "constant"),
    ("rectangle", "elastic"),
    ("rand", "constant"),
)

#: Cell status values in ``MANIFEST.json``.
STATUS_PLANNED = "planned"
STATUS_COMPLETE = "complete"
STATUS_REPLAY_FAILED = "replay_failed"
STATUS_CURVATURE_BREACH = "curvature_breach"
STATUS_ERROR = "error"

#: Sub-experiment names.
SUB_MAIN = "main"
SUB_CHORD = "chord"
SUBS = (SUB_MAIN, SUB_CHORD)

# --------------------------------------------------------------------------
# Sub-experiment S1 — chord-cutting against replanning and carrot resolution
# --------------------------------------------------------------------------

#: Written before the run. Evaluated by :func:`evaluate_s1` after it.
S1_PREDICTION = (
    "With pre-compensation OFF, the flown radius converges monotonically on "
    "R cos(L/R) as dt_s falls, and the residual above the floor is below "
    "0.5 m by 40 Hz; with it ON, the radius is within 0.1 m of R at every "
    "cell. If either fails, the ISSUE-M6 probe was wrong.")
S1_RESIDUAL_BOUND_M = 0.5
S1_RESIDUAL_RATE_HZ = 40.0
S1_PRECOMP_TOL_M = 0.1

S1_DT_S = (0.2, 0.1, 0.05, 0.025, 0.01)
S1_LOOK_AHEAD_M = (70.0, 50.0, 35.0, 25.0, 15.0, 8.0)
S1_PRECOMPENSATE = (True, False)
#: Path sampling: default, and four times finer (`D8`, kept).
S1_SAMPLING = ("dflt", "fine")
S1_SAMPLING_FINE_FACTOR = 4.0
#: Targets: stationary isolates chord-cutting from prediction lead; the moving
#: case checks the answer survives motion.
S1_TARGETS = ("point", "straight")
S1_STRAIGHT_RATIO = ("half", 0.5)


# --------------------------------------------------------------------------
# Grid expansion
# --------------------------------------------------------------------------

def ratio_table(ratios=None):
    """``[(name, ratio), ...]`` — the default table, or an override.

    An override given as bare numbers is named ``r<value>`` so the cell ids
    stay sortable and self-describing; a value present in :data:`SPEED_RATIOS`
    keeps its tabulated name.
    """
    if ratios is None:
        return list(SPEED_RATIOS)
    named = dict((r, n) for n, r in SPEED_RATIOS)
    out = []
    for item in ratios:
        if isinstance(item, (tuple, list)):
            out.append((str(item[0]), float(item[1])))
        else:
            value = float(item)
            out.append((named.get(value, "r%g" % value), value))
    return out


def cell_id(arm_id, base, pace, ratio_name=None, seed=None):
    """``<arm>-<base>-<pace>-<ratio-name>[-s<seed>]``; ``<arm>-point`` for the
    stationary target, which has neither a pace nor a speed axis."""
    if base == "point":
        return "%s-point" % arm_id
    out = "%s-%s-%s-%s" % (arm_id, base, pace, ratio_name)
    if seed is not None:
        out += "-s%02d" % int(seed)
    return out


def _fixed_config_fields(arm_id, arm_set=DEFAULT_ARM_SET):
    """The spec ``aircraft`` and ``algorithm`` sections for one arm, from
    :func:`benchmark.build_config` — so the arm's configuration is the
    benchmark's, not a re-typing of it."""
    arm = arm_table(arm_set)[0][arm_id]
    cfg = benchmark.build_config(arm)
    aircraft = dict((name, getattr(cfg, name))
                    for name in experiment.AIRCRAFT_FIELDS)
    from . import algorithms
    cls = algorithms.REGISTRY[arm["algorithm"]]
    overrides = dict((k, v) for k, v in arm["overrides"].items()
                     if k not in experiment.ALGORITHM_CONFIG_FIELDS)
    algorithm = {
        "name": arm["algorithm"],
        "estimate": bool(getattr(cls, "requires_estimate", False)
                         or cfg.lookahead_steps),
        "lookahead_steps": int(cfg.lookahead_steps),
        "replan_every": cfg.replan_every,
        "hold_policy": cfg.hold_policy,
        "orbit_precompensate": cfg.orbit_precompensate,
        "overrides": overrides,
    }
    return aircraft, algorithm, cfg


def kangaroo_legs(base, pace, speed_ms, seed=None, duration_s=DURATION_S,
                  heading_deg=None):
    """The spec legs for one mode permutation at one speed.

    ``elastic`` legs carry ``elastic_base`` (`D2`) so ``elastic x circle`` and
    ``elastic x rectangle`` are reachable. ``rand`` expands the seeded
    schedule into explicit legs covering the run (:func:`kangaroo.rand_legs`).
    """
    heading = (INITIAL_CONDITIONS["kangaroo_heading_deg"] if heading_deg is None
               else heading_deg)
    if base == "point":
        return [{"duration_s": duration_s, "mode": "point",
                 "heading_deg": heading, "speed_ms": 0.0}]
    if base == "rand":
        if pace != "constant":
            raise ValueError("kangaroo_rand has no elastic form: it is already "
                             "a schedule")
        legs = kang.rand_legs(seed, speed_ms, RAND_MIN_S, RAND_MAX_S, duration_s)
        return [experiment.leg_dict(leg) for leg in legs]
    if pace == "constant":
        return [{"duration_s": duration_s, "mode": base,
                 "heading_deg": heading, "speed_ms": speed_ms}]
    if pace == "elastic":
        return [{"duration_s": duration_s, "mode": kang.ELASTIC_MODE,
                 "heading_deg": heading, "speed_ms": speed_ms,
                 kang.ELASTIC_BASE_FIELD: base}]
    raise ValueError("unknown pace %r" % pace)


def build_spec(arm_id, base, pace, ratio_name=None, speed_ratio=None,
               seed=None, duration_s=DURATION_S, objective=None,
               arm_set=DEFAULT_ARM_SET):
    """One complete experiment spec for one cell. Viewer off, every field
    explicit, initial conditions from :data:`INITIAL_CONDITIONS`."""
    aircraft, algorithm, cfg = _fixed_config_fields(arm_id, arm_set)
    speed_ms = (0.0 if base == "point"
                else speed_for(speed_ratio, cfg.airspeed_ms))
    spec = experiment.default_spec()
    spec["experiment_id"] = cell_id(arm_id, base, pace, ratio_name, seed)
    spec["objective"] = objective or (
        "TASK-045 grid cell: arm %s (%s) against a %s kangaroo at %s pace, "
        "speed ratio %s" % (arm_id, algorithm["name"], base, pace,
                            "n/a" if speed_ratio is None else speed_ratio))
    spec["aircraft"] = aircraft
    spec["algorithm"] = algorithm
    spec["initial_conditions"] = dict(INITIAL_CONDITIONS)
    spec["kangaroo"] = dict(KANGAROO_GEOMETRY)
    spec["kangaroo"]["seed"] = None if seed is None else int(seed)
    spec["kangaroo"]["legs"] = kangaroo_legs(base, pace, speed_ms, seed,
                                             duration_s)
    spec["zone"] = dict(ZONE)
    spec["run"] = {"duration_s": duration_s, "visualise": False}
    return experiment.validate_spec(spec)


def _cell(sub, arm_id, base, pace, ratio_name, speed_ratio, seed, spec,
          n_a_max_steps, extra=None, arm_set=DEFAULT_ARM_SET):
    cfg = experiment.config_from_spec(spec)
    speed_ms = 0.0 if base == "point" else speed_for(speed_ratio, cfg.airspeed_ms)
    out = {
        "cell_id": spec["experiment_id"],
        "sub": sub,
        "arm_set": arm_set,
        "arm": arm_id,
        "algorithm": spec["algorithm"]["name"],
        "mode_base": base,
        "mode_pace": pace,
        "ratio_name": ratio_name,
        "speed_ratio": speed_ratio,
        "target_speed_ms": speed_ms,
        "seed": seed,
        "n_a_max_steps": n_a_max_steps,
        "feasibility": benchmark.feasibility(
            cfg.orbit_radius_m, cfg.turn_radius_m, cfg.airspeed_ms, speed_ms),
        "spec": spec,
    }
    out.update(extra or {})
    return out


def expand_grid(arms=None, modes=None, ratios=None, seeds=None,
                duration_s=DURATION_S, arm_set=DEFAULT_ARM_SET):
    """The main grid as a list of cell dicts, in reporting order.

    Args:
        arms: Arm ids, filtered to the arm set's order. Default all.
        modes: Permutation filters — ``"straight"`` (both paces) or
            ``"straight-elastic"``. Default all eight.
        ratios: Ratio table override; see :func:`ratio_table`.
        seeds: ``kangaroo_rand`` seeds. Default :data:`RAND_SEEDS`.
        arm_set: Which arm table (`benchmark.ARM_SETS`); default the
            `TASK-045` one.
    """
    _arms, order = arm_table(arm_set)
    arm_ids = [a for a in order if a in (arms or order)]
    table = ratio_table(ratios)
    seeds = list(RAND_SEEDS if seeds is None else seeds)
    wanted = _permutations(modes)
    n_a = int(round(TRANSIT_WINDOW_S / benchmark.FIXED["dt_s"]))
    cells = []
    for base, pace in wanted:
        if base == "point":
            for arm_id in arm_ids:
                spec = build_spec(arm_id, base, pace, duration_s=duration_s,
                                  arm_set=arm_set)
                cells.append(_cell(SUB_MAIN, arm_id, base, pace, None, None,
                                   None, spec, n_a, arm_set=arm_set))
            continue
        for name, ratio in table:
            cell_seeds = seeds if base == "rand" else [None]
            for seed in cell_seeds:
                for arm_id in arm_ids:
                    spec = build_spec(arm_id, base, pace, name, ratio, seed,
                                      duration_s, arm_set=arm_set)
                    cells.append(_cell(SUB_MAIN, arm_id, base, pace, name,
                                       ratio, seed, spec, n_a, arm_set=arm_set))
    return cells


def _permutations(modes):
    if not modes:
        return list(MODE_PERMUTATIONS)
    out = []
    for base, pace in MODE_PERMUTATIONS:
        for wanted in modes:
            if wanted == base or wanted == "%s-%s" % (base, pace):
                out.append((base, pace))
                break
    unknown = [m for m in modes
               if not any(m == b or m == "%s-%s" % (b, p)
                          for b, p in MODE_PERMUTATIONS)]
    if unknown:
        raise ValueError("unknown mode filter(s) %s; use a base (%s) or "
                         "base-pace (e.g. circle-elastic)"
                         % (", ".join(unknown),
                            ", ".join(sorted(set(b for b, _ in MODE_PERMUTATIONS)))))
    return out


def s1_cell_id(target, dt_s, look_ahead_m, precompensate, sampling):
    """``S1-<target>-dt<dt_s>-L<look_ahead>-precomp<ON|OFF>-<dflt|fine>``."""
    return "S1-%s-dt%.3f-L%d-precomp%s-%s" % (
        target, dt_s, int(round(look_ahead_m)),
        "ON" if precompensate else "OFF", sampling)


def expand_s1(duration_s=DURATION_S):
    """The 240 S1 cells: baseline only, every axis crossed."""
    defaults = HarnessConfig()
    cells = []
    for target in S1_TARGETS:
        for dt_s in S1_DT_S:
            for look in S1_LOOK_AHEAD_M:
                for precomp in S1_PRECOMPENSATE:
                    for sampling in S1_SAMPLING:
                        if target == "point":
                            base, ratio_name, ratio = "point", None, None
                        else:
                            base = "straight"
                            ratio_name, ratio = S1_STRAIGHT_RATIO
                        spec = build_spec("0", base, "constant", ratio_name,
                                          ratio, None, duration_s)
                        spec["experiment_id"] = s1_cell_id(target, dt_s, look,
                                                           precomp, sampling)
                        spec["objective"] = (
                            "TASK-045 S1: chord-cutting against resolution — "
                            "%s target, %.3f s control step, %g m look-ahead, "
                            "pre-compensation %s, %s path sampling"
                            % (target, dt_s, look, "ON" if precomp else "OFF",
                               "default" if sampling == "dflt" else "4x finer"))
                        spec["aircraft"]["dt_s"] = dt_s
                        spec["aircraft"]["look_ahead_m"] = look
                        spec["algorithm"]["orbit_precompensate"] = precomp
                        if sampling == "fine":
                            spec["algorithm"]["overrides"] = {
                                "delta_d_m": defaults.delta_d_m / S1_SAMPLING_FINE_FACTOR,
                                "delta_psi_rad": defaults.delta_psi_rad / S1_SAMPLING_FINE_FACTOR,
                            }
                        experiment.validate_spec(spec)
                        n_a = int(round(TRANSIT_WINDOW_S / dt_s))
                        cells.append(_cell(
                            SUB_CHORD, "0", base, "constant", ratio_name, ratio,
                            None, spec, n_a,
                            extra={"s1": {"target": target, "dt_s": dt_s,
                                          "rate_hz": 1.0 / dt_s,
                                          "look_ahead_m": look,
                                          "precompensate": precomp,
                                          "sampling": sampling}}))
    return cells


def expand(sub=SUB_MAIN, **kwargs):
    if sub == SUB_MAIN:
        return expand_grid(**kwargs)
    if sub == SUB_CHORD:
        if kwargs.get("arm_set", DEFAULT_ARM_SET) != DEFAULT_ARM_SET:
            raise ValueError("S1 (--sub chord) is a baseline-resolution study "
                             "and runs on the base arm set only")
        return expand_s1(duration_s=kwargs.get("duration_s", DURATION_S))
    raise ValueError("unknown sub-experiment %r; use one of %s"
                     % (sub, ", ".join(SUBS)))


# --------------------------------------------------------------------------
# Manifest and specs on disk
# --------------------------------------------------------------------------

def manifest_path(out_dir):
    return os.path.join(out_dir, "MANIFEST.json")


def spec_path(out_dir, cell_id_):
    return os.path.join(out_dir, "spec", cell_id_ + ".json")


def bundle_path(out_dir, cell_id_):
    return os.path.join(out_dir, cell_id_)


def load_manifest(out_dir):
    path = manifest_path(out_dir)
    if not os.path.isfile(path):
        return None
    with open(path) as handle:
        return json.load(handle)


def save_manifest(manifest, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    with open(manifest_path(out_dir), "w") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    return manifest_path(out_dir)


def _manifest_entry(cell):
    feas = cell["feasibility"]
    entry = {
        "status": STATUS_PLANNED,
        "sub": cell["sub"],
        "arm_set": cell.get("arm_set", DEFAULT_ARM_SET),
        "arm": cell["arm"],
        "algorithm": cell["algorithm"],
        "mode_base": cell["mode_base"],
        "mode_pace": cell["mode_pace"],
        "ratio_name": cell["ratio_name"],
        "speed_ratio": cell["speed_ratio"],
        "target_speed_ms": cell["target_speed_ms"],
        "seed": cell["seed"],
        "n_a_max_steps": cell["n_a_max_steps"],
        "feasible": feas["feasible"],
        "unreachable": feas["unreachable"],
        "spec": None,
        "bundle": None,
        "replay_ok": None,
        "curvature_ok": None,
        "partial": None,
        "wall_clock_s": None,
        "error": None,
    }
    if "s1" in cell:
        entry["s1"] = dict(cell["s1"])
    return entry


def plan(out_dir, sub=SUB_MAIN, arms=None, modes=None, ratios=None,
         seeds=None, duration_s=DURATION_S, arm_set=DEFAULT_ARM_SET):
    """Expand the grid and write ``MANIFEST.json`` plus one spec per cell.

    Runs nothing. A second ``plan`` for another sub-experiment adds to the
    manifest rather than replacing it; re-planning the same sub replaces that
    sub's cells (and resets their status).

    Raises:
        ValueError: If a generated spec's initial conditions differ from the
            manifest's — the like-for-like guarantee, checked rather than
            assumed.
    """
    cells = expand(sub, arms=arms, modes=modes, ratios=ratios, seeds=seeds,
                   duration_s=duration_s, arm_set=arm_set)
    arms_table, arm_order = arm_table(arm_set)
    for cell in cells:
        if cell["spec"]["initial_conditions"] != INITIAL_CONDITIONS:
            raise ValueError(
                "cell %s carries initial conditions %r, which differ from "
                "INITIAL_CONDITIONS; every cell must start from the same place"
                % (cell["cell_id"], cell["spec"]["initial_conditions"]))

    manifest = load_manifest(out_dir) or {
        "campaign": CAMPAIGN_IDS[arm_set],
        "task": "TASK-045" if arm_set == DEFAULT_ARM_SET else "TASK-048",
        "arm_set": arm_set,
        "module": "py_harness.Py_Sweep_Experiment",
        "created_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(timespec="seconds"),
        "not_a_flight_configuration": (
            "Harness configuration for offline geometric validation. No value "
            "here is an approved flight-safety limit (SR-004)."),
        "fixed": dict(benchmark.FIXED),
        "duration_s": duration_s,
        "transit_window_s": TRANSIT_WINDOW_S,
        "initial_conditions": dict(INITIAL_CONDITIONS),
        "kangaroo_geometry": dict(KANGAROO_GEOMETRY),
        "zone": dict(ZONE),
        "arms": dict((a, {"algorithm": arms_table[a]["algorithm"],
                          "label": arms_table[a]["label"]})
                     for a in arm_order),
        "mode_permutations": [list(p) for p in MODE_PERMUTATIONS],
        "subs": {},
        "cells": {},
    }
    if manifest["initial_conditions"] != INITIAL_CONDITIONS:
        raise ValueError(
            "MANIFEST.json at %s was planned with initial conditions %r; this "
            "module's INITIAL_CONDITIONS are %r. Plan into a fresh directory "
            "rather than mixing starts." % (out_dir,
                                            manifest["initial_conditions"],
                                            INITIAL_CONDITIONS))
    if manifest.get("arm_set", DEFAULT_ARM_SET) != arm_set:
        raise ValueError(
            "MANIFEST.json at %s holds the %r arm set; refusing to plan the %r "
            "set into it. One campaign directory per arm set."
            % (out_dir, manifest.get("arm_set", DEFAULT_ARM_SET), arm_set))

    table = ratio_table(ratios)
    sub_block = {
        "cell_count": len(cells),
        "planned_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(timespec="seconds"),
        "arms": [a for a in arm_order if a in (arms or arm_order)],
        "speed_ratios": [list(r) for r in table],
        "speed_ratios_overridden": ratios is not None,
        "seeds": list(RAND_SEEDS if seeds is None else seeds),
    }
    if sub == SUB_CHORD:
        sub_block.update({
            "prediction": S1_PREDICTION,
            "dt_s": list(S1_DT_S), "look_ahead_m": list(S1_LOOK_AHEAD_M),
            "precompensate": list(S1_PRECOMPENSATE),
            "sampling": list(S1_SAMPLING), "targets": list(S1_TARGETS),
            "a_val_001": ("Python-harness result: kinematic aircraft, no L1, "
                          "no airframe response. Bounds what the geometry "
                          "contributes; says nothing about what L1 adds."),
        })
    manifest["subs"][sub] = sub_block
    # Replace this sub's cells; keep the others.
    manifest["cells"] = dict((k, v) for k, v in manifest["cells"].items()
                             if v.get("sub") != sub)

    os.makedirs(os.path.join(out_dir, "spec"), exist_ok=True)
    for cell in cells:
        path = spec_path(out_dir, cell["cell_id"])
        experiment.save_spec(cell["spec"], path)
        entry = _manifest_entry(cell)
        entry["spec"] = os.path.relpath(path, out_dir)
        manifest["cells"][cell["cell_id"]] = entry
    save_manifest(manifest, out_dir)
    return manifest, cells


def cells_from_manifest(manifest, out_dir, sub=None, only=None, resume=False):
    """Rebuild the cell list from the manifest and the specs on disk."""
    cells = []
    arm_set = manifest.get("arm_set", DEFAULT_ARM_SET)
    for cid, entry in manifest["cells"].items():
        if sub is not None and entry["sub"] != sub:
            continue
        if only is not None and cid not in only:
            continue
        if resume and entry["status"] == STATUS_COMPLETE:
            continue
        spec = experiment.load_spec(os.path.join(out_dir, entry["spec"]))
        cell = _cell(entry["sub"], entry["arm"], entry["mode_base"],
                     entry["mode_pace"], entry["ratio_name"],
                     entry["speed_ratio"], entry["seed"], spec,
                     entry["n_a_max_steps"],
                     extra={"s1": entry["s1"]} if "s1" in entry else None,
                     arm_set=arm_set)
        cells.append(cell)
    return cells


# --------------------------------------------------------------------------
# Running a cell
# --------------------------------------------------------------------------

def _identity(cell):
    return {"cell_id": cell["cell_id"], "arm": cell["arm"],
            "algorithm": cell["algorithm"], "mode_base": cell["mode_base"],
            "mode_pace": cell["mode_pace"], "speed_ratio": cell["speed_ratio"],
            "seed": cell["seed"]}


def s1_block(session, cell):
    """The S1 record block: flown radius over the **post-contact window**
    (comparable across rates, unlike a tail fraction), the `A-VAL-005` floor
    ``R cos(L/R)``, and the residual above it."""
    from . import metrics
    cfg = session.config
    R, L = cfg.orbit_radius_m, cfg.look_ahead_m
    predicted = R * math.cos(L / R)
    errors = metrics.signed_radial_errors(session.history, R, "target")
    i_c = metrics.first_contact_index(errors)
    flown = None
    if i_c is not None:
        post = [e + R for e in errors[i_c:] if e is not None]
        flown = sum(post) / len(post) if post else None
    block = dict(cell["s1"])
    block.update({
        "flown_radius_m": flown,
        "predicted_radius_m": predicted,
        "residual_m": None if flown is None else flown - predicted,
        "contact_tick": i_c,
        "a_val_001": ("Kinematic aircraft, no L1: this bounds what the "
                      "geometry contributes and says nothing about what L1 "
                      "adds."),
    })
    return block


def run_cell(cell, out_dir, render=False, verify=True, visualise=False):
    """Run one cell through the `TASK-040` bundle path.

    Returns ``(record, written, directory)``. A watched run (``visualise``)
    writes under ``<out>/watched/`` so it never replaces the campaign's cell.
    """
    spec = json.loads(json.dumps(cell["spec"]))
    session = experiment.session_from_spec(spec)
    started = time.time()
    if visualise:
        from . import scenario_driver
        scenario_driver.ScenarioView(session).show()
        spec["run"]["duration_s"] = session.t_s
        directory = os.path.join(out_dir, "watched", cell["cell_id"])
    else:
        session.run(spec["run"]["duration_s"])
        directory = bundle_path(out_dir, cell["cell_id"])
    elapsed = time.time() - started

    last = (session.history[-1].get("algorithm_state") or {}) if session.history else {}
    zone_time_outside = None
    if session.zone is not None:
        zone_time_outside = sum(b["duration_s"] for b in
                                session.zone.breaches(session.history, "plane"))
    cell_block = _identity(cell)
    cell_block.update({
        "sub": cell["sub"],
        "ratio_name": cell["ratio_name"],
        "target_speed_ms": cell["target_speed_ms"],
        "n_a_max_steps": cell["n_a_max_steps"],
        "initial_conditions": dict(INITIAL_CONDITIONS),
        "candidates_scored": last.get("candidates_scored"),
        "rollout_steps": last.get("rollout_steps"),
        "zone_plane_time_outside_s": zone_time_outside,
        "wall_clock_s": elapsed,
        "tick_wall_clock_ms": (elapsed / len(session.history) * 1000.0
                               if session.history else None),
    })
    cell_block.update(cell["feasibility"])
    extra = {"cell": cell_block}
    if cell["sub"] == SUB_CHORD:
        extra["s1"] = s1_block(session, cell)

    written = experiment.write_bundle(
        spec, session, verify=verify, render=render,
        n_a_max_steps=cell["n_a_max_steps"], directory=directory,
        cell=_identity(cell), extra_record=extra)
    with open(written["record"]) as handle:
        record = json.load(handle)
    return record, written, directory


def _update_entry(entry, record, directory, out_dir, error=None):
    if error is not None:
        entry["status"] = STATUS_ERROR
        entry["error"] = error
        return entry
    verification = record.get("replay_verification") or {}
    metrics_block = record.get("metrics") or {}
    entry["bundle"] = os.path.relpath(directory, out_dir)
    entry["replay_ok"] = (verification.get("history_matches")
                          if verification.get("attempted") else None)
    entry["curvature_ok"] = metrics_block.get("curvature_ok")
    entry["partial"] = metrics_block.get("partial")
    entry["wall_clock_s"] = (record.get("cell") or {}).get("wall_clock_s")
    entry["error"] = None
    if verification.get("attempted") and not verification.get("history_matches"):
        entry["status"] = STATUS_REPLAY_FAILED
    elif metrics_block.get("curvature_ok") is False:
        entry["status"] = STATUS_CURVATURE_BREACH
    else:
        entry["status"] = STATUS_COMPLETE
    return entry


def run_cells(cells, manifest, out_dir, render=False, verify=True,
              progress=None):
    """Run cells in order, updating and saving the manifest after each one.

    Returns the ids of cells that did not complete cleanly (replay failure,
    curvature breach or error). Nothing is retried with different settings:
    a cell that stops early is a bundle with ``partial = true``.
    """
    failed = []
    for index, cell in enumerate(cells):
        cid = cell["cell_id"]
        entry = manifest["cells"][cid]
        started = time.time()
        try:
            record, _written, directory = run_cell(cell, out_dir, render,
                                                   verify)
            _update_entry(entry, record, directory, out_dir)
        except Exception as exc:                     # recorded, not raised
            record = None
            _update_entry(entry, None, None, out_dir,
                          error="%s: %s" % (type(exc).__name__, exc))
        save_manifest(manifest, out_dir)
        if entry["status"] != STATUS_COMPLETE:
            failed.append(cid)
        if progress is not None:
            progress(index, len(cells), cid, entry, record,
                     time.time() - started)
    return failed


# --------------------------------------------------------------------------
# S1 evaluation
# --------------------------------------------------------------------------

def evaluate_s1(manifest, out_dir):
    """Evaluate :data:`S1_PREDICTION` against the recorded S1 bundles.

    Returns a dict with the per-clause outcomes and the numbers behind them.
    Pass or fail is recorded, not adjusted.
    """
    rows = []
    for cid, entry in manifest["cells"].items():
        if entry["sub"] != SUB_CHORD or entry["status"] != STATUS_COMPLETE:
            continue
        with open(os.path.join(out_dir, entry["bundle"], "record.json")) as fh:
            rows.append(json.load(fh)["s1"])

    # Clause 1: OFF, monotone convergence on the floor as dt_s falls, per
    # (target, look-ahead, sampling); residual < bound at every rate >= 40 Hz.
    off = [r for r in rows if not r["precompensate"] and r["flown_radius_m"] is not None]
    groups = {}
    for r in off:
        groups.setdefault((r["target"], r["look_ahead_m"], r["sampling"]), []).append(r)
    monotone_failures, residual_failures = [], []
    for key, group in groups.items():
        group.sort(key=lambda r: -r["dt_s"])          # coarse -> fine
        residuals = [r["residual_m"] for r in group]
        for a, b in zip(residuals, residuals[1:]):
            if b > a + 1e-9:
                monotone_failures.append({"group": list(key), "residuals": residuals})
                break
        for r in group:
            if r["rate_hz"] >= S1_RESIDUAL_RATE_HZ - 1e-9 and r["residual_m"] >= S1_RESIDUAL_BOUND_M:
                residual_failures.append({"cell": key, "rate_hz": r["rate_hz"],
                                          "residual_m": r["residual_m"]})
    # Clause 2: ON, |flown − R| < tol everywhere.
    on = [r for r in rows if r["precompensate"]]
    R = benchmark.FIXED["orbit_radius_m"]
    on_failures = [{"target": r["target"], "dt_s": r["dt_s"],
                    "look_ahead_m": r["look_ahead_m"], "sampling": r["sampling"],
                    "flown_radius_m": r["flown_radius_m"]}
                   for r in on
                   if r["flown_radius_m"] is None
                   or abs(r["flown_radius_m"] - R) >= S1_PRECOMP_TOL_M]
    return {
        "prediction": S1_PREDICTION,
        "cells_evaluated": len(rows),
        "clause_off_monotone": {"pass": not monotone_failures,
                                "groups": len(groups),
                                "failures": monotone_failures},
        "clause_off_residual_below_bound_at_40hz": {
            "pass": not residual_failures, "bound_m": S1_RESIDUAL_BOUND_M,
            "failures": residual_failures},
        "clause_on_within_tolerance": {"pass": not on_failures,
                                       "tolerance_m": S1_PRECOMP_TOL_M,
                                       "cells": len(on),
                                       "failures": on_failures},
        "outcome": ("PASS" if not (monotone_failures or residual_failures
                                   or on_failures) else "FAIL"),
    }


# --------------------------------------------------------------------------
# Aggregation — master.csv
# --------------------------------------------------------------------------

#: One row per cell, every scalar in ``record.json`` flattened. Explicit so
#: the figures can rely on the column set.
MASTER_COLUMNS = (
    "cell_id", "sub", "arm_set", "arm", "algorithm", "mode_base", "mode_pace",
    "ratio_name", "speed_ratio", "target_speed_ms", "seed",
    "feasible", "unreachable", "required_curvature_1pm",
    "status", "replay_ok", "partial", "stopped_reason", "steps", "duration_s",
    "lookahead_steps", "replan_every", "n_a_max_steps",
    "rms_ring_error_m", "max_ring_error_m", "mean_radius_m", "drift_m",
    "settled", "min_orbit_distance_m",
    "rms_ring_error_held_m", "max_ring_error_held_m", "mean_radius_held_m",
    "settled_held", "min_orbit_distance_held_m", "mean_prediction_lead_m",
    "deformation_spread_m", "deformation_samples",
    "rms_e_tan_m", "mean_e_tan_m", "max_abs_e_tan_m", "rms_cross_track_m",
    "median_n_a_steps", "e_tan_coverage", "e_tan_scored",
    "contact_tick", "t_contact_s", "post_contact_window_s",
    "post_contact_cum_radial_m_s", "post_contact_mean_radial_m",
    "post_contact_max_abs_radial_m", "post_contact_coverage",
    "t_contact_held_s", "post_contact_cum_radial_held_m_s",
    "post_contact_mean_radial_held_m",
    "plane_velocity_error_rms_ms", "plane_velocity_error_max_ms",
    "target_velocity_estimate_error_rms_ms",
    "target_velocity_estimate_error_max_ms",
    "target_velocity_estimate_error_post_contact_rms_ms",
    "max_curvature_1pm", "curvature_bound_1pm", "curvature_ok",
    "zone_plane_breaches", "zone_plane_max_depth_m", "zone_plane_time_outside_s",
    "zone_target_breaches", "zone_containment_turns",
    "reconvergence_events",
    "candidates_scored", "rollout_steps",
    "wall_clock_s", "tick_wall_clock_ms",
    "s1_target", "s1_dt_s", "s1_rate_hz", "s1_look_ahead_m",
    "s1_precompensate", "s1_sampling", "s1_flown_radius_m",
    "s1_predicted_radius_m", "s1_residual_m",
)


def _get(d, *path):
    for key in path:
        if not isinstance(d, dict):
            return None
        d = d.get(key)
    return d


def master_row(cid, entry, record):
    """Flatten one cell into a :data:`MASTER_COLUMNS` row. A cell without a
    record (error, not run) is a row of nulls with its identity and status —
    never an absent row (`VR-012`)."""
    m = (record or {}).get("metrics") or {}
    cell = (record or {}).get("cell") or {}
    s1 = (record or {}).get("s1") or {}
    ring_t = _get(m, "ring", "target") or {}
    ring_r = _get(m, "ring", "ring") or {}
    pc_t = _get(m, "post_contact", "target") or {}
    pc_r = _get(m, "post_contact", "ring") or {}
    vel_p = _get(m, "velocity", "plane") or {}
    vel_e = _get(m, "velocity", "target_estimate") or {}
    tan = m.get("tangent") or {}
    zone = m.get("zone") or {}
    cfg = (record or {}).get("config") or {}
    row = {
        "cell_id": cid, "sub": entry["sub"],
        "arm_set": entry.get("arm_set", DEFAULT_ARM_SET), "arm": entry["arm"],
        "algorithm": entry["algorithm"], "mode_base": entry["mode_base"],
        "mode_pace": entry["mode_pace"], "ratio_name": entry["ratio_name"],
        "speed_ratio": entry["speed_ratio"],
        "target_speed_ms": entry["target_speed_ms"], "seed": entry["seed"],
        "feasible": entry["feasible"], "unreachable": entry["unreachable"],
        "required_curvature_1pm": cell.get("required_curvature_1pm"),
        "status": entry["status"], "replay_ok": entry.get("replay_ok"),
        "partial": m.get("partial"), "stopped_reason": m.get("stopped_reason"),
        "steps": m.get("steps"), "duration_s": m.get("duration_s"),
        "lookahead_steps": cfg.get("lookahead_steps"),
        "replan_every": cfg.get("replan_every"),
        "n_a_max_steps": entry.get("n_a_max_steps"),
        "rms_ring_error_m": ring_t.get("rms_ring_error_m"),
        "max_ring_error_m": ring_t.get("max_ring_error_m"),
        "mean_radius_m": ring_t.get("mean_radius_m"),
        "drift_m": ring_t.get("drift_m"), "settled": ring_t.get("settled"),
        "min_orbit_distance_m": ring_t.get("min_orbit_distance_m"),
        "rms_ring_error_held_m": ring_r.get("rms_ring_error_m"),
        "max_ring_error_held_m": ring_r.get("max_ring_error_m"),
        "mean_radius_held_m": ring_r.get("mean_radius_m"),
        "settled_held": ring_r.get("settled"),
        "min_orbit_distance_held_m": ring_r.get("min_orbit_distance_m"),
        "mean_prediction_lead_m": _get(m, "ring", "mean_prediction_lead_m"),
        "deformation_spread_m": _get(m, "deformation", "spread_m"),
        "deformation_samples": _get(m, "deformation", "samples"),
        "rms_e_tan_m": tan.get("rms_e_tan_m"),
        "mean_e_tan_m": tan.get("mean_e_tan_m"),
        "max_abs_e_tan_m": tan.get("max_abs_e_tan_m"),
        "rms_cross_track_m": tan.get("rms_cross_track_m"),
        "median_n_a_steps": tan.get("median_n_a_steps"),
        "e_tan_coverage": tan.get("coverage"), "e_tan_scored": tan.get("scored"),
        "contact_tick": pc_t.get("contact_tick"),
        "t_contact_s": pc_t.get("t_contact_s"),
        "post_contact_window_s": pc_t.get("post_contact_window_s"),
        "post_contact_cum_radial_m_s": pc_t.get("post_contact_cum_radial_m_s"),
        "post_contact_mean_radial_m": pc_t.get("post_contact_mean_radial_m"),
        "post_contact_max_abs_radial_m": pc_t.get("post_contact_max_abs_radial_m"),
        "post_contact_coverage": pc_t.get("post_contact_coverage"),
        "t_contact_held_s": pc_r.get("t_contact_s"),
        "post_contact_cum_radial_held_m_s": pc_r.get("post_contact_cum_radial_m_s"),
        "post_contact_mean_radial_held_m": pc_r.get("post_contact_mean_radial_m"),
        "plane_velocity_error_rms_ms": vel_p.get("rms_ms"),
        "plane_velocity_error_max_ms": vel_p.get("max_ms"),
        "target_velocity_estimate_error_rms_ms": vel_e.get("rms_ms"),
        "target_velocity_estimate_error_max_ms": vel_e.get("max_ms"),
        "target_velocity_estimate_error_post_contact_rms_ms":
            _get(vel_e, "post_contact", "rms_ms"),
        "max_curvature_1pm": m.get("max_curvature_1pm"),
        "curvature_bound_1pm": m.get("curvature_bound_1pm"),
        "curvature_ok": m.get("curvature_ok"),
        "zone_plane_breaches": zone.get("plane_breaches"),
        "zone_plane_max_depth_m": zone.get("plane_max_depth_m"),
        "zone_plane_time_outside_s": cell.get("zone_plane_time_outside_s"),
        "zone_target_breaches": zone.get("target_breaches"),
        "zone_containment_turns": zone.get("containment_turns"),
        "reconvergence_events": (len(m["reconvergence"])
                                 if m.get("reconvergence") is not None else None),
        "candidates_scored": cell.get("candidates_scored"),
        "rollout_steps": cell.get("rollout_steps"),
        "wall_clock_s": cell.get("wall_clock_s"),
        "tick_wall_clock_ms": cell.get("tick_wall_clock_ms"),
        "s1_target": s1.get("target"), "s1_dt_s": s1.get("dt_s"),
        "s1_rate_hz": s1.get("rate_hz"), "s1_look_ahead_m": s1.get("look_ahead_m"),
        "s1_precompensate": s1.get("precompensate"),
        "s1_sampling": s1.get("sampling"),
        "s1_flown_radius_m": s1.get("flown_radius_m"),
        "s1_predicted_radius_m": s1.get("predicted_radius_m"),
        "s1_residual_m": s1.get("residual_m"),
    }
    return row


def aggregate(manifest, out_dir):
    """Write ``master.csv`` from every ``record.json`` the manifest names.

    Returns ``(path, rows)``. A `None` is written as an empty cell; nothing here
    averages anything, so a `None` cannot be flattened into a `0` by accident.
    """
    rows = []
    for cid in sorted(manifest["cells"]):
        entry = manifest["cells"][cid]
        record = None
        if entry.get("bundle"):
            path = os.path.join(out_dir, entry["bundle"], "record.json")
            if os.path.isfile(path):
                with open(path) as handle:
                    record = json.load(handle)
        rows.append(master_row(cid, entry, record))
    path = os.path.join(out_dir, "master.csv")
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(MASTER_COLUMNS)
        for row in rows:
            writer.writerow([experiment._csv_cell(row.get(c))
                             for c in MASTER_COLUMNS])
    return path, rows


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(
        prog="python3 -m py_harness.Py_Sweep_Experiment",
        description="TASK-045: every guidance arm x every kangaroo mode "
                    "permutation x five speed ratios, one bundle per cell, "
                    "plus sub-experiment S1 (--sub chord).")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--plan", action="store_true",
                        help="Expand the grid, write MANIFEST.json and one spec "
                             "per cell. Runs nothing.")
    action.add_argument("--all", action="store_true",
                        help="Run every planned cell of the sub-experiment.")
    action.add_argument("--cell", metavar="ID",
                        help="Run one cell by id (rendered unless --no-render).")
    action.add_argument("--aggregate", action="store_true",
                        help="Write master.csv from every record.json.")
    action.add_argument("--figures", nargs="*", metavar="F",
                        help="Produce the thesis figures (all, or the named "
                             "ones, e.g. F1 F4 F10) from master.csv and the "
                             "bundles. Never re-runs anything.")
    action.add_argument("--evaluate-s1", action="store_true",
                        help="Evaluate the S1 prediction against its bundles.")
    parser.add_argument("--sub", default=SUB_MAIN, choices=SUBS,
                        help="Sub-experiment: main (default) or chord (S1).")
    parser.add_argument("--arm-set", default=DEFAULT_ARM_SET,
                        choices=sorted(benchmark.ARM_SETS),
                        help="Arm table: base (TASK-045, CAMP-002) or hyst "
                             "(TASK-048, orbit-sense hysteresis, CAMP-003).")
    parser.add_argument("--out-dir", default=None,
                        help="Campaign directory (default: the arm set's "
                             "campaign under experiments/campaigns/).")
    parser.add_argument("--arms", nargs="+", default=None,
                        help="Subset of arm ids of the arm set (plan / run).")
    parser.add_argument("--modes", nargs="+", default=None,
                        help="Subset of mode permutations: a base (straight) "
                             "or base-pace (circle-elastic).")
    parser.add_argument("--speed-ratios", nargs="+", type=float, default=None,
                        help="Override the ratio table for a one-off; recorded "
                             "in MANIFEST.json.")
    parser.add_argument("--duration-s", type=float, default=DURATION_S,
                        help="Run length per cell (default: %(default)s).")
    parser.add_argument("--no-render", dest="render", action="store_false",
                        default=None,
                        help="Skip the PNGs (default for --all; D6).")
    parser.add_argument("--render", dest="render", action="store_true",
                        help="Draw the PNGs (default for --cell).")
    parser.add_argument("--no-verify", dest="verify", action="store_false",
                        help="Skip the replay verification. Recorded; the "
                             "acceptance run must verify.")
    parser.add_argument("--visualise", action="store_true",
                        help="Watch one --cell in the scenario driver. Refused "
                             "with --all.")
    parser.add_argument("--resume", action="store_true",
                        help="With --all: run only cells not yet complete.")
    parser.add_argument("--quiet", action="store_true",
                        help="Suppress the per-cell progress lines.")
    return parser


def _progress(index, total, cid, entry, record, elapsed):
    m = (record or {}).get("metrics") or {}
    ring = _get(m, "ring", "target", "rms_ring_error_m")
    contact = _get(m, "post_contact", "target", "t_contact_s")
    sys.stderr.write(
        "[%3d/%3d] %-40s %-14s ring=%s t_c=%s %5.1fs\n"
        % (index + 1, total, cid, entry["status"],
           "   -  " if ring is None else "%6.2f" % ring,
           "  -  " if contact is None else "%5.1f" % contact, elapsed))


def main(argv=None):
    args = build_parser().parse_args(argv)
    out_dir = args.out_dir or default_out_dir(args.arm_set)
    if args.arms:
        unknown = [a for a in args.arms if a not in arm_table(args.arm_set)[1]]
        if unknown:
            print("unknown arm(s) %s for arm set %r; available: %s"
                  % (", ".join(unknown), args.arm_set,
                     ", ".join(arm_table(args.arm_set)[1])), file=sys.stderr)
            return 2

    if args.visualise and not args.cell:
        print("REFUSED: --visualise is for one --cell only; the grid runs "
              "headless (TASK-045 reproducibility rules)", file=sys.stderr)
        return 2

    if args.plan:
        try:
            manifest, cells = plan(out_dir, sub=args.sub, arms=args.arms,
                                   modes=args.modes, ratios=args.speed_ratios,
                                   seeds=None, duration_s=args.duration_s,
                                   arm_set=args.arm_set)
        except ValueError as exc:
            print("REFUSED: %s" % exc, file=sys.stderr)
            return 2
        print("planned %d %s cells -> %s" % (len(cells), args.sub,
                                              manifest_path(out_dir)))
        for cell in cells:
            print("  %s" % cell["cell_id"])
        return 0

    manifest = load_manifest(out_dir)
    if manifest is None:
        print("no MANIFEST.json under %s; run --plan first" % out_dir,
              file=sys.stderr)
        return 2

    if args.aggregate:
        path, rows = aggregate(manifest, out_dir)
        print("wrote %s (%d rows)" % (path, len(rows)))
        return 0

    if args.evaluate_s1:
        result = evaluate_s1(manifest, out_dir)
        manifest["subs"].setdefault(SUB_CHORD, {})["evaluation"] = result
        save_manifest(manifest, out_dir)
        print(json.dumps(result, indent=2))
        return 0 if result["outcome"] == "PASS" else 1

    if args.figures is not None:
        from . import arm_figures
        made = arm_figures.make_figures(out_dir, args.figures or None)
        for name, path in made:
            print("  %-4s %s" % (name, path))
        return 0

    render = args.render
    if args.cell:
        if args.cell not in manifest["cells"]:
            print("unknown cell %r; see MANIFEST.json" % args.cell,
                  file=sys.stderr)
            return 2
        cells = cells_from_manifest(manifest, out_dir, only={args.cell})
        cell = cells[0]
        if args.visualise:
            record, written, directory = run_cell(cell, out_dir,
                                                  render=render is not False,
                                                  verify=args.verify,
                                                  visualise=True)
            print("watched cell written -> %s (not the campaign cell)"
                  % directory)
            return 0
        render = True if render is None else render
        failed = run_cells(cells, manifest, out_dir, render=render,
                           verify=args.verify,
                           progress=None if args.quiet else _progress)
        entry = manifest["cells"][args.cell]
        print("cell %s: %s -> %s" % (args.cell, entry["status"],
                                     bundle_path(out_dir, args.cell)))
        if entry["error"]:
            print("  error: %s" % entry["error"], file=sys.stderr)
        return 1 if failed else 0

    # --all
    render = False if render is None else render
    cells = cells_from_manifest(manifest, out_dir, sub=args.sub,
                                resume=args.resume)
    if args.arms:
        cells = [c for c in cells if c["arm"] in args.arms]
    if args.modes:
        wanted = _permutations(args.modes)
        cells = [c for c in cells if (c["mode_base"], c["mode_pace"]) in wanted]
    if args.speed_ratios:
        cells = [c for c in cells if c["speed_ratio"] in args.speed_ratios
                 or c["speed_ratio"] is None]
    print("running %d %s cells -> %s" % (len(cells), args.sub, out_dir))
    failed = run_cells(cells, manifest, out_dir, render=render,
                       verify=args.verify,
                       progress=None if args.quiet else _progress)
    statuses = {}
    for cid in manifest["cells"]:
        if manifest["cells"][cid]["sub"] == args.sub:
            st = manifest["cells"][cid]["status"]
            statuses[st] = statuses.get(st, 0) + 1
    print("status: " + ", ".join("%s=%d" % kv for kv in sorted(statuses.items())))
    if failed:
        print("%d cell(s) did not complete cleanly:" % len(failed),
              file=sys.stderr)
        for cid in failed:
            entry = manifest["cells"][cid]
            print("  %s: %s%s" % (cid, entry["status"],
                                  (" — " + entry["error"]) if entry["error"] else ""),
                  file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
