"""Stage the live kangaroo demonstration for ``sim_vehicle.py`` (``TASK-058``).

The campaign flies one fixed Python cell per SITL run and cannot show the
baseline against every mode and speed in a way someone can watch. This stages
``kangaroo_demo.lua`` instead: the baseline guidance law, a kangaroo whose
mode, pace, speed and heading are ``KDEM_*`` parameters changed from MAVProxy
during the flight, and an ``ADSB_VEHICLE`` broadcast so the map shows it.

Nothing is typed by hand (`TASK-052` P4): the algorithm, its flattened
HarnessConfig, the control interval, the estimator settings, the mode geometry
and the default speed and start range all come from a Python cell's
``spec.json`` (default the baseline's ``0H-straight-constant-half`` in
``CAMP-003``), through :func:`schedule.cell_table`, the function the campaign
uses. The ``rand`` mode carries a recorded leg list expanded from a fixed seed
by ``kangaroo.rand_legs`` (the schedule, not the seed, per
``harness_segments.lua``); the operator's speed is applied to it on the vehicle.

``--plan --arm <id>`` stages the physical-validation run plan instead of a
cell (:mod:`pv_plan`, 2026-10-10): that arm's spec, built from
``pv_plan.json`` and checked against the plan's fence, flown in the demo's
suite mode (``KDEM_MODE`` 5) from the site reference, so the plan can be
flown in SITL arm by arm before the on-board scripts are finished.

Staging reuses :mod:`stage_scripts`'s state file, so ``--restore`` (or
``python3 -m kangaroo_follow.stage_scripts --restore``) puts the flight scripts
back, and a campaign cell refuses to stage over a live demonstration.

Usage, from ``src/ardupilot/Tools/autotest`` (the package lives there, so
``python3 -m kangaroo_follow...`` fails with "No module named
'kangaroo_follow'" from anywhere else, as for every command in the README)::

    cd src/ardupilot/Tools/autotest
    python3 -m kangaroo_follow.demo --stage      # prints the sim_vehicle command
    ... fly, from src/ardupilot ...
    python3 -m kangaroo_follow.demo --restore    # again from Tools/autotest
"""

import argparse
import json
import math
import os
import shutil

from . import paths, schedule, stage_scripts
from . import fence as fence_mod

from py_harness import kangaroo as kang

#: The Lua a demonstration flies, beside the campaign's in ``ArduPlane_Tests``.
DEMO_SCRIPT = "kangaroo_demo.lua"
ADSB_MODULE = "sitl_adsb.lua"
#: Parameters layered on kangaroo-follow.parm for the demonstration only.
DEMO_PARAM_FILE = os.path.join(paths.PARAMS_DIR, "kangaroo-demo.parm")

#: The Python cell the demonstration's configuration is taken from.
DEFAULT_CAMPAIGN = os.path.join(paths.CAMPAIGNS_DIR, "CAMP-003-arm-mode-ratio-hyst")
DEFAULT_CELL = "0H-straight-constant-half"

#: KDEM_MODE codes, in ``kangaroo.MODES`` order then ``kangaroo_rand``;
#: must match MODE_NAMES in kangaroo_demo.lua.
MODE_CODES = {"point": 0, "straight": 1, "circle": 2, "rectangle": 3, "rand": 4,
              "suite": 5}

#: The ``kangaroo_rand`` expansion carried to the vehicle: the harness's own
#: defaults (``kangaroo.build`` rand_min_s, rand_max_s, rand_horizon_s) and
#: the composite schedule's seed (``kangaroo.COMPOSITE_RAND_SEED``).
RAND_SEED = kang.COMPOSITE_RAND_SEED
RAND_MIN_S = 5.0
RAND_MAX_S = 20.0
RAND_HORIZON_S = 3600.0

#: Default containment radius about the start, m: half the side of the 2 km
#: zone the recorded campaigns ran in. KDEM_BOUND_M changes it in flight.
#: Used only when no fence is staged (``--no-fence``): with one (the default,
#: `ADR-012`) the kangaroo is contained in the fence instead.
DEFAULT_BOUND_M = 1000.0

#: ``fence_path`` default: the fence ``environment.json`` pins.
ENVIRONMENT_FENCE = "environment"

#: MAVProxy command the operator runs, from the ArduPilot checkout.
SIM_VEHICLE = ("Tools/autotest/sim_vehicle.py -v ArduPlane -L {location} "
               "--console --map --add-param-file={param_file} "
               "--add-param-file={demo_param_file}")


def _cell_spec(campaign_dir, cell_id):
    from . import campaign
    cells = campaign.python_cells(campaign_dir)
    if cell_id not in cells:
        raise KeyError("%s is not a complete Python cell of %s"
                       % (cell_id, paths.rel(campaign_dir)))
    directory, entry = cells[cell_id]
    with open(os.path.join(directory, entry["spec"])) as handle:
        return json.load(handle), entry


def with_look_ahead(spec, look_ahead_m):
    """The spec with its carrot look-ahead replaced, as the thesis's carrot
    sweep re-ran cells with ``look_ahead_m`` overridden (the short-carrot
    baseline is 5 m, two ticks at 25 m/s and 10 Hz). ``None`` leaves it."""
    if look_ahead_m is None:
        return spec
    if not look_ahead_m > 0.0:
        raise ValueError("look_ahead_m must be > 0, got %r" % look_ahead_m)
    spec = json.loads(json.dumps(spec))
    spec["aircraft"]["look_ahead_m"] = float(look_ahead_m)
    return spec


def with_algorithm(spec, name):
    """The spec with its algorithm replaced by another registered arm, the
    kangaroo, aircraft and start pose kept (`TASK-060`: fly arm F, which has no
    campaign cell, against a recorded cell's set-up).

    An arm that owns its horizon gets ``lookahead_steps`` 0 and one that needs
    the estimator gets ``estimate`` true, the combination the campaign's
    ``cell_table`` requires; everything else is the cell's. ``None`` leaves the
    spec as it is.

    Raises:
        KeyError: ``name`` is not a registered algorithm.
    """
    if name is None:
        return spec
    from py_harness import algorithms
    if name not in algorithms.REGISTRY:
        raise KeyError("%r is not a registered algorithm; choose from %s"
                       % (name, ", ".join(sorted(algorithms.IMPLEMENTED))))
    built = algorithms.build(name, {})
    spec = json.loads(json.dumps(spec))
    spec["algorithm"]["name"] = name
    if getattr(built, "owns_horizon", False):
        spec["algorithm"]["lookahead_steps"] = 0
    if getattr(built, "requires_estimate", False):
        spec["algorithm"]["estimate"] = True
    return spec


def demo_table(spec, cell_id, mode_base=None, alt_m=None,
               bound_m=DEFAULT_BOUND_M, look_ahead_m=None, roll_limit_deg=None,
               fence_path=None, suite_runs=None, cs_sampling=None):
    """The flat ``cfg`` the demonstration reads from the vehicle ``spec.json``
    (:func:`schedule.spec_cfg`, `ADR-011`): the cell's configuration exactly
    as the campaign runner gets it, plus the demonstration's own fields.

    ``look_ahead_m`` overrides the spec's carrot before the configuration is
    flattened, so it passes through the same validation as any cell.
    ``roll_limit_deg`` overrides the bank limit (default the configuration's
    ``bank_limit_deg``, 60). ``fence_path`` adds the ``fence`` field the
    demonstration and ``hardware_val.lua`` contain the kangaroo in
    (:func:`fence.spec_block`, `ADR-012`); ``None`` stages none.
    ``suite_runs`` (:func:`pv_plan.run_table`) makes it a suite: mode 5,
    the spec's legs flown from the site reference, each run announced.
    ``cs_sampling`` ({fine_m, coarse_m}, from pv_plan.json) writes
    ``cs_fine_m`` / ``cs_coarse_m``: the CS path's straights are sampled every
    ``delta_d_m`` for the first fine_m metres and every coarse_m after
    (harness_dubins; the flight's heap). Absent: the whole path every
    ``delta_d_m``, as the Python harness."""
    spec = with_look_ahead(spec, look_ahead_m)
    table = schedule.cell_table(spec, cell_id)
    speeds = [leg["speed_ms"] for leg in table["legs"] if leg["speed_ms"] > 0.0]
    rand_legs = [{"duration_s": float(d), "mode": str(m), "heading_deg": float(h)}
                 for (d, m, h, _s) in kang.rand_legs(RAND_SEED, 0.0, RAND_MIN_S,
                                                     RAND_MAX_S, RAND_HORIZON_S)]
    if alt_m is None:
        from . import campaign
        with open(paths.ENVIRONMENT_FILE) as handle:
            fa = json.load(handle).get("flight_area") or {}
        alt_m = float(fa.get("alt_m", campaign.DEFAULT_ALT_M))
    extra = {}
    if cs_sampling is not None:
        extra["cs_fine_m"] = float(cs_sampling["fine_m"])
        extra["cs_coarse_m"] = float(cs_sampling["coarse_m"])
    if suite_runs is not None:
        extra["suite_runs"] = [{"name": r["name"], "t_start_s": float(r["t_start_s"]),
                                "t_end_s": float(r["t_end_s"])} for r in suite_runs]
        mode_base = "suite"
    if fence_path is not None:
        extra["fence"] = fence_mod.spec_block(fence_path,
                                              fence_mod.containment_margin_m(spec))
    return schedule.spec_cfg(spec, cell_id, roll_limit_deg=roll_limit_deg, extra=dict(extra, **{
        "source_cell": str(cell_id),
        "mode_code": MODE_CODES.get(mode_base, MODE_CODES["straight"]),
        "pace_code": 0,
        "speed_ms": speeds[0] if speeds else 12.5,
        "start_range_m": math.hypot(table["target_n_m"], table["target_e_m"]),
        "alt_m": float(alt_m),
        "bound_m": float(bound_m),
        "rand_seed": RAND_SEED,
        "rand_legs": rand_legs,
    }))


def stage(campaign_dir=DEFAULT_CAMPAIGN, cell_id=DEFAULT_CELL,
          scripts_dir=paths.SCRIPTS_DIR, lua_dir=paths.LUA_DIR, alt_m=None,
          bound_m=DEFAULT_BOUND_M, spec=None, look_ahead_m=None, algorithm=None,
          roll_limit_deg=None, fence_path=ENVIRONMENT_FENCE, plan_arm=None,
          plan_path=None):
    """Stage the demonstration. Returns the staging record.

    ``algorithm`` flies another registered arm against the cell's set-up
    (:func:`with_algorithm`); the record's ``cell_id`` then names both.
    ``fence_path`` is the fence the kangaroo is contained in: the
    environment's by default, a path, or ``None`` for none (the
    ``KDEM_BOUND_M`` circle).

    ``plan_arm`` stages that arm of the physical-validation plan
    (``plan_path``, default :data:`pv_plan.PLAN_FILE`) in suite mode instead
    of a cell; the plan's own fence replaces ``fence_path``.

    Raises:
        RuntimeError: If something (a cell or a demonstration) is already staged.
        pv_plan.PlanError: The plan arm is unknown or the plan does not fit.
    """
    if stage_scripts.staged_state(scripts_dir) is not None:
        raise RuntimeError(
            "%s exists: something is staged and not restored; run %s first"
            % (paths.rel(os.path.join(scripts_dir, stage_scripts.STATE_FILE)),
               restore_command()))
    mode_base = None
    suite_runs = None
    cs_sampling = None
    if plan_arm is not None:
        from . import pv_plan
        plan = pv_plan.load(plan_path or pv_plan.PLAN_FILE)
        spec = pv_plan.build_spec(plan, plan_arm)
        suite_runs = pv_plan.run_table(plan)
        cell_id = "pv:%s" % plan_arm
        fence_path = pv_plan.fence_path(plan)
        roll_limit_deg = float(plan["aircraft"]["bank_limit_deg"])
        cs_sampling = plan.get("cs_sampling")
    if spec is None:
        spec, entry = _cell_spec(campaign_dir, cell_id)
        mode_base = entry.get("mode_base")
    if algorithm is not None and algorithm != spec["algorithm"]["name"]:
        spec = with_algorithm(spec, algorithm)
        cell_id = "%s+%s" % (cell_id, algorithm)
    if fence_path == ENVIRONMENT_FENCE:
        fence_path = fence_mod.environment_fence_path()
    table = demo_table(spec, cell_id, mode_base=mode_base, alt_m=alt_m, bound_m=bound_m,
                       look_ahead_m=look_ahead_m, roll_limit_deg=roll_limit_deg,
                       fence_path=fence_path, suite_runs=suite_runs,
                       cs_sampling=cs_sampling)

    modules_dir = os.path.join(scripts_dir, "modules")
    os.makedirs(modules_dir, exist_ok=True)
    moved = []
    for name in sorted(os.listdir(scripts_dir)):
        if not name.endswith(".lua") or name == DEMO_SCRIPT:
            continue
        src = os.path.join(scripts_dir, name)
        if os.path.isfile(src):
            os.rename(src, src + stage_scripts.OFF_SUFFIX)
            moved.append(name)

    script_dst = os.path.join(scripts_dir, DEMO_SCRIPT)
    shutil.copyfile(os.path.join(lua_dir, DEMO_SCRIPT), script_dst)
    arms_dst = os.path.join(modules_dir, paths.ARMS_MODULE)
    adsb_dst = os.path.join(modules_dir, ADSB_MODULE)
    spec_mod_dst = os.path.join(modules_dir, paths.SPEC_MODULE)
    # Support modules already present and identical are left in place, so a
    # restore does not delete the copy hardware_val.lua needs (stage_scripts).
    placed = stage_scripts.place_modules(
        [(os.path.join(lua_dir, name), dst) for name, dst in (
            (paths.ARMS_MODULE, arms_dst), (ADSB_MODULE, adsb_dst),
            (paths.SPEC_MODULE, spec_mod_dst))], scripts_dir)
    cfg_dst = os.path.join(scripts_dir, paths.SPEC_FILE)
    schedule.write_spec(table, cfg_dst)

    loaded = [script_dst, arms_dst, adsb_dst, spec_mod_dst, cfg_dst]
    hashes = {}
    for path in loaded + [os.path.join(modules_dir, m) for m in paths.HARNESS_MODULES]:
        if not os.path.isfile(path):
            raise FileNotFoundError("staging needs %s" % paths.rel(path))
        hashes[os.path.relpath(path, scripts_dir)] = stage_scripts.sha256(path)
    record = {
        "cell_id": "demo:%s" % cell_id,
        "scripts_dir": paths.rel(scripts_dir),
        "moved_aside": moved,
        "staged": ([os.path.relpath(p, scripts_dir) for p in (script_dst, cfg_dst)]
                   + placed["staged"]),
        "kept": placed["kept"],
        "modules_moved_aside": placed["modules_moved_aside"],
        "hashes": hashes,
        "demo": dict({k: table[k] for k in ("algorithm", "dt_s", "estimate", "speed_ms",
                                            "start_range_m", "alt_m", "bound_m",
                                            "rand_seed", "roll_limit_deg")},
                     look_ahead_m=table["look_ahead_m"],
                     fence=(table["fence"]["file"] if "fence" in table else None),
                     suite=(len(suite_runs) if suite_runs is not None else None)),
    }
    with open(os.path.join(scripts_dir, stage_scripts.STATE_FILE), "w") as handle:
        json.dump(record, handle, indent=2)
        handle.write("\n")
    return record


def restore_command():
    """The restore command with the directory it must be run from."""
    return "`cd %s && python3 -m kangaroo_follow.demo --restore`" % paths.rel(paths.AUTOTEST_DIR)


def location_name():
    """The site the environment pins (a ``locations.txt`` entry)."""
    with open(paths.ENVIRONMENT_FILE) as handle:
        return json.load(handle)["location"]["name"]


def sim_vehicle_command():
    return SIM_VEHICLE.format(
        location=location_name(),
        param_file=os.path.relpath(paths.PARAM_FILE, paths.ARDUPILOT_DIR),
        demo_param_file=os.path.relpath(DEMO_PARAM_FILE, paths.ARDUPILOT_DIR))


OPERATOR_NOTES = """\
In MAVProxy once the console shows "KDEM: loaded":
  mode TAKEOFF ; arm throttle          climb to TKOFF_ALT
  mode GUIDED                          kangaroo placed KDEM_RANGE ahead; follow starts
  param set KDEM_MODE <n>              0 point, 1 straight, 2 circle, 3 rectangle, 4 rand
  param set KDEM_PACE <0|1>            constant | elastic (not point or rand)
  param set KDEM_SPD <m/s>             sweep: 6.25 12.5 18.75 25 37.5 (ratios 0.25 to 1.5)
  param set KDEM_HDG <deg>             -1 = aircraft heading at start
  param set KDEM_RESET 1               re-place the kangaroo ahead of the aircraft
  param set KDEM_LOOK <m>              carrot look-ahead: 50 thesis default, 5 short-carrot baseline,
                                       30 the physical-validation plan (--plan sets it)
  param set KDEM_CHAN <1|0>            steer at the guidance point (default) |
                                       loiter about it, as the campaign runner does
With --plan: KDEM_MODE is 5 (suite) and the run plan starts on GUIDED, from
the site reference; KDEM_RESET 1 restarts it. Watch for "KDEM: run i/n" and
"KDEM: suite complete; least spare ...". Restore and re-stage with the next
--arm when it finishes (one arm across every run, then the next).
The kangaroo is the ADS-B contact KANGAROO (`module load adsb` if the map
does not show it)."""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--stage", action="store_true")
    action.add_argument("--restore", action="store_true")
    parser.add_argument("--campaign", default=DEFAULT_CAMPAIGN,
                        help="Python campaign directory the cell is read from")
    parser.add_argument("--cell", default=DEFAULT_CELL,
                        help="Python cell whose spec configures the demonstration")
    parser.add_argument("--alt-m", type=float, default=None)
    parser.add_argument("--bound-m", type=float, default=DEFAULT_BOUND_M,
                        help="containment radius about the start, m, used only "
                             "with --no-fence")
    fence_group = parser.add_mutually_exclusive_group()
    fence_group.add_argument("--fence", default=None,
                             help="fence file the kangaroo is contained in (default "
                                  "environment.json flight_area.fence_file; ADR-012)")
    fence_group.add_argument("--no-fence", action="store_true",
                             help="stage no fence: contain the kangaroo in the "
                                  "--bound-m circle instead")
    parser.add_argument("--look-ahead-m", type=float, default=None,
                        help="carrot look-ahead, m (default the cell's, 50; the "
                             "thesis's short-carrot baseline is 5)")
    parser.add_argument("--algorithm", default=None,
                        help="fly this registered arm against the cell's set-up "
                             "(e.g. carrot_shift_cs_hyst, arm F, which has no "
                             "campaign cell); default the cell's own")
    parser.add_argument("--plan", nargs="?", const="default", default=None,
                        help="stage the physical-validation run plan (default "
                             "kangaroo_follow/pv_plan.json) instead of a cell; "
                             "needs --arm")
    parser.add_argument("--arm", default=None,
                        help="with --plan: the plan arm to stage (0H, FH, AH)")
    parser.add_argument("--roll-limit-deg", type=float, default=None,
                        help="bank limit written into spec.json, deg (default the "
                             "cell configuration's bank_limit_deg, 60; ADR-011). "
                             "Set ROLL_LIMIT_DEG to the same value: the script warns "
                             "on a mismatch")
    args = parser.parse_args(argv)
    if args.restore:
        print(json.dumps(stage_scripts.restore(), indent=2))
        return 0
    if (args.plan is None) != (args.arm is None):
        parser.error("--plan and --arm go together")
    record = stage(args.campaign, args.cell, alt_m=args.alt_m, bound_m=args.bound_m,
                   look_ahead_m=args.look_ahead_m, algorithm=args.algorithm,
                   roll_limit_deg=args.roll_limit_deg,
                   fence_path=(None if args.no_fence else
                               os.path.abspath(args.fence) if args.fence else
                               ENVIRONMENT_FENCE),
                   plan_arm=args.arm,
                   plan_path=(None if args.plan in (None, "default")
                              else os.path.abspath(args.plan)))
    print(json.dumps(record["demo"], indent=2))
    print("\nStaged %s from %s. From %s run:\n\n  %s\n\n%s\n\n"
          "When finished, put control_cont.lua and kangaroo_MAV.lua back with\n"
          "(from the repository root):\n\n  %s\n" % (
              DEMO_SCRIPT, ("plan arm %s" % args.arm) if args.arm else args.cell,
              paths.rel(paths.ARDUPILOT_DIR),
              sim_vehicle_command(), OPERATOR_NOTES, restore_command().strip("`")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
