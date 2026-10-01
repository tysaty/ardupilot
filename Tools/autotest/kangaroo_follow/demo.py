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

from py_harness import kangaroo as kang

#: The Lua a demonstration flies, beside the campaign's in ``ArduPlane_Tests``.
DEMO_SCRIPT = "kangaroo_demo.lua"
DEMO_CFG_MODULE = "kangaroo_demo_cfg.lua"
ADSB_MODULE = "sitl_adsb.lua"
#: Parameters layered on kangaroo-follow.parm for the demonstration only.
DEMO_PARAM_FILE = os.path.join(paths.PARAMS_DIR, "kangaroo-demo.parm")

#: The Python cell the demonstration's configuration is taken from.
DEFAULT_CAMPAIGN = os.path.join(paths.CAMPAIGNS_DIR, "CAMP-003-arm-mode-ratio-hyst")
DEFAULT_CELL = "0H-straight-constant-half"

#: KDEM_MODE codes, in ``kangaroo.MODES`` order then ``kangaroo_rand``;
#: must match MODE_NAMES in kangaroo_demo.lua.
MODE_CODES = {"point": 0, "straight": 1, "circle": 2, "rectangle": 3, "rand": 4}

#: The ``kangaroo_rand`` expansion carried to the vehicle: the harness's own
#: defaults (``kangaroo.build`` rand_min_s, rand_max_s, rand_horizon_s) and
#: the composite schedule's seed (``kangaroo.COMPOSITE_RAND_SEED``).
RAND_SEED = kang.COMPOSITE_RAND_SEED
RAND_MIN_S = 5.0
RAND_MAX_S = 20.0
RAND_HORIZON_S = 3600.0

#: Default containment radius about the start, m: half the side of the 2 km
#: zone the recorded campaigns ran in. KDEM_BOUND_M changes it in flight.
DEFAULT_BOUND_M = 1000.0

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


def demo_table(spec, cell_id, mode_base=None, alt_m=None,
               bound_m=DEFAULT_BOUND_M, look_ahead_m=None):
    """The plain dict ``kangaroo_demo_cfg.lua`` returns, from a cell's spec.

    ``look_ahead_m`` overrides the spec's carrot before the configuration is
    flattened, so it passes through the same validation as any cell."""
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
    return {
        "source_cell": str(cell_id),
        "algorithm": table["algorithm"],
        "cfg": table["cfg"],
        "dt_s": table["dt_s"],
        "estimate": table["estimate"],
        "lookahead_steps": table["lookahead_steps"],
        "estimator": table["estimator"],
        "geometry": table["geometry"],
        "mode_code": MODE_CODES.get(mode_base, MODE_CODES["straight"]),
        "pace_code": 0,
        "speed_ms": speeds[0] if speeds else 12.5,
        "start_range_m": math.hypot(table["target_n_m"], table["target_e_m"]),
        "alt_m": float(alt_m),
        "bound_m": float(bound_m),
        "rand_seed": RAND_SEED,
        "rand_legs": rand_legs,
    }


def render_demo_module(table):
    return ("-- Generated by kangaroo_follow/demo.py from %s. Do not edit.\n"
            "return %s\n" % (table["source_cell"], schedule.to_lua(table)))


def stage(campaign_dir=DEFAULT_CAMPAIGN, cell_id=DEFAULT_CELL,
          scripts_dir=paths.SCRIPTS_DIR, lua_dir=paths.LUA_DIR, alt_m=None,
          bound_m=DEFAULT_BOUND_M, spec=None, look_ahead_m=None):
    """Stage the demonstration. Returns the staging record.

    Raises:
        RuntimeError: If something (a cell or a demonstration) is already staged.
    """
    if stage_scripts.staged_state(scripts_dir) is not None:
        raise RuntimeError(
            "%s exists: something is staged and not restored; run %s first"
            % (paths.rel(os.path.join(scripts_dir, stage_scripts.STATE_FILE)),
               restore_command()))
    mode_base = None
    if spec is None:
        spec, entry = _cell_spec(campaign_dir, cell_id)
        mode_base = entry.get("mode_base")
    table = demo_table(spec, cell_id, mode_base=mode_base, alt_m=alt_m, bound_m=bound_m,
                       look_ahead_m=look_ahead_m)

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
    shutil.copyfile(os.path.join(lua_dir, paths.ARMS_MODULE), arms_dst)
    adsb_dst = os.path.join(modules_dir, ADSB_MODULE)
    shutil.copyfile(os.path.join(lua_dir, ADSB_MODULE), adsb_dst)
    cfg_dst = os.path.join(modules_dir, DEMO_CFG_MODULE)
    with open(cfg_dst, "w") as handle:
        handle.write(render_demo_module(table))

    staged = [script_dst, arms_dst, adsb_dst, cfg_dst]
    hashes = {}
    for path in staged + [os.path.join(modules_dir, m) for m in paths.HARNESS_MODULES]:
        if not os.path.isfile(path):
            raise FileNotFoundError("staging needs %s" % paths.rel(path))
        hashes[os.path.relpath(path, scripts_dir)] = stage_scripts.sha256(path)
    record = {
        "cell_id": "demo:%s" % cell_id,
        "scripts_dir": paths.rel(scripts_dir),
        "moved_aside": moved,
        "staged": [os.path.relpath(p, scripts_dir) for p in staged],
        "hashes": hashes,
        "demo": dict({k: table[k] for k in ("algorithm", "dt_s", "estimate", "speed_ms",
                                            "start_range_m", "alt_m", "bound_m",
                                            "rand_seed")},
                     look_ahead_m=table["cfg"]["look_ahead_m"]),
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
  param set KDEM_LOOK <m>              carrot look-ahead: 50 thesis default, 5 short-carrot baseline
  param set KDEM_CHAN <1|0>            steer at the guidance point (default) |
                                       loiter about it, as the campaign runner does
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
    parser.add_argument("--bound-m", type=float, default=DEFAULT_BOUND_M)
    parser.add_argument("--look-ahead-m", type=float, default=None,
                        help="carrot look-ahead, m (default the cell's, 50; the "
                             "thesis's short-carrot baseline is 5)")
    args = parser.parse_args(argv)
    if args.restore:
        print(json.dumps(stage_scripts.restore(), indent=2))
        return 0
    record = stage(args.campaign, args.cell, alt_m=args.alt_m, bound_m=args.bound_m,
                   look_ahead_m=args.look_ahead_m)
    print(json.dumps(record["demo"], indent=2))
    print("\nStaged %s from %s. From %s run:\n\n  %s\n\n%s\n\n"
          "When finished, put control_cont.lua and kangaroo_MAV.lua back with\n"
          "(from the repository root):\n\n  %s\n" % (
              DEMO_SCRIPT, args.cell, paths.rel(paths.ARDUPILOT_DIR),
              sim_vehicle_command(), OPERATOR_NOTES, restore_command().strip("`")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
