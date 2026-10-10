"""Build the SD-card flight package for the physical-validation plan.

One complete ``APM/scripts/`` tree per arm (0H, FH, AH), each with that arm's
``spec.json``, so the field step is "copy the arm's folder to the card":

    flight_package/<plan_id>/
        README.md            what to copy, the parameters, the order of the day
        MANIFEST.json        source commit and the SHA-256 of every file
        flight.parm          experiment parameters for the flight card
        fence/<fence file>   the fence, for the ground station to upload to ArduPlane
        0H/APM/scripts/...   arm 0H (baseline)          HVAL_ARM 0
        FH/APM/scripts/...   arm FH (adaptive carrot)   HVAL_ARM 2
        AH/APM/scripts/...   arm AH (adaptive baseline) HVAL_ARM 1

Every file is copied from its source in this repository (nothing is
retyped), and each ``spec.json`` is built by ``pv_plan.py`` exactly as
``demo.py --stage --plan --arm`` writes it, so the card flies what SITL flew.

Usage, from ``src/ardupilot/Tools/autotest``::

    python3 -m kangaroo_follow.sdcard            # writes flight_package/<plan_id>/
    python3 -m kangaroo_follow.sdcard --check    # rebuild in a temp dir, compare
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

from . import demo, paths, pv_plan, schedule

#: The flight scripts, from scripts/.
SCRIPTS = ("hardware_val.lua",)      # one script: the kangaroo is its section 3b (2026-10-10)
#: Modules from scripts/modules/ (arms 0H, FH and AH, the kangaroo, the fence).
MODULES = ("sitl_arms.lua", "sitl_adsb.lua",
           "harness_geom.lua", "harness_dubins.lua", "harness_orbit.lua",
           "harness_cs_orbit.lua", "harness_carrot_shift.lua", "harness_adaptive_db.lua",
           "harness_kangaroo.lua", "harness_segments.lua", "harness_estimator.lua",
           "harness_zone.lua")
#: Modules kept with the SITL test assets (Tools/autotest/ArduPlane_Tests/KangarooFollow/).
ASSET_MODULES = ("sitl_spec.lua",)
#: Upstream MAVLink encoder: needed only by the two-script layout's
#: FOLLOW_TARGET output (archived in working_folder_lua/); the one script sends
#: no MAVLink messages of its own except ADS-B (sitl_adsb), so none are copied.
MAVLINK_DIR = os.path.join(paths.ARDUPILOT_DIR, "libraries", "AP_Scripting", "modules",
                           "MAVLink")
MAVLINK = ()

#: HVAL_ARM for each plan arm (hardware_val.lua ARM_NAMES).
HVAL_ARM = {"0H": 0, "FH": 2, "AH": 1}

DEFAULT_OUT = os.path.join(paths.PACKAGE_DIR, "flight_package")


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        h.update(handle.read())
    return h.hexdigest()


def _git(*cmd):
    try:
        return subprocess.check_output(("git",) + cmd, cwd=paths.ARDUPILOT_DIR,
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


def _copy(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(src, dst)


def arm_tree(plan, arm, root, roll_limit_deg):
    """Write one arm's APM/scripts tree under ``root``; returns the spec table."""
    scripts = os.path.join(root, "APM", "scripts")
    for name in SCRIPTS:
        _copy(os.path.join(paths.SCRIPTS_DIR, name), os.path.join(scripts, name))
    for name in MODULES:
        _copy(os.path.join(paths.MODULES_DIR, name), os.path.join(scripts, "modules", name))
    for name in ASSET_MODULES:
        _copy(os.path.join(paths.LUA_DIR, name), os.path.join(scripts, "modules", name))
    for name in MAVLINK:
        _copy(os.path.join(MAVLINK_DIR, name),
              os.path.join(scripts, "modules", "MAVLink", name))
    spec = pv_plan.build_spec(plan, arm)
    table = demo.demo_table(spec, "pv-%s" % arm, alt_m=float(plan.get("alt_m", 60.0)),
                            fence_path=pv_plan.fence_path(plan),
                            suite_runs=pv_plan.run_table(plan),
                            roll_limit_deg=roll_limit_deg,
                            cs_sampling=plan.get("cs_sampling"))
    # rand_legs is the SITL demo's random-mode schedule (KDEM_MODE 4); neither
    # flight script reads it, and both parse spec.json, so it would hold about
    # 60 kB of heap in each (2026-10-10: the two scripts must fit 750 kB).
    table.pop("rand_legs", None)
    schedule.write_spec(table, os.path.join(scripts, paths.SPEC_FILE))
    return table


def flight_parm(plan):
    card = plan.get("flight_card") or {}
    lines = [
        "# Physical-validation experiment parameters (flight card). Load on top of",
        "# the airframe's own parameters; none of these replaces an airframe limit.",
        "SCR_ENABLE        1",
        "SCR_HEAP_SIZE     %d    # the board's limit; all three arms flew the plan at 750000 in SITL"
        % int(card.get("SCR_HEAP_SIZE_min_bytes", 750000)),
        "SCR_VM_I_COUNT    1000000    # as SITL; measure on the bench",
        "ROLL_LIMIT_DEG    %d    # must equal spec.json roll_limit_deg (the script checks)"
        % int(card.get("ROLL_LIMIT_DEG", 60)),
        "AIRSPEED_CRUISE   %g    # pv_plan.json aircraft.airspeed_ms, the speed the plan assumes;"
        % float(plan["aircraft"]["airspeed_ms"]),
        "#                    keep it within the airframe's AIRSPEED_MIN / AIRSPEED_MAX",
        "GUIDED_P          15000    # the heading-command gain the SITL runs used",
        "RC7_OPTION        303    # activation switch (HVAL_ACT_FN 303); any free channel",
        "ADSB_TYPE         %d    # the aircraft must ignore its own kangaroo"
        % int(card.get("ADSB_TYPE", 0)),
        "AVD_ENABLE        0",
        "FENCE_TYPE        4    # polygon: upload fence/%s from the ground station"
        % os.path.basename(plan["fence_file"]),
        "# FENCE_ENABLE and FENCE_ACTION: set on the field per the safety case (a real action).",
        "",
        "# HVAL_* parameters exist only once hardware_val.lua has loaded; set them after boot:",
        "#   HVAL_ARM  0 (0H) / 2 (FH) / 1 (AH), matching the spec.json on the card",
        "#   HVAL_TGT  3 (the site kangaroo in hardware_val.lua)",
        "#   HVAL_OUT  1 (active; 0 = shadow)",
        "#   HVAL_ENABLE 1 when ready; then the RC7 switch low -> high engages in GUIDED",
        "# KSRC_RUN 5 starts the plan (hardware_val.lua section 3b); set it after engaging.",
    ]
    return "\n".join(lines) + "\n"


def readme(plan, tables, manifest):
    arms = plan["order"]
    run_lines = "\n".join("| %s | %.0f | %.0f |" % (r["name"], r["t_start_s"], r["t_end_s"])
                          for r in pv_plan.run_table(plan))
    arm_lines = "\n".join("| `%s/` | %s | `HVAL_ARM %d` | %s |" % (
        a, plan["arms"][a].get("label", ""), HVAL_ARM[a], tables[a]["algorithm"]) for a in arms)
    return """# Flight package: %(plan_id)s

Generated by `python3 -m kangaroo_follow.sdcard` from `pv_plan.json` and the
scripts in this repository (commit `%(commit)s`). Do not edit these files;
regenerate them.

## Arms (fly in this order, one per sortie)

| Folder | Arm | Set | Algorithm |
|---|---|---|---|
%(arm_lines)s

For an arm, copy the **contents** of its `APM/` folder onto the card's `APM/`
(the card ends up with `APM/scripts/hardware_val.lua`,
`spec.json` and `modules/`). Nothing else may be in `APM/scripts/`: every
`.lua` there runs.

## Before each sortie

1. Parameters: `flight.parm` on top of the airframe's own; set `FENCE_ENABLE`
   and `FENCE_ACTION` per the safety case.
2. ArduPlane fence: upload `fence/%(fence)s` from the ground station. It must
   match the fence corners in `spec.json` (the scripts cannot read the fence).
3. Boot. Expect `HVAL: loaded disabled; algorithm <arm>` and
   `KSRC: kangaroo at 30 N 0 E of the site`.
4. Set `HVAL_ARM` (table above), `HVAL_TGT 3` and `HVAL_OUT 1`, with
   `HVAL_ENABLE 0`. `hardware_val.lua` selects an arm only while disabled, in
   GUIDED and armed (or at boot, from the saved `HVAL_ARM`).
5. Take off and climb, fly to the site centre in GUIDED and wait for
   `HVAL: selected <algorithm>`; then set `HVAL_ENABLE 1` and switch low then
   high to engage.
6. When the aircraft is orbiting the point, set `KSRC_RUN 5`. The plan runs
   %(dur).0f s and ends with `KSRC: plan complete; least spare ... m`.
7. Switch low to hand back, land, and copy the `.BIN` log.

## The plan (each arm, %(dur).0f s)

| Run | Start (s) | End (s) |
|---|---|---|
%(run_lines)s

`MANIFEST.json` lists the SHA-256 of every file here.
""" % {"plan_id": plan.get("plan_id", "PV"), "commit": manifest["source"]["ardupilot_commit"],
       "arm_lines": arm_lines, "fence": os.path.basename(plan["fence_file"]),
       "dur": tables[arms[0]]["duration_s"], "run_lines": run_lines}


def build(out_root=DEFAULT_OUT, plan_path=None, roll_limit_deg=60.0):
    """Write the package; returns its directory."""
    plan = pv_plan.load(plan_path or pv_plan.PLAN_FILE)
    pkg = os.path.join(out_root, plan.get("plan_id", "PV"))
    if os.path.isdir(pkg):
        shutil.rmtree(pkg)
    os.makedirs(pkg)
    tables = {}
    for arm in plan["order"]:
        tables[arm] = arm_tree(plan, arm, os.path.join(pkg, arm), roll_limit_deg)
    fence_src = pv_plan.fence_path(plan)
    _copy(fence_src, os.path.join(pkg, "fence", os.path.basename(fence_src)))
    with open(os.path.join(pkg, "flight.parm"), "w") as handle:
        handle.write(flight_parm(plan))
    files = {}
    for dirpath, _dirs, names in os.walk(pkg):
        for name in sorted(names):
            p = os.path.join(dirpath, name)
            files[os.path.relpath(p, pkg)] = _sha256(p)
    manifest = {
        "plan_id": plan.get("plan_id"),
        "arms": {a: {"hval_arm": HVAL_ARM[a], "algorithm": tables[a]["algorithm"],
                     "duration_s": tables[a]["duration_s"]} for a in plan["order"]},
        "source": {"ardupilot_commit": _git("rev-parse", "--short", "HEAD"),
                   "dirty": bool(_git("status", "--porcelain", "--", "scripts",
                                      "Tools/autotest/kangaroo_follow",
                                      "Tools/autotest/ArduPlane_Tests/KangarooFollow"))},
        "files": dict(sorted(files.items())),
    }
    with open(os.path.join(pkg, "MANIFEST.json"), "w") as handle:
        json.dump(manifest, handle, indent=1)
        handle.write("\n")
    with open(os.path.join(pkg, "README.md"), "w") as handle:
        handle.write(readme(plan, tables, manifest))
    return pkg


def check(out_root=DEFAULT_OUT):
    """True when the committed package equals a fresh build (ignoring the
    manifest's commit/dirty and the README's commit line)."""
    plan = pv_plan.load(pv_plan.PLAN_FILE)
    have = os.path.join(out_root, plan.get("plan_id", "PV"))
    with tempfile.TemporaryDirectory() as tmp:
        fresh = build(tmp)
        a = json.load(open(os.path.join(have, "MANIFEST.json")))["files"]
        b = json.load(open(os.path.join(fresh, "MANIFEST.json")))["files"]
        skip = {"README.md"}
        diff = sorted(k for k in set(a) | set(b) if k not in skip and a.get(k) != b.get(k))
    return diff


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", default=DEFAULT_OUT)
    parser.add_argument("--plan", default=None)
    parser.add_argument("--check", action="store_true",
                        help="compare the existing package with a fresh build")
    args = parser.parse_args(argv)
    if args.check:
        diff = check(args.out)
        print("package is current" if not diff else "out of date: %s" % ", ".join(diff))
        return 1 if diff else 0
    pkg = build(args.out, args.plan)
    m = json.load(open(os.path.join(pkg, "MANIFEST.json")))
    print("wrote %s (%d files)" % (paths.rel(pkg), len(m["files"])))
    for arm, a in m["arms"].items():
        print("  %s  HVAL_ARM %d  %-26s %.0f s" % (arm, a["hval_arm"], a["algorithm"],
                                                   a["duration_s"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
