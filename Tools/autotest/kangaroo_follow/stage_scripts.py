"""Stage the exact Lua files a cell flies into the SITL scripts directory, and
put everything back afterwards (``TASK-052`` "script staging").

ArduPilot SITL loads every ``*.lua`` in ``scripts/`` relative to its working
directory, which for the AutoTest tree is the ArduPilot root: the project's
``src/ardupilot/scripts/``. That directory already holds the shipping flight
scripts (``control_cont.lua``, ``kangaroo_MAV.lua``), which would command the
vehicle and the target bus at the same time as the runner. Staging therefore:

1. copies ``sitl_harness_runner.lua`` into ``scripts/`` and ``sitl_arms.lua``
   into ``scripts/modules/``;
2. writes the generated ``sitl_cell.lua`` into ``scripts/modules/``;
3. moves every other ``scripts/*.lua`` aside as ``<name>.sitl-off`` and records
   the moves in ``scripts/.sitl-staged.json`` so :func:`restore` is exact;
4. hashes every file the cell will load (runner, registry, cell, the ported
   ``harness_*.lua`` modules) so the bundle is tied to the files that flew
   (`VR-010`).

The ported modules are **not copied**: they already live in
``scripts/modules/`` and are staged unchanged; their hashes are recorded.

Restore is idempotent and is run in a ``finally`` by the campaign runner, so
a killed cell does not leave the flight scripts disabled.
"""

import argparse
import hashlib
import json
import os
import shutil

from . import paths, schedule

STATE_FILE = ".sitl-staged.json"
OFF_SUFFIX = ".sitl-off"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _state_path(scripts_dir):
    return os.path.join(scripts_dir, STATE_FILE)


def staged_state(scripts_dir=paths.SCRIPTS_DIR):
    path = _state_path(scripts_dir)
    if not os.path.isfile(path):
        return None
    with open(path) as handle:
        return json.load(handle)


def stage(spec, cell_id, scripts_dir=paths.SCRIPTS_DIR, lua_dir=paths.LUA_DIR,
          heading_source=schedule.DEFAULT_HEADING_SOURCE):
    """Stage one cell. Returns the staging record (also written to the state file).

    Raises:
        RuntimeError: If a previous staging was not restored (the state file
            exists), so two cells never interleave.
    """
    if staged_state(scripts_dir) is not None:
        raise RuntimeError(
            "%s exists: a previous cell was staged and not restored; run "
            "`python3 -m kangaroo_follow.stage_scripts --restore` first"
            % paths.rel(_state_path(scripts_dir)))
    modules_dir = os.path.join(scripts_dir, "modules")
    os.makedirs(modules_dir, exist_ok=True)

    moved = []
    for name in sorted(os.listdir(scripts_dir)):
        if not name.endswith(".lua") or name == paths.RUNNER_SCRIPT:
            continue
        src = os.path.join(scripts_dir, name)
        if os.path.isfile(src):
            os.rename(src, src + OFF_SUFFIX)
            moved.append(name)

    runner_dst = os.path.join(scripts_dir, paths.RUNNER_SCRIPT)
    shutil.copyfile(os.path.join(lua_dir, paths.RUNNER_SCRIPT), runner_dst)
    arms_dst = os.path.join(modules_dir, paths.ARMS_MODULE)
    shutil.copyfile(os.path.join(lua_dir, paths.ARMS_MODULE), arms_dst)
    cell_dst = os.path.join(modules_dir, paths.CELL_MODULE)
    table = schedule.write_cell_module(spec, cell_id, cell_dst, heading_source)

    hashes = {}
    for path in [runner_dst, arms_dst, cell_dst] + [
            os.path.join(modules_dir, m) for m in paths.HARNESS_MODULES]:
        if not os.path.isfile(path):
            raise FileNotFoundError("staging needs %s" % paths.rel(path))
        hashes[os.path.relpath(path, scripts_dir)] = sha256(path)

    record = {
        "cell_id": cell_id,
        "scripts_dir": paths.rel(scripts_dir),
        "moved_aside": moved,
        "staged": [os.path.relpath(p, scripts_dir)
                   for p in (runner_dst, arms_dst, cell_dst)],
        "hashes": hashes,
        "cell_table": {k: table[k] for k in ("algorithm", "duration_s", "dt_s",
                                             "estimate", "lookahead_steps",
                                             "heading_source", "target_n_m",
                                             "target_e_m")},
    }
    with open(_state_path(scripts_dir), "w") as handle:
        json.dump(record, handle, indent=2)
        handle.write("\n")
    return record


def restore(scripts_dir=paths.SCRIPTS_DIR):
    """Undo :func:`stage`. Safe to call when nothing is staged."""
    state = staged_state(scripts_dir)
    if state is None:
        # Still sweep stray *.sitl-off files from an interrupted run.
        restored = []
        for name in sorted(os.listdir(scripts_dir)):
            if name.endswith(OFF_SUFFIX):
                src = os.path.join(scripts_dir, name)
                os.rename(src, src[:-len(OFF_SUFFIX)])
                restored.append(name[:-len(OFF_SUFFIX)])
        return {"restored": restored, "removed": [], "had_state": False}
    for rel in state.get("staged", []):
        path = os.path.join(scripts_dir, rel)
        if os.path.isfile(path):
            os.remove(path)
    restored = []
    for name in state.get("moved_aside", []):
        src = os.path.join(scripts_dir, name + OFF_SUFFIX)
        if os.path.isfile(src):
            os.rename(src, os.path.join(scripts_dir, name))
            restored.append(name)
    os.remove(_state_path(scripts_dir))
    return {"restored": restored, "removed": list(state.get("staged", [])),
            "had_state": True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--stage", action="store_true")
    action.add_argument("--restore", action="store_true")
    action.add_argument("--status", action="store_true")
    parser.add_argument("--spec")
    parser.add_argument("--cell-id")
    parser.add_argument("--heading-source", default=schedule.DEFAULT_HEADING_SOURCE,
                        choices=schedule.HEADING_SOURCES)
    args = parser.parse_args(argv)
    if args.stage:
        if not (args.spec and args.cell_id):
            parser.error("--stage needs --spec and --cell-id")
        with open(args.spec) as handle:
            spec = json.load(handle)
        record = stage(spec, args.cell_id, heading_source=args.heading_source)
        print(json.dumps(record, indent=2))
    elif args.restore:
        print(json.dumps(restore(), indent=2))
    else:
        print(json.dumps(staged_state(), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
