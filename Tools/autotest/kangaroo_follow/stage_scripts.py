"""Stage the exact Lua files a cell flies into the SITL scripts directory, and
put everything back afterwards (``TASK-052`` "script staging").

ArduPilot SITL loads every ``*.lua`` in ``scripts/`` relative to its working
directory, which for the AutoTest tree is the ArduPilot root: the project's
``src/ardupilot/scripts/``. That directory already holds the shipping flight
scripts (``control_cont.lua``, ``kangaroo_MAV.lua``), which would command the
vehicle and the target bus at the same time as the runner. Staging therefore:

1. copies ``sitl_harness_runner.lua`` into ``scripts/``, and ``sitl_arms.lua``
   and ``sitl_spec.lua`` into ``scripts/modules/``;
2. writes the generated shared configuration ``scripts/spec.json``
   (:func:`schedule.spec_cfg`, `ADR-011`);
3. moves every other ``scripts/*.lua`` aside as ``<name>.sitl-off`` and records
   the moves in ``scripts/.sitl-staged.json`` so :func:`restore` is exact;
4. hashes every file the cell will load (runner, registry, cell, the ported
   ``harness_*.lua`` modules) so the bundle is tied to the files that flew
   (`VR-010`).

The ported modules are **not copied**: they already live in
``scripts/modules/`` and are staged unchanged; their hashes are recorded.

Support modules (``sitl_arms.lua``, ``sitl_spec.lua``, and the demo's
``sitl_adsb.lua``) are placed by :func:`place_module`: a module already in
``scripts/modules/`` and byte-identical to its source is left alone
(``kept``), one that differs is set aside as ``<name>.sitl-off`` and put back
by :func:`restore` (``modules_moved_aside``), and only a module that was not
there is copied in and later deleted (``staged``). ``hardware_val.lua`` needs
``modules/sitl_arms.lua``, so a restore must not delete a copy that was there
before staging (TASK-057 Part A, 2026-10-07).

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


def place_module(src, dst):
    """Put the support module ``src`` at ``dst`` without destroying what is there.

    Returns ``"kept"`` (``dst`` was already byte-identical to ``src``; nothing
    written), ``"moved_aside"`` (``dst`` differed: renamed to
    ``dst + OFF_SUFFIX`` and ``src`` copied in) or ``"staged"`` (``dst`` was
    absent: ``src`` copied in). Only a ``"staged"`` file is deleted by
    :func:`restore`; a ``"moved_aside"`` one is put back.
    """
    if os.path.isfile(dst):
        if sha256(dst) == sha256(src):
            return "kept"
        aside = dst + OFF_SUFFIX
        if os.path.exists(aside):
            raise RuntimeError("%s exists: an earlier staging was not restored"
                               % paths.rel(aside))
        os.rename(dst, aside)
        shutil.copyfile(src, dst)
        return "moved_aside"
    shutil.copyfile(src, dst)
    return "staged"


def place_modules(pairs, scripts_dir):
    """:func:`place_module` over ``[(src, dst), ...]``; returns the three
    lists the staging record carries, as paths relative to ``scripts_dir``."""
    out = {"staged": [], "kept": [], "modules_moved_aside": []}
    for src, dst in pairs:
        how = place_module(src, dst)
        key = "modules_moved_aside" if how == "moved_aside" else how
        out[key].append(os.path.relpath(dst, scripts_dir))
    return out


def _matches_source(path, lua_dir):
    """True when ``path`` is byte-identical to the file of the same name in
    ``lua_dir`` (the support modules' source)."""
    src = os.path.join(lua_dir, os.path.basename(path))
    return os.path.isfile(path) and os.path.isfile(src) and sha256(path) == sha256(src)


def staged_state(scripts_dir=paths.SCRIPTS_DIR):
    path = _state_path(scripts_dir)
    if not os.path.isfile(path):
        return None
    with open(path) as handle:
        return json.load(handle)


def stage(spec, cell_id, scripts_dir=paths.SCRIPTS_DIR, lua_dir=paths.LUA_DIR,
          heading_source=schedule.DEFAULT_HEADING_SOURCE, legs=None,
          roll_limit_deg=None):
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
    spec_mod_dst = os.path.join(modules_dir, paths.SPEC_MODULE)
    placed = place_modules([(os.path.join(lua_dir, paths.ARMS_MODULE), arms_dst),
                            (os.path.join(lua_dir, paths.SPEC_MODULE), spec_mod_dst)],
                           scripts_dir)
    cell_dst = os.path.join(scripts_dir, paths.SPEC_FILE)
    table = schedule.write_spec(
        schedule.spec_cfg(spec, cell_id, heading_source, legs=legs,
                          roll_limit_deg=roll_limit_deg), cell_dst)

    hashes = {}
    for path in [runner_dst, arms_dst, spec_mod_dst, cell_dst] + [
            os.path.join(modules_dir, m) for m in paths.HARNESS_MODULES]:
        if not os.path.isfile(path):
            raise FileNotFoundError("staging needs %s" % paths.rel(path))
        hashes[os.path.relpath(path, scripts_dir)] = sha256(path)

    record = {
        "cell_id": cell_id,
        "scripts_dir": paths.rel(scripts_dir),
        "moved_aside": moved,
        "staged": ([os.path.relpath(p, scripts_dir) for p in (runner_dst, cell_dst)]
                   + placed["staged"]),
        "kept": placed["kept"],
        "modules_moved_aside": placed["modules_moved_aside"],
        "hashes": hashes,
        "cell_table": {k: table[k] for k in ("algorithm", "duration_s", "dt_s",
                                             "estimate", "lookahead_steps",
                                             "heading_source", "target_n_m",
                                             "target_e_m", "roll_limit_deg",
                                             "legs_source")},
    }
    with open(_state_path(scripts_dir), "w") as handle:
        json.dump(record, handle, indent=2)
        handle.write("\n")
    return record


def _sweep_off(directory, prefix=""):
    """Rename every ``*.sitl-off`` in ``directory`` back; returns the names."""
    restored = []
    if not os.path.isdir(directory):
        return restored
    for name in sorted(os.listdir(directory)):
        if name.endswith(OFF_SUFFIX):
            src = os.path.join(directory, name)
            os.rename(src, src[:-len(OFF_SUFFIX)])
            restored.append(prefix + name[:-len(OFF_SUFFIX)])
    return restored


def restore(scripts_dir=paths.SCRIPTS_DIR, lua_dir=paths.LUA_DIR):
    """Undo :func:`stage`. Safe to call when nothing is staged.

    Deletes only what staging created and puts back what it set aside. For a
    state file written before :func:`place_module` existed (no ``kept`` key),
    which lists ``modules/sitl_arms.lua`` as staged even when a copy was there
    first, a module identical to its source in ``lua_dir`` is not deleted."""
    state = staged_state(scripts_dir)
    if state is None:
        # Still sweep stray *.sitl-off files from an interrupted run.
        restored = _sweep_off(scripts_dir) + _sweep_off(
            os.path.join(scripts_dir, "modules"), prefix="modules/")
        return {"restored": restored, "removed": [], "kept": [], "had_state": False}
    removed, kept = [], list(state.get("kept", []))
    # A record without "kept" predates place_module: its "staged" modules may
    # have been there before staging, so one identical to its source is kept.
    legacy = "kept" not in state
    for rel in state.get("staged", []):
        path = os.path.join(scripts_dir, rel)
        if not os.path.isfile(path):
            continue
        if legacy and rel.startswith("modules/") and _matches_source(path, lua_dir):
            kept.append(rel)
            continue
        os.remove(path)
        removed.append(rel)
    restored = []
    for name in state.get("moved_aside", []):
        src = os.path.join(scripts_dir, name + OFF_SUFFIX)
        if os.path.isfile(src):
            os.rename(src, os.path.join(scripts_dir, name))
            restored.append(name)
    for rel in state.get("modules_moved_aside", []):
        dst = os.path.join(scripts_dir, rel)
        if os.path.isfile(dst + OFF_SUFFIX):
            os.replace(dst + OFF_SUFFIX, dst)
            restored.append(rel)
    os.remove(_state_path(scripts_dir))
    return {"restored": restored, "removed": removed, "kept": kept, "had_state": True}


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
