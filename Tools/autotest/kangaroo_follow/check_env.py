"""Check the live environment against the pinned one before any cell is flown
(``TASK-052`` P3).

Every check is a row: ``OK``, ``WARN`` or ``FAIL`` with the thing checked, the
pinned value and the live value. The script exits non-zero on any ``FAIL`` and
names the missing thing, so a later reader on a fresh machine is told what to
install rather than watching SITL fail to start.

``--dry-run`` relaxes the checks that need an ArduPilot **build** (the SITL
binary) to warnings, because the dry run is exactly for a machine that cannot
build (`TASK-052` P7). ``--allow-commit`` accepts a different ArduPilot commit
and is recorded in provenance by the campaign runner, never silently.

Usage::

    python3 -m kangaroo_follow.check_env [--dry-run] [--allow-commit] [--json]   (from Tools/autotest)
"""

import argparse
import importlib
import json
import os
import platform
import subprocess
import sys

from . import paths

OK, WARN, FAIL = "OK", "WARN", "FAIL"


def load_environment(path=paths.ENVIRONMENT_FILE):
    with open(path) as handle:
        return json.load(handle)


def _git(*args, cwd=paths.ARDUPILOT_DIR):
    try:
        out = subprocess.run(("git",) + args, capture_output=True, text=True,
                             timeout=10, cwd=cwd)
    except Exception as exc:                          # pragma: no cover
        return None, str(exc)
    if out.returncode != 0:
        return None, out.stderr.strip()
    return out.stdout.strip(), None


def _package_version(name):
    try:
        module = importlib.import_module(name)
    except Exception as exc:
        return None, "%s: %s" % (type(exc).__name__, exc)
    version = getattr(module, "__version__", None)
    if version is None:
        try:
            from importlib import metadata
            version = metadata.version(name)
        except Exception:
            version = "unknown"
    return str(version), None


def _row(level, what, pinned=None, live=None, hint=None):
    return {"level": level, "what": what, "pinned": pinned, "live": live,
            "hint": hint}


def run_checks(env=None, dry_run=False, allow_commit=False,
               location_name=None):
    """Return the list of check rows. Pure apart from reading the file system."""
    env = env or load_environment()
    rows = []

    # -- layout ---------------------------------------------------------
    for label, path in (("repository root", paths.REPO_ROOT),
                        ("ArduPilot checkout", paths.ARDUPILOT_DIR),
                        ("Tools/autotest", paths.AUTOTEST_DIR),
                        ("py_harness", paths.PY_HARNESS_DIR),
                        ("Lua modules dir", paths.MODULES_DIR)):
        rows.append(_row(OK if os.path.isdir(path) else FAIL, label,
                         live=paths.rel(path),
                         hint=None if os.path.isdir(path) else
                         "git submodule update --init src/ardupilot"))
    for label, name in (("AutoTest suite", "vehicle_test_suite.py"),
                        ("AutoTest plane", "arduplane.py"),
                        ("AutoTest runner", "autotest.py"),
                        ("AutoTest locations", "locations.txt")):
        path = os.path.join(paths.AUTOTEST_DIR, name)
        rows.append(_row(OK if os.path.isfile(path) else FAIL, label,
                         live=paths.rel(path)))
    plane_py = os.path.join(paths.AUTOTEST_DIR, "arduplane.py")
    has_test = False
    if os.path.isfile(plane_py):
        with open(plane_py) as handle:
            has_test = "def KangarooFollowCell(" in handle.read()
    rows.append(_row(OK if has_test else FAIL,
                     "plane test suite carries KangarooFollowCell",
                     live="present" if has_test else "missing",
                     hint=None if has_test else
                     "the TASK-052 test methods are not in this arduplane.py"))

    # -- ArduPilot commit -------------------------------------------------
    pinned = env["ardupilot"]["commit"]
    live, err = _git("rev-parse", "HEAD")
    if live is None:
        rows.append(_row(FAIL, "ArduPilot commit", pinned, err,
                         "the checkout is not a git repository"))
    elif live == pinned:
        rows.append(_row(OK, "ArduPilot commit", pinned, live))
    else:
        rows.append(_row(WARN if allow_commit else FAIL, "ArduPilot commit",
                         pinned, live,
                         "git -C src/ardupilot checkout %s, or pass "
                         "--allow-commit (recorded in provenance)" % pinned[:12]))
    gitlink, _ = _git("ls-files", "--stage", "src/ardupilot", cwd=paths.REPO_ROOT)
    if gitlink:
        link = gitlink.split()[1]
        rows.append(_row(OK if link == pinned else WARN,
                         "parent gitlink for src/ardupilot", pinned, link,
                         None if link == pinned else
                         "the parent repository records a different submodule "
                         "commit; the checkout is what flies"))
    dirty, _ = _git("status", "--porcelain", "--untracked-files=no")
    rows.append(_row(OK if not dirty else WARN, "ArduPilot tree clean",
                     "clean", "dirty" if dirty else "clean",
                     None if not dirty else "tracked files modified in the "
                     "checkout; recorded as ardupilot_dirty in provenance"))

    # -- build ------------------------------------------------------------
    binary = os.path.join(paths.ARDUPILOT_DIR, env["build"]["binary"])
    have = os.path.isfile(binary) and os.access(binary, os.X_OK)
    rows.append(_row(OK if have else (WARN if dry_run else FAIL),
                     "SITL binary", env["build"]["binary"],
                     "present" if have else "missing",
                     None if have else "cd src/ardupilot && %s && %s" % (
                         " ".join(env["build"]["configure"]),
                         " ".join(env["build"]["build"]))))

    # -- Python -----------------------------------------------------------
    minor = "%d.%d" % sys.version_info[:2]
    want = env["python"]["minor"]
    rows.append(_row(OK if minor == want else WARN, "Python minor version",
                     want, minor))
    for name, pinned_version in env["python"]["packages"].items():
        live_version, err = _package_version(name)
        # MAVProxy is a hard import of the AutoTest tree
        # (vehicle_test_suite.py imports MAVProxy.modules.lib); only the dry
        # run can do without it.
        optional = name == "MAVProxy" and dry_run
        if live_version is None:
            rows.append(_row(WARN if optional else FAIL, "package %s" % name,
                             pinned_version, "missing",
                             "python3 -m pip install %s==%s"
                             % (name, pinned_version)))
        elif live_version == pinned_version:
            rows.append(_row(OK, "package %s" % name, pinned_version,
                             live_version))
        else:
            rows.append(_row(WARN, "package %s" % name, pinned_version,
                             live_version, "version differs from the pin"))
    rows.append(_row(OK, "platform", env["captured_on"]["os"],
                     platform.platform()))

    # -- harness importable and the Lua sandbox loads the modules --------
    try:
        importlib.import_module("py_harness.experiment")
        rows.append(_row(OK, "py_harness imports", live=paths.rel(paths.PY_HARNESS_DIR)))
    except Exception as exc:
        rows.append(_row(FAIL, "py_harness imports", live=str(exc)))
    try:
        from py_harness import luadiff
        sandbox = luadiff.LuaSandbox()
        sandbox.execute('package.path = "%s/?.lua;" .. package.path'
                        % paths.LUA_DIR.replace("\\", "/"))
        for name in env["lua"]["modules"] + ["sitl_arms.lua"]:
            sandbox.require(name[:-4])
        rows.append(_row(OK, "Lua sandbox loads every module",
                         live=sandbox.info.describe()))
    except Exception as exc:
        rows.append(_row(FAIL, "Lua sandbox loads every module", live=str(exc),
                         hint="python3 -m pip install lupa==%s"
                         % env["python"]["packages"].get("lupa", "")))

    # -- files this task ships -------------------------------------------
    for label, path in (("runner script", os.path.join(paths.LUA_DIR, paths.RUNNER_SCRIPT)),
                        ("arm registry", os.path.join(paths.LUA_DIR, paths.ARMS_MODULE)),
                        ("parameter file", paths.PARAM_FILE),
                        ("parameter sources", os.path.join(paths.PARAMS_DIR, "SOURCES.md"))):
        rows.append(_row(OK if os.path.isfile(path) else FAIL, label,
                         live=paths.rel(path)))
    for name in env["lua"]["modules"]:
        path = os.path.join(paths.MODULES_DIR, name)
        rows.append(_row(OK if os.path.isfile(path) else FAIL,
                         "ported module %s" % name, live=paths.rel(path),
                         hint=None if os.path.isfile(path) else
                         "the TASK-006 port is missing from the checkout"))

    # -- location ---------------------------------------------------------
    name = location_name or env["location"]["name"]
    found = False
    if os.path.isfile(paths.LOCATIONS_FILE):
        with open(paths.LOCATIONS_FILE) as handle:
            for line in handle:
                if line.split("=")[0].strip() == name:
                    found = True
                    break
    rows.append(_row(OK if found else FAIL, "SITL location", name,
                     "found" if found else "not in Tools/autotest/locations.txt"))
    return rows


def summarise(rows):
    fails = [r for r in rows if r["level"] == FAIL]
    warns = [r for r in rows if r["level"] == WARN]
    return {"ok": not fails, "fail": len(fails), "warn": len(warns),
            "rows": rows}


def format_rows(rows):
    lines = []
    for r in rows:
        line = "%-4s %s" % (r["level"], r["what"])
        if r["pinned"] is not None or r["live"] is not None:
            line += "  [pinned: %s | live: %s]" % (r["pinned"], r["live"])
        if r["hint"]:
            line += "\n     -> %s" % r["hint"]
        lines.append(line)
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-commit", action="store_true")
    parser.add_argument("--location", default=None)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    rows = run_checks(dry_run=args.dry_run, allow_commit=args.allow_commit,
                      location_name=args.location)
    result = summarise(rows)
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(format_rows(rows))
        print("\n%d FAIL, %d WARN" % (result["fail"], result["warn"]))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
