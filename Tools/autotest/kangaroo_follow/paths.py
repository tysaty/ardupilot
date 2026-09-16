"""Where everything is, relative to the ArduPilot checkout (``TASK-052`` P2).

This package lives in the ArduPilot AutoTest tree
(``Tools/autotest/kangaroo_follow/``) beside ``arduplane.py``, which carries
the ``KangarooFollowCell`` / ``KangarooFollowCampaign`` tests; the assets a
cell flies (the runner and arm-registry Lua, the parameter file) are under
``ArduPlane_Tests/KangarooFollow/`` like every other plane test's files. The
harness it repeats is ``scripts/py_harness`` in the same checkout, and the
campaigns it reads and writes are in the parent repository's
``experiments/campaigns/``.

No absolute paths and no machine names: everything is derived from this
file's location. Importing it puts the harness on ``sys.path`` exactly as the
parent repository's ``tests/unit/conftest.py`` does.
"""

import os
import sys

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
AUTOTEST_DIR = os.path.abspath(os.path.join(PACKAGE_DIR, os.pardir))
ARDUPILOT_DIR = os.path.abspath(os.path.join(AUTOTEST_DIR, os.pardir, os.pardir))
#: The parent (research) repository: ``src/ardupilot`` is its submodule.
REPO_ROOT = os.path.abspath(os.path.join(ARDUPILOT_DIR, os.pardir, os.pardir))
SCRIPTS_DIR = os.path.join(ARDUPILOT_DIR, "scripts")
MODULES_DIR = os.path.join(SCRIPTS_DIR, "modules")
PY_HARNESS_DIR = os.path.join(SCRIPTS_DIR, "py_harness")
LOCATIONS_FILE = os.path.join(AUTOTEST_DIR, "locations.txt")
AUTOTEST_PY = os.path.join(AUTOTEST_DIR, "autotest.py")
SITL_BINARY = os.path.join(ARDUPILOT_DIR, "build", "sitl", "bin", "arduplane")
SITL_LOGS_DIR = os.path.join(ARDUPILOT_DIR, "logs")
#: Test assets, in the ArduPlane_Tests convention.
ASSETS_DIR = os.path.join(AUTOTEST_DIR, "ArduPlane_Tests", "KangarooFollow")
LUA_DIR = ASSETS_DIR
PARAMS_DIR = ASSETS_DIR
PARAM_FILE = os.path.join(ASSETS_DIR, "kangaroo-follow.parm")
ENVIRONMENT_FILE = os.path.join(PACKAGE_DIR, "environment.json")
CAMPAIGNS_DIR = os.path.join(REPO_ROOT, "experiments", "campaigns")

#: Environment variable through which ``autotest.py test.Plane.KangarooFollowCell``
#: receives its cell plan (a ``plan.json`` path) and
#: ``test.Plane.KangarooFollowCampaign`` its manifest directory.
PLAN_ENV = "KANGAROO_FOLLOW_PLAN"
CAMPAIGN_ENV = "KANGAROO_FOLLOW_CAMPAIGN"

#: The Lua files a cell flies, by role. The runner and the arm registry are
#: this task's; the modules are TASK-006's ported libraries, staged unchanged.
RUNNER_SCRIPT = "sitl_harness_runner.lua"
ARMS_MODULE = "sitl_arms.lua"
CELL_MODULE = "sitl_cell.lua"
HARNESS_MODULES = (
    "harness_geom.lua", "harness_dubins.lua", "harness_orbit.lua",
    "harness_cs_orbit.lua", "harness_kangaroo.lua", "harness_segments.lua",
    "harness_estimator.lua", "harness_adaptive_db.lua",
    "harness_adaptive_horizon.lua", "harness_rh_geometric.lua",
)
#: Scripts that would fight the runner for the vehicle and are moved aside
#: for the duration of a cell (`stage_scripts.py`).
CONFLICTING_SCRIPTS = ("control_cont.lua", "kangaroo_MAV.lua")


def add_harness_to_path():
    """Make ``py_harness`` and this package importable. Idempotent."""
    for path in (SCRIPTS_DIR, AUTOTEST_DIR, REPO_ROOT):
        if path not in sys.path:
            sys.path.insert(0, path)


def rel(path):
    """``path`` relative to the parent repository root, for records and messages."""
    return os.path.relpath(os.path.abspath(path), REPO_ROOT)


add_harness_to_path()
