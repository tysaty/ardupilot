"""SITL AutoTest campaign for the harness experiments (``TASK-052``).

Campaign machinery in the ArduPilot AutoTest tree, beside ``arduplane.py``
whose ``KangarooFollowCell`` and ``KangarooFollowCampaign`` tests fly the
cells (the same place the mid-term ``DubinsSweep`` lived; `TASK-052` D1 as
directed by the author, 2026-09-16). It reads the Python harness at
``scripts/py_harness`` and the ported Lua at ``scripts/modules`` in the same
checkout through :mod:`kangaroo_follow.paths`, and writes the same bundle
format the Python campaigns write, so no figure code changes.

Start at ``kangaroo_follow/README.md``.
"""
