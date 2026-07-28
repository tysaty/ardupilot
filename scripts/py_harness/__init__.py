"""Offline geometric validation harness.

Two modules with separated responsibilities, per ``DEC-2026-06-25-01``:

* :mod:`py_harness.state` — plane and target state, time stepping, the
  recorded history and the read-only plotter;
* :mod:`py_harness.algorithms` — substitutable geometric algorithms behind the
  single interface defined in :mod:`py_harness.interface`.

Python is the **validation** language here and Lua is the **target** language.
This is a scratch harness for establishing that a geometry behaves as intended
before it is committed to Lua. It is not a second implementation to be
maintained in parallel with the controller, and it does not replace SITL.

Status: SKELETON — structure only, no geometry implemented. See
``tasks/active/TASK-001-python-geometric-validation-harness.md``.
"""

__all__ = ["algorithms", "interface", "state"]
