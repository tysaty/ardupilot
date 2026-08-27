"""Adaptive Dubins circle — orbit a *predicted* target position (``TASK-033``).

**Status: SKELETON. Nothing is implemented.** The task file
``tasks/active/TASK-033-adaptive-DB-circle.md`` is the specification; this module
is the place its geometry will go. It is deliberately not registered in
:mod:`py_harness.algorithms`, so importing it changes no existing behaviour.

The intent
----------
Every orbit family in the harness today centres the standoff ring on where the
target **is** (``snapshot["target_n_m"]``/``["target_e_m"]``). This module is to
centre it on where the target is predicted to be at a window ``k`` control
intervals ahead, so both the Dubins approach and the orbit hold are planned
against the prediction rather than against a position the aircraft can only ever
arrive behind.

The prediction machinery already exists and does not need rebuilding:

* :mod:`py_harness.estimator` — constant-velocity Kalman filter, transliterated
  from ``modules/state_estimator.lua`` (``TASK-012``).
* :mod:`py_harness.lookahead` — ``predict(estimate, dt_s, n_steps)``, the
  constant-velocity projection (``TASK-017``). The horizon in seconds is
  ``k * dt_s``.
* The harness publishes the projected estimate to algorithms as
  ``snapshot["target_est"]`` — ``{n_m, e_m, vn_ms, ve_ms}``, or ``None`` when no
  estimator is running.

So the open work is geometric, not estimative: what the ring is centred on, and
whether it leads the target only on approach or permanently.

Constraints inherited from the rest of the harness
--------------------------------------------------
* Frame ``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).
* Plain ``math``, plain floats and dicts; no ``numpy``; no module-level mutable
  state — any accumulated state travels through ``algorithm_state``
  (``VR-015``, ``A-VAL-003``).
* Orbit curvature bound ``R >= turn_radius_m``.
* ``A-TGT-002`` bounds how far the constant-velocity projection stays useful; a
  manoeuvring kangaroo violates it continuously.
"""

import math  # noqa: F401  (kept so the module opens ready to write geometry in)


def guidance(px, py, psi_i, tx, ty, tvn, tve, orbit_radius_m, turn_radius_m,
             look_ahead_m, horizon_steps, dt_s):
    """One guidance point about the predicted target position.

    Signature is a starting point, not a contract — change it freely while
    populating ``TASK-033``.

    Args:
        px: Aircraft East position, metres.
        py: Aircraft North position, metres.
        psi_i: Aircraft heading, radians clockwise from North.
        tx: Target (or estimated target) East position, metres.
        ty: Target (or estimated target) North position, metres.
        tvn: Target North velocity, metres per second.
        tve: Target East velocity, metres per second.
        orbit_radius_m: Commanded standoff ring radius, metres.
        turn_radius_m: Minimum turn radius, metres.
        look_ahead_m: Guidance-point arc advance along the ring, metres.
        horizon_steps: The window ``k``, in whole control intervals, ``>= 0``.
            ``0`` is the present-position behaviour.
        dt_s: Control interval, seconds. The horizon is ``horizon_steps * dt_s``.

    Returns:
        Not decided. The other orbit modules return a dict of at least
        ``{"gx", "gy"}``; see :mod:`py_harness.geometry.dubins_target_orbit`.

    Raises:
        NotImplementedError: Always, until ``TASK-033`` is implemented.
    """
    raise NotImplementedError(
        "TASK-033 (adaptive Dubins circle) is not implemented. See "
        "tasks/active/TASK-033-adaptive-DB-circle.md")
