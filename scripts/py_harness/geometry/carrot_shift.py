"""The carrot lead of arm F: the estimated velocity over one control tick (``TASK-060``).

Arm F builds the baseline CS-onto-ring guidance (``dubins_target_orbit``)
about the raw state estimate and then moves only the guidance point it
returns, the carrot, by the estimator's velocity over ``step_ticks`` control
ticks::

    g' = g + v_est * dt_s * step_ticks

The path, ring, phase, orbit sense and curvature stay exactly the baseline's.
This module holds only that lead, so the arm differs from the baseline in one
function and nothing else (``VR-009``), as ``velocity_db_circle`` holds only
arm D's stepped centre.

Not arm D: arm D moves the RING CENTRE by the same one-tick lead and re-solves
the path about it; arm F keeps the path and moves the commanded point.

Lua counterpart: ``M.carrot_shift`` in ``modules/harness_carrot_shift.lua``,
which returns ``nil, reason`` where this raises ``ValueError``.

Frame: the estimate is ``{"n_m", "e_m", "vn_ms", "ve_ms"}`` (North/East);
the lead is returned as ``(east, north)`` metres, the geometry frame.
Stateless (``VR-015``, ``A-VAL-003``).
"""

import math


def carrot_shift(est_raw, dt_s, step_ticks=1):
    """The lead to add to the carrot, as ``(shift_e_m, shift_n_m)`` metres.

    Args:
        est_raw: The un-projected estimate ``snapshot["target_est_raw"]``.
        dt_s: Control interval, seconds (``cfg["dt_s"]``).
        step_ticks: Whole control ticks to lead (``cfg["af_step_ticks"]``).
            1 is the arm as specified.

    Returns:
        ``(shift_e_m, shift_n_m)``.

    Raises:
        ValueError: If ``est_raw`` is None, ``dt_s`` is not a positive finite
            number, or ``step_ticks`` is below 1. The messages match the Lua
            port's refusals.
    """
    if est_raw is None:
        raise ValueError("carrot_shift_cs requires the state estimator")
    if not isinstance(dt_s, (int, float)) or not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be a positive number")
    if not isinstance(step_ticks, (int, float)) or step_ticks < 1:
        raise ValueError(
            "af_step_ticks must be a number >= 1 (is it in config_dict?)")
    lead_s = dt_s * step_ticks
    return est_raw["ve_ms"] * lead_s, est_raw["vn_ms"] * lead_s
