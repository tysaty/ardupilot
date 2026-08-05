"""State-estimate look-ahead / prediction horizon (``TASK-017``).

Projects a target estimate forward on its estimated constant velocity, so guidance
can plan toward where the target *will* be rather than where it is. This reinstates
the finite-horizon prediction the mid-term report used (``MTR-KF``) and discharges
``FR-003`` (a configurable future prediction horizon), which was lost when
``predict_position`` was removed with the Dubins build (``ADR-001``).

    pos(k+n) = pos(k) + v_est * n * dt        (velocity held constant)

Pure, stateless, plain ``math``/dicts (no ``numpy``) so it transliterates to Lua
(``VR-015``). It reads the target-estimate dict the state module builds from the
Kalman filter — ``{n_m, e_m, vn_ms, ve_ms}`` — and returns the same shape. The
horizon is ``n_steps`` control intervals (``horizon = n_steps * dt``); ``n = 0`` is
the identity, so the look-ahead is opt-in and off by default (``A-TGT-002`` bounds
how far the constant-velocity projection stays useful).
"""


def predict(estimate, dt_s, n_steps):
    """Project a target estimate ``n_steps`` control intervals ahead.

    Args:
        estimate: ``{n_m, e_m, vn_ms, ve_ms}`` target estimate (from the Kalman
            filter), or ``None``.
        dt_s: Control interval, seconds (the harness step, ``DT_S``).
        n_steps: Horizon in whole steps, ``>= 0``. ``0`` returns the estimate
            unchanged; the horizon in seconds is ``n_steps * dt_s``.

    Returns:
        A new ``{n_m, e_m, vn_ms, ve_ms}`` dict advanced on constant velocity, or
        ``None`` when ``estimate`` is ``None``. The velocity is carried through
        unchanged (constant-velocity model, ``A-TGT-002``).

    Raises:
        ValueError: For a negative ``n_steps``.
    """
    if n_steps < 0:
        raise ValueError("n_steps must be >= 0, got %r" % n_steps)
    if estimate is None:
        return None
    horizon_s = n_steps * dt_s
    return {
        "n_m": estimate["n_m"] + estimate["vn_ms"] * horizon_s,
        "e_m": estimate["e_m"] + estimate["ve_ms"] * horizon_s,
        "vn_ms": estimate["vn_ms"],
        "ve_ms": estimate["ve_ms"],
    }
