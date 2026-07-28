"""2D constant-velocity state estimator, ported from ``modules/state_estimator.lua``.

A Kalman filter over ``[x, y, vx, vy]`` that estimates target velocity (and
position) from position measurements alone. ``TASK-012`` ports it into the harness
so a later task (`TASK-010`) can use the estimated trajectory; the estimation
*use* is not wired into guidance here.

**This preserves the Lua's approximations — it is not a full Kalman filter**
(`ADR`-style "reproduce, not fix", as with the weave's `A-DEC-009`):

* **Simplified covariance update** ``P = (I - K.H).P_pred`` (the Lua author's own
  comment: "this is simplified"). *Not* the numerically-stable Joseph form.
* **Diagonal constant process noise** ``Q = process_noise * I4`` — not a proper
  constant-velocity ``Q`` with ``dt^3/3``, ``dt^2/2`` coupling.
* **Diagonal measurement noise** ``R = measurement_noise * I2``.
* ``P`` starts at ``I4``; :meth:`init` resets only the state, not ``P``.
* The guards: skip-update on a singular ``S``, reset ``P`` on overflow (>1e6) or
  NaN, and reject an update whose ``P`` fails the SPD (Cholesky) check.

It holds its own recursive state, which — with the :class:`~py_harness.state.Harness`
— is the only legitimately stateful part of the harness. It is **not** a
``GeometricAlgorithm`` and is not selected through the algorithm registry. Plain
``math`` and plain lists throughout, so it transliterates back to Lua. The
matrix helpers mirror ``math_helpers.lua``.
"""

import math


# --------------------------------------------------------------------------
# Matrix helpers (plain lists; mirror math_helpers.lua)
# --------------------------------------------------------------------------

def mat_mul(a, b):
    """Matrix product of ``a`` (n x m) and ``b`` (m x p)."""
    n, m, p = len(a), len(b), len(b[0])
    return [[sum(a[i][k] * b[k][j] for k in range(m)) for j in range(p)]
            for i in range(n)]


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def mat_sub(a, b):
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def transpose(a):
    return [[a[i][j] for i in range(len(a))] for j in range(len(a[0]))]


def eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def invert_22(s):
    """Inverse of a 2x2 matrix, or ``None`` if singular (mirrors the Lua guard)."""
    a, b = s[0][0], s[0][1]
    c, d = s[1][0], s[1][1]
    det = a * d - b * c
    if abs(det) < 1e-12:
        return None
    inv_det = 1.0 / det
    return [[d * inv_det, -b * inv_det], [-c * inv_det, a * inv_det]]


def is_spd(m, tol=1e-6):
    """Symmetric positive-definite check via Cholesky (ported from the Lua).

    Returns ``(ok, reason)`` — ``reason`` is ``None`` when ``ok``.
    """
    n = len(m)
    for i in range(n):
        for j in range(i + 1, n):
            if abs(m[i][j] - m[j][i]) > tol:
                return False, "P not symmetric at [%d][%d]" % (i, j)
    lower = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(i + 1):
            s = m[i][j]
            for k in range(j):
                s -= lower[i][k] * lower[j][k]
            if i == j:
                if s <= 0.0:
                    return False, "P not positive definite: pivot %.3e at [%d][%d]" % (s, i, i)
                lower[i][j] = math.sqrt(s)
            else:
                lower[i][j] = s / lower[j][j]
    return True, None


def _big_p():
    """The ``1e4`` diagonal reset used by the Lua overflow/NaN guards."""
    return [[1e4 if i == j else 0.0 for j in range(4)] for i in range(4)]


class KalmanFilter:
    """The 2D constant-velocity filter, faithful to ``state_estimator.lua``.

    State ``x = [x, y, vx, vy]``. Construct, :meth:`init` at a first position,
    then :meth:`update` with each ``(meas_x, meas_y, dt)``.
    """

    def __init__(self, process_noise=0.1, measurement_noise=5.0):
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.x = [0.0, 0.0, 0.0, 0.0]
        self.P = eye(4)
        #: Reason string when the last update was rejected, else None.
        self.last_warning = None

    def init(self, x0, y0):
        """Set the position state; velocity to zero. ``P`` is **not** reset."""
        self.x = [float(x0), float(y0), 0.0, 0.0]

    def update(self, meas_x, meas_y, dt):
        """One predict-correct step. Returns ``{x, y, vx, vy}`` or ``None``.

        ``None`` is returned (and the estimate not committed) when ``S`` is
        singular, ``P`` degrades below SPD, or a NaN appears — mirroring the Lua
        guards, including the ``P`` reset on overflow or NaN.
        """
        self.last_warning = None
        F = [[1.0, 0.0, dt, 0.0],
             [0.0, 1.0, 0.0, dt],
             [0.0, 0.0, 1.0, 0.0],
             [0.0, 0.0, 0.0, 1.0]]
        H = [[1.0, 0.0, 0.0, 0.0],
             [0.0, 1.0, 0.0, 0.0]]
        q = self.process_noise
        Q = [[q, 0.0, 0.0, 0.0], [0.0, q, 0.0, 0.0],
             [0.0, 0.0, q, 0.0], [0.0, 0.0, 0.0, q]]
        r = self.measurement_noise
        R = [[r, 0.0], [0.0, r]]

        x_col = [[v] for v in self.x]

        # Predict.
        x_pred = mat_mul(F, x_col)
        P_pred = mat_add(mat_mul(mat_mul(F, self.P), transpose(F)), Q)

        # Correct.
        z = [[meas_x], [meas_y]]
        y = mat_sub(z, mat_mul(H, x_pred))
        S = mat_add(mat_mul(mat_mul(H, P_pred), transpose(H)), R)

        s_inv = invert_22(S)
        if s_inv is None:
            self.last_warning = "S singular, skipping update"
            if any(self.P[i][i] > 1e6 for i in range(4)):
                self.P = _big_p()
                self.last_warning = "P reset due to overflow"
            return None

        K = mat_mul(mat_mul(P_pred, transpose(H)), s_inv)
        x_new = mat_add(x_pred, mat_mul(K, y))

        # Simplified covariance update (NOT Joseph form) — preserved from the Lua.
        P_new = mat_mul(mat_sub(eye(4), mat_mul(K, H)), P_pred)

        ok, reason = is_spd(P_new)
        if not ok:
            self.last_warning = "covariance degraded - %s" % reason
            return None

        self.x = [row[0] for row in x_new]
        self.P = P_new

        # NaN guard.
        if self.x[0] != self.x[0] or self.x[2] != self.x[2]:
            self.x = [0.0, 0.0, 0.0, 0.0]
            self.P = _big_p()
            self.last_warning = "NaN in state, resetting"
            return None

        return {"x": self.x[0], "y": self.x[1], "vx": self.x[2], "vy": self.x[3]}
