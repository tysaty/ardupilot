-- =========================================================
--  harness_estimator -- constant-velocity target estimator and look-ahead
--  created 2026-09-03
--  TASK-006 Tranche 3 (reverse check, plus a small forward port).
--
--  A Kalman filter over [x, y, vx, vy] estimating target velocity from position
--  measurements alone, plus the look-ahead projection (TASK-017).
--
--  Direction of work
--  -----------------
--  The FILTER is a reverse check: py_harness/estimator.py was transliterated
--  FROM modules/state_estimator.lua under TASK-012, so this compares two Lua
--  implementations of the same filter through a Python intermediary. The
--  LOOK-AHEAD is a forward port -- roughly ten lines, and n_steps = 0 must be
--  exactly the identity.
--
--  This preserves the original's APPROXIMATIONS. It is not a correct Kalman
--  filter and must not be improved here (TASK-006's no-behaviour-change rule):
--
--    * simplified covariance update P = (I - K.H).P_pred, NOT the numerically
--      stable Joseph form -- the original author's own comment says "this is
--      simplified";
--    * diagonal constant process noise Q = q*I4, not a constant-velocity Q with
--      dt^3/3 and dt^2/2 coupling;
--    * diagonal measurement noise R = r*I2;
--    * P starts at I4 and init() resets only the state, NOT P.
--
--  The differential test must confirm the APPROXIMATIONS AGREE, not that the
--  filter is correct.
--
--  The four guards -- singular S, covariance not SPD, overflow, NaN -- are the
--  cases most likely to diverge, because each depends on a floating-point
--  comparison against a threshold. The Tranche 3 gate requires at least one run
--  that trips each.
--
--  Statefulness: this is the ONE ported module that legitimately holds state,
--  and it holds it in an EXPLICIT filter object the caller owns, never at module
--  level. `new()` returns that object; the module itself stays a table of
--  functions (VR-015, A-VAL-003).
-- =========================================================

local M = {}

-- ---------------------------------------------------------
-- Matrix helpers
-- ---------------------------------------------------------
-- Local rather than taken from modules/math_helpers.lua: that module is the
-- SHIPPING controller's and TASK-006 must not couple a port to flight code it
-- is not also porting. The implementations are equivalent; invert_22's
-- singularity threshold differs and is called out below.

local function mat_mul(a, b)
    local n, m, p = #a, #b, #b[1]
    local res = {}
    for i = 1, n do
        res[i] = {}
        for j = 1, p do
            local s = 0.0
            for k = 1, m do
                s = s + a[i][k] * b[k][j]
            end
            res[i][j] = s
        end
    end
    return res
end

local function mat_add(a, b)
    local res = {}
    for i = 1, #a do
        res[i] = {}
        for j = 1, #a[1] do
            res[i][j] = a[i][j] + b[i][j]
        end
    end
    return res
end

local function mat_sub(a, b)
    local res = {}
    for i = 1, #a do
        res[i] = {}
        for j = 1, #a[1] do
            res[i][j] = a[i][j] - b[i][j]
        end
    end
    return res
end

local function transpose(a)
    local res = {}
    for i = 1, #a[1] do
        res[i] = {}
        for j = 1, #a do
            res[i][j] = a[j][i]
        end
    end
    return res
end

local function eye(n)
    local res = {}
    for i = 1, n do
        res[i] = {}
        for j = 1, n do
            res[i][j] = (i == j) and 1.0 or 0.0
        end
    end
    return res
end

--- Inverse of a 2x2 matrix, or nil when singular.
--  The threshold is 1e-12, matching py_harness/estimator.py. NOTE:
--  modules/math_helpers.lua uses 1e-9 for the same guard. The harness value is
--  used here because the harness is the reference the differential test scores
--  against; the difference is recorded rather than silently reconciled, and it
--  is a real (if narrow) behavioural gap between the two Lua modules.
local function invert_22(s)
    local a, b = s[1][1], s[1][2]
    local c, d = s[2][1], s[2][2]
    local det = a * d - b * c
    if math.abs(det) < 1e-12 then
        return nil
    end
    local inv = 1.0 / det
    return { { d * inv, -b * inv }, { -c * inv, a * inv } }
end

--- Symmetric positive-definite check via Cholesky. Returns ok, reason.
local function is_spd(m, tol)
    tol = tol or 1e-6
    local n = #m
    for i = 1, n do
        for j = i + 1, n do
            if math.abs(m[i][j] - m[j][i]) > tol then
                return false, "P not symmetric"
            end
        end
    end
    local lower = {}
    for i = 1, n do
        lower[i] = {}
        for j = 1, n do
            lower[i][j] = 0.0
        end
    end
    for i = 1, n do
        for j = 1, i do
            local s = m[i][j]
            for k = 1, j - 1 do
                s = s - lower[i][k] * lower[j][k]
            end
            if i == j then
                if s <= 0.0 then
                    return false, "P not positive definite"
                end
                lower[i][j] = math.sqrt(s)
            else
                lower[i][j] = s / lower[j][j]
            end
        end
    end
    return true, nil
end

--- The 1e4 diagonal reset used by the overflow and NaN guards.
local function big_p()
    local res = {}
    for i = 1, 4 do
        res[i] = {}
        for j = 1, 4 do
            res[i][j] = (i == j) and 1e4 or 0.0
        end
    end
    return res
end

M.mat_mul = mat_mul
M.mat_add = mat_add
M.mat_sub = mat_sub
M.transpose = transpose
M.eye = eye
M.invert_22 = invert_22
M.is_spd = is_spd

-- ---------------------------------------------------------
-- The filter
-- ---------------------------------------------------------

local Filter = {}
Filter.__index = Filter

--- A new filter. State zeroed, P = I4, and the noise parameters bound.
function M.new(process_noise, measurement_noise)
    local self = setmetatable({}, Filter)
    self.process_noise = process_noise or 0.1
    self.measurement_noise = measurement_noise or 5.0
    self.x = { 0.0, 0.0, 0.0, 0.0 }
    self.P = eye(4)
    self.last_warning = nil
    return self
end

--- Set the position state; velocity to zero. P is deliberately NOT reset --
--  faithful to the original, and a real difference from a textbook filter.
function Filter:init(x0, y0)
    self.x = { x0, y0, 0.0, 0.0 }
end

--- One predict-correct step. Returns a table {x, y, vx, vy}, or nil when the
--  update was rejected (S singular, P not SPD, or NaN) -- in which case the
--  estimate is NOT committed and `last_warning` says why.
function Filter:update(meas_x, meas_y, dt)
    self.last_warning = nil
    local F = { { 1.0, 0.0, dt,  0.0 },
                { 0.0, 1.0, 0.0, dt  },
                { 0.0, 0.0, 1.0, 0.0 },
                { 0.0, 0.0, 0.0, 1.0 } }
    local H = { { 1.0, 0.0, 0.0, 0.0 },
                { 0.0, 1.0, 0.0, 0.0 } }
    local q = self.process_noise
    local Q = { { q, 0.0, 0.0, 0.0 }, { 0.0, q, 0.0, 0.0 },
                { 0.0, 0.0, q, 0.0 }, { 0.0, 0.0, 0.0, q } }
    local r = self.measurement_noise
    local R = { { r, 0.0 }, { 0.0, r } }

    local x_col = {}
    for i = 1, 4 do
        x_col[i] = { self.x[i] }
    end

    -- Predict.
    local x_pred = mat_mul(F, x_col)
    local P_pred = mat_add(mat_mul(mat_mul(F, self.P), transpose(F)), Q)

    -- Correct.
    local z = { { meas_x }, { meas_y } }
    local y = mat_sub(z, mat_mul(H, x_pred))
    local S = mat_add(mat_mul(mat_mul(H, P_pred), transpose(H)), R)

    local s_inv = invert_22(S)
    if s_inv == nil then
        self.last_warning = "S singular, skipping update"
        local overflow = false
        for i = 1, 4 do
            if self.P[i][i] > 1e6 then
                overflow = true
            end
        end
        if overflow then
            self.P = big_p()
            self.last_warning = "P reset due to overflow"
        end
        return nil
    end

    local K = mat_mul(mat_mul(P_pred, transpose(H)), s_inv)
    local x_new = mat_add(x_pred, mat_mul(K, y))

    -- Simplified covariance update (NOT Joseph form) -- preserved deliberately.
    local P_new = mat_mul(mat_sub(eye(4), mat_mul(K, H)), P_pred)

    local ok, reason = is_spd(P_new)
    if not ok then
        self.last_warning = "covariance degraded - " .. tostring(reason)
        return nil
    end

    for i = 1, 4 do
        self.x[i] = x_new[i][1]
    end
    self.P = P_new

    -- NaN guard. `v ~= v` is the Lua idiom for isnan.
    if self.x[1] ~= self.x[1] or self.x[3] ~= self.x[3] then
        self.x = { 0.0, 0.0, 0.0, 0.0 }
        self.P = big_p()
        self.last_warning = "NaN in state, resetting"
        return nil
    end

    return { x = self.x[1], y = self.x[2], vx = self.x[3], vy = self.x[4] }
end

M.Filter = Filter

-- ---------------------------------------------------------
-- Look-ahead -- TASK-017, forward port
-- ---------------------------------------------------------

--- Project a target estimate n_steps control intervals ahead on constant
--  velocity: pos(k+n) = pos(k) + v*n*dt. Velocity is carried through unchanged
--  (A-TGT-002).
--
--  n_steps = 0 is EXACTLY the identity, which is what makes the look-ahead
--  opt-in and off by default. Returns nil for a nil estimate, and nil plus a
--  reason for a negative n_steps.
--
--  `estimate` is a table {n_m, e_m, vn_ms, ve_ms} -- the harness frame, not the
--  geometry frame.
function M.predict(estimate, dt_s, n_steps)
    if n_steps < 0 then
        return nil, "n_steps must be >= 0"
    end
    if estimate == nil then
        return nil
    end
    local horizon_s = n_steps * dt_s
    return {
        n_m = estimate.n_m + estimate.vn_ms * horizon_s,
        e_m = estimate.e_m + estimate.ve_ms * horizon_s,
        vn_ms = estimate.vn_ms,
        ve_ms = estimate.ve_ms,
    }
end

return M
