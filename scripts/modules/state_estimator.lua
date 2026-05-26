-- =========================================================
-- 2D State Estimator for velocity 
-- =========================================================

-- calling helpers
local math_helpers = require("math_helpers")

-- intiialise estimator
local kf = {}
-- process noise - Q Diagonal, higher values insinuate more trust
kf.process_noise = 0.1
-- measurement noise - for R diagonal, higher values weigh more trust on the model
kf.measurement_noise = 5.0

-- State vector
-- x = {x, y, vx, vy}
kf.x = {0, 0, 0, 0}

-- Covariance matrix
kf.P = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
}

-- ---------------------------------------------------------
-- Check if matrix M is symmetric positive-definite.
-- Cholesky decomposition attempt — if any pivot is <= 0
-- the matrix is not positive definete; 
-- asymmetry beyond tolerance is flagged
-- Returns: ok (bool) and the reason (string or nil)
-- ---------------------------------------------------------
local function is_spd(M)
    local n = #M
    local tol = 1e-6
    -- symmetry check
    for i = 1, n do
        for j = i + 1, n do
            if math.abs(M[i][j] - M[j][i]) > tol then
                return false, string.format("P not symmetric at [%d][%d]: %.3e vs %.3e", i, j, M[i][j], M[j][i])
            end
        end
    end
    -- Cholesky: build lower-triangular L; only needs L[i][k] for k < i
    local L = {}
    for i = 1, n do
        L[i] = {}
        for j = 1, n do L[i][j] = 0 end
    end
    for i = 1, n do
        for j = 1, i do
            local s = M[i][j]
            for k = 1, j - 1 do
                s = s - L[i][k] * L[j][k]
            end
            if i == j then
                if s <= 0 then
                    return false, string.format("P not Positive Definite: non-positive pivot at [%d][%d] val=%.3e", i, i, s)
                end
                L[i][j] = math.sqrt(s)
            else
                L[i][j] = s / L[j][j]
            end
        end
    end
    return true, nil
end

-- ---------------------------------------------------------
-- init estimator
-- ---------------------------------------------------------
function kf.init(x0, y0)
    kf.x = {x0, y0, 0, 0}
end

-- ---------------------------------------------------------
-- Update estimator
-- taking measured target position and timestep
-- ---------------------------------------------------------
function kf.update(meas_x, meas_y, dt)

    -- state transition matrix
    local F = {
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    }
    -- measurment matrix (i.e. only taking x and y)
    local H = {
        {1, 0, 0, 0},
        {0, 1, 0, 0}
    }

    local Q = {
        {kf.process_noise, 0, 0, 0},
        {0, kf.process_noise, 0, 0},
        {0, 0, kf.process_noise, 0},
        {0, 0, 0, kf.process_noise}
    }

    local R = {
        {kf.measurement_noise, 0},
        {0, kf.measurement_noise}
    }

    -- converting to column
    local x_col = math_helpers.vec_to_col(kf.x)

    -- -----------------------------------------------------
    -- Predict step
    -- -----------------------------------------------------

    -- prediction of x: x_pred = F . x
    local x_pred = math_helpers.mat_mul(F, x_col)
    
    -- prediction of covariance matrix: P_pred = F . P . F(transpose) + Q
    local P_pred = math_helpers.mat_add(
        math_helpers.mat_mul(math_helpers.mat_mul(F, kf.P),
        math_helpers.transpose(F)),
        Q
    )

    -- -----------------------------------------------------
    -- Correction step
    -- -----------------------------------------------------
    -- calling observed value, reorgnaising
    local z = math_helpers.vec_to_col({meas_x, meas_y})

    -- updating y values: y = z - H . x_pred
    local y = math_helpers.mat_sub(z, math_helpers.mat_mul(H, x_pred))

    -- invoking covariance/resiudal covariance
    -- S = H . P_pred . H(transpose) + R
    local S = math_helpers.mat_add(
        math_helpers.mat_mul(math_helpers.mat_mul(H, P_pred), math_helpers.transpose(H)),
        R
    )

    -- inverting S
    local S_inv = math_helpers.invert_22(S)
    if not S_inv then
        gcs:send_text(4, "KF: S singular, skipping update")
        -- Reset P if covariance has exploded to prevent permanent filter lockout
        if kf.P[1][1] > 1e6 or kf.P[2][2] > 1e6 or kf.P[3][3] > 1e6 or kf.P[4][4] > 1e6 then
            kf.P = {{1e4,0,0,0},{0,1e4,0,0},{0,0,1e4,0},{0,0,0,1e4}}
            gcs:send_text(4, "KF: P reset due to overflow")
        end
        return nil
    end

    -- Kalman gain
    -- K = P_pred . H(transpose) . inverse_S
    local K = math_helpers.mat_mul(
        math_helpers.mat_mul(P_pred, math_helpers.transpose(H)),
        S_inv
    )

    -- update state
    -- x = x_pred + K.y
    local x_new = math_helpers.mat_add(
        x_pred,
        math_helpers.mat_mul(K, y)
    )

    -- identity matrix
    local I = math_helpers.eye(4)

    -- Correct covariance
    -- update prediction -- P_new = (I - K.H) . P_pred
    -- note this is simplified - check any issues
    local P_new = math_helpers.mat_mul(
        math_helpers.mat_sub(I, math_helpers.mat_mul(K, H)),
        P_pred
    )

    -- guard: check P_new is still symmetric positive-definite before committing
    local spd_ok, spd_reason = is_spd(P_new)
    if not spd_ok then
        gcs:send_text(3, "KF WARNING: covariance degraded — " .. tostring(spd_reason))
        return nil
    end

    -- flipping values
    kf.x = math_helpers.col_to_vec(x_new)
    kf.P = P_new

    -- Guard against NaN propagating into state (e.g. from extreme noise params)
    if kf.x[1] ~= kf.x[1] or kf.x[3] ~= kf.x[3] then
        kf.x = {0, 0, 0, 0}
        kf.P = {{1e4,0,0,0},{0,1e4,0,0},{0,0,1e4,0},{0,0,0,1e4}}
        gcs:send_text(4, "KF: NaN in state, resetting")
        return nil
    end

    -- return values
    return {
        x = kf.x[1],
        y = kf.x[2],
        vx = kf.x[3],
        vy = kf.x[4]
    }
end

-- ---------------------------------------------------------
-- Return estimator
-- ---------------------------------------------------------
return kf
