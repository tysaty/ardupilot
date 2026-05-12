-- =========================================================
-- Simple 2D State Estimator (Constant Velocity Kalman Filter)
-- =========================================================

-- calling helpers
local math_helpers = require("math_helpers")

-- Tunable noise parameters
local process_noise = 0.01
local measurement_noise = 5.0

-- intiialise estimator
local kf = {}

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

    local q = 0.1

    local r = 5.0

    local Q = {
        {q, 0, 0, 0},
        {0, q, 0, 0},
        {0, 0, q, 0},
        {0, 0, 0, q}
    }

    local R = {
        {r, 0},
        {0, r}
    }

    -- converting to column
    local x_col = math_helpers.vec_to_col(kf.x)

    -- -----------------------------------------------------
    -- Predict
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
    local y = math_helpers.at_sub(z, math_helpers.mat_mul(H, x_pred))

    -- invoking covariance/resiudal covariance
    -- S = H . P_pred . H(transpose) + R
    local S = math_helpers.mat_add(
        math_helpers.mat_mul(math_helpers.mat_mul(H, P_pred), math_helpers.transpose(H)),
        R
    )

    -- inverting S
    local S_inv = math_helpers.invert_22(S)
    if not S_inv then
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
    local P_new = math_helpers.mat_mul(
        math_helpers.mat_sub(I, math_helpers.mat_mul(K, H)),
        P_pred
    )

    -- flipping values
    kf.x = col_to_vec(x_new)
    kf.P = P_new

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
