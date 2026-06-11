-- =========================================================
-- Control function
-- =========================================================
-- =========================================================
-- 1. Initialising 
-- =========================================================

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- reporting variables
local REPORT_INTERVAL_MS = 2000
local last_report_ms = 0
local last_kf_report_ms = 0
-- local last_reported_dubins_index = nil  -- Dubins reporting, removed

-- kangaroo variables
local kangaroo_loc_pending = nil

-- maintain aircraft 150m above virtual target altitude
-- below CASR requriements
local PLANE_ABOVE_TARGET_M = 120.0

-- KBUS_ bus params (owned by kangaroo_MAV.lua)
-- bounded loosely because control.lua will boot first (alpha betical)
local kbus_seq_param = Parameter()
local kbus_t_s_param = Parameter()
local kbus_lat_param = Parameter()
local kbus_lon_param = Parameter()
local kbus_vn_param  = Parameter()
local kbus_ve_param  = Parameter()
local kbus_all_ready = false
-- refresh for the bus
local last_bus_seq_seen = 0

-- import modules
-- local dubins_points = require("dubins_weave_full")  -- no Dubins or orbit paths remain
local math_helpers = require("math_helpers")
local param_helpers = require("param_helpers")
local kf = require("state_estimator")
local continuous_weave = require("continuous_weave")

-- continuous_weave.lua calls clamp() as a bare global; bridge it from math_helpers
clamp = function(v, lo, hi) return math_helpers.clamp(v, lo, hi) end

-- continuous weave state (arc-length accumulator and phase)
local cw_s        = 0.0
local cw_phase    = 0.0
local last_cw_ms  = nil

-- Kalman filter initialisiation for the filter
-- initialising for KF filter - reference fised 
local kf_ref_loc = nil
-- latest state reading
local kf_state = nil
-- timestamp of last KF update (ms)
local last_kf_t_ms = nil
local kf_initialized = false


-- boot message for MAV
gcs:send_text(4, "Control: loaded at boot")

-- ---------------------------------------------------------
-- Parameter table for control variables
-- ---------------------------------------------------------
local CTRL_TABLE_PREFIX = "CTRL_"
local CTRL_TABLE_KEY = nil

-- establish parameter table key
for key = 0, 200 do
    if param:add_table(key, CTRL_TABLE_PREFIX, 9) then
        CTRL_TABLE_KEY = key
        break
    end
end
assert(CTRL_TABLE_KEY ~= nil, "CTRL: no free param table key")

-- -----------------------------------------------------------------------
-- Parameter value declaration — continuous weave
-- -----------------------------------------------------------------------
-- Weave spatial wavelength (m)
-- local CTRL_CW_LAMBDA  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_LAMBDA", 1, 300)  -- original (R_min=150)
-- local CTRL_CW_LAMBDA  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_LAMBDA", 1, 110)
-- local CTRL_CW_LAMBDA  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_LAMBDA", 1, 20)
local CTRL_CW_LAMBDA  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_LAMBDA", 1, 60)
-- Minimum aircraft turn radius (m)
-- local CTRL_CW_RMIN    = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_RMIN",2, 150)  -- original
local CTRL_CW_RMIN    = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_RMIN",2, 20)
-- Maximum weave amplitude cap (m)
-- local CTRL_CW_ACAP = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_ACAP", 3, 80) -- original
-- local CTRL_CW_ACAP = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_ACAP", 3, 20) -- prev
local CTRL_CW_ACAP = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_ACAP", 3, 5)
-- Distance at which weave begins (d_start, must be > d_full)
-- local CTRL_CW_DSTART = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_DSTART", 4, 600)  -- original
-- local CTRL_CW_DSTART = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_DSTART", 4, 220) -- prev
local CTRL_CW_DSTART = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_DSTART", 4, 300)
-- Curvature safety margin (0–1)
local CTRL_CW_ETA = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_ETA", 5, 0.8)

-- Distance at which weave reaches full amplitude (d_full, must be < d_start)
-- local CTRL_CW_DFULL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_DFULL", 6, 150)   -- original
-- local CTRL_CW_DFULL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_DFULL", 6, 55) -- prev
local CTRL_CW_DFULL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_DFULL", 6, 60)
-- Look-ahead distance along approach vector (m), used to compute u
-- local CTRL_CW_LOOK  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_LOOK", 7, 50) -- original
local CTRL_CW_LOOK  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "C_LOOK", 7, 20)

-- -----------------------------------------------------------------------
-- KF_ parameter table for Kalman filter noise tuning
-- -----------------------------------------------------------------------
local KF_TABLE_PREFIX = "KF_"
local KF_TABLE_KEY = nil
for key = 0, 200 do
    if param:add_table(key, KF_TABLE_PREFIX, 2) then
        KF_TABLE_KEY = key
        break
    end
end
assert(KF_TABLE_KEY ~= nil, "CTRL: no free KF param table key")

local KF_PROC_NOISE  = param_helpers.bind_add_param(KF_TABLE_KEY, KF_TABLE_PREFIX, "PROC_NOISE", 1, 0.1)  -- process noise (Q diagonal)
local KF_MEAS_NOISE  = param_helpers.bind_add_param(KF_TABLE_KEY, KF_TABLE_PREFIX, "MEAS_NOISE", 2, 5.0)  -- measurement noise (R diagonal)


-- =========================================================
-- 2. Bus Input 
-- =========================================================

local function ensure_kbus()
    if kbus_all_ready then return true end
    kbus_all_ready = kbus_seq_param:init("KBUS_SEQ")
                 and kbus_t_s_param:init("KBUS_T_S")
                 and kbus_lat_param:init("KBUS_LAT")
                 and kbus_lon_param:init("KBUS_LON")
                 and kbus_vn_param:init("KBUS_VN")
                 and kbus_ve_param:init("KBUS_VE")
    return kbus_all_ready
end

-- read bus target
local function read_bus_target()

    -- not all values are ready
    if not ensure_kbus() then
        return nil
    end

    -- get sequence
    local seq_1 = kbus_seq_param:get()
    if seq_1 == nil then
        return nil
    end

    -- if sequence is odd
    if (seq_1 % 2) ~= 0 then
        return nil
    end

    -- write values
    local t_s     = kbus_t_s_param:get()
    local lat_deg = kbus_lat_param:get()
    local lon_deg = kbus_lon_param:get()
    local vn      = kbus_vn_param:get()
    local ve      = kbus_ve_param:get()
    -- bin if any are nil
    if t_s == nil or lat_deg == nil or lon_deg == nil or vn == nil or ve == nil then
        return nil
    end

    local seq_2 = kbus_seq_param:get()
    if seq_2 == nil then
        return nil
    end

    -- checking consistency of sample
    if seq_1 ~= seq_2 or (seq_2 % 2) ~= 0 then
        return nil
    end

    -- if not a new sample
    if seq_2 == last_bus_seq_seen then
        return nil -- no new sample
    end

    -- testing if sample is stale
    local now_s = millis():toint() * 0.001
    if (now_s - t_s) > 1.0 then
        return nil -- stale sample
    end

    -- updating location
    local loc = Location()
    loc:lat(math_helpers.deg_to_e7(lat_deg))
    loc:lng(math_helpers.deg_to_e7(lon_deg))

    last_bus_seq_seen = seq_2

    return {
        loc = loc,
        vn = vn,
        ve = ve,
        seq = seq_2,
        timestamp_ms = math.floor(t_s * 1000.0 + 0.5)
    }
end

-- =========================================================
-- 3. Kalman Filter - State Prediction
-- =========================================================

-- predict_position removed — was only used in the Dubins build block
-- local function predict_position(kangaroo_state, t) ... end

-- update Kalman filter
local function update_kf(sample)
    -- first init
    if not kf_initialized then
        kf_ref_loc   = sample.loc:copy()
        last_kf_t_ms = sample.timestamp_ms
        kf.init(0, 0)
        kf_initialized = true
        return
    end
    -- time interval
    local dt_s = (sample.timestamp_ms - last_kf_t_ms) * 0.001

    -- guard against stale or reversed samples
    if dt_s <= 0 or dt_s > 10.0 then 
        return 
    end 

    -- convert lat/lon to NE metres relative to fixed origin
    local ne = kf_ref_loc:get_distance_NE(sample.loc)
    if ne == nil then 
        return 
    end

    -- measurement noise 
    kf.process_noise = KF_PROC_NOISE:get()
    kf.measurement_noise = KF_MEAS_NOISE:get()
    local result = kf.update(ne:x(), ne:y(), dt_s)

    if result then
        kf_state      = result
        last_kf_t_ms  = sample.timestamp_ms
    end
end


-- =========================================================
-- 4. Misc
-- =========================================================

-- get home altitude - shouldget it from home (0)
local function get_home_alt_m()
    if ahrs:home_is_set() then
        local home = ahrs:get_home()
        if home ~= nil and home:alt() ~= nil then
            return home:alt() * 0.01
        end
    end

    local pos = ahrs:get_position()
    if pos ~= nil then
        pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
        local pos_alt_cm = pos:alt()
        if pos_alt_cm ~= nil then
            return pos_alt_cm * 0.01
        end
    end

    return nil
end

-- get_point_accept_radius_m removed — waypoint reach detection no longer used
-- local function get_point_accept_radius_m(point, next_point) ... end

-- compute_track_er removed — L1/L2 error reporting no longer used
-- local function compute_track_er(pos_loc, target_loc) ... end


-- =========================================================
-- 5. Continuous Weave (delegated to continuous_weave.lua)
-- =========================================================
-- need to add stuff

-- =========================================================
-- 7. Update loop (main)
-- =========================================================

function update()
    local now_ms = millis():toint()

    -- absorb latest bus sample and run KF on every tick
    local sample = read_bus_target()
    if sample then
        kangaroo_loc_pending = sample
        update_kf(sample)
    end

    local pos = ahrs:get_position()

    -- periodic KF health report
    if kf_state and kf_ref_loc and kangaroo_loc_pending and (now_ms - last_kf_report_ms) >= REPORT_INTERVAL_MS then
        last_kf_report_ms = now_ms
        local kf_loc = kf_ref_loc:copy()
        kf_loc:offset(kf_state.x, kf_state.y)
        local err_ne = kf_loc:get_distance_NE(kangaroo_loc_pending.loc)
        if err_ne then
            local err_dist = math.sqrt(err_ne:x()^2 + err_ne:y()^2)
            gcs:send_text(4, string.format("KF err: %.1fm vel N%.2f E%.2f m/s",
                err_dist, kf_state.vx, kf_state.vy))
        end
    end

    -- mode gate: only command in AUTO or GUIDED
    local current_mode = vehicle:get_mode()
    if current_mode ~= MODE_AUTO and current_mode ~= MODE_GUIDED then
        return update, 100
    end

    -- wait for KF to produce a state estimate and valid position
    if kf_state == nil or kf_ref_loc == nil or pos == nil then
        return update, 100
    end

    -- plane and kangaroo NE positions relative to KF origin
    local plane_ne = kf_ref_loc:get_distance_NE(pos)
    if plane_ne == nil then return update, 100 end
    local plane_n = plane_ne:x()
    local plane_e = plane_ne:y()
    local kang_n  = kf_state.x
    local kang_e  = kf_state.y

    -- distance to kangaroo and look-ahead fraction
    local dx      = kang_n - plane_n
    local dy      = kang_e - plane_e
    local dist    = math.sqrt(dx*dx + dy*dy)
    local u       = dist > 0 and math.min(CTRL_CW_LOOK:get() / dist, 1.0) or 0

    -- advance arc-length accumulator by distance flown this tick
    if last_cw_ms == nil then last_cw_ms = now_ms end
    local dt_s     = (now_ms - last_cw_ms) * 0.001
    last_cw_ms     = now_ms
    local vel      = ahrs:get_velocity_NED()
    local speed_ms = vel and math.sqrt(vel:x()^2 + vel:y()^2) or 0
    cw_s = cw_s + speed_ms * dt_s

    -- generate next reference point from continuous_weave module
    local ref_n, ref_e = continuous_weave.straight_weave(plane_n, plane_e,kang_n,  kang_e,u, cw_s,
        CTRL_CW_LAMBDA:get(), CTRL_CW_RMIN:get(), CTRL_CW_ACAP:get(), CTRL_CW_DSTART:get(), CTRL_CW_DFULL:get(), CTRL_CW_ETA:get(), cw_phase)

    -- command the reference point at the required altitude
    local home_alt_m = get_home_alt_m()
    if home_alt_m == nil then
        gcs:send_text(4, "CW: no home alt, skipping command")
        return update, 100
    end

    local ref_loc = kf_ref_loc:copy()
    ref_loc:offset(ref_n, ref_e)
    ref_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
    ref_loc:set_alt_m(home_alt_m + PLANE_ABOVE_TARGET_M, ALT_FRAME_ABSOLUTE)

    if not vehicle:set_target_location(ref_loc) then
        gcs:send_text(4, "CW: set_target_location failed")
    end

    -- periodic status report
    if (now_ms - last_report_ms) >= REPORT_INTERVAL_MS then
        gcs:send_text(4, string.format("CW: s=%.0fm dist=%.0fm spd=%.1fm/s",
            cw_s, dist, speed_ms))
        last_report_ms = now_ms
    end

    return update, 100
end

-- updated wrapper for logging
local function protected_update()
    local ok, err = pcall(update)
    if not ok then
        gcs:send_text(3, "Control: " .. tostring(err))
        return protected_update, 1000
    end
    return protected_update, 100
end

return protected_update()
