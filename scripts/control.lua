-- =========================================================
-- Control function
-- =========================================================

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- local variable states
local dubins_point_index = nil
local dubins_point_count = nil
local kangaroo_loc_pending = nil
local kangaroo_loc_active = nil
local dubins_points_active = nil
local REPORT_INTERVAL_MS = 2000
local last_report_ms = 0

-- orbiting state variables
local orbit_active    = false
local orbit_points    = nil   -- absolute waypoint list (approach + arc)
local orbit_index     = 1
local orbit_count     = 0
local orbit_rho       = nil   -- R_min at last build
local orbit_dir_active = 0    -- +1 or -1
local orbit_psi_cur   = nil   -- current arc angle on orbit circle (for replenishment)
local last_orbit_build_ms = 0

-- maintain aircraft 150m above virtual target altitude
local PLANE_ABOVE_TARGET_M = 150.0

-- KBUS_ bus params (owned by kangaroo_MAV.lua)
-- bounded loosely because control.lua will boot first (alpha betical)
local kbus_seq_param = Parameter()
local kbus_t_s_param = Parameter()
local kbus_lat_param = Parameter()
local kbus_lon_param = Parameter()
local kbus_vn_param  = Parameter()
local kbus_ve_param  = Parameter()
local kbus_all_ready = false

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

--

-- refresh for the bus
local last_bus_seq_seen = 0

-- import modules
local dubins_points = require("dubins_weave_full")
local math_helpers = require("math_helpers")
local param_helpers = require("param_helpers")
local kf = require("state_estimator")

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
    if param:add_table(key, CTRL_TABLE_PREFIX, 16) then
        CTRL_TABLE_KEY = key
        break
    end
end
assert(CTRL_TABLE_KEY ~= nil, "CTRL: no free param table key")

-- -- -----------------------------------------------------------------------
-- -- Paramater value declaration for control 
-- -- -----------------------------------------------------------------------
-- Dubins path rebuild interval (ms)
local CTRL_REBUILD_MS = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "REBUILD_MS", 1, 6000)
-- local CTRL_REBUILD_MS = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "REBUILD_MS", 1, 2000)
-- Waypoint acceptance radius (m). Range: 5–100
local CTRL_WP_RAD = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "WP_RAD", 2, 40)
-- Minimum waypoint acceptance radius (m). Range: 1–50
local CTRL_MIN_WP = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "MIN_WP", 3, 20)
-- Consecutive position samples inside radius before waypoint is marked reached
local CTRL_STREAK = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "STREAK", 4, 1)
-- local CTRL_STREAK = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "STREAK", 4, 2)
-- Minimum distance that the follower is travelling at for the dubins controller to activate
--- this requires tuning... (750 m in the radisu)
local CTRL_DUBINS_ON_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_DIST", 5, 500)
--local CTRL_DUBINS_ON_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_DIST", 5, 750)

-- local CTRL_DUBINS_ON_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_DIST", 5, 400)
-- Velocity that the follower vehicle is travelling at for the dubins controller to activate
local CTRL_DUBINS_ON_VEL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_VEL", 6, 30)
-- local CTRL_DUBINS_ON_VEL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_VEL", 6, 15)
-- additional swapping control 3 May
-- Minimum improvement in final-point distance (m) before swapping to a new Dubins path, set to 50 m
local CTRL_SWAP_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "SWAP_DIST", 7, 50)
-- Cooldown (ms) after a swap before another swap is allowed
local CTRL_SWAP_COOL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "SWAP_COOL", 8, 1000)
-- Cost function weights (w1–w4); heading terms in rad^2, distance terms in m^2
local CTRL_W_HDG_KANG = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "W_HDG_KANG",  9,  0.4)  -- w1: kangaroo heading alignment
local CTRL_W_HDG_CHG  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "W_HDG_CHG",  10,  0.2)  -- w2: change in bearing to kangaroo
local CTRL_W_DIST_PLN = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "W_DIST_PLN", 11,  0.2)  -- w3: plane travel to next waypoint
local CTRL_W_DIST_KNG = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "W_DIST_KNG", 12,  0.2)  -- w4: next waypoint proximity to predicted kangaroo
-- additional control function for orbiting behaviour
local CTRL_ORBIT_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "ORBIT_DIST", 13, 200)  -- Distance for orbiting behaviour to kick in
local CTRL_ORBIT_DIR  = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "ORBIT_DIR",  14,   0)  -- -1, Counter CW, 0 default, 1 CW


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


-- get home altitude 
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

-- fly to point - point.loc is absolute from dubins_weave
function fly_to_dubins_point(point)
    if point == nil or point.loc == nil then
        return false
    end

    local home_alt_m = get_home_alt_m()
    if home_alt_m == nil then
        return false
    end

    local target_loc = point.loc:copy()
    target_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
    -- setting altitude to 150 
    local target_alt_m = home_alt_m + PLANE_ABOVE_TARGET_M
    target_loc:set_alt_m(target_alt_m, ALT_FRAME_ABSOLUTE)
    return vehicle:set_target_location(target_loc)
end

-- Error function  - flags for target reach
local reached_streak = 0
local reached_index = -1

-- acceptance radius of the point being reached
local function get_point_accept_radius_m(point, next_point)
    local accept_radius_m = CTRL_WP_RAD:get()
    -- nil case
    if point == nil or point.loc == nil or next_point == nil or next_point.loc == nil then
        return accept_radius_m
    end
    local spacing_m = point.loc:get_distance(next_point.loc)
    if spacing_m == nil or spacing_m <= 0 then
        return accept_radius_m
    end
    -- never let radius exceed 45% of spacing (avoid double-counting)
    -- but never go below CTRL_MIN_WP
    local max_radius = spacing_m * 0.45
    return math.max(CTRL_MIN_WP:get(), math.min(accept_radius_m, max_radius))
end



-- if the point has been reached.
function dubins_point_reached(point, next_point)
    -- if no points, return a false
    if point == nil then
        reached_streak = 0
        return false
    end

    -- i.e. a new point, resetting the streak
    if reached_index ~= dubins_point_index then
        reached_index = dubins_point_index
        reached_streak = 0
    end

    local pos = ahrs:get_position()
    -- nil case
    if pos == nil or point.loc == nil then
        reached_streak = 0
        return false
    end

    -- horizontal distance
    local dist_m = pos:get_distance_NE(point.loc)

    -- take euclidean distance if not nil
    if dist_m ~= nil then
        dist_m = math.sqrt(dist_m:x() * dist_m:x() + dist_m:y() * dist_m:y())
    end

    -- if the location is nil
    if dist_m == nil then
        reached_streak = 0
        return false
    end

    local accept_radius_m = get_point_accept_radius_m(point, next_point)

    -- if distance is less 
    if dist_m <= accept_radius_m then
        reached_streak = reached_streak + 1
        return reached_streak >= math.floor(CTRL_STREAK:get())
    end
    -- reseting reached streak, ending loop in control loop
    reached_streak = 0
    return false
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

-- to avoid a rebulid storm... for small arcs generated
local MIN_DUBINS_POINTS = 5

-- to calculate the optimal next point
local dubins_points_new = nil
local dubins_new_info   = nil
local kangaroo_loc_new  = nil

-- function to update build
local function update_build(kangaroo_loc_target)

    -- generate build
    local build_result, build_info = dubins_points.build_path(kangaroo_loc_target)

    -- reject short paths
    if build_result ~= nil and #build_result < MIN_DUBINS_POINTS then
        gcs:send_text(4, string.format("Dubins build rejected: only %d points", #build_result))
        build_result = nil
        build_info = "degenerate_path"
    end

    if build_result == nil then
        if build_info ~= nil then
            gcs:send_text(6, "Dubins build failed: " .. tostring(build_info))
        end
        return false, build_info
    end

    -- stage the new path; commit_dubins_path() decides when to apply it
    dubins_points_new = build_result
    dubins_new_info   = build_info
    kangaroo_loc_new  = kangaroo_loc_target
    return true, build_info
end

local function get_path_final_loc(path)
    if path == nil or #path == 0 then return nil end
    local last = path[#path]
    return last and last.loc or nil
end

-- Error Variables and cost function intilisation
local cum_L1_error = 0.0
local cum_L2_error = 0.0
local error_samples = 0
local min_J_seen = math.huge

-- predictive step - pull from state machine data
local function predict_position(kangaroo_state, t)
    -- predict kangaroo values
    local x = kangaroo_state.x + kangaroo_state.vx * t
    local y = kangaroo_state.y + kangaroo_state.vy * t
    local heading = math.atan(kangaroo_state.vy, kangaroo_state.vx)
    return x, y, heading
end

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
    kf.process_noise     = KF_PROC_NOISE:get()
    kf.measurement_noise = KF_MEAS_NOISE:get()
    local result = kf.update(ne:x(), ne:y(), dt_s)
    if result then
        kf_state      = result
        last_kf_t_ms  = sample.timestamp_ms
    end
end

-- period (T) may require extension to match flight time
-- in the event that the plane is further away than one rebuild
local function compute_lookahead_s(plane_loc, wp_loc)
    -- rebuild interval is the minimum horizon
    local floor_s = CTRL_REBUILD_MS:get() * 0.001

    -- initisalising case
    if plane_loc == nil or wp_loc == nil then 
        return floor_s 
    end

    -- actual velocity
    local vel = ahrs:get_velocity_NED()
    local speed = vel and math.sqrt(vel:x()^2 + vel:y()^2) or 0
    if speed < 5.0 then 
        return floor_s 
    end

    -- location from waypoint
    local dn = plane_loc:get_distance_NE(wp_loc)
    if dn == nil then return floor_s end
    local dist = math.sqrt(dn:x()^2 + dn:y()^2)

    -- T >= rebuild interval; extend if flight time to wp is longer
    return math.max(floor_s, dist / speed)
end

-- cost function
-- next_loc: next plane waypoint A(k+1)
-- plane_loc: current plane position A(k); kangaroo_loc: current kangaroo position B(k)
-- kangaroo_state: KF output {x,y,vx,vy}; lookahead_s: prediction horizon (s)

local function cost_function(next_loc, plane_loc, kangaroo_loc, kangaroo_state, lookahead_s)
    if not next_loc or not plane_loc or not kangaroo_loc or not kangaroo_state then
        return math.huge
    end
    if kangaroo_state.vx == nil or kangaroo_state.vy == nil then return math.huge end

    -- B(k+1): predict kangaroo ahead by lookahead_s
    local pred_loc = kangaroo_loc:copy()
    pred_loc:offset(kangaroo_state.vx * lookahead_s, kangaroo_state.vy * lookahead_s)

    -- dn1: next_loc → pred_loc  (psi_{A(k+1),B(k+1)} and term 4)
    local dn1 = next_loc:get_distance_NE(pred_loc)
    if not dn1 then return math.huge end

    -- dn2: plane_loc → next_loc  (term 3)
    local dn2 = plane_loc:get_distance_NE(next_loc)
    if not dn2 then return math.huge end

    -- dn3: plane_loc → kangaroo_loc  (psi_{A(k),B(k)})
    local dn3 = plane_loc:get_distance_NE(kangaroo_loc)
    if not dn3 then return math.huge end

    local psi_B            = math.atan(kangaroo_state.vy, kangaroo_state.vx)
    local psi_next_to_kang = math.atan(dn1:y(), dn1:x())  -- psi_{A(k+1),B(k+1)}
    local psi_curr_to_kang = math.atan(dn3:y(), dn3:x())  -- psi_{A(k),B(k)}

    -- normalise distance terms by DUB_DIST² so all four terms are dimensionless and
    -- comparable to the heading terms (rad²); w3 needs larger values than w4 because
    -- waypoint spacing << DUB_DIST, so (spacing/DUB_DIST)² is small
    local dub_dist = CTRL_DUBINS_ON_DIST:get()
    local ref_sq   = dub_dist * dub_dist

    -- local J = CTRL_W_HDG_KANG:get() * math_helpers.wrap_pi(psi_B - psi_next_to_kang)^2
    --         + CTRL_W_HDG_CHG:get()  * math_helpers.wrap_pi(psi_next_to_kang - psi_curr_to_kang)^2
    --         + CTRL_W_DIST_PLN:get() * (dn2:x()^2 + dn2:y()^2)
    --         + CTRL_W_DIST_KNG:get() * (dn1:x()^2 + dn1:y()^2)
    local J = CTRL_W_HDG_KANG:get() * math_helpers.wrap_pi(psi_B - psi_next_to_kang)^2
            + CTRL_W_HDG_CHG:get()  * math_helpers.wrap_pi(psi_next_to_kang - psi_curr_to_kang)^2
            + CTRL_W_DIST_PLN:get() * (dn2:x()^2 + dn2:y()^2) / ref_sq
            + CTRL_W_DIST_KNG:get() * (dn1:x()^2 + dn1:y()^2) / ref_sq

    return J
end


-- select optimal route — compares next waypoint of active path vs next waypoint of new path
local function should_swap_dubin_path(plane_loc, plane_heading, kangaroo_loc, kangaroo_state, lookahead_s)
    -- maintaining state machine
    if not kangaroo_loc or not kangaroo_state then
        return false
    end
    if dubins_points_active == nil or dubins_point_index == nil then
        return false
    end
    if dubins_points_new == nil then
        return false
    end

    -- next waypoint of the active path (what the plane is currently flying toward)
    local active_next = dubins_points_active[dubins_point_index]

    -- next waypoint of the newly generated path - index starts at 1
    local new_next = dubins_points_new[1]

    if not active_next or not active_next.loc or not new_next or not new_next.loc then
        return false
    end

    -- the current J of the active path next point
    local J_active = cost_function(active_next.loc, plane_loc, kangaroo_loc, kangaroo_state, lookahead_s)

    -- newly updated J of the new path next point
    local J_new    = cost_function(new_next.loc, plane_loc, kangaroo_loc, kangaroo_state, lookahead_s)

    if J_new < J_active then
        return true, J_active, J_new
    end

    return false, J_active, J_new
end

local function commit_dubins_path()
    dubins_points_active = dubins_points_new
    dubins_point_index   = 1
    dubins_point_count   = #dubins_points_active
    kangaroo_loc_active  = kangaroo_loc_new
    dubins_points_new    = nil
    dubins_new_info      = nil
    kangaroo_loc_new     = nil
end


-- build orbit from the current state
local function build_orbit_path_from_state()
    -- parameters
    local orbit_dir = CTRL_ORBIT_DIR:get()
    local abs_points, info = dubins_points.build_orbit_path(kangaroo_loc_pending, orbit_dir)

    if abs_points == nil then
        return nil
    end

    if abs_points ~= nil then
        -- set module-level orbit state variables
        orbit_points     = abs_points
        orbit_index      = 1
        orbit_count      = #orbit_points
        orbit_rho        = info.rho
        orbit_dir_active = info.dir
        orbit_psi_cur    = info.psi_arc_end
        --rho=info.rho, orbit_dir_active=info.dir, orbit_psi_cur=info.psi_arc_end
        -- Logs via gcs:send_text
        gcs:send_text(4, string.format("Orbit build: %d pts dir=%d rho=%.0fm", #orbit_points, info.dir, info.rho))
        return true
    end
end

-- add comements - 20 May
-- append orbit arc to the orbit path (i.e. curve onto centre)
local function append_orbit_arc(center_loc, psi_start, n_steps)
    local sign = orbit_dir_active  -- +1 CW, -1 CCW
    local delta = math.rad(15)
    local home_alt_m = get_home_alt_m()
    for i = 1, n_steps do
        local psi = psi_start + sign * i * delta
        local east_m  = orbit_rho * math.sin(psi)
        local north_m = orbit_rho * math.cos(psi)
        local wp = center_loc:copy()
        wp:offset(north_m, east_m)
        wp:change_alt_frame(ALT_FRAME_ABSOLUTE)
        if home_alt_m then wp:set_alt_m(home_alt_m + PLANE_ABOVE_TARGET_M, ALT_FRAME_ABSOLUTE) end
        orbit_points[#orbit_points + 1] = {loc = wp, psi = psi}
        orbit_count = orbit_count + 1
    end
    orbit_psi_cur = psi_start + sign * n_steps * delta
end

--- error function for comparison of best run - L1 and L2 distance
local function compute_track_er(pos_loc, target_loc)
    -- distance from target
    local dn = pos_loc:get_distance_NE(target_loc)
    -- return nil if the distance
    if dn == nil then 
        return nil 
    end
    local dx, dy = dn:x(), dn:y()
    local L1_error = math.abs(dx) + math.abs(dy)
    local L2_error = math.sqrt( dx * dx + dy * dy)
    return L1_error, L2_error
end

-- function to trigger the weaving functionality - else just follow per guided/auto
-- currentyl impemented as a state machine betewen idle and active, build is occuring every interval
local function engage_control_state(current_mode, dubins_point_index, pos_loc, target_loc)
    -- nil guards
    if pos_loc == nil or target_loc == nil then
        return nil
    end
    
    -- parameters
    local min_distance = CTRL_DUBINS_ON_DIST:get()
    local min_speed = CTRL_DUBINS_ON_VEL:get()
    local orbit_distance = CTRL_ORBIT_DIST:get()

    -- distance
    local dn = pos_loc:get_distance_NE(target_loc)
    local dist_m = dn and math.sqrt(dn:x()*dn:x() + dn:y()*dn:y()) or math.huge
    -- relative velocity between plane and kangaroo (closing rate)
    local plane_velocity = ahrs:get_velocity_NED()
    -- old: absolute plane speed — always ~20-30 m/s, so speed_ms <= min_speed was never true
    ---local speed_ms = plane_velocity and math.sqrt(plane_velocity:x()*plane_velocity:x() + plane_velocity:y()*plane_velocity:y()) or math.huge
    local rel_speed_ms = math.huge
    if plane_velocity and kf_state then
        local rel_vn = plane_velocity:x() - kf_state.vx
        local rel_ve = plane_velocity:y() - kf_state.vy
        rel_speed_ms = math.sqrt(rel_vn*rel_vn + rel_ve*rel_ve)
    end

    -- handle negative case - IDLE
    if current_mode ~= MODE_AUTO and current_mode ~= MODE_GUIDED then
        return {
        idle   = true,
        build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
        active = false,
        orbit = false
        }
    end

    -- trigger: close enough AND nearly caught up (low closing rate relative to kangaroo)
    local dubins_trigger_met = dist_m <= min_distance and rel_speed_ms <= min_speed
    -- old trigger (before 7 May)
    ---local trigger_met = dist_m <= min_distance or speed_ms <= min_speed
    -- absolute speed trigger (replaced 14 May — plane cruises at 20-30 m/s so condition was never true)
    ---local trigger_met = dist_m <= min_distance and speed_ms <= min_speed

    -- orbit is a sub-case of dubins (same conditions + within orbit_distance); check FIRST so it isn't shadowed
    local orbit_trigger_met = dubins_trigger_met and dist_m <= orbit_distance

    if orbit_trigger_met then
        return {
            idle   = false,
            build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
            active = false,
            orbit  = true
        }
    end

    if dubins_trigger_met then
        return {
        idle   = false,
        build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
        active = dubins_points_active ~= nil and dubins_point_index ~= nil,
        orbit  = false
        }
    end

    -- else case
    return {
        idle   = false,
        build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
        active = false,
        orbit  = false
    }

end


-- -----------------------------------------------------------------------
-- Revised update() — rebuild Dubins path every second from latest bus target,
-- walk through points as they are reached.

-- Three target states for state machine
-- 1. kanagroo_loc_latest, the newest bus sample
-- 2. kangaroo_loc_pending, the allocated target to build from
-- 3. kangaroo_loc_active, the target currently being optmised through weaving behvaiour
-- 4. 17 May - need to add radius bound (circling)
-- -----------------------------------------------------------------------
local last_rebuild_ms     = 0
local last_swap_ms        = 0

function update()
    local current_mode = vehicle:get_mode()

    local now_ms = millis():toint()

    -- always absorb the latest bus sample; track if KF ran this tick
    local kf_ran = false
    local kangaroo_loc_latest = read_bus_target()
    -- target
    if kangaroo_loc_latest then
        kangaroo_loc_pending = kangaroo_loc_latest
        -- update the filter with the latest position
        update_kf(kangaroo_loc_latest)
        kf_ran = true
    end
   
    local pos = ahrs:get_position()

    -- establishing the state
    local state = engage_control_state(current_mode, dubins_point_index, pos, kangaroo_loc_pending and kangaroo_loc_pending.loc)

    -- handling nil case
    if state == nil then 
        return update, 100 
    end

    -- handling idle case (i.e. condition isn't met)
    if state.idle then
        return update, 100
    end

    -- rebuild every second from the latest known target
    if state.build then
        -- handling timing errors
        if not kf_ran and (now_ms - last_rebuild_ms) >= CTRL_REBUILD_MS:get() then
            last_rebuild_ms = now_ms
            --local success, build_info = update_build(kangaroo_loc_pending)
            -- predicting based on future target
            local build_target = kangaroo_loc_pending

            -- if the kalman filter has a state response, build the target
            -- adjust this 
            if kf_state then
                local T = CTRL_REBUILD_MS:get() * 0.001
                -- local pred_loc = kangaroo_loc_pending.loc:copy()
                -- pred_loc:offset(kf_state.vx * T, kf_state.vy * T)
                local px, py, _ = predict_position(kf_state, T)
                local pred_loc = kf_ref_loc:copy()
                pred_loc:offset(px, py)
                build_target = {loc = pred_loc, vn = kf_state.vx, ve = kf_state.vy,
                seq = kangaroo_loc_pending.seq, timestamp_ms = kangaroo_loc_pending.timestamp_ms
                }
            end

            local success, build_info = update_build(build_target)
            -- if it builds successfully
            if success then
                if dubins_points_active == nil then
                    -- no active path yet, commit immediately
                    local n_pts = #dubins_points_new
                    commit_dubins_path()
                    last_swap_ms = now_ms
                    gcs:send_text(4, string.format("Dubins initial build: %d pts type=%s rho=%.0fm",
                        n_pts,
                        (type(build_info) == "table" and build_info.path_type) or "?",
                        (type(build_info) == "table" and build_info.rho_m) or 0))
                else
                    local active_final = get_path_final_loc(dubins_points_active)
                    local new_final    = get_path_final_loc(dubins_points_new)
                    local kang_loc     = kangaroo_loc_pending and kangaroo_loc_pending.loc                    
                    -- update w/ cost function
                    local plane_heading = ahrs:get_yaw_rad() or 0
                    local next_wp = dubins_points_active[dubins_point_index]
                    local lookahead_s = compute_lookahead_s(pos, next_wp and next_wp.loc)

                    local swap, J_active, J_new = should_swap_dubin_path(pos, plane_heading, kang_loc, kf_state, lookahead_s)
                    if J_active and J_active < math.huge then
                        min_J_seen = math.min(min_J_seen, J_active)
                        gcs:send_text(4, string.format("Dubins J:%.4f", J_active))
                    end
                    --manage k state
                    if kf_state == nil then
                        swap = false
                    end

                    -- cooldown - hystersisi 
                    local cooldown_elapsed = (now_ms - last_swap_ms) >= CTRL_SWAP_COOL:get()
                    -- guarding from swapping too quickly...
                    if swap and cooldown_elapsed then
                        local n_pts = #dubins_points_new
                        commit_dubins_path()
                        last_swap_ms = now_ms
                        gcs:send_text(4, string.format("Dubins swapped: %d pts J_new=%.3f J_act=%.3f type=%s rho=%.0fm",
                            n_pts, J_new, J_active,
                            (type(build_info) == "table" and build_info.path_type) or "?",
                            (type(build_info) == "table" and build_info.rho_m) or 0))
                    end
                end
            end
        end
    end

    -- fly the active path
    if state.active then
        --- Establish current point and next point
        local point = dubins_points_active[dubins_point_index]
        local next_point = dubins_points_active[dubins_point_index + 1]
       
        -- second position pull for reporting
        local pos2 = ahrs:get_position()

        if pos2 and point and point.loc then
            local l1, l2 = compute_track_er(pos2, point.loc)
            if l1 then
                cum_L1_error = cum_L1_error + l1
                cum_L2_error = cum_L2_error + l2
                error_samples = error_samples + 1
            end
        end

        -- periodic distance report
        if dubins_point_count ~= nil and (now_ms - last_report_ms) >= REPORT_INTERVAL_MS then

            local dist_2d = -1

            -- reporting block
            if pos2 and point then
                local dn = pos2:get_distance_NE(point.loc)
                if dn then 
                    dist_2d = math.sqrt(dn:x()*dn:x() + dn:y()*dn:y()) 
                end
            end

            gcs:send_text(4, string.format("Dubins point %d/%d dist=%.1fm",
                dubins_point_index, dubins_point_count, dist_2d))
            local n = math.max(error_samples, 1)
            local j_report = min_J_seen < math.huge and min_J_seen or -1
            gcs:send_text(4, string.format("Dubins error L1 Norm:%.1fm L2 Norm:%.1fm J_min:%.4f",
                cum_L1_error / n, cum_L2_error / n, j_report))
            last_report_ms = now_ms
        end

        -- fly to point
        if point and fly_to_dubins_point(point) then
            if dubins_point_reached(point, next_point) then
                gcs:send_text(4, string.format("Reached Dubins point %d/%d",
                    dubins_point_index, dubins_point_count or 0))
                dubins_point_index = dubins_point_index + 1
            end

        -- debugging - for no target case
        elseif point then
            gcs:send_text(4, "Failed to set target for Dubins point")
        end

        -- clamp at end of path — next rebuild will reset the index
        if dubins_point_count ~= nil and dubins_point_index > dubins_point_count then
            dubins_point_index = dubins_point_count
        end
    --
    -- fly the orbit path
    -- comment up 20 May
    if state.orbit then
        -- Initial build / rebuild if not yet active
        if not orbit_active or orbit_points == nil then
            build_orbit_path_from_state()
            orbit_active = true
        end

        local point      = orbit_points[orbit_index]
        local next_point = orbit_points[orbit_index + 1]

        -- Replenish arc when within 5 waypoints of end
        if orbit_index >= orbit_count - 5 then
            local kang_loc = kangaroo_loc_pending and kangaroo_loc_pending.loc
            if kang_loc then
                append_orbit_arc(kang_loc, orbit_psi_cur, 12)  -- ~180° more arc
            end
        end

        -- Fly and advance (same logic as Dubins active block)
        if point and fly_to_dubins_point(point) then
            if dubins_point_reached(point, next_point) then
                gcs:send_text(4, string.format("Orbit wp %d/%d", orbit_index, orbit_count))
                orbit_index = orbit_index + 1
            end
        end

        if orbit_index > orbit_count then orbit_index = orbit_count end
        return update, 100
    end

    -- Reset orbit state if we leave orbit trigger zone
    if not state.orbit then
        orbit_active = false
        orbit_points = nil
    end
        -- else fly direct to the kangaroo if the velocity or distance metric isn't met
    else
        -- following the same structure to direct it to the direct location
        if kangaroo_loc_pending and kangaroo_loc_pending.loc then
            local home_alt_m = get_home_alt_m()
            if home_alt_m then
                local direct_loc = kangaroo_loc_pending.loc:copy()
                direct_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
                direct_loc:set_alt_m(home_alt_m + PLANE_ABOVE_TARGET_M, ALT_FRAME_ABSOLUTE)
                vehicle:set_target_location(direct_loc)
            end
        end

    end

    return update, 1000
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
