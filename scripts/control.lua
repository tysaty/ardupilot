-- establish a control loop - state handling

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- local variable states
local controller_busy = nil
local dubins_point_index = nil
local dubins_point_count = nil
local kangaroo_loc_pending = nil
local kangaroo_loc_active = nil
local dubins_points_active = nil
local REPORT_INTERVAL_MS = 2000
local last_report_ms = 0

-- maintain aircraft 150m above virtual target altitude
local PLANE_ABOVE_TARGET_M = 150.0
local KANG_ALT_M_FALLBACK = 80.0


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
local kang_alt_m_param = Parameter()
local kang_alt_m_ready = kang_alt_m_param:init("KANG_ALT_M")

-- refresh for the bus
local last_bus_seq_seen = 0

-- import modules
local dubins_points = require("dubins_weave_full")
local math_helpers = require("math_helpers")
local param_helpers = require("param_helpers")

-- boot message
gcs:send_text(4, "Control: loaded at boot")

-- ---------------------------------------------------------
-- Parameter tabel for control variables
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

-- -- -----------------------------------------------------------------------
-- -- Paramater value declaration for control algorithm
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
local CTRL_DUBINS_ON_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_DIST", 5, 750)
-- local CTRL_DUBINS_ON_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_DIST", 5, 400)
-- Velocity that the follower vehicle is travelling at for the dubins controller to activate
local CTRL_DUBINS_ON_VEL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_VEL", 6, 30)
-- local CTRL_DUBINS_ON_VEL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "DUB_VEL", 6, 15)
-- additional swapping control 3 May
-- Minimum improvement in final-point distance (m) before swapping to a new Dubins path, set to 50 m
local CTRL_SWAP_DIST = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "SWAP_DIST", 7, 50)
-- Cooldown (ms) after a swap before another swap is allowed
local CTRL_SWAP_COOL = param_helpers.bind_add_param(CTRL_TABLE_KEY, CTRL_TABLE_PREFIX, "SWAP_COOL", 8, 1000)


-- -- -----------------------------------------------------------------------
-- -- MAVProxy map visualisation: broadcast each Dubins waypoint as an
-- -- ADSB_VEHICLE so they appear as dots on the MAVProxy map.
-- -- ICAO addresses 0xDB0001..0xDBFFFF are reserved for path points.
-- -- -----------------------------------------------------------------------
-- mavlink:init(1, 0)

-- local DUBINS_VIS_ICAO_BASE   = 0xDB0001
-- local DUBINS_VIS_INTERVAL_MS = 2000     -- re-send before MAVProxy times contacts out
-- local DUBINS_VIS_STRIDE      = 3        -- send every Nth point to reduce clutter
-- local last_vis_ms            = 0

-- local ADSB_VIS_FLAGS = 1 + 2 + 64      -- VALID_COORDS + VALID_ALTITUDE + SIMULATED
--                                         -- no VALID_CALLSIGN (16) = no label shown

-- local function send_vis_point(icao, lat_deg, lng_deg, alt_m)
--     local cs = string.sub("\0\0\0\0\0\0\0\0\0", 1, 9)  -- empty callsign
--     local payload = string.pack("<I4i4i4i4 I2I2i2I2I2 Bc9BB",
--         icao,
--         math.floor(lat_deg * 1e7),
--         math.floor(lng_deg * 1e7),
--         math.floor(alt_m   * 1000),
--         0, 0, 0,            -- heading, hor_velocity, ver_velocity
--         ADSB_VIS_FLAGS,
--         0,                  -- squawk
--         0,                  -- altitude_type
--         cs,
--         0,                  -- emitter_type
--         0)                  -- tslc
--     for chan = 0, 5 do
--         mavlink:send_chan(chan, 246, payload)
--     end
-- end

-- setting altitude (not captured in the )
local function get_kangaroo_alt_m()
    if not kang_alt_m_ready then
        kang_alt_m_ready = kang_alt_m_param:init("KANG_ALT_M")
    end
    if kang_alt_m_ready then
        local alt_m = kang_alt_m_param:get()
        if alt_m ~= nil then
            return alt_m
        end
    end
    return KANG_ALT_M_FALLBACK
end

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

-- local function broadcast_dubins_vis()
--     if dubins_points_active == nil then 
--         return 
--     end

--     local home_alt = get_home_alt_m()
--     local alt_m = (home_alt or 0) + get_kangaroo_alt_m() + PLANE_ABOVE_TARGET_M

--     local icao = DUBINS_VIS_ICAO_BASE
--     for i = 1, #dubins_points_active, DUBINS_VIS_STRIDE do
--         local pt = dubins_points_active[i]
--         if pt and pt.loc then
--             send_vis_point(
--                 icao,
--                 pt.loc:lat() * 1.0e-7,
--                 pt.loc:lng() * 1.0e-7,
--                 alt_m)
--             icao = icao + 1
--         end
--     end
-- end

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
    local target_alt_m = home_alt_m + get_kangaroo_alt_m() + PLANE_ABOVE_TARGET_M
    target_loc:set_alt_m(target_alt_m, ALT_FRAME_ABSOLUTE)

    return vehicle:set_target_location(target_loc)
end


-- Error function 
-- flags for target reach
local reached_streak = 0
local reached_index = -1
-- error variables
local cum_L1_error = 0.0
local cum_L2_error = 0.0
local error_samples = 0

-- local function get_point_accept_radius_m(point, next_point)
    
local function get_point_accept_radius_m(point, next_point)
    local accept_radius_m = CTRL_WP_RAD:get()
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



-- if the point has been reached...
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

    -- checking distance to the active Dubins point location
    --local dist_m = pos:get_distance(point.loc)

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

-- ready bus target
local function read_bus_target()

    -- not all values are ready
    if not ensure_kbus() then
        return nil
    end

    local seq_1 = kbus_seq_param:get()
    if seq_1 == nil then
        return nil
    end

    -- if sequence is odd
    if (seq_1 % 2) ~= 0 then
        return nil
    end

    -- writing values
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

-- to avoid a rebulid storm...
local MIN_DUBINS_POINTS = 5

-- staged (not yet committed) Dubins build
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

    -- -- reset counts (moved to commit_dubins_path)
    -- dubins_points_active = build_result
    -- dubins_point_index = 1
    -- dubins_point_count = (dubins_points_active ~= nil) and #dubins_points_active or nil
    -- controller_busy = (dubins_points_active ~= nil and #dubins_points_active > 0)

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

-- Swap to new Dubins path if its final point is closer to the kangaroo
local function should_swap_dubins_path(kangaroo_loc, active_final_loc, new_final_loc, margin_m)
    if not kangaroo_loc or not active_final_loc or not new_final_loc then
        return false
    end

    margin_m = margin_m or 0

    local active_dist = kangaroo_loc:get_distance(active_final_loc)
    local new_dist    = kangaroo_loc:get_distance(new_final_loc)

    -- swap only if new path is closer by at least margin_m
    if new_dist + margin_m < active_dist then
        return true, active_dist, new_dist
    end

    return false, active_dist, new_dist
end

local function commit_dubins_path()
    dubins_points_active = dubins_points_new
    dubins_point_index   = 1
    dubins_point_count   = #dubins_points_active
    controller_busy      = true
    kangaroo_loc_active  = kangaroo_loc_new
    dubins_points_new    = nil
    dubins_new_info      = nil
    kangaroo_loc_new     = nil
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
local function engage_control_state(current_mode, dubins_point_active, dubins_point_index, pos_loc, target_loc)
    
    -- nil guards
    if pos_loc == nil or target_loc == nil then
        return nil
    end
    
    -- parameters
    local min_distance = CTRL_DUBINS_ON_DIST:get()
    local min_speed = CTRL_DUBINS_ON_VEL:get()
    -- distance
    local dn = pos_loc:get_distance_NE(target_loc)
    local dist_m = dn and math.sqrt(dn:x()*dn:x() + dn:y()*dn:y()) or math.huge
    -- 2d velocity
    local plane_velocity = ahrs:get_velocity_NED()
    local speed_ms = plane_velocity and math.sqrt(plane_velocity:x()*plane_velocity:x() + plane_velocity:y()*plane_velocity:y()) or math.huge

    -- handle negative case - IDLE
    if current_mode ~= MODE_AUTO and current_mode ~= MODE_GUIDED then
        return {
        idle   = true,
        build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
        active = false
        }
    end

    -- new trigger
    local trigger_met = dist_m <= min_distance and speed_ms <= min_speed
    -- old trigger (before 7 May)
    -- note rerun autotest
    ---local trigger_met = dist_m <= min_distance or speed_ms <= min_speed

    if trigger_met then
        return {
        idle   = false,
        build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
        active = dubins_points_active ~= nil and dubins_point_index ~= nil
        }
    end

    -- else case
    return {
        idle   = false,
        build  = kangaroo_loc_pending ~= nil and kangaroo_loc_pending.loc ~= nil,
        active = false
    }

end


-- need to add hysterisis to deal with edge case


-- -----------------------------------------------------------------------
-- Revised update() — rebuild Dubins path every second from latest bus target,
-- walk through points as they are reached.

-- Three target states for state machine
-- 1. kanagroo_loc_latest, the newest bus sample
-- 2. kangaroo_loc_pending, the allocated target to build from
-- 3. kangaroo_loc_active, the target currently being weaved in

--- 19 April to do 
--- dubins path - cost function for two paths - select optimal path

-- -----------------------------------------------------------------------
local last_rebuild_ms     = 0
local last_swap_ms        = 0

function update()
    local current_mode = vehicle:get_mode()

    local now_ms = millis():toint()

    -- always absorb the latest bus sample
    local kangaroo_loc_latest = read_bus_target()
    if kangaroo_loc_latest then
        kangaroo_loc_pending = kangaroo_loc_latest
    end

   
    local pos = ahrs:get_position()
    -- establishing the state
    local state = engage_control_state(current_mode, dubins_point_active, dubins_point_index, pos, kangaroo_loc_pending and kangaroo_loc_pending.loc)

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
        if (now_ms - last_rebuild_ms) >= CTRL_REBUILD_MS:get() then
            last_rebuild_ms = now_ms
            local success, build_info = update_build(kangaroo_loc_pending)
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
                    -- manage switching trade off
                    local swap, adist, ndist = should_swap_dubins_path(kang_loc, active_final, new_final, CTRL_SWAP_DIST:get())
                    
                    -- cooldown - hystersisi 
                    local cooldown_elapsed = (now_ms - last_swap_ms) >= CTRL_SWAP_COOL:get()
                    -- guarding from swapping too quickly...
                    if swap and cooldown_elapsed then
                        local n_pts = #dubins_points_new
                        commit_dubins_path()
                        last_swap_ms = now_ms
                        gcs:send_text(4, string.format("Dubins swapped: %d pts new=%.0fm < active=%.0fm type=%s rho=%.0fm",
                            n_pts, ndist, adist,
                            (type(build_info) == "table" and build_info.path_type) or "?",
                            (type(build_info) == "table" and build_info.rho_m) or 0))
                    end
                end
            end
        end
    end

    -- -- periodically re-broadcast path points to MAVProxy map
    -- if (now_ms - last_vis_ms) >= DUBINS_VIS_INTERVAL_MS then
    --     last_vis_ms = now_ms
    --     broadcast_dubins_vis()
    -- end

    -- fly the active path
    if state.active then
        --- Establish current point and next point
        local point = dubins_points_active[dubins_point_index]
        local next_point = dubins_points_active[dubins_point_index + 1]

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
            gcs:send_text(4, string.format("Dubins error L1 Norm:%.1fm L2 Norm:%.1fm",
                cum_L1_error / n, cum_L2_error / n))
            last_report_ms = now_ms
        end

        -- fly to point
        if point and fly_to_dubins_point(point) then
            if dubins_point_reached(point, next_point) then
                gcs:send_text(4, string.format("Reached Dubins point %d/%d",
                    dubins_point_index, dubins_point_count or 0))
                dubins_point_index = dubins_point_index + 1
            end
        -- debugging
        -- no target
        elseif point then
            gcs:send_text(4, "Failed to set target for Dubins point")
        end

        -- clamp at end of path — next rebuild will reset the index
        if dubins_point_count ~= nil and dubins_point_index > dubins_point_count then
            dubins_point_index = dubins_point_count
        end

        -- else fly direct to the kangaroo if the velocity or distance metric isn't met
    else
        -- following the same structure to direct it to the direct location
        if kangaroo_loc_pending and kangaroo_loc_pending.loc then
            local home_alt_m = get_home_alt_m()
            if home_alt_m then
                local direct_loc = kangaroo_loc_pending.loc:copy()
                direct_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
                direct_loc:set_alt_m(home_alt_m + get_kangaroo_alt_m() + PLANE_ABOVE_TARGET_M, ALT_FRAME_ABSOLUTE)
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
