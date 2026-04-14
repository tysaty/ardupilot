-- establish a control loop - state handling

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- states
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


-- establishing flags for the parameters in 
local bus_seq_param = Parameter()
local bus_seq_ready = bus_seq_param:init("SCR_USER1")
assert(bus_seq_ready, "missing SCR_USER1")

local bus_t_s_param = Parameter()
local bus_t_s_ready = bus_t_s_param:init("SCR_USER2")
assert(bus_t_s_ready, "missing SCR_USER2")

local bus_lat_param = Parameter()
local bus_lat_ready = bus_lat_param:init("SCR_USER3")
assert(bus_lat_ready, "missing SCR_USER3")

local bus_lon_param = Parameter()
local bus_lon_ready = bus_lon_param:init("SCR_USER4")
assert(bus_lon_ready, "missing SCR_USER4")

local bus_vn_param = Parameter()
local bus_vn_ready = bus_vn_param:init("SCR_USER5")
assert(bus_vn_ready, "missing SCR_USER5")

local bus_ve_param = Parameter()
local bus_ve_ready = bus_ve_param:init("SCR_USER6")
assert(bus_ve_ready, "missing SCR_USER6")

local kang_alt_m_param = Parameter()
local kang_alt_m_ready = kang_alt_m_param:init("KANG_ALT_M")

local last_bus_seq_seen = 0

-- import the dubins weaving module
local dubins_points = require("dubins_weave_full")
-- LSR implementation
--local dubins_points = require("dubins_weave")
--math helpers
local math_helpers = require("math_helpers")

-- boot message
gcs:send_text(4, "Control: loaded at boot")

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
    local target_alt_m = home_alt_m + get_kangaroo_alt_m() + PLANE_ABOVE_TARGET_M
    target_loc:set_alt_m(target_alt_m, ALT_FRAME_ABSOLUTE)

    return vehicle:set_target_location(target_loc)
end


-- tuning aparameters
-- handle an acceptable radius from point
--local WP_ACCEPT_RADIUS_M = 30.0
--local MIN_WP_ACCEPT_RADIUS_M = 1.0
-- test
local WP_ACCEPT_RADIUS_M = 40.0
local MIN_WP_ACCEPT_RADIUS_M = 20.0 

-- consecutive hits required to be in radius before proceeding to next point
local REACHED_STREAK_REQUIRED = 1

-- flags for target reach
local reached_streak = 0
local reached_index = -1

-- old function
-- local function get_point_accept_radius_m(point, next_point)
    
--     local accept_radius_m = WP_ACCEPT_RADIUS_M
--     if point == nil or point.loc == nil or next_point == nil or next_point.loc == nil then
--         return accept_radius_m
--     end

--     local spacing_m = point.loc:get_distance(next_point.loc)
--     if spacing_m == nil or spacing_m <= 0 then
--         return accept_radius_m
--     end

--     -- keep acceptance radius below half waypoint spacing so adjacent points
--     -- cannot both be counted as reached from a single position sample.
--     local max_non_overlapping_radius_m = spacing_m * 0.45
--     if max_non_overlapping_radius_m < accept_radius_m then
--         accept_radius_m = math.max(MIN_WP_ACCEPT_RADIUS_M, max_non_overlapping_radius_m)
--     end

--     return accept_radius_m
-- end

-- test - revised
local function get_point_accept_radius_m(point, next_point)
    local accept_radius_m = WP_ACCEPT_RADIUS_M
    if point == nil or point.loc == nil or next_point == nil or next_point.loc == nil then
        return accept_radius_m
    end
    local spacing_m = point.loc:get_distance(next_point.loc)
    if spacing_m == nil or spacing_m <= 0 then
        return accept_radius_m
    end
    -- never let radius exceed 45% of spacing (avoid double-counting)
    -- but never go below MIN_WP_ACCEPT_RADIUS_M
    local max_radius = spacing_m * 0.45
    return math.max(MIN_WP_ACCEPT_RADIUS_M, math.min(accept_radius_m, max_radius))
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
        return reached_streak >= REACHED_STREAK_REQUIRED
    end
    -- reseting reached streak, ending loop in control loop
    reached_streak = 0
    return false
end

-- bus function
local function all_bus_ready()
    return bus_seq_ready and bus_t_s_ready and bus_lat_ready and bus_lon_ready and bus_vn_ready and bus_ve_ready
end

local function read_bus_target()
    -- not all values are ready
    if not all_bus_ready() then
        return nil
    end
    local seq_1 = bus_seq_param:get()
    if seq_1 == nil then
        return nil
    end

    -- if sequence is odd
    if (seq_1 % 2) ~= 0 then
        return nil
    end

    -- writing values
    local t_s = bus_t_s_param:get()
    local lat_deg = bus_lat_param:get()
    local lon_deg = bus_lon_param:get()
    local vn = bus_vn_param:get()
    local ve = bus_ve_param:get()
    -- bin if any are nil
    if t_s == nil or lat_deg == nil or lon_deg == nil or vn == nil or ve == nil then
        return nil
    end

    local seq_2 = bus_seq_param:get()
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

-- function to update build
local function update_build(kangaroo_loc_active)

    local build_result, build_info = dubins_points.build_path(kangaroo_loc_active)

    -- reject short paths
    if build_result ~= nil and #build_result < MIN_DUBINS_POINTS then
        gcs:send_text(4, string.format("Dubins build rejected: only %d points", #build_result))
        build_result = nil
        build_info = "degenerate_path"
    end

    -- generate result
    dubins_points_active = build_result
    dubins_point_index = 1
    dubins_point_count = (dubins_points_active ~= nil) and #dubins_points_active or nil
    controller_busy = (dubins_points_active ~= nil and #dubins_points_active > 0)

    if not controller_busy and build_info ~= nil then
        gcs:send_text(6, "Dubins build failed: " .. tostring(build_info))
    end
    -- return true for controller_busy
    return controller_busy
end

-- update to run main logic
-- function update()
--     -- only run if vehcile is in AUTO or GUIDED
--     local current_mode = vehicle:get_mode()
--     -- if statement to manage when not AUTO or not GUIDED
--     if current_mode ~= MODE_AUTO and current_mode ~= MODE_GUIDED then 
--         return update, 100
--     end

--     -------
--     -- Three target states for state machine
--     -- 1. kanagroo_loc_latest, the newest bus sample
--     -- 2. kangaroo_loc_pending, the allocated target to build from
--     -- 3. kangaroo_loc_active, the target currently being weaved in
--     -------

--     -------
--     --- State 1: Building a new weave when the controller is idle
--     local kangaroo_loc_latest = read_bus_target()
    
--     if kangaroo_loc_latest then
--         kangaroo_loc_pending = kangaroo_loc_latest
--     end

--     if not controller_busy and kangaroo_loc_pending and kangaroo_loc_pending.loc then
--         kangaroo_loc_active = kangaroo_loc_pending
--         kangaroo_loc_pending = nil
--         update_build(kangaroo_loc_active)
--     end

--     -----
--     --- State 2: Controller is busy, locked into state
--     if controller_busy and dubins_points_active then
--         local point = dubins_points_active[dubins_point_index]
--         local next_point = dubins_points_active[dubins_point_index + 1]

--         -- update next point in dubins weave, fly to the point, reach the point, move onto next
--         -- may be better suited with a 
--         if point and fly_to_dubins_point(point) then
--             if dubins_point_reached(point, next_point) then
--                 --gcs:send_text(4, string.format("Dubins idx=%d/%d", dubins_point_index or 0, dubins_point_count or 0))
--                 dubins_point_index = dubins_point_index + 1
--             end
--            -- gcs:send_text(4, string.format("Dubins idx=%d/%d", dubins_point_index or 0, dubins_point_count or 0))

--         elseif point then
--             -- handle failure to set target
--             gcs:send_text(6, "Failed to set target for Dubins point")
--         end

--         -- if the index is greater than the number of active points, i.e. end loop and go to next step
--         if dubins_point_count ~= nil and dubins_point_index > dubins_point_count then
--             -- test for fresh sample
--             -- clear path exeuction state for rebuild
--             controller_busy = false
--             dubins_points_active = nil
--             dubins_point_index = nil
--             dubins_point_count = nil
--             reached_streak = 0
--             reached_index = -1

--             --- State 3 - Target is waiting fora  reset
--             if kangaroo_loc_pending == nil then
--                 -- hold it until there is a fresh sample in 
--                 kangaroo_loc_pending = kangaroo_loc_active
--             end

--             --clear out kangaroo_loc_active
--             kangaroo_loc_active = nil
--         end

--     end

--     return update, 100
-- end


-- update logic
function update()
    local current_mode = vehicle:get_mode()
    -- only operate in GUIDED or AUTO....
    if current_mode ~= MODE_AUTO and current_mode ~= MODE_GUIDED then 
        return update, 100
    end

    local kangaroo_loc_latest = read_bus_target()

    if kangaroo_loc_latest then
        kangaroo_loc_pending = kangaroo_loc_latest
    end

    -- if the controller has finished and there is a kangaroo location
    if not controller_busy and kangaroo_loc_pending and kangaroo_loc_pending.loc then
        local success = update_build(kangaroo_loc_pending)
        if success then
            kangaroo_loc_active = kangaroo_loc_pending
            kangaroo_loc_pending = nil
            gcs:send_text(4, "New Dubins Build!")
        end
        -- on failure kangaroo_loc_pending stays so the next tick retries with a fresh sample
    end

   -- build the points of the Dubins path
    if controller_busy and dubins_points_active then
        local point = dubins_points_active[dubins_point_index]
        local next_point = dubins_points_active[dubins_point_index + 1]

        -- count
        -- if dubins_point_count ~= nil and dubins_point_index ~= nil then
        --     local now_ms = millis():toint()
        --     if (now_ms - last_report_ms) >= REPORT_INTERVAL_MS then
        --         local idx = math.min(dubins_point_index, dubins_point_count)
        --         gcs:send_text(4, string.format("Dubins point %d/%d", idx, dubins_point_count))
                
        --         last_report_ms = now_ms
        --     end
        -- end
        if dubins_point_count ~= nil and dubins_point_index ~= nil then
            local now_ms = millis():toint()
            if (now_ms - last_report_ms) >= REPORT_INTERVAL_MS then
                local pos2 = ahrs:get_position()
                local pt = dubins_points_active[dubins_point_index]
                local dist_2d = -1
                if pos2 and pt then
                    local dn = pos2:get_distance_NE(pt.loc)
                    if dn then
                        dist_2d = math.sqrt(dn:x() * dn:x() + dn:y() * dn:y())
                    end
                end
                gcs:send_text(4, string.format("Dubins point %d/%d dist=%.1fm",
                    dubins_point_index, dubins_point_count, dist_2d))
                last_report_ms = now_ms
            end
        end

        -- update next point in dubins weave, fly to the point, reach the point, move onto next
        -- may be better suited with a 
        if point and fly_to_dubins_point(point) then
            if dubins_point_reached(point, next_point) then

                gcs:send_text(4, string.format("Reached Dubins point %d/%d", dubins_point_index, dubins_point_count or 0))

                -- increase increment
                dubins_point_index = dubins_point_index + 1
                --gcs:send_text(4, string.format("Dubins idx=%d/%d", dubins_point_index or 0, dubins_point_count or 0))

            end
           -- gcs:send_text(4, string.format("Dubins idx=%d/%d", dubins_point_index or 0, dubins_point_count or 0))

        elseif point then
            -- handle failure to set target
            gcs:send_text(4, "Failed to set target for Dubins point")
        end

        --- old try
        -- -- if the index is greater than the number of active points, i.e. end loop and go to next step
        -- if dubins_point_count ~= nil and dubins_point_index > dubins_point_count then
        --     gcs:send_text(4, string.format("End of Dubins Curve!"))
        --     controller_busy = false
        --     dubins_points_active = nil
        --     dubins_point_index = nil
        --     dubins_point_count = nil
        --     -- new content
        --     reached_streak = 0
        --     reached_index = -1
        --     -- test 13 April
        --     if kangaroo_loc_pending == nil then
        --         -- hold it until there is a fresh sample in 
        --         kangaroo_loc_pending = kangaroo_loc_active
        --     end
        --     -- clear path exeuction state for rebuild
        --     --dubins_points_active = nil
        --     --dubins_point_index = nil
        --     --dubins_point_count = nil

        --     kangaroo_loc_active = nil
        -- end

        --- new try
        if dubins_point_count ~= nil and dubins_point_index > dubins_point_count then
            gcs:send_text(4, "End of Dubins Curve!")

            -- Clear execution state
            controller_busy = false
            dubins_points_active = nil
            dubins_point_index = nil
            dubins_point_count = nil
            reached_streak = 0
            reached_index = -1

            -- If no fresh bus sample is already waiting, recycle the last known location
            -- so the rebuild trigger fires next tick even if the bus is momentarily quiet
            if kangaroo_loc_pending == nil and kangaroo_loc_active ~= nil then
                kangaroo_loc_pending = kangaroo_loc_active
                gcs:send_text(6, "Control: recycling active loc for rebuild")
            end

            kangaroo_loc_active = nil
            -- Rebuild happens next tick via the normal not-busy + pending check
        end
    end
    
    return update, 200
end

return update()
