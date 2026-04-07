-- establish a control loop - state handling

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- states
local controller_busy = nil
local dubins_point_index = nil
local kangaroo_loc_pending = nil
local kangaroo_loc_active = nil
local dubins_points_active = nil
local REPORT_INTERVAL_MS = 2000
local last_report_ms = 0

-- maintain aircraft 150m above virtual target altitude
local PLANE_ABOVE_TARGET_M = 150.0
local KANG_ALT_M_FALLBACK = 80.0
local kang_alt_m_param = Parameter()
local kang_alt_m_ready = kang_alt_m_param:init("KANG_ALT_M")


-- import kangaroo_bus (target) and dubins outputs (path)
local kangaroo_loc = require("kangaroo_bus")
local dubins_points = require("dubins_weave")

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
local WP_ACCEPT_RADIUS_M = 1.0
-- consecutive hits required to be in radius before proceeding to next point
local REACHED_STREAK_REQUIRED = 2

-- flags for target reach
local reached_streak = 0
local reached_index = -1

function dubins_point_reached(point)
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
    local weave_target = vehicle:get_target_location()

    -- nil case
    if pos == nil or weave_target == nil then
        reached_streak = 0
        return false
    end

    -- checking distance to current location
    local dist_m = pos:get_distance(weave_target)

    -- if the location is nil
    if dist_m == nil then
        reached_streak = 0
        return false
    end

    -- if distance is less 
    if dist_m <= WP_ACCEPT_RADIUS_M then
        reached_streak = reached_streak + 1
        return reached_streak >= REACHED_STREAK_REQUIRED
    end
    -- reseting reached streak, ending loop in control loop
    reached_streak = 0
    return false
end

-- function to update build
local function update_build()
    if kangaroo_loc_pending == nil then
        return false
    end

    kangaroo_loc_active = kangaroo_loc_pending
    kangaroo_loc_pending = nil

    local build_result, build_info = dubins_points.build_path(kangaroo_loc_active)
    dubins_points_active = build_result
    dubins_point_index = 1
    controller_busy = (dubins_points_active ~= nil and #dubins_points_active > 0)

    if not controller_busy and build_info ~= nil then
        gcs:send_text(6, "Dubins build failed: " .. tostring(build_info))
    end

    return controller_busy
end

-- update logic
function update()
    local current_mode = vehicle:get_mode()
    -- only operate in GUIDED or AUTO....
    if current_mode ~= MODE_AUTO and current_mode ~= MODE_GUIDED then 
        return update, 100
    end

    local kangaroo_loc_latest = kangaroo_loc.get()

    -- Case 1: kangaroo hasn't moved
    if kangaroo_loc_latest then
        kangaroo_loc_pending = kangaroo_loc_latest
    end

    -- Case 2: kangaroo has moved, Dubins curve to be activated
    -- if controller isn't busy and the kangaroo_location is pending
    -- if not controller_busy and kangaroo_loc_pending then
    --     kangaroo_loc_active = kangaroo_loc_pending
    --     kangaroo_loc_pending = nil
    -- end 
    if not controller_busy then
        update_build()   -- start path when idle
    end

    -- build the points of the Dubins path
    if controller_busy and dubins_points_active then
        local point = dubins_points_active[dubins_point_index]

        -- update next point in dubins weave, fly to the point, reach the ooint, move onto next
        if point and fly_to_dubins_point(point) and dubins_point_reached(point) then
            dubins_point_index = dubins_point_index + 1
        end
        -- reporting in mavlink
        local now_ms = millis():toint()
        if now_ms - last_report_ms >= REPORT_INTERVAL_MS then
            last_report_ms = now_ms
            gcs:send_text(6, string.format("Dubins idx=%d/%d", dubins_point_index or 0, #dubins_points_active))
        end
        -- if the index is greater than the number of active points, i.e. end loop and go to next step
        if dubins_point_index > #dubins_points_active then
            controller_busy = false
            dubins_points_active = nil
            dubins_point_index = nil
            update_build()
        end
    end

    return update, 100
end

return update()
