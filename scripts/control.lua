-- establish a control loop - state handling

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15

-- states
local latest_target = nil 
local active_path = nil 
local pending_target = nil
local controller_busy = nil
local dubins_point_index = nil
local kangaroo_loc_pending = nil
local REPORT_INTERVAL_MS = 2000
local last_report_ms = 0

-- setting local altittude
local plane_altitude = 150.0

-- import kangaroo_bus (target) and dubins outputs (path)
local kangaroo_loc = require("kangaroo_bus")
local dubins_points = require("dubins_weave")

-- fly to point
function fly_to_dubins_point(point)
    -- handle negative case
   if point == nil or kangaroo_loc_active == nil or kangaroo_loc_active.loc == nil then
        return false
    end
    -- initialsing lcoation
    local target_loc = Location()
    -- setting points
    local north_m = point.x or 0.0
    local east_m = point.y or 0.0
    target_loc:offset(north_m, east_m)

    -- Setting altitutde to be 150 m above 
    target_loc:change_alt_frame(0)
    local target_alt_m = (kangaroo_loc_active.loc:alt() * 0.01) + plane_altitude

    -- set altittude to the target_loc value
    target_loc:set_alt_m(target_alt_m, 0)

    -- update messaging
    gcs:send_text(6, string.format("Travelling on Dubins Path", "x:",north_m, "y:",east_m))
    -- set location
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

-- update logic
function update()
    local current_mode = vehicle:get_mode()
    -- only operate in GUIDED or AUTO....
    if current_mode = 10 OR 15
        local kangaroo_loc_latest = kangaroo_loc.get()
        -- Case 1: kangaroo hasn't moved
        if kangaroo_loc_latest then
            kangaroo_loc_pending = kangaroo_loc_latest
        end

        -- Case 2: kangaroo has moved, Dubins curve to be activated
        -- if controller isn't busy and the kangaroo_location is pending
        if not controller_busy and kangaroo_loc_pending then
            kangaroo_loc_active = kangaroo_loc_pending
            kangaroo_loc_pending = nil

        -- build the points of the Dubins path
            --dubins_points_active = dubins_points(kangaroo_loc_active)
            dubins_points_active = dubins_points.build_path(kangaroo_loc_active)
            dubins_point_index = 1
            controller_busy = dubins_points_active ~= nil and #dubins_points_active > 0
        end

        if controller_busy and dubins_points_active then
            local point = dubins_points_active[dubins_point_index]

            -- update next point in dubins weave
            if point then
                -- fly to point
                fly_to_dubins_point(point)
                -- check if point is reached
                if dubins_point_reached(point) then
                    dubins_point_index = dubins_point_index + 1
                end
            
                local now_ms = millis():toint()
                if now_ms - last_report_ms >= REPORT_INTERVAL_MS then
                    last_report_ms = now_ms
                    gcs:send_text(6, "Travelling on Dubins Path")
                end
            end


            -- if the index is greater than the number of active points, i.e. end loop and go to next step
            if dubins_point_index > #dubins_points_active then
                controller_busy = false
                dubins_points_active = nil
                dubins_point_index = nil

                -- double check logic
                if kangaroo_loc_pending then
                    kangaroo_loc_active = kangaroo_loc_pending
                    kangaroo_loc_pending = nil
                    --dubins_points_active = dubins_points(kangaroo_loc_active)
                    dubins_points_active = dubins_points.build_path(kangaroo_loc_active)
                    dubins_point_index = 1
                    controller_busy = dubins_points_active ~= nil and #dubins_points_active > 0
                end
            end
        end
    return update, 100
end

return update()