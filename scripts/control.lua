-- establish a control loop in 
-- need to keep path active... essentially a state machine 

-- if in gudied or auto
local MODE_AUTO = 10
local MODE_GUIDED = 15

-- states
local latest_target = nil 
local active_path = nil 
local pending target = nil
controller_busy = nil
dubins_point_index = nil
kangaroo_loc_pending = nil

-- import kangaroo_bus (target) and dubins outputs (path)
local kangaroo_loc = require("kangaroo_bus")
local dubins_points = require("dubins_weave")

-- producer script
function publish_target(x, y)
    if not controller_busy then
        target_buffer = {
            x = x, 
            y = y
        }
    end
end

-- need if follow mode = guided logic
-- core logic
function update()

    local kangaroo_loc_latest = kangaroo_loc.get()

    if kangaroo_loc_latest then
        kangaroo_loc_pending = kangaroo_loc_latest
    end

    if not controller_busy and kangaroo_loc_pending then
        kangaroo_loc_active = kangaroo_loc_pending
        kangaroo_loc_pending = nil

        -- name function not build (5 April)
        dubins_points_active = dubins_points.build(kangaroo_loc_active)
        dubins_point_index = 1
        controller_busy = dubins_points_active ~= nil and #dubins_points_active > 0
    end

    if controller_busy and dubins_points_active then
        local point = dubins_points_active[dubins_point_index]

        -- clean up logic from here - introduce two new functions - one to fly to the point, set the message
        -- one to update to the next point 
        -- take it outside of the update loops...
        if point then
            -- update point location
            fly_to_dubins_point(point)

            -- only use active values here
            local cmd = {
                lat = target_active.lat,
                lng = target_active.lng,
                alt = target_active.alt
            }
            vehicle:set_target_location(cmd)
            --throttle messaging to 2000 ms
            local now_ms = millis()
            if distance and now_ms - last_report_ms >= REPORT_INTERVAL_MS then
                last_report_ms = now_ms
                gcs:send_text(6, string.format("Travelling on Dubins Path"))
            end

            if dubins_point_reached(point) then
                dubins_point_index = dubins_point_index + 1
            end
        end

        -- if the index is greater than the number of active points
        if dubins_point_index > #dubins_points_active then
            controller_busy = false
            dubins_points_active = nil
            dubins_point_index = nil

            if kangaroo_loc_pending then
                kangaroo_loc_active = kangaroo_loc_pending
                kangaroo_loc_pending = nil
                dubins_points_active = dubins_points.build(kangaroo_loc_active)
                dubins_point_index = 1
                controller_busy = dubins_points_active ~= nil and #dubins_points_active > 0
            end
        end
    end
    return update, 100
end


-- disregard comments
-- dubins generate
-- while it executes, don't pull from new kangaroo
-- pull fromt the new kangaroo and regenerate the dubins curve using settarget
-- vehicle:get_target_location()
-- NED position
--vehicle:set_target_pos_NED()
--vehicle:set_target_posvel_NED()
--vehicle:set_target_posvelaccel_NED()
