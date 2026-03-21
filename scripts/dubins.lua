-- Dubins path implementation 

-- Section 1: Configuration 
-- captures turn radius, waypoint acceptance, starting condition (i.e. left or right)


-- Section 2: Current state values - vehicle configuration and waypoint
local function process_imu_sample(sample)
    -- local values gyro, accelaration views
    local gyro_x = sample.gyro:x()
    local gyro_y = sample.gyro:y()
    local gyro_z = sample.gyro:z()
    local accel_x = sample.accel:x()
    local accel_y = sample.accel:y()
    local accel_z = sample.accel:z()
    local pos = ahrs:get_position()
    local lat, lng, alt = nil, nil, nil
    if pos then
        lat = pos:lat()
        lng = pos:lng()
        alt = pos:alt()
    end
    -- clamp function
    --local normalized_command = clamp(gyro_x / 1.0, -1.0, 1.0)
    return {
        gyro_x = gyro_x,
        gyro_y = gyro_y,
        gyro_z = gyro_z,
        accel_x = accel_x,
        accel_y = accel_y,
        accel_z = accel_z,
        --normalized_command = normalized_command,
        timestamp_ms = sample.timestamp_ms,
        -- EKF/AHRS estimated vehicle position at roughly this time
        lat = lat,
        lng = lng,
        alt = alt
    }
end

-- getting waypoints function

local function mission_wps(item)
    if not item then
        return nil
    end
    local loc = Location()
    loc:lat(item:x())
    loc:lng(item:y())
    loc:alt(math.floor(item:z() * 100))
    return loc
end

-- Section 3: Euclidean distance between current state and waypoint


local function euclid_to_waypoint(current_loc, wp_loc)
    if not current_loc or not wp_loc then
        return nil
    end
    -- local ne = current_loc:get_distance_NE(wp_loc)
    -- *** double check the math here
    local ne = current_loc:get_distance_NE(wp_loc)
    distance = math.sqrt(ne:x() * ne:x() + ne:y() * ne:y())
    -- return current_loc:get_distance(wp_loc)
    return distance

end

-- Section 4: Coordinate references


-- Section 5: Turning circle


-- Section 6: Path sampling

-- Throttling

-- MAIN

local REPORT_INTERVAL_MS = 2000
local last_report_ms = millis()

local function update()

    -- getting waypoints
    local index = mission:get_current_nav_index()
    local item = mission:get_item(index)
    local wp = mission_wps(item)
    -- local position
    local pos = ahrs:get_position()
    -- local wp = vehicle:get_target_location()  
    -- alternative mission:get_current_nav_cmd()
    local distance = euclid_to_waypoint(pos, wp)

    --throttle messaging to 2000 ms
    local now_ms = millis()
    if distance and now_ms - last_report_ms >= REPORT_INTERVAL_MS then
        last_report_ms = now_ms
        gcs:send_text(6, string.format("Dubins distance: %.1f m", distance))
    end

--    --local sample, err = read_imu_sample()
--    --if sample == nil then
--        --  send_info("Structure.lua: " .. err)
--         return update, UPDATE_PERIOD_MS
--     end

--     local control_result = process_imu_sample(sample)
--     write_output(control_result)

--     -- Optional debug output, throttled to avoid flooding the GCS link.
--     if control_result.timestamp_ms - last_debug_ms >= 1000 then
--         last_debug_ms = control_result.timestamp_ms
--         send_info(string.format(
--             "IMU gx=%.2f ax=%.2f cmd=%.2f",
--             control_result.gyro_x,
--             control_result.accel_x,
--             control_result.normalized_command
--         ))
--     end

    return update, 100
end

return update()
