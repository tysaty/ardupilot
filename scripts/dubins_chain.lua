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

    local ne = current_loc:get_distance_NE(wp_loc)
    local distance = math.sqrt(ne:x() * ne:x() + ne:y() * ne:y())
    return distance
end

-- Section 4: Coordinate references


-- Section 5: Generate Dubins

local TURN_RADIUS_M = 25
local SAMPLE_SPACING_M = 25
local TWO_PI = 2 * math.pi
local EPSILON = 1e-6

local function mod2pi(theta)
    theta = theta % TWO_PI
    if theta < 0 then
        theta = theta + TWO_PI
    end
    return theta
end

local function clamp(x, lo, hi)
    if x < lo then
        return lo
    end
    if x > hi then
        return hi
    end
    return x
end

local function vec2(north, east)
    return { north = north, east = east }
end

local function vec_add(a, b)
    return vec2(a.north + b.north, a.east + b.east)
end

local function vec_sub(a, b)
    return vec2(a.north - b.north, a.east - b.east)
end

local function vec_scale(v, scalar)
    return vec2(v.north * scalar, v.east * scalar)
end

local function vec_dot(a, b)
    return a.north * b.north + a.east * b.east
end

local function vec_cross_z(a, b)
    return a.north * b.east - a.east * b.north
end

local function vec_length(v)
    return math.sqrt(vec_dot(v, v))
end

local function vec_unit(v)
    local length = vec_length(v)
    if length < EPSILON then
        return nil
    end
    return vec_scale(v, 1.0 / length)
end

local function vec_left_normal(v)
    return vec2(-v.east, v.north)
end

local function vec_right_normal(v)
    return vec2(v.east, -v.north)
end

local function vec_lerp(a, b, t)
    return vec2(
        a.north + (b.north - a.north) * t,
        a.east + (b.east - a.east) * t
    )
end

local function location_to_local_ne(ref_loc, loc)
    if not ref_loc or not loc then
        return nil
    end

    local ne = ref_loc:get_distance_NE(loc)
    return vec2(ne:x(), ne:y())
end

local function local_ne_to_location(ref_loc, ne, alt_cm)
    local loc = ref_loc:copy()
    loc:offset(ne.north, ne.east)
    if alt_cm then
        loc:alt(alt_cm)
    end
    return loc
end

local function build_dubins_chain(start_loc, waypoint_loc, next_waypoint_loc, turn_radius_m)
    if not start_loc or not waypoint_loc then
        return nil
    end

    local ref_loc = waypoint_loc
    local start_ne = location_to_local_ne(ref_loc, start_loc)
    local waypoint_ne = vec2(0, 0)
    local waypoint_alt_cm = waypoint_loc:alt()

    if not next_waypoint_loc then
        return {
            ref_loc = ref_loc,
            alt_cm = waypoint_alt_cm,
            curved = false,
            segments = {
                {
                    type = "S",
                    start = start_ne,
                    finish = waypoint_ne
                }
            }
        }
    end

    local next_ne = location_to_local_ne(ref_loc, next_waypoint_loc)
    local inbound = vec_sub(waypoint_ne, start_ne)
    local outbound = vec_sub(next_ne, waypoint_ne)
    local inbound_length = vec_length(inbound)
    local outbound_length = vec_length(outbound)

    if inbound_length < EPSILON or outbound_length < EPSILON then
        return {
            ref_loc = ref_loc,
            alt_cm = waypoint_alt_cm,
            curved = false,
            segments = {
                {
                    type = "S",
                    start = start_ne,
                    finish = waypoint_ne
                }
            }
        }
    end

    local inbound_unit = vec_unit(inbound)
    local outbound_unit = vec_unit(outbound)
    local turn_angle = math.acos(clamp(-vec_dot(inbound_unit, outbound_unit), -1.0, 1.0))

    if turn_angle < 1e-3 or math.abs(math.pi - turn_angle) < 1e-3 then
        return {
            ref_loc = ref_loc,
            alt_cm = waypoint_alt_cm,
            curved = false,
            segments = {
                {
                    type = "S",
                    start = start_ne,
                    finish = waypoint_ne
                }
            }
        }
    end

    local tangent_distance = turn_radius_m * math.tan(turn_angle * 0.5)
    if tangent_distance >= inbound_length or tangent_distance >= outbound_length then
        return {
            ref_loc = ref_loc,
            alt_cm = waypoint_alt_cm,
            curved = false,
            segments = {
                {
                    type = "S",
                    start = start_ne,
                    finish = waypoint_ne
                }
            }
        }
    end

    local entry = vec_sub(waypoint_ne, vec_scale(inbound_unit, tangent_distance))
    local exit = vec_add(waypoint_ne, vec_scale(outbound_unit, tangent_distance))
    local turn_left = vec_cross_z(inbound_unit, outbound_unit) > 0
    local normal = turn_left and vec_left_normal(inbound_unit) or vec_right_normal(inbound_unit)
    local center = vec_add(entry, vec_scale(normal, turn_radius_m))
    local start_angle = math.atan2(entry.east - center.east, entry.north - center.north)
    local end_angle = math.atan2(exit.east - center.east, exit.north - center.north)
    local sweep = turn_left and mod2pi(end_angle - start_angle) or mod2pi(start_angle - end_angle)

    return {
        ref_loc = ref_loc,
        alt_cm = waypoint_alt_cm,
        curved = true,
        entry = entry,
        exit = exit,
        center = center,
        turn_left = turn_left,
        segments = {
            {
                type = "S",
                start = start_ne,
                finish = entry
            },
            {
                type = turn_left and "L" or "R",
                center = center,
                radius = turn_radius_m,
                start_angle = start_angle,
                sweep = sweep
            },
            {
                type = "S",
                start = exit,
                finish = next_ne
            }
        }
    }
end

-- Section 6: Path sampling


local function append_location_sample(samples, chain, ne)
    samples[#samples + 1] = local_ne_to_location(chain.ref_loc, ne, chain.alt_cm)
end

local function sample_straight_segment(samples, chain, segment, spacing_m)
    local segment_vec = vec_sub(segment.finish, segment.start)
    local segment_length = vec_length(segment_vec)
    local steps = math.max(1, math.ceil(segment_length / spacing_m))

    for i = 0, steps do
        local t = i / steps
        append_location_sample(samples, chain, vec_lerp(segment.start, segment.finish, t))
    end
end

local function sample_arc_segment(samples, chain, segment, spacing_m)
    local arc_length = segment.radius * segment.sweep
    local steps = math.max(1, math.ceil(arc_length / spacing_m))

    for i = 0, steps do
        local t = i / steps
        local angle = segment.turn_left and
            (segment.start_angle + segment.sweep * t) or
            (segment.start_angle - segment.sweep * t)
        local point = vec2(
            segment.center.north + segment.radius * math.cos(angle),
            segment.center.east + segment.radius * math.sin(angle)
        )
        append_location_sample(samples, chain, point)
    end
end

local function sample_dubins_chain(chain, spacing_m)
    if not chain then
        return {}
    end

    local samples = {}
    local spacing = spacing_m or SAMPLE_SPACING_M

    for i = 1, #chain.segments do
        local segment = chain.segments[i]
        if segment.type == "S" then
            sample_straight_segment(samples, chain, segment, spacing)
        else
            segment.turn_left = (segment.type == "L")
            sample_arc_segment(samples, chain, segment, spacing)
        end
    end

    return samples
end




-- Throttling

-- MAIN

local REPORT_INTERVAL_MS = 2000
local last_report_ms = millis()
local latest_dubins_chain = nil
local latest_dubins_samples = {}

local function update()

    -- getting waypoints
    local index = mission:get_current_nav_index()
    local item = mission:get_item(index)
    local wp = mission_wps(item)
    local next_item = mission:get_item(index + 1)
    local next_wp = mission_wps(next_item)
    -- local position
    local pos = ahrs:get_position()

    latest_dubins_chain = build_dubins_chain(pos, wp, next_wp, TURN_RADIUS_M)
    latest_dubins_samples = sample_dubins_chain(latest_dubins_chain, SAMPLE_SPACING_M)

    local distance = euclid_to_waypoint(pos, wp)


    --throttle messaging to 2000 ms
    local now_ms = millis()
    if distance and now_ms - last_report_ms >= REPORT_INTERVAL_MS then
        last_report_ms = now_ms
        gcs:send_text(6, string.format(
            "Dubins distance: %.1f m, samples=%d",
            distance,
            #latest_dubins_samples
        ))
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

    return update, 1000
end

return update()
