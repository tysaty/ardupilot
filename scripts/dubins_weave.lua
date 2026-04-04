-- Dubins path implementation 
-- Implemented just as LSR for the time being - will expand following testing

-- Section 1: Configuration 
-- captures turn radius, waypoint acceptance, starting condition (i.e. left or right)
-- setting absolute to be the same altitutde as the reference (MAJOR ASSUMPTION IN CODE - NEED TO FIX)
local ALT_FRAME_ABSOLUTE = 0
local grav = 9.807
local PHI_MAX_RAD = math.rad(45)

-- Section 2: Current state values - vehicle configuration 
local function process_imu_sample(sample)
    local pos = ahrs:get_position()
    local vel = ahrs:get_velocity_NED()
    local yaw = ahrs:get_yaw_rad()
    -- local values gyro, accelaration views 
    -- local gyro_x = sample.gyro:x()
    -- local gyro_y = sample.gyro:y()
    -- local gyro_z = sample.gyro:z()
    -- local accel_x = sample.accel:x()
    -- local accel_y = sample.accel:y()
    -- local accel_z = sample.accel:z()
    -- nil case
    if pos == nil or vel == nil or yaw == nil then
        return nil
    end
    -- udpate absolute frame reference
    pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
    -- calculate V_T
    local vn = vel:x()
    local ve = vel:y()
    local vd = vel:z()
    local vt = math.sqrt(vn * vn + ve * ve)
    -- establishing lat, long, alt
    local lat, lng, alt = nil, nil, nil
    if pos then
        lat = pos:lat()
        lng = pos:lng()
        alt = pos:alt()
    end
    -- ms 
    local timestamp_ms = millis():toint()
    if sample ~= nil then
        timestamp_ms = sample.timestamp_ms
    end
    -- clamp function
    --local normalized_command = clamp(gyro_x / 1.0, -1.0, 1.0)
    return {
        -- gyro_x = gyro_x,
        -- gyro_y = gyro_y,
        -- gyro_z = gyro_z,
        -- accel_x = accel_x,
        -- accel_y = accel_y,
        -- accel_z = accel_z,
        --normalized_command = normalized_command,
        timestamp_ms = timestamp_ms,
        -- EKF/AHRS estimated vehicle position at roughly this time
        pos = pos,
        lat = lat,
        lng = lng,
        alt = alt,
        psi = yaw,
        vn = vn,
        ve = ve,
        vd = vd,
        Vt = vt
    }
end

-- getting target value
local function chase_target()
    local target = vehicle:get_target_location()
    if target == nil then
        return nil
    end
    local loc = target:copy()
    return loc
end

--  states for dubins calculations
local function build_state(sample)
    local current_state = process_imu_sample(sample)
    local target = chase_target()
    if current_state == nil or target == nil then
        return nil
    end
    --
    local rel_ne = current_state.pos:get_distance_NE(target)
    if rel_ne == nil then
        return nil
    end

    return{
        current_state = current_state,
        target = target,
        xi = 0.0,
        yi = 0.0,
        psi_i = current_state.psi,
        -- don'tknow where I got this
        xf = rel_ne:x(),
        yf = rel_ne:y(),
        -- heading is a bit of a tba - this is an apprximation 
        psi_f = current_state.pos:get_bearing(target),
        -- tba if this is right
        rho = min_turn_radius(math.max(current_state.Vt, 1.0), PHI_MAX_RAD, grav)
    }
end

-- Section 2: onstructing Dubins values
local PI = math.pi

local function clamp(x, low, hi)
    if x < low then 
        return low 
    end
    if x > hi then 
        return hi 
    end
    return x
end

local function wrap_pi(i)
    while i > PI do
        i = i - 2.0 * PI 
    end
    while i < -PI do 
        i = i + 2.0 * PI 
    end
    return i
end

-- Euclidean distance
local function dist2d(x1, y1, x2, y2)
    local dx = x2 - x1
    local dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)
end

-- ---------------------------------------------------------
-- Dubins kinematic equations
-- ---------------------------------------------------------
local function dubins_kinematics(x, y, psi, Vt, omega, dt)
    local x_next = x + Vt * math.cos(psi) * dt
    local y_next = y + Vt * math.sin(psi) * dt
    local psi_next = psi + omega * dt
    return x_next, y_next, wrap_pi(psi_next)
end

-- ---------------------------------------------------------
-- (2) Heading rate induced by roll
-- omega = g / Vt * tan(phi)
-- ---------------------------------------------------------
local function heading_rate_from_roll(phi, Vt, g)
    return (g / Vt) * math.tan(phi)
end

-- ---------------------------------------------------------
-- (3) Minimum turn radius
-- rho = Vt^2 / (g * tan(phi_max))
-- ---------------------------------------------------------
local function min_turn_radius(Vt, phi_max, g)
    return (Vt * Vt) / (g * math.tan(phi_max))
end

-- ---------------------------------------------------------
-- (4) Circle centers
-- ---------------------------------------------------------
local function circle_center_right(x, y, psi, rho)
    local xc = x + rho * math.cos(psi)
    local yc = y - rho * math.sin(psi)
    return xc, yc
end

local function circle_center_left(x, y, psi, rho)
    local xc = x - rho * math.cos(psi)
    local yc = y + rho * math.sin(psi)
    return xc, yc
end

local function dubins_circle_centers(xi, yi, psi_i, xf, yf, psi_f, rho)
    local xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    local xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    local xLf, yLf = circle_center_left(xf, yf, psi_f, rho)
    return {
        CRi = {x = xRi, y = yRi},
        CLi = {x = xLi, y = yLi},
        CRf = {x = xRf, y = yRf},
        CLf = {x = xLf, y = yLf},
    }
end

-- ---------------------------------------------------------
-- LSR Geometry
-- theta = eta + gamma - pi/2
-- eta   = pi/2 + atan2(yRf - yLi, xRf - xLi)
-- gamma = acos(2*rho / d)
-- straight_length = sqrt(l^2 - 4*rho^2)
-- l = distance(CL_i, CR_f) (i.e. distance between centroids)
--  straight_length = sqrt(l^2 - 4*rho^2)
--  gamma = acos( clamp(2*rho / l, -1, 1) )
-- ---------------------------------------------------------
local function lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho)
    local l = dist2d(xLi, yLi, xRf, yRf)
    local straight_length = math.sqrt(math.max(0.0, l * l - 4.0 * rho * rho))
    local eta = (PI / 2.0) + math.atan(yRf - yLi, xRf - xLi)
    local gamma = math.acos(clamp((2.0 * rho) / l, -1.0, 1.0))
    local theta = eta + gamma - (PI / 2.0)
    return theta, straight_length, eta, gamma, l
end

-- ---------------------------------------------------------
-- (9) Arc point generation
-- pn = [xc + rho*sin(psi_n), yc + rho*cos(psi_n)]
-- Used by the paper for both right and left arc point updates
-- ---------------------------------------------------------
local function arc_point(xc, yc, rho, psi_n)
    local x = xc + rho * math.sin(psi_n)
    local y = yc + rho * math.cos(psi_n)
    return x, y
end

-- ---------------------------------------------------------
-- (10) Straight segment point generation
-- x_n = x_(n-1) + delta_d * sin(theta)
-- y_n = y_(n-1) + delta_d * cos(theta)
-- ---------------------------------------------------------
local function straight_step(x_prev, y_prev, theta, delta_d)
    local x = x_prev + delta_d * math.sin(theta)
    local y = y_prev + delta_d * math.cos(theta)
    return x, y
end

-- Section 3 - generating segments
-- generating arc points
local function generate_arc_points(points, xc, yc, rho, psi_start, psi_end, delta_psi, increasing)
    local psi = psi_start
    if increasing then
        while psi <= psi_end do
            local x, y = arc_point(xc, yc, rho, psi)
            points[#points + 1] = {x = x, y = y, psi = psi}
            psi = psi + delta_psi
        end
    else
        while psi >= psi_end do
            local x, y = arc_point(xc, yc, rho, psi)
            points[#points + 1] = {x = x, y = y, psi = psi}
            psi = psi - delta_psi
        end
    end
end
-- generate striaght points
local function generate_straight_points(points, x_start, y_start, theta, total_d, delta_d)
    local x = x_start
    local y = y_start
    local dsum = 0.0
    while dsum <= total_d do
        x, y = straight_step(x, y, theta, delta_d)
        points[#points + 1] = {x = x, y = y, psi = theta}
        dsum = dsum + delta_d
    end

    return x, y
end

-- generating LSR
local function generate_LSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local points = {}
    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    local xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    local theta, straight_len = lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho)
    -- Generate Left
    generate_arc_points(points, xLi, yLi, rho, psi_i, theta, delta_psi, true)
    -- Straight
    local last = points[#points]
    local xs, ys = generate_straight_points(points, last.x, last.y, theta, straight_len, delta_d)
    -- Generate right
    generate_arc_points(points, xRf, yRf, rho, theta, psi_f, delta_psi, false)
    return points
end


-- establish a control loop in 
-- need to keep path active... with a state machine
-- path index...


-- MAIN

local REPORT_INTERVAL_MS = 2000
local last_report_ms = millis()

local function update()
    -- getting target
    local target = chase_target()
    if target == nil then
        return update, 100
    end
    -- local position
    local pos = ahrs:get_position()
    -- local wp = vehicle:get_target_location()  

    -- Dubins geometry
    local points = generate_LSR(
        dubins.xi, dubins.yi, dubins.psi_i,
        dubins.xf, dubins.yf, dubins.psi_f,
        dubins.rho,
        math.rad(5), 5.0
    )

    --no points case, update for 100ms
    if #points == 0 then
        return update, 100
    end

    --throttle messaging to 2000 ms
    local now_ms = millis()

    --update this part for an error check
    -- if distance and now_ms - last_report_ms >= REPORT_INTERVAL_MS then
    --     last_report_ms = now_ms
    --     gcs:send_text(6, string.format("Dubins distance: %.1f m", distance))
    -- end

    return update, 100
end

return update()
