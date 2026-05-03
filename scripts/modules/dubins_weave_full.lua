-- ---------------------------------------------------------
-- Dubins Path Implementation

-- a Dubins path generator, that returns the optimal
-- path between two points (the plane and the target).
-- Currently only implemneted in 2-dimensions, requries
-- extnesion into 3D following successfuly tests. 

-- Main reference for equeations:
-- Lugo-Cárdenas, Israel & Flores, Gerardo & Salazar, Sergio & Lozano, R.. (2014). 
-- Dubins path generation for a fixed wing UAV. 339-346. 10.1109/ICUAS.2014.6842272. 
-- ---------------------------------------------------------

-- ---------------------------------------------------------
-- Section 1: Configuration 
-- captures turn radius, waypoint acceptance, starting condition (i.e. left or right)
-- setting absolute to be the same altitutde as the reference (MAJOR ASSUMPTION IN CODE - NEED TO FIX)
-- ---------------------------------------------------------

local ALT_FRAME_ABSOLUTE = 0
local grav = 9.807
local PHI_MAX_RAD = math.rad(45)

local math_helpers = require("math_helpers")

-- ---------------------------------------------------------
-- Section 2: Current state values - vehicle configuration 
-- this is from the actual vehicle - update code to process imu_sample from leader acrchitecture
-- ---------------------------------------------------------


-- ---------------------------------------------------------
-- Section 3: constructing Dubins curves
-- ---------------------------------------------------------

local PI = math.pi

-- ---------------------------------------------------------
-- kinematic equations
-- ---------------------------------------------------------
-- local function dubins_kinematics(x, y, psi, Vt, omega, dt)
--     local x_next = x + Vt * math.cos(psi) * dt
--     local y_next = y + Vt * math.sin(psi) * dt
--     local psi_next = psi + omega * dt
--     return x_next, y_next, math_helpers.wrap_pi(psi_next)
-- end


-- ---------------------------------------------------------
-- Minimum turn radius (rho)
-- rho = Vt^2 / (g * tan(phi_max))
-- ---------------------------------------------------------
local function min_turn_radius(Vt, phi_max, g)
    return (Vt * Vt) / (g * math.tan(phi_max))
end

-- ---------------------------------------------------------
-- generate circle centres - consistent for intial and final circles
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

-- ---------------------------------------------------------
-- LSR Geometry
-- theta = eta + gamma - pi/2
-- eta   = pi/2 + atan2(yRf - yLi, xRf - xLi)
-- l = distance(CL_i, CR_f) (i.e. distance between centroids)
--   straight_length = sqrt(l^2 - 4*rho^2)
--  gamma = acos( clamp(2*rho / l, -1, 1) )
-- ---------------------------------------------------------
local function lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho)
    local l = math_helpers.dist2d(xLi, yLi, xRf, yRf)
    if l < 2.0 * rho then 
        return nil, nil 
    end
    local straight_length = math.sqrt(math.max(0.0, l * l - 4.0 * rho * rho))
    local eta = (PI / 2.0) + math.atan(yRf - yLi, xRf - xLi)
    local gamma = math.acos(math_helpers.clamp((2.0 * rho) / straight_length, -1.0, 1.0))
    --local gamma = math.acos(math_helpers.clamp((2.0 * rho) / l, -1.0, 1.0))
    -- math in paper
    local theta = eta + gamma - (PI / 2.0)
    -- appears to work better
    --local theta = gamma - eta + (PI / 2.0)
    return theta, straight_length
end

-- ---------------------------------------------------------
-- LSL Geometry
-- theta = pi/2 - atan2(yLf - yLi, xLf - xLi)
-- straight_length = sqrt((xLf - xLi)^2 + (yLf - yLi)^2)
-- no gamma because of external tangents
-- ---------------------------------------------------------
local function lsl_theta_and_distance(xLi, yLi, xLf, yLf, rho)
    local dx = xLf - xLi
    local dy = yLf - yLi
    local straight_length = math.sqrt(dx * dx + dy * dy)
    local theta = (PI / 2.0) - math.atan(dy, dx)
    return theta, straight_length
end

-- ---------------------------------------------------------
-- RSL Geometry
-- theta = eta - gamma + pi/2
-- eta   = pi/2 - atan2(yLf - yRi, xLf - xRi)
-- gamma = atan(2*rho / straight_length)
-- l = distance(CR_i, CL_f)
-- straight_length = sqrt(l^2 - 4*rho^2)
-- ---------------------------------------------------------

local function rsl_theta_and_distance(xRi, yRi, xLf, yLf, rho)
    local l = math_helpers.dist2d(xRi, yRi, xLf, yLf)
    if l < 2.0 * rho then 
        return nil, nil 
    end
    local straight_length = math.sqrt(math.max(0.0, l * l - 4.0 * rho * rho))
    local eta = (PI / 2.0) - math.atan(yLf - yRi, xLf - xRi)
    local gamma = math.acos(math_helpers.clamp((2.0 * rho) / l, -1.0, 1.0))
    --local gamma = math.atan(math_helpers.clamp((2.0 * rho) / straight_length, -1.0, 1.0))
    local theta = eta - gamma + (PI/2.0)
    return theta, straight_length
end

-- ---------------------------------------------------------
-- RSR Geometry
-- theta = pi/2 - atan2(yRf - yRi, xRf - xRi)
-- straight_length = sqrt((xRf - xRi)^2 + (yRf - yRi)^2)
-- no gamma because of external tangents 
-- ---------------------------------------------------------
local function rsr_theta_and_distance(xRi, yRi, xRf, yRf, rho)
    local dx = xRf - xRi
    local dy = yRf - yRi
    local straight_length = math.sqrt(dx * dx + dy * dy)
    local theta = (PI / 2.0) - math.atan(dy, dx)
    return theta, straight_length
end

-- ---------------------------------------------------------
-- RLR Geometry
-- TBA
-- no gamma because of external tangents 
-- ---------------------------------------------------------
local function rlr_theta_and_distance(xRi, yRi, xRf, yRf, rho)
    local dx = xRf - xRi
    local dy = yRf - yRi
    local straight_length = math.sqrt(dx * dx + dy * dy)
    local theta = (PI / 2.0) - math.atan(dy, dx)
    return theta, straight_length
end

-- ---------------------------------------------------------
-- LRL Geometry
-- TBA
-- no gamma because of external tangents 
-- ---------------------------------------------------------
local function lrl_theta_and_distance(xRi, yRi, xRf, yRf, rho)
    local dx = xRf - xRi
    local dy = yRf - yRi
    local straight_length = math.sqrt(dx * dx + dy * dy)
    local theta = (PI / 2.0) - math.atan(dy, dx)
    return theta, straight_length
end


-- ---------------------------------------------------------
-- Compute the angular sweep (radians) traversed by generate_arc_points
-- with the same normalisation logic used there.
-- ---------------------------------------------------------
local function arc_sweep_rad(psi_start, psi_end, increasing)
    if increasing then
        if psi_end < psi_start then psi_end = psi_end + 2 * PI end
        return psi_end - psi_start
    else
        if psi_end > psi_start then psi_end = psi_end - 2 * PI end
        return psi_start - psi_end
    end
end

-- ---------------------------------------------------------
-- Arc point generation of the nth point
-- pn = xc + rho*sin(psi_n), yc + rho*cos(psi_n)
-- Used by the paper for both right and left arc point updates
-- Usable for intial and final values
-- x is east
-- y is north
-- ---------------------------------------------------------
local function arc_point(xc, yc, rho, psi_n)
    local x = xc + rho * math.sin(psi_n)
    local y = yc + rho * math.cos(psi_n)
    return x, y
end


-- ---------------------------------------------------------
-- Straight segment point generation of the nth point
-- x_n = x_(n-1) + delta_d * sin(theta)
-- y_n = y_(n-1) + delta_d * cos(theta)
-- ---------------------------------------------------------
local function straight_step(x_prev, y_prev, theta, delta_d)
    local x = x_prev + delta_d * math.sin(theta)
    local y = y_prev + delta_d * math.cos(theta)
    return x, y
end


-- ---------------------------------------------------------
-- Section 4: generating curves
-- ---------------------------------------------------------

local function generate_arc_points(points, xc, yc, rho, psi_start, psi_end, delta_psi, increasing)
    local psi = psi_start
    if increasing then
        -- normalise so psi_end >= psi_start (left/CCW arc); cap at one full circle
        if psi_end < psi_start then
            psi_end = psi_end + 2 * PI
        end
        if psi_end - psi_start > 2 * PI then
            psi_end = psi_start + 2 * PI
        end
        while psi <= psi_end + 1e-9 do
            local x, y = arc_point(xc, yc, rho, psi)
            points[#points + 1] = {x = x, y = y, psi = psi}
            psi = psi + delta_psi
        end
    else
        -- normalise so psi_end <= psi_start (right/CW arc); cap at one full circle
        if psi_end > psi_start then
            psi_end = psi_end - 2 * PI
        end
        if psi_start - psi_end > 2 * PI then
            psi_end = psi_start - 2 * PI
        end
        while psi >= psi_end - 1e-9 do
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
    local LSR_points = {}
    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    local xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    local theta, straight_len = lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho)
    if theta == nil then return nil, math.huge end
    -- Generate Left
    --generate_arc_points(LSR_points, xLi, yLi, rho, psi_i, theta, delta_psi, true)
    generate_arc_points(LSR_points, xLi, yLi, rho,  psi_i + PI/2,  theta + PI/2,  delta_psi, false)

    if #LSR_points == 0 then
        return nil, math.huge
    end

    -- Straight
    local last = LSR_points[#LSR_points]
    generate_straight_points(LSR_points, last.x, last.y, theta, straight_len, delta_d)
    -- Generate right
    --generate_arc_points(LSR_points, xRf, yRf, rho, theta, psi_f, delta_psi, false)
    generate_arc_points(LSR_points, xRf, yRf, rho,  theta - PI/2,  psi_f - PI/2,  delta_psi, true)

    local sweep1 = arc_sweep_rad(psi_i + PI/2, theta + PI/2, false)
    local sweep2 = arc_sweep_rad(theta - PI/2, psi_f - PI/2, true)
    local total_length = rho * (sweep1 + sweep2) + straight_len
    return LSR_points, total_length
end

-- generate LSL
local function generate_LSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local LSL_points = {}
    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    local xLf, yLf = circle_center_left(xf, yf, psi_f, rho)
    local theta, straight_len = lsl_theta_and_distance(xLi, yLi, xLf, yLf, rho)
    -- Generate Left (first arc)
    generate_arc_points(LSL_points, xLi, yLi, rho, psi_i + PI/2, theta + PI/2, delta_psi, false)
    --generate_arc_points(LSL_points, xLi, yLi, rho, psi_i, theta, delta_psi, true)

    if #LSL_points == 0 then
        return nil, math.huge
    end
    -- Straight
    local last = LSL_points[#LSL_points]
    generate_straight_points(LSL_points, last.x, last.y, theta, straight_len, delta_d)
    -- Generate Left
     --   generate_arc_points(LSL_points, xLf, yLf, rho, theta, psi_f, delta_psi, true)

    generate_arc_points(LSL_points, xLf, yLf, rho, theta + PI/2, psi_f + PI/2, delta_psi, false)

    local sweep1 = arc_sweep_rad(psi_i + PI/2, theta + PI/2, false)
    local sweep2 = arc_sweep_rad(theta + PI/2, psi_f + PI/2, false)
    local total_length = rho * (sweep1 + sweep2) + straight_len
    return LSL_points, total_length
end

-- generate RSL
local function generate_RSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local RSL_oints = {}
    local xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    local xLf, yLf = circle_center_left(xf, yf, psi_f, rho)
    local theta, straight_len = rsl_theta_and_distance(xRi, yRi, xLf, yLf, rho)
    if theta == nil then return nil, math.huge end
    -- generate right
    generate_arc_points(RSL_oints, xRi, yRi, rho, psi_i - PI/2, theta - PI/2, delta_psi, true)
    if #RSL_oints == 0 then
        return nil, math.huge
    end
    -- Straight
    local last = RSL_oints[#RSL_oints]
    generate_straight_points(RSL_oints, last.x, last.y, theta, straight_len, delta_d)
    -- generate left
    generate_arc_points(RSL_oints, xLf, yLf, rho, theta + PI/2, psi_f + PI/2, delta_psi, false)

    local sweep1 = arc_sweep_rad(psi_i - PI/2, theta - PI/2, true)
    local sweep2 = arc_sweep_rad(theta + PI/2, psi_f + PI/2, false)
    local total_length = rho * (sweep1 + sweep2) + straight_len
    return RSL_oints, total_length
end

-- generate RSR loop
local function generate_RSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local RSR_points = {}
    local xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    local xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    local theta, straight_len = rsr_theta_and_distance(xRi, yRi, xRf, yRf, rho)
    -- generate right
    generate_arc_points(RSR_points, xRi, yRi, rho, psi_i- PI/2, theta- PI/2, delta_psi, true)
    if #RSR_points == 0 then
        return nil, math.huge
    end
    -- Straight
    local last = RSR_points[#RSR_points]
    generate_straight_points(RSR_points, last.x, last.y, theta, straight_len, delta_d)
    -- generate right
    generate_arc_points(RSR_points, xRf, yRf, rho, theta- PI/2, psi_f- PI/2, delta_psi, true)

    local sweep1 = arc_sweep_rad(psi_i - PI/2, theta - PI/2, true)
    local sweep2 = arc_sweep_rad(theta - PI/2, psi_f - PI/2, true)
    local total_length = rho * (sweep1 + sweep2) + straight_len
    return RSR_points, total_length
end


-- ---------------------------------------------------------
-- Section 5: Optimal paths
--- add in a measurement/optimsiation function
-- --------------------------------------------------------

local function optimal_path(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local optimal_points = {}
    -- generate each iteration of points
    local LSR_points, LSR_distance = generate_LSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local LSL_points, LSL_distance = generate_LSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local RSL_points, RSL_distance = generate_RSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local RSR_points, RSR_distance = generate_RSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)

    -- calculate distance - with guards if the value is a nil case
    --local LSR_distance = LSR_points and (#LSR_points * delta_d) or math.huge
    --local LSL_distance = LSL_points and (#LSL_points * delta_d) or math.huge
    --local RSL_distance = RSL_points and (#RSL_points * delta_d) or math.huge
    --local RSR_distance = RSR_points and (#RSR_points * delta_d) or math.huge

    -- candidate list: keeping all values together 
    local candidates = {
        {distance = LSR_distance, points = LSR_points, name = "LSR"},
        {distance = LSL_distance, points = LSL_points, name = "LSL"},
        {distance = RSL_distance, points = RSL_points, name = "RSL"},
        {distance = RSR_distance, points = RSR_points, name = "RSR"},
    }

    local min_val = nil

    -- looping through minimum valuables
    for _, candidate in ipairs(candidates) do
        if not min_val or candidate.distance < min_val.distance then
            min_val = candidate
        end
    end

    -- return the optimal values
    return min_val.distance, min_val.points, min_val.name
end


-- ---------------------------------------------------------
-- Section 6: Local to Absolute Values
-- convert local-frame points into absolute Location waypoints
-- altitude is intentionally not owned here; control.lua sets altitude policy
-- --------------------------------------------------------

local function to_absolute_points(origin_loc_abs, rel_points)
    if origin_loc_abs == nil or rel_points == nil or #rel_points == 0 then
        return nil
    end

    local abs_points = {}
    for i = 1, #rel_points do
        local rel = rel_points[i]
        if rel ~= nil and rel.x ~= nil and rel.y ~= nil then
            local wp = origin_loc_abs:copy()
            wp:offset(rel.y, rel.x)
            --wp:offset(rel.x, rel.y)
            abs_points[#abs_points + 1] = {
                loc = wp,
                psi = rel.psi,
                x = rel.x,
                y = rel.y
            }
        end
    end

    if #abs_points == 0 then
        return nil
    end

    return abs_points
end


-- ---------------------------------------------------------
-- Section 7: Build Path
-- get the target location from the kangaroo bus (consumed in control.lua)
-- --------------------------------------------------------

local function build_path(kangaroo_state)
    -- handling 0 case
    if kangaroo_state == nil or kangaroo_state.loc == nil then
        return nil, "invalid_kangaroo_state"
    end
    -- current position of plane
    local pos = ahrs:get_position()
    local vel = ahrs:get_velocity_NED()
    local yaw = ahrs:get_yaw_rad()
    if pos == nil or vel == nil or yaw == nil then
        return nil, "missing_aircraft_state"
    end

    local pos_abs = pos:copy()
    local target_abs = kangaroo_state.loc:copy()
    pos_abs:change_alt_frame(ALT_FRAME_ABSOLUTE)

    -- distance from kangaroo
    local rel_ne = pos_abs:get_distance_NE(target_abs)

    -- ArduPilot: rel_ne:x() = North metres,  rel_ne:y() = East metres
    if rel_ne == nil then
        return nil, "rel_ne_unavailable"
    end

    -- velocity
    local vt = math.sqrt(vel:x() * vel:x() + vel:y() * vel:y())
    local rho = min_turn_radius(math.max(vt, 1.0), PHI_MAX_RAD, grav)

    -- updated math on psi_f
    local vn_f = kangaroo_state.vn or 0
    local ve_f = kangaroo_state.ve or 0
    local speed_f = math.sqrt(vn_f * vn_f + ve_f * ve_f)
    local psi_f

    -- safety 
    if speed_f > 0.5 then   -- only trust heading if target is actually moving
        psi_f = math.atan(ve_f, vn_f)
    else
        -- fall back: aim the terminal heading toward the kangaroo from the plane
        psi_f = math.atan(rel_ne:y(), rel_ne:x())
    end

    -- tune these points
    --- optimal path
    --local function optimal_path(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local _, rel_points, path_type = optimal_path(0.0, 0.0, yaw, rel_ne:y(), rel_ne:x(), psi_f, rho, math.rad(15), 50.0)

    if rel_points == nil or #rel_points == 0 then
        return nil, "empty_relative_path"
    end

    -- converting relative values to absolute points
    local abs_points = to_absolute_points(pos_abs, rel_points)

    -- nil case
    if abs_points == nil or #abs_points == 0 then
        return nil, "absolute_conversion_failed"
    end

    -- otherwise return
    return abs_points, {
        rho_m = rho,
        target_distance_m = math.sqrt(rel_ne:x() * rel_ne:x() + rel_ne:y() * rel_ne:y()),
        point_count = #abs_points,
        frame = "absolute",
        path_type = path_type
    }
end

return {
    build_path = build_path
}
