-- ---------------------------------------------------------
-- Dubins Path Implementation - 2 Dimensional

-- The core Dubins path generator, that returns the optimal
-- path between two points (the plane and the kangaroo).
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


local PI = math.pi

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
--  l = distance(CL_i, CR_f) (i.e. distance between centroids)
--  straight_length = sqrt(l^2 - 4*rho^2)
--  gamma = acos( clamp(2*rho / l, -1, 1) )
-- ---------------------------------------------------------
local function lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho)
    local l = math_helpers.dist2d(xLi, yLi, xRf, yRf)
    if l < 2.0 * rho then 
        return nil, nil 
    end
    local straight_length = math.sqrt(math.max(0.0, l * l - 4.0 * rho * rho))
    local phi = math.atan(yRf - yLi, xRf - xLi)
    local eta = (PI / 2.0) + phi
    -- commented out 24 May 
    local gamma = math.acos(math_helpers.clamp((2.0 * rho) / straight_length, -1.0, 1.0))
    --local gamma = math.acos(math_helpers.clamp((2.0 * rho) / l, -1.0, 1.0))


    --local phi = math.atan(yRf - yLi, xRf - xLi)
    --local theta = gamma - phi

    -- appears to work better
    -- changed back from 72/73
        -- math in paper
    local theta = eta + gamma - (PI / 2.0)
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
    local phi = math.atan(yLf - yRi, xLf - xRi)
    local eta = (PI / 2.0) - phi
    --local gamma = math.acos(math_helpers.clamp((2.0 * rho) / straight_length, -1.0, 1.0))
   --local gamma = math.atan(math_helpers.clamp((2.0 * rho) / straight_length, -1.0, 1.0))
    local gamma = math.atan((2.0 * rho) / straight_length)
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
-- RLR Geometry using Shkel/Lumelsky Eq. (11)


--   0 = North, positive = clockwise
-- Shkel/Lumelsky alpha:
--   0 = East, positive = counter-clockwise
local function lua_heading_to_paper_alpha(psi)
    return math_helpers.wrap_2pi((PI / 2.0) - psi)
end

-----------------------------------------------------------
-- Inputs use Lua heading convention:
--   psi = 0 North, positive clockwise
-- Internally converted to:
--   alpha = 0 East, positive counter-clockwise
-- ---------------------------------------------------------

local function rlr_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    local dx = xf - xi
    local dy = yf - yi
    local D = math.sqrt(dx * dx + dy * dy)
    local d = D / rho
    local theta_goal = (D > 1e-9) and math.atan(dy, dx) or 0.0
    local alpha = math_helpers.wrap_2pi(lua_heading_to_paper_alpha(psi_i) - theta_goal)
    local beta  = math_helpers.wrap_2pi(lua_heading_to_paper_alpha(psi_f) - theta_goal)

    local value = (6.0 - d*d + 2.0 * math.cos(alpha - beta)+ 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if value < -1.0 or value > 1.0 then
        return nil, nil, nil
    end
    local p = math_helpers.wrap_2pi(2.0 * PI - math.acos(math_helpers.clamp(value, -1.0, 1.0)))
    local atan_term = math.atan(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta))
    local t = math_helpers.wrap_2pi(alpha - atan_term + p / 2.0)
    local q = math_helpers.wrap_2pi(alpha - beta - t + p)

    return t, p, q
end


-- ---------------------------------------------------------
-- LRL Geometry using Shkel/Lumelsky Eq. (13)
-- Inputs use Lua heading convention:
--   psi = 0 North, positive clockwise
-- Internally converted to:
--   alpha = 0 East, positive counter-clockwise
-- ---------------------------------------------------------
local function lrl_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    local dx = xf - xi
    local dy = yf - yi
    local D = math.sqrt(dx * dx + dy * dy)
    local d = D / rho
    local theta_goal = (D > 1e-9) and math.atan(dy, dx) or 0.0
    local alpha = math_helpers.wrap_2pi(lua_heading_to_paper_alpha(psi_i) - theta_goal)
    local beta  = math_helpers.wrap_2pi(lua_heading_to_paper_alpha(psi_f) - theta_goal)
    -- sin(beta) - sin(alpha), opposite sign to RLR
    local value = (6.0 - d*d + 2.0 * math.cos(alpha - beta) + 2.0 * d * (math.sin(beta) - math.sin(alpha))) / 8.0
    if value < -1.0 or value > 1.0 then
        return nil, nil, nil
    end
    local p = math_helpers.wrap_2pi(2.0 * PI - math.acos(math_helpers.clamp(value, -1.0, 1.0)))
    local atan_term = math.atan(-math.cos(alpha) + math.cos(beta), d + math.sin(alpha) - math.sin(beta))
    local t = math_helpers.wrap_2pi(-alpha + atan_term + p / 2.0)
    local q = math_helpers.wrap_2pi(beta - alpha - t + p)
    return t, p, q
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
    -- Skip degenerate, zero sweep arcs
    -- Previosuly emitting one phantom point at psi_start that doesn't connect to the
    -- adjacent straight segment.
    if math.abs(psi_end - psi_start) < 1e-9 then
        return
    end

    -- Trust the caller's 'increasing' flag for direction; wrap psi_end to make the signed sweep have the right sign.
    -- Controlling pi (e.g. so it doesn't backtrack/U-turns), so don't shrink to "shortest signed".
    local sweep
    local sign
    -- increasing case for clockwise
    if increasing then
        if psi_end < psi_start then 
            psi_end = psi_end + 2 * PI 
        end
        sweep = psi_end - psi_start
        sign = 1
    -- else it's counterclockwise
    else
        if psi_end > psi_start then 
            psi_end = psi_end - 2 * PI 
        end
        sweep = psi_start - psi_end
        sign = -1
    end

    -- Cap sweep at one full circle to prevent runaway on noisy inputs
    if sweep > 2 * PI then 
        sweep = 2 * PI 
    end

    -- Steps of delta_psi, starting AT psi_start + sign*delta_psi
    local n_steps = math.floor(sweep / delta_psi)
    -- skip psi_start itself; it's already on the previous segment
    for i = 1, n_steps do
        local psi = psi_start + sign * i * delta_psi
        local x, y = arc_point(xc, yc, rho, psi)
        points[#points + 1] = {x = x, y = y, psi = psi}
    end
    -- Preivously not meeting end point
    -- Snap to exact, final endpoint if the last full step didn't reach it
    if sweep - n_steps * delta_psi > 1e-6 then
        local psi = psi_start + sign * sweep
        local x, y = arc_point(xc, yc, rho, psi)
        points[#points + 1] = {x = x, y = y, psi = psi}
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
    -- nil case for theta
    if theta == nil then 
        return nil, math.huge 
    end
    -- Generate Left
    --generate_arc_points(LSR_points, xLi, yLi, rho, psi_i, theta, delta_psi, true)
    generate_arc_points(LSR_points, xLi, yLi, rho,  psi_i + PI/2,  theta + PI/2,  delta_psi, false)

    -- handle degenartive points
    local sx, sy
    if #LSR_points > 0 then
        sx, sy = LSR_points[#LSR_points].x, LSR_points[#LSR_points].y
    else
        sx, sy = xi, yi
    end
    generate_straight_points(LSR_points, sx, sy, theta, straight_len, delta_d)
    -- if #LSR_points == 0 then
    --     return nil, math.huge
    -- end
    
    -- -- Straight
    -- local last = LSR_points[#LSR_points]
    
    -- generate_straight_points(LSR_points, last.x, last.y, theta, straight_len, delta_d)
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

    -- if #LSL_points == 0 then
    --     return nil, math.huge
    -- end
    -- -- Straight
    -- local last = LSL_points[#LSL_points]
    -- generate_straight_points(LSL_points, last.x, last.y, theta, straight_len, delta_d)

    -- hanlde colinear case
    local sx, sy
    if #LSL_points > 0 then
        sx, sy = LSL_points[#LSL_points].x, LSL_points[#LSL_points].y
    else
        sx, sy = xi, yi
    end
    -- generate straight line
    generate_straight_points(LSL_points, sx, sy, theta, straight_len, delta_d)

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



    -- if #RSL_oints == 0 then
    --     return nil, math.huge
    -- end
    -- -- Straight
    -- local last = RSL_oints[#RSL_oints]
    -- generate_straight_points(RSL_oints, last.x, last.y, theta, straight_len, delta_d)

    -- handle colinear case 
    local sx, sy
    if #RSL_oints > 0 then
        sx, sy = RSL_oints[#RSL_oints].x, RSL_oints[#RSL_oints].y
    else
        sx, sy = xi, yi
    end
    --- generate straight
    generate_straight_points(RSL_oints, sx, sy, theta, straight_len, delta_d)


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

    -- handle colinear case 
    local sx, sy
    if #RSR_points > 0 then
        sx, sy = RSR_points[#RSR_points].x, RSR_points[#RSR_points].y
    else
        sx, sy = xi, yi
    end
    --- generate straight
    generate_straight_points(RSR_points, sx, sy, theta, straight_len, delta_d)

    -- if #RSR_points == 0 then
    --     return nil, math.huge
    -- end
    -- -- Straight
    -- local last = RSR_points[#RSR_points]
    -- generate_straight_points(RSR_points, last.x, last.y, theta, straight_len, delta_d)
    -- generate right
    generate_arc_points(RSR_points, xRf, yRf, rho, theta- PI/2, psi_f- PI/2, delta_psi, true)

    local sweep1 = arc_sweep_rad(psi_i - PI/2, theta - PI/2, true)
    local sweep2 = arc_sweep_rad(theta - PI/2, psi_f - PI/2, true)
    local total_length = rho * (sweep1 + sweep2) + straight_len
    return RSR_points, total_length
end

-- generate RLR loop
local function generate_RLR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local RLR_points = {}

    local t, p, q = rlr_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    if t == nil then
        return nil, math.huge
    end

    local xRi, yRi = circle_center_right(xi, yi, psi_i, rho)

    -- First R arc
    local psi_1_end = psi_i + t
    generate_arc_points(RLR_points, xRi, yRi, rho, psi_i - PI/2, psi_1_end - PI/2, delta_psi, true)

    local x1, y1
    if #RLR_points > 0 then
        x1, y1 = RLR_points[#RLR_points].x, RLR_points[#RLR_points].y
    else
        x1, y1 = xi, yi
    end

    -- Middle L arc centre is to the left of heading psi_1_end
    local xLm, yLm = circle_center_left(x1, y1, psi_1_end, rho)

    local psi_2_end = psi_1_end - p
    generate_arc_points(RLR_points, xLm, yLm, rho, psi_1_end + PI/2, psi_2_end + PI/2, delta_psi, false)

    local x2, y2
    if #RLR_points > 0 then
        x2, y2 = RLR_points[#RLR_points].x, RLR_points[#RLR_points].y
    else
        x2, y2 = x1, y1
    end

    -- Final R arc centre is to the right of heading psi_2_end
    local xRf, yRf = circle_center_right(x2, y2, psi_2_end, rho)

    local psi_3_end = psi_2_end + q
    generate_arc_points(RLR_points, xRf, yRf, rho, psi_2_end - PI/2, psi_3_end - PI/2, delta_psi, true)

    local total_length = rho * (t + p + q)
    return RLR_points, total_length
end

-- generate LRL loop
local function generate_LRL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    local LRL_points = {}

    local t, p, q = lrl_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    if t == nil then
        return nil, math.huge
    end

    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)

    -- First L arc
    local psi_1_end = psi_i - t
    generate_arc_points(LRL_points, xLi, yLi, rho, psi_i + PI/2, psi_1_end + PI/2, delta_psi, false)

    local x1, y1
    if #LRL_points > 0 then
        x1, y1 = LRL_points[#LRL_points].x, LRL_points[#LRL_points].y
    else
        x1, y1 = xi, yi
    end

    -- Middle R arc centre is to the right of heading psi_1_end
    local xRm, yRm = circle_center_right(x1, y1, psi_1_end, rho)

    local psi_2_end = psi_1_end + p
    generate_arc_points(LRL_points, xRm, yRm, rho, psi_1_end - PI/2, psi_2_end - PI/2, delta_psi, true)

    local x2, y2
    if #LRL_points > 0 then
        x2, y2 = LRL_points[#LRL_points].x, LRL_points[#LRL_points].y
    else
        x2, y2 = x1, y1
    end

    -- Final L arc centre is to the left of heading psi_2_end
    local xLf, yLf = circle_center_left(x2, y2, psi_2_end, rho)

    local psi_3_end = psi_2_end - q
    generate_arc_points(LRL_points, xLf, yLf, rho, psi_2_end + PI/2, psi_3_end + PI/2, delta_psi, false)

    local total_length = rho * (t + p + q)
    return LRL_points, total_length
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
    -- additional points for LRL and RLR
    local RLR_points, RLR_distance = nil, math.huge
    local LRL_points, LRL_distance = nil, math.huge
    local D = math_helpers.dist2d(xi, yi, xf, yf)
    -- only calculate the LRL and RLR curves if it fits the criteria around distance between point and rho
    -- mathematical proof in Shkel and Lumelsky, 2001
    if D < 4.0 * rho then
        RLR_points, RLR_distance = generate_RLR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
        LRL_points, LRL_distance = generate_LRL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    end
    -- candidate list: keeping all values together 
    local candidates = {
        {distance = LSR_distance, points = LSR_points, name = "LSR"},
        {distance = LSL_distance, points = LSL_points, name = "LSL"},
        {distance = RSL_distance, points = RSL_points, name = "RSL"},
        {distance = RSR_distance, points = RSR_points, name = "RSR"},
        {distance = RLR_distance, points = RLR_points, name = "RLR"},
        {distance = LRL_distance, points = LRL_points, name = "LRL"},
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

-- curve onto orbit -- separate behaviour from the dubins sweep
-- takes the kanagaroo state (read from the bus) and the orbit direction (CW or CCW)
local function build_orbit_path(kangaroo_state, orbit_dir)
    -- read aircraft state
    if kangaroo_state == nil or kangaroo_state.loc == nil then
        return nil, "invalid_kangaroo_state"
    end
    -- current position of plane
    local pos = ahrs:get_position()
    local vel = ahrs:get_velocity_NED()
    local yaw = ahrs:get_yaw_rad()

    --nil case
    if pos == nil or vel == nil or yaw == nil then
        return nil, "missing_aircraft_state"
    end

    -- takes copy of predicted kangaroo state and changes frame to absolute reference
    local pos_abs = pos:copy()
    local target_abs = kangaroo_state.loc:copy()
    pos_abs:change_alt_frame(ALT_FRAME_ABSOLUTE)

    -- distance from kangaroo
    local rel_ne = pos_abs:get_distance_NE(target_abs)
    if rel_ne == nil then return nil, "rel_ne_unavailable" end
    -- north and east distance from kangaroo state

    -- Critical to impementation
    -- ArduPilot: rel_ne:x() = North metres,  rel_ne:y() = East metres
    -- kx/ky using rel_ne:y() and rel_ne:x() - flipped convention
    local kx, ky = rel_ne:y(), rel_ne:x()

    --  rho via min_turn_radius; velocity across two dimensions
    local vt = math.sqrt(vel:x() * vel:x() + vel:y() * vel:y())
    local rho = min_turn_radius(math.max(vt, 1.0), PHI_MAX_RAD, grav)

    -- distance to kangaroo; plane must be outside the orbit circle to compute tangent
    local d = math.sqrt(kx * kx + ky * ky)
    if d <= rho then 
        return nil, "inside_orbit_radius" 
    end

    -- tangent geometry
    -- from point (P) to kangaroo (k), extend a tangent (T) out by the given radius
    -- generates two values - beta and alpha
    -- beta value - bearing from plane to kangaroo
    local beta = math.atan(kx, ky)
    -- half angle of the tangent (i.e. the inner angle of the triangle KPT) 
    local alpha = math.asin(math.min(rho / d, 1.0))

    -- straight tangent length between the kangaroo and the tangent
    -- the plane will follow this ditance
    local d_t = math.sqrt(d * d - rho * rho)

    -- beta_tangent (Clockwise): subtract alpha from bearing to kangaroo
    local Beta_tangent_cw  = beta - alpha
    -- beta tangent (counter clockwise): add alpha
    local Beta_tangent_ccw = beta + alpha

    -- select direction: orbit_dir 1=CW, -1=CCW, 0=auto (least heading change)
    -- takes orbit direction param from control.lua
    local dir = orbit_dir
    -- create a figure for this one - for yaw (20 May)
    if dir == 0 or (dir ~= 1 and dir ~= -1) then
        local dh_right = math.abs(math_helpers.wrap_pi(Beta_tangent_cw  - yaw))
        local dh_left  = math.abs(math_helpers.wrap_pi(Beta_tangent_ccw - yaw))
        dir = (dh_right <= dh_left) and 1 or -1
    end
    local Beta_tangent = (dir == 1) and Beta_tangent_cw or Beta_tangent_ccw

    -- tangent point T on the orbit circle (plane at origin)
    local tx = d_t * math.sin(Beta_tangent)
    local ty = d_t * math.cos(Beta_tangent)

    -- bearing from orbit centre K to T — arc start angle on the orbit circle
    local psi_KT = math.atan(tx - kx, ty - ky)

    local rel_points = {}

    -- orbit has three parts
    -- Part 1: generate straight points from plane to tangent point T
    generate_straight_points(rel_points, 0.0, 0.0, Beta_tangent, d_t, 50.0)
    local orbit_start_idx = #rel_points + 1

    -- Part 2: generate the arc points for the half arc onto the circle
    local psi_arc_end = psi_KT + dir * PI
    generate_arc_points(rel_points, kx, ky, rho, psi_KT, psi_arc_end, math.rad(15), dir == 1)
    
    -- combining points
    local abs_points = to_absolute_points(pos_abs, rel_points)

    -- note Part 3: handled in control.lua (append_orbit_arc extends arc as plane progresses)

    -- returning for nil case
    if abs_points == nil or #abs_points == 0 then
        return nil, "absolute_conversion_failed"
    end

    -- else return points
    return abs_points, {
        rho             = rho,
        dir             = dir,
        psi_KT          = psi_KT,
        psi_arc_end     = psi_arc_end,
        orbit_start_idx = orbit_start_idx,
        point_count     = #abs_points,
    }
end



return {
    build_path = build_path,
    build_orbit_path = build_orbit_path
}
