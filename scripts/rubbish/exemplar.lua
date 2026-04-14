-- =========================================================
-- dubins.lua
-- 2D Dubins path generation based on the attached PDF
-- Supports:
--   generate_RSR
--   generate_RSL
--   generate_LSL
--   generate_LSR
--   select_shortest_path
--
-- Angles in radians
-- Coordinates in local 2D plane
-- =========================================================

local Dubins = {}

local PI = math.pi

-- ---------------------------------------------------------
-- Helpers
-- ---------------------------------------------------------
local function clamp(x, lo, hi)
    if x < lo then return lo end
    if x > hi then return hi end
    return x
end

local function dist2d(x1, y1, x2, y2)
    local dx = x2 - x1
    local dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)
end

local function wrap_2pi(a)
    while a < 0.0 do a = a + 2.0 * PI end
    while a >= 2.0 * PI do a = a - 2.0 * PI end
    return a
end

local function mod2pi(a)
    return wrap_2pi(a)
end

-- CCW positive angle from 'from' to 'to'
local function delta_ccw(from, to)
    return mod2pi(to - from)
end

-- CW positive angle from 'from' to 'to'
local function delta_cw(from, to)
    return mod2pi(from - to)
end

local function heading_from_center_right(x, y, xc, yc)
    -- From paper arc form:
    -- x = xc + rho sin(psi), y = yc + rho cos(psi)
    return math.atan(x - xc, y - yc)
end

local function heading_from_center_left(x, y, xc, yc)
    return math.atan(x - xc, y - yc)
end

local function circle_center_right(x, y, psi, rho)
    return x + rho * math.cos(psi), y - rho * math.sin(psi)
end

local function circle_center_left(x, y, psi, rho)
    return x - rho * math.cos(psi), y + rho * math.sin(psi)
end

local function arc_point(xc, yc, rho, psi)
    return xc + rho * math.sin(psi), yc + rho * math.cos(psi)
end

local function straight_step(x, y, theta, ds)
    return x + ds * math.sin(theta), y + ds * math.cos(theta)
end

local function append_arc(points, xc, yc, rho, psi_start, psi_end, dpsi, turn)
    -- turn = "R" or "L"
    local travel
    if turn == "R" then
        travel = delta_cw(psi_start, psi_end)
        local steps = math.max(1, math.ceil(travel / dpsi))
        for i = 0, steps do
            local frac = i / steps
            local psi = psi_start - frac * travel
            psi = mod2pi(psi)
            local x, y = arc_point(xc, yc, rho, psi)
            points[#points + 1] = {x = x, y = y, psi = psi, segment = "arc_" .. turn}
        end
    else
        travel = delta_ccw(psi_start, psi_end)
        local steps = math.max(1, math.ceil(travel / dpsi))
        for i = 0, steps do
            local frac = i / steps
            local psi = psi_start + frac * travel
            psi = mod2pi(psi)
            local x, y = arc_point(xc, yc, rho, psi)
            points[#points + 1] = {x = x, y = y, psi = psi, segment = "arc_" .. turn}
        end
    end
    return travel * rho
end

local function append_straight(points, x0, y0, theta, length, ds)
    local steps = math.max(1, math.ceil(length / ds))
    local x, y = x0, y0
    for i = 1, steps do
        local step = math.min(ds, length - (i - 1) * ds)
        x, y = straight_step(x, y, theta, step)
        points[#points + 1] = {x = x, y = y, psi = theta, segment = "straight"}
    end
    return x, y
end

local function build_result(path_type, points, total_length, meta)
    return {
        path_type = path_type,
        points = points,
        total_length = total_length,
        meta = meta
    }
end

-- ---------------------------------------------------------
-- RSR
-- Uses same-turn external tangent geometry from the paper:
-- theta = pi/2 - atan2(yRf-yRi, xRf-xRi)
-- d = distance(CRi, CRf)
-- ---------------------------------------------------------
function Dubins.generate_RSR(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    local xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    local xRf, yRf = circle_center_right(xf, yf, psi_f, rho)

    local theta = (PI / 2.0) - math.atan(yRf - yRi, xRf - xRi)
    theta = mod2pi(theta)

    local d = dist2d(xRi, yRi, xRf, yRf)

    local x_t1 = xRi + rho * math.sin(theta)
    local y_t1 = yRi + rho * math.cos(theta)
    local x_t2 = xRf + rho * math.sin(theta)
    local y_t2 = yRf + rho * math.cos(theta)

    local points = {}
    local arc1 = append_arc(points, xRi, yRi, rho, psi_i, theta, dpsi, "R")
    local x_end, y_end = append_straight(points, x_t1, y_t1, theta, d, ds)
    local arc2 = append_arc(points, xRf, yRf, rho, theta, psi_f, dpsi, "R")

    return build_result("RSR", points, arc1 + d + arc2, {
        theta = theta,
        straight_length = d,
        centers = {
            CRi = {x = xRi, y = yRi},
            CRf = {x = xRf, y = yRf}
        },
        tangent_start = {x = x_t1, y = y_t1},
        tangent_end   = {x = x_t2, y = y_t2},
        final_sample = {x = x_end, y = y_end}
    })
end

-- ---------------------------------------------------------
-- LSL
-- Same-turn external tangent geometry from the paper:
-- theta = pi/2 - atan2(yLf-yLi, xLf-xLi)
-- d = distance(CLi, CLf)
-- ---------------------------------------------------------
function Dubins.generate_LSL(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    local xLf, yLf = circle_center_left(xf, yf, psi_f, rho)

    local theta = (PI / 2.0) - math.atan(yLf - yLi, xLf - xLi)
    theta = mod2pi(theta)

    local d = dist2d(xLi, yLi, xLf, yLf)

    local x_t1 = xLi + rho * math.sin(theta)
    local y_t1 = yLi + rho * math.cos(theta)
    local x_t2 = xLf + rho * math.sin(theta)
    local y_t2 = yLf + rho * math.cos(theta)

    local points = {}
    local arc1 = append_arc(points, xLi, yLi, rho, psi_i, theta, dpsi, "L")
    local x_end, y_end = append_straight(points, x_t1, y_t1, theta, d, ds)
    local arc2 = append_arc(points, xLf, yLf, rho, theta, psi_f, dpsi, "L")

    return build_result("LSL", points, arc1 + d + arc2, {
        theta = theta,
        straight_length = d,
        centers = {
            CLi = {x = xLi, y = yLi},
            CLf = {x = xLf, y = yLf}
        },
        tangent_start = {x = x_t1, y = y_t1},
        tangent_end   = {x = x_t2, y = y_t2},
        final_sample = {x = x_end, y = y_end}
    })
end

-- ---------------------------------------------------------
-- RSL
-- Opposite-turn internal tangent using paper geometry:
-- eta = pi/2 - atan2(yLf-yRi, xLf-xRi)
-- gamma = atan(2*rho / d_straight)
-- theta = eta - gamma + pi/2
-- d_straight = sqrt(l^2 - 4*rho^2), l = distance(CRi, CLf)
-- ---------------------------------------------------------
function Dubins.generate_RSL(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    local xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    local xLf, yLf = circle_center_left(xf, yf, psi_f, rho)

    local l = dist2d(xRi, yRi, xLf, yLf)
    if l < 2.0 * rho then
        return nil, "RSL infeasible: circle centers too close"
    end

    local d = math.sqrt(math.max(0.0, l * l - 4.0 * rho * rho))
    local eta = (PI / 2.0) - math.atan(yLf - yRi, xLf - xRi)
    local gamma = math.atan(2.0 * rho, d)
    local theta = eta - gamma + (PI / 2.0)
    theta = mod2pi(theta)

    local x_t1 = xRi + rho * math.sin(theta)
    local y_t1 = yRi + rho * math.cos(theta)
    local x_t2 = xLf + rho * math.sin(theta)
    local y_t2 = yLf + rho * math.cos(theta)

    local points = {}
    local arc1 = append_arc(points, xRi, yRi, rho, psi_i, theta, dpsi, "R")
    local x_end, y_end = append_straight(points, x_t1, y_t1, theta, d, ds)
    local arc2 = append_arc(points, xLf, yLf, rho, theta, psi_f, dpsi, "L")

    return build_result("RSL", points, arc1 + d + arc2, {
        theta = theta,
        eta = eta,
        gamma = gamma,
        straight_length = d,
        center_distance = l,
        centers = {
            CRi = {x = xRi, y = yRi},
            CLf = {x = xLf, y = yLf}
        },
        tangent_start = {x = x_t1, y = y_t1},
        tangent_end   = {x = x_t2, y = y_t2},
        final_sample = {x = x_end, y = y_end}
    })
end

-- ---------------------------------------------------------
-- LSR
-- Opposite-turn internal tangent using paper geometry:
-- eta = pi/2 + atan2(yRf-yLi, xRf-xLi)
-- gamma = acos(2*rho / l)
-- theta = eta + gamma - pi/2
-- d = sqrt(l^2 - 4*rho^2), l = distance(CLi, CRf)
-- ---------------------------------------------------------
function Dubins.generate_LSR(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    local xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    local xRf, yRf = circle_center_right(xf, yf, psi_f, rho)

    local l = dist2d(xLi, yLi, xRf, yRf)
    if l < 2.0 * rho then
        return nil, "LSR infeasible: circle centers too close"
    end

    local d = math.sqrt(math.max(0.0, l * l - 4.0 * rho * rho))
    local eta = (PI / 2.0) + math.atan(yRf - yLi, xRf - xLi)
    local gamma = math.acos(clamp((2.0 * rho) / l, -1.0, 1.0))
    local theta = eta + gamma - (PI / 2.0)
    theta = mod2pi(theta)

    local x_t1 = xLi + rho * math.sin(theta)
    local y_t1 = yLi + rho * math.cos(theta)
    local x_t2 = xRf + rho * math.sin(theta)
    local y_t2 = yRf + rho * math.cos(theta)

    local points = {}
    local arc1 = append_arc(points, xLi, yLi, rho, psi_i, theta, dpsi, "L")
    local x_end, y_end = append_straight(points, x_t1, y_t1, theta, d, ds)
    local arc2 = append_arc(points, xRf, yRf, rho, theta, psi_f, dpsi, "R")

    return build_result("LSR", points, arc1 + d + arc2, {
        theta = theta,
        eta = eta,
        gamma = gamma,
        straight_length = d,
        center_distance = l,
        centers = {
            CLi = {x = xLi, y = yLi},
            CRf = {x = xRf, y = yRf}
        },
        tangent_start = {x = x_t1, y = y_t1},
        tangent_end   = {x = x_t2, y = y_t2},
        final_sample = {x = x_end, y = y_end}
    })
end

-- ---------------------------------------------------------
-- Select shortest feasible path
-- Returns best_result, all_results
-- ---------------------------------------------------------
function Dubins.select_shortest_path(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    local results = {}

    local rsr = Dubins.generate_RSR(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    if rsr then results[#results + 1] = rsr end

    local rsl = Dubins.generate_RSL(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    if rsl then results[#results + 1] = rsl end

    local lsl = Dubins.generate_LSL(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    if lsl then results[#results + 1] = lsl end

    local lsr = Dubins.generate_LSR(xi, yi, psi_i, xf, yf, psi_f, rho, dpsi, ds)
    if lsr then results[#results + 1] = lsr end

    if #results == 0 then
        return nil, {}
    end

    local best = results[1]
    for i = 2, #results do
        if results[i].total_length < best.total_length then
            best = results[i]
        end
    end

    return best, results
end

return Dubins