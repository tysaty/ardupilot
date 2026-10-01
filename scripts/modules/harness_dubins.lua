-- =========================================================
--  harness_dubins -- Dubins primitives  
--  Ported 2026-09-03
--
--  Circle centres, arc sweep, arc- and straight-segment point generation, and
--  path arc-length sampling. 
--  source: py_harness/geometry/dubins.py and py_harness/geometry/orbit.py.
--
--  Relationship to modules/dubins_weave_full.lua - it is the old hanress that implements
-- the full six family Dubins generator that was presented in the mid-term and since superseded.
--  Its arc and straight generators are equivalent in intent, but they are entangled 
--  with the former controller's data, shapes and bookkepping, so resuing that code
--  would have coupled alive port to dead code and made a divergence attributable to either.
-- 
--  This code is generated from the harness instead, which is what the differential
--  test compares against. dubins_weave_full.lua is not referenced in this file.
--
--  Point representation - a path is a one-indexed array of {x, y, psi} tables, in
--  the geometry frame x = East, y = North, positive psi from North clockwise.
-- 
--
--  Stateless - the caller owns every path it is handed.
-- Read concurrent with harness_orbit.lua
-- =========================================================

local geom = require("harness_geom")

local M = {}

local PI = math.pi

-- ---------------------------------------------------------
-- Generate turn circles (starting circles)
-- ---------------------------------------------------------

--- Centre of the turn circle to the aircraft's right of radius rho.
function M.circle_center_right(x, y, psi, rho)
    return x + rho * math.cos(psi), y - rho * math.sin(psi)
end

--- Centre of the turn circle to the aircraft's left of radius rho.
function M.circle_center_left(x, y, psi, rho)
    return x - rho * math.cos(psi), y + rho * math.sin(psi)
end

-- Maximum turn radius
--- rho = V^2/(g tan phi); where phi_max is measured in radians
--  The harness fixes airspeed and derivevs the bank instead so nothing calls this

function M.min_turn_radius(v_t, phi_max, g)
    return (v_t * v_t) / (g * math.tan(phi_max))
end

-- ---------------------------------------------------------
-- Arc generation
-- ---------------------------------------------------------

-- Sweep angel (radians) for starting heading to required end heading
--- Swept angle of an arc from psi_start to psi_end, radians, always >= 0.
-- `increasing` selects the sense; the branch that adds or subtracts 2*pi is
--  what makes the sweep the one actually flown rather than its complement.
function M.arc_sweep_rad(psi_start, psi_end, increasing)
    -- if increasing then the 
    if increasing then
        if psi_end < psi_start then
            psi_end = psi_end + 2.0 * PI
        end
        return psi_end - psi_start
    end
    -- else it is increasing it hits the other side
    if psi_end > psi_start then
        psi_end = psi_end - 2.0 * PI
    end
    return psi_start - psi_end
end


--- One point on a circle at phase psi_n, in (x = East, y = North).
--  Note the sin/cos order: x uses sin and y uses cos (because psi is measured
--  from North clockwise under NE convention).
function M.arc_point(xc, yc, rho, psi_n)
    return xc + rho * math.sin(psi_n), yc + rho * math.cos(psi_n)
end

-- straight step
--- One straight step of delta_d along heading theta.
function M.straight_step(x_prev, y_prev, theta, delta_d)
    return x_prev + delta_d * math.sin(theta), y_prev + delta_d * math.cos(theta)
end


-- GENERATE ARC POINTS

-- Append the arc between the start heading angle and the final heading angle
--  Sampled every delta_psi (change in heading), then snapped to the actual endpoint
--  whole step fell short. The snap mattersl; without it the path ends up to one
--  sampling interval short of the actual tangent, and every downstream length
--  and guidance point inherits the error.
--
--  Faithful to the harness including its integer floor: `n_steps` is
--  floor(sweep / delta_psi). 
-- Lua 5.3 has an integer subtype and `//` is floor  division on floats, so `sweep // delta_psi` 
--  gives a float whose value is the floor -- matching Python's `//`. 

function M.generate_arc_points(points, xc, yc, rho, psi_start, psi_end,
                               delta_psi, increasing)
    -- if the heading change is 0 return nothing
    if math.abs(psi_end - psi_start) < 1e-9 then
        return
    end
    local sweep, sign

    if increasing then
-- if the heading angle is larger handle the sense as pos
        if psi_end < psi_start then
            psi_end = psi_end + 2.0 * PI
        end
        sweep = psi_end - psi_start
        sign = 1
    else
        -- if the heading angle is larger at the end, then sweep as a neg
        if psi_end > psi_start then
            psi_end = psi_end - 2.0 * PI
        end
        sweep = psi_start - psi_end
        sign = -1
    end
    -- if the sweep over rotates
    if sweep > 2.0 * PI then
        sweep = 2.0 * PI
    end
    local n_steps = math.floor(sweep / delta_psi)
    -- generate points
    for i = 1, n_steps do
        local psi = psi_start + sign * i * delta_psi
        local x, y = M.arc_point(xc, yc, rho, psi)
        points[#points + 1] = { x = x, y = y, psi = psi }
    end
    -- generate final points
    if sweep - n_steps * delta_psi > 1e-6 then
        local psi = psi_start + sign * sweep
        local x, y = M.arc_point(xc, yc, rho, psi)
        points[#points + 1] = { x = x, y = y, psi = psi }
    end
end


-- GENERATE STRAIGHT POINTS

-- Differnential test for port from python
--- Append a straight run of total_d along theta onto `points`.

--  The `while dsum <= total_d` bound is the harness's, reproduced
--  including its consequence: the last emitted point can overshoot total_d by
--  up to delta_d. That is as-built behaviour, not a defect to be corrected
--  during a port, a "fix" here would make the differential test pass while
--  hiding the real difference.

function M.generate_straight_points(points, x_start, y_start, theta, total_d,
                                    delta_d)
    local x, y = x_start, y_start
    local dsum = 0.0
    -- generate along the points along theta
    while dsum <= total_d do
        x, y = M.straight_step(x, y, theta, delta_d)
        points[#points + 1] = { x = x, y = y, psi = theta }
        -- regenerate new points
        dsum = dsum + delta_d
    end
    return x, y
end

-- ---------------------------------------------------------
-- Path arc length
-- ---------------------------------------------------------

--- Cumulative planar arc length along a path. Returns a 1-indexed array whose
--  first entry is 0 and whose i-th entry is the length up to points[i].
function M.cumulative_arc_length(points)
    -- count of arc length points
    local out = { 0.0 }
    for i = 2, #points do
        local seg = geom.dist2d(points[i - 1].x, points[i - 1].y,
                                points[i].x, points[i].y)
        out[i] = out[i - 1] + seg
    end
    return out
end

--- The point arc_length_m along a path, linearly interpolated, clamped to its
--  ends. Returns x, y.

--  Interpolating rather than snapping to the nearest sample is deliberate: the
--  guidance point would otherwise jump by the sampling interval every time the
--  bracketing pair changed, and that jump would appear as a dicontunity/plan change.
--
--  Returns nil for an empty path, which is the Lua equivalent of the harness
--  raising ValueError (the interface contract's transliteration note: raise
--  where a Lua port would return nil).

function M.point_at_arc_length(points, arc_length_m)
    local n = #points
    if n == 0 then
        return nil
    end
    -- next point
    if n == 1 then
        return points[1].x, points[1].y
    end
    local lengths = M.cumulative_arc_length(points)
    -- if the arc length is 0, generate the next point
    if arc_length_m <= 0.0 then
        return points[1].x, points[1].y
    end
    -- if the length is greater than the next point jump to the length
    if arc_length_m >= lengths[n] then
        return points[n].x, points[n].y
    end
    -- return the length interopolation
    for i = 2, n do
        if lengths[i] >= arc_length_m then
            local span = lengths[i] - lengths[i - 1]
            local frac = 0.0
            if span > 0.0 then
                frac = (arc_length_m - lengths[i - 1]) / span
            end
            local x0, y0 = points[i - 1].x, points[i - 1].y
            local x1, y1 = points[i].x, points[i].y
            return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)
        end
    end
    return points[n].x, points[n].y
end

--- Arc length of the point on `points` nearest to (px, py) metres.
--  Steers aricraft towards a carrot along the projected curve; turn-rate limited.
--
--  Plain loop with a clamped point-to-segment projection. Returns 0.0 for a
--  single-point path and nil for an empty one.

function M.progress_along(points, px, py)
    local n = #points
    if n == 0 then
        return nil
    end
    if n == 1 then
        return 0.0
    end
    local best_d2 = nil
    local best_s = 0.0
    local s = 0.0

    for i = 2, n do
        local x0, y0 = points[i - 1].x, points[i - 1].y
        local x1, y1 = points[i].x, points[i].y
        local vx, vy = x1 - x0, y1 - y0
        local seg = math.sqrt(vx * vx + vy * vy)
        if seg > 1e-12 then
            local t = ((px - x0) * vx + (py - y0) * vy) / (seg * seg)
            if t < 0.0 then t = 0.0 elseif t > 1.0 then t = 1.0 end
            local cx, cy = x0 + t * vx, y0 + t * vy
            local d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy)
            if best_d2 == nil or d2 < best_d2 then
                best_d2 = d2
                best_s = s + t * seg
            end
            s = s + seg
        end
    end
    return best_s
end

return M
