-- =========================================================
--  harness_cs_orbit -- the CS target-circle approach and its orbit continuation
--  created 3 September 2026
--  algorithm ported to Lua - verified no structural change to logic
-- ported from the baseline algorithm CS-orbit:
--          py_harness/geometry/dubins_target_circle.py and
--          dubins_target_orbit.py , 
--          with the ring pre-compensation through tharness orbit
--
--  What the construction is
--  ------------------------
--  Standard Dubins puts the FINAL turn circle beside a terminal POSE, at radius
--  rho. This replaces it with a circle centred on the TARGET, of radius R -- the
--  standoff ring -- so the aircraft arrives TANGENT TO THE RING rather than
--  flying at the target point. 

-- Per increment the algorithm:
--    1. picks the aircraft's initial turn circle C1 (left or right, radius rho),
--       and handles the left and right of the orbit
--                      (i.e. sense s1 = +1 right / -1 left).
--    2. picks the ring sense s2 (which tangent of the ring the path meets).
--    3. solve the common tangent between C1 (rho) and the ring (R):
--           sin(theta - phi) = (rho*s1 - R*s2) / D,   
--           theta = phi + asin(k)
--       where O1 is C1's centre, D = |T - O1| and phi the heading O1 -> T. That
--       is the external tangent when the senses match and the internal tangent
--       when they oppose -- one formula covering both, which is why there is no
--       branch here
--    4. emit the C1 arc then the straight, ending at the tangent point. There
--       is no terminal arc and no psi_f: it is a CS path, not CSC, and the orbit
--       continues from the tangency point.
--
--  All four (s1, s2) pairs are tried and the least TURN-IN cost (rho*sweep + L)
--  wins. The open-ended orbit is deliberately not scored, so it cannot bias the
--  choice. This ranking is geometric sense selection inside one plan; it is not
--  a cost-based selection.
--
--  Constraints
--  -------------------------
--  R < rho is refused (the orbit would exceed the curvature bounds), 
--  as is an aircraft at the ring centre, as is a start inside the ring.
--  Each returns nil plus a reason -- the Lua equivalent of the harness raising,
--  per the interface contract's transliteration note. A refusal that did not
--  transfer would be a safety divergence, not a numeric one, so the differential
--  tests require each to trigger identically.

-- Frame of reference
--  Frame x = East, y = North, psi from North clockwise. 
--  Geoemtry is stateless.
-- =========================================================
-- Initialising values

local dubins = require("harness_dubins")
local orbit = require("harness_orbit")

-- initialising dictionary
local M = {}

-- importing pi
local PI = math.pi

-- ---------------------------------------------------------
-- Part 1: Generating arrival direction onto tangent pairs
-- ---------------------------------------------------------
--- Build the Circle-Straight path for one sense pair.
--  Returns a table {points, reach, direction, arrival_x, arrival_y, s1, s2}
--  OR nil when this pair has no tangent.
--
--  sampling (optional, 10 October 2026; heap): nil samples the whole path as
--  before. false returns the candidate WITHOUT points (reach, direction and
--  arrival are closed form, so a candidate can be scored without sampling
--  it; sense_costs does this and shortest_path samples only the winner). A
--  table {fine_m, coarse_m} keeps a straight's points within fine_m of the
--  path's start and every coarse_m beyond (dubins generate_straight_points:
--  the same points, fewer of them stored); arcs are sampled by angle either
--  way.
-- determining reach
function M.reach_path(px, py, psi_i, tx, ty, R, rho, s1, s2, delta_psi, delta_d, sampling)
    -- The arc sweep moves a phase by at most one turn, so it is correct only
    -- for a heading in the harness's range, [-pi, pi). A heading outside it
    -- (the SITL scripts passed [0, 2 pi) until 24 September 2026) gave the
    -- right-turn candidate a negative sweep, a phantom turn-in cost and a
    -- path starting away from the aircraft. Only an
    -- out-of-range heading is wrapped, so every in-range heading is
    -- unchanged bit for bit. Mirrors dubins_target_circle._reach_path.
    if psi_i >= PI or psi_i < -PI then
        psi_i = (psi_i + PI) % (2.0 * PI) - PI
    end
    local o1x, o1y
    if s1 > 0 then
        o1x, o1y = dubins.circle_center_right(px, py, psi_i, rho)
    else
        o1x, o1y = dubins.circle_center_left(px, py, psi_i, rho)
    end

    local dx, dy = tx - o1x, ty - o1y
    local D = math.sqrt(dx * dx + dy * dy)
    if D < 1e-9 then
        return nil
    end
    -- Heading O1 -> T, from North CW: atan2(East, North) = atan(dx, dy).
    local phi = math.atan(dx, dy)
    local k = (rho * s1 - R * s2) / D
    -- Handling nil case
    if math.abs(k) > 1.0 then
        return nil                      
    end
    local off = math.asin(k)
    local theta = phi + off
    -- = D*sqrt(1 - k^2) >= 0
    local L = D * math.cos(off)         
    if L < -1e-9 then
        return nil
    end
    if L < 0.0 then
        L = 0.0
    end

    -- starboard(theta): the fixed perpendicular the radii are measured along.
    local nx, ny = math.cos(theta), -math.sin(theta)
    local arrival_x = tx - R * s2 * nx
    local arrival_y = ty - R * s2 * ny

    local points = (sampling ~= false) and {} or nil
    local start_ph, end_ph, inc
    -- right / clockwise initial arc
    if s1 > 0 then                      
        start_ph, end_ph, inc = psi_i - PI / 2.0, theta - PI / 2.0, true
    else                                
        -- left / counter-clockwise initial arc
        start_ph, end_ph, inc = psi_i + PI / 2.0, theta + PI / 2.0, false
    end
    local sweep1 = dubins.arc_sweep_rad(start_ph, end_ph, inc)
    if points ~= nil then
        -- generate arc
        dubins.generate_arc_points(points, o1x, o1y, rho, start_ph, end_ph, delta_psi, inc)
        local sx, sy = px, py
        if #points > 0 then
            sx, sy = points[#points].x, points[#points].y
        end
        if L > 1e-6 then
            local keep = nil
            if type(sampling) == "table" then
                -- fine within fine_m of where the path starts (the pose it
                -- was planned from), every coarse_m beyond
                keep = { x0 = px, y0 = py, fine_m = sampling.fine_m,
                         coarse_m = sampling.coarse_m }
            end
            dubins.generate_straight_points(points, sx, sy, theta, L, delta_d, keep)
        end
    end

    -- No terminal arc: the path flies into the tangent and ends on the ring.
    local direction = "ccw"
    -- else retun the clockwise
    if s2 > 0 then
        direction = "cw"
    end
    return {
        points = points,
        reach = rho * sweep1 + L,
        direction = direction,
        arrival_x = arrival_x,
        arrival_y = arrival_y,
        s1 = s1,
        s2 = s2,
    }
end

--- The sampling table for reach_path from spec.json (cfg.cs_fine_m and
--  cfg.cs_coarse_m, written for the flight by pv_plan.json), or nil (the
--  whole path every delta_d, as the Python harness and the campaigns).
function M.sampling(cfg)
    if cfg ~= nil and cfg.cs_fine_m ~= nil and cfg.cs_coarse_m ~= nil then
        return { fine_m = cfg.cs_fine_m, coarse_m = cfg.cs_coarse_m }
    end
    return nil
end

-- ---------------------------------------------------------
-- Part 2a: Calculating the lowest cost
-- ---------------------------------------------------------

--- The least turn-in cost CS path onto the ring.
--  Returns the same table shape as reach_path, or nil plus a reason.
--  preferred_direction / sense_margin_m (optional; orbit-sense hysteresis): hold the
--  previous tick's orbit sense unless the other is cheaper by more than the
--  margin. Both nil reproduces the argmin used before 14 September 2026 exactly.
--  sampling (optional): see reach_path. Only the chosen candidate is sampled
--  (10 October 2026): the four were sampled and three discarded every tick,
--  most of the script's heap. The choice uses the closed-form reach, so it is
--  unchanged, and the chosen path is the same points as before.
function M.shortest_path(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                         delta_psi, delta_d, preferred_direction, sense_margin_m,
                         sampling)
    -- handle curvature constraints
    if orbit_radius_m < turn_radius_m - 1e-9 then
        return nil, "target circle radius < minimum turn radius: the orbit " ..
                    "would exceed the curvature bound"
    end
    local dx, dy = px - tx, py - ty
    --inside the ring, the outward tangent may exist, but the approach will not have a tnagent
    if math.sqrt(dx * dx + dy * dy) < orbit_radius_m - 1e-6 then
        return nil, "aircraft is inside the target ring; no approach tangent"
    end

    -- see part 2b for sense_cost, choose_sense
    -- calculate cost
    local costs = M.sense_costs(px, py, psi_i, tx, ty, orbit_radius_m,
                                turn_radius_m, delta_psi, delta_d)
    -- choose best
    local best = M.choose_sense(costs, preferred_direction, sense_margin_m)
    if best == nil then
        return nil, "no target-circle tangent solves this configuration"
    end
    -- sample only the winner (the same pair, so the same path)
    return M.reach_path(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                        best.s1, best.s2, delta_psi, delta_d, sampling)
end

-- ---------------------------------------------------------
-- Part 2b: Hysterisis implementation ('Orbit sense')
-- ---------------------------------------------------------

--- The best candidate per orbit sense, and the baseline's argmin.
--  Returns {cw = cand|nil, ccw = cand|nil, argmin = cand|nil}. `argmin` is
--  the first-visited least-cost candidate over all four pairs, in the
--  (+1,+1), (+1,-1), (-1,+1), (-1,-1) order with strict-less replacement.
--  The candidates carry no points (reach_path sampling false): they are
--  scored on the closed-form reach only.

function M.sense_costs(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                       delta_psi, delta_d)
    local out = { cw = nil, ccw = nil, argmin = nil }
    for _, s1 in ipairs({ 1, -1 }) do
        for _, s2 in ipairs({ 1, -1 }) do
            local cand = M.reach_path(px, py, psi_i, tx, ty, orbit_radius_m,
                                      turn_radius_m, s1, s2, delta_psi, delta_d, false)
            if cand ~= nil then
                local key = cand.direction
                if out[key] == nil or cand.reach < out[key].reach then
                    out[key] = cand
                end
                if out.argmin == nil or cand.reach < out.argmin.reach then
                    out.argmin = cand
                end
            end
        end
    end
    return out
end

--- Pick the candidate, with optional hysteresis.
--  With no preferred_direction: the baseline argmin. With one ("cw"/"ccw"):
--  the held sense's candidate unless it does not exist or the other sense is
--  cheaper by MORE than sense_margin_m metres of turn-in cost. Returns the
--  candidate table, or nil when neither sense has one.

function M.choose_sense(costs, preferred_direction, sense_margin_m)
    local cw, ccw = costs.cw, costs.ccw
    if cw == nil and ccw == nil then
        return nil
    end
    if preferred_direction == "cw" or preferred_direction == "ccw" then
        local held = costs[preferred_direction]
        local other = ccw
        if preferred_direction == "ccw" then
            other = cw
        end
        if held == nil then
            return other
        end
        local margin = sense_margin_m or 0.0
        if other == nil or other.reach >= held.reach - margin then
            return held
        end
        return other
    end
    return costs.argmin
end

--- One guidance point a look-ahead along the shortest CS path (the approach
--  only, no orbit continuation). Returns a table, or nil plus a reason.

function M.approach_guidance(px, py, psi_i, tx, ty, orbit_radius_m,
                             turn_radius_m, look_ahead_m, delta_psi, delta_d,
                             preferred_direction, sense_margin_m, sampling)
    local path, reason = M.shortest_path(px, py, psi_i, tx, ty, orbit_radius_m,
                                         turn_radius_m, delta_psi, delta_d,
                                         preferred_direction, sense_margin_m,
                                         sampling)
    if path == nil then
        return nil, reason
    end
    local gx, gy = dubins.point_at_arc_length(path.points, look_ahead_m)
    return {
        gx = gx,
        gy = gy,
        direction = path.direction,
        reach_length_m = path.reach,
        curvature = 1.0 / orbit_radius_m,
        arrival_e = path.arrival_x,
        arrival_n = path.arrival_y,
    }
end

-- ---------------------------------------------------------
-- Part 4: Orbit after hitting the tangent
-- ---------------------------------------------------------

--- One guidance point: CS approach outside the ring, orbit continuation on it.
--  `phase` is "approach" (outside) or "orbit" (on/inside) -- a DISCRETE
--  geometric switch on d against R, never a blended ramp weight. It is
--  continuous without a ramp because the approach arrives tangent to the ring,
--  which is the property the orbit continuation relies on and the one a port could
--  most easily lose.
--
--  Returns a table {gx, gy, phase, direction, curvature, ring_angle_rad?} or
--  nil plus a reason.

function M.guidance(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                    look_ahead_m, delta_psi, delta_d, precompensate,
                    preferred_direction, sense_margin_m, sampling)
    local R = orbit_radius_m
    if R < turn_radius_m - 1e-9 then
        return nil, "target circle radius < minimum turn radius: the orbit " ..
                    "would exceed the curvature bound"
    end
    local dx, dy = px - tx, py - ty
    local d = math.sqrt(dx * dx + dy * dy)
    if d < 1e-6 then
        return nil, "aircraft is at the ring centre; no orbit angle"
    end

    if d > R then
        local g, reason = M.approach_guidance(px, py, psi_i, tx, ty, R,
                                              turn_radius_m, look_ahead_m,
                                              delta_psi, delta_d,
                                              preferred_direction, sense_margin_m,
                                              sampling)
        if g == nil then
            return nil, reason
        end
        return {
            gx = g.gx,
            gy = g.gy,
            phase = "approach",
            direction = g.direction,
            curvature = g.curvature,
        }
    end

    -- On / inside the ring: continue around it. Closed-form advance, so no
    -- sampling resolution enters the guidance output.
    local psi0 = orbit.entry_angle(px, py, tx, ty)
    local direction = orbit.orbit_direction(psi0, psi_i)
    local gx, gy, psi = orbit.orbit_guidance_point(tx, ty, R, psi0, direction,
                                                   look_ahead_m, precompensate)
    if gx == nil then
        return nil, gy               -- second return is the refusal reason
    end
    local sense = "ccw"
    if direction > 0 then
        sense = "cw"
    end
    return {
        gx = gx,
        gy = gy,
        phase = "orbit",
        direction = sense,
        curvature = 1.0 / R,
        ring_angle_rad = psi,
    }
end

return M
