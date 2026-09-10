-- =========================================================
--  harness_cs_orbit -- the CS target-circle approach and its orbit continuation
--  created 2026-09-03
--  TASK-006 Tranche 5 (forward port) -- THE TRANCHE THE PORT EXISTS FOR.
--
--  On its own this discharges VR-015's stated acceptance: "one algorithm ported
--  to Lua with no structural change to its logic".
--
--  Ported from py_harness/geometry/dubins_target_circle.py (TASK-024) and
--  dubins_target_orbit.py (TASK-025), with the ring pre-compensation of
--  TASK-027 reached through harness_orbit.
--
--  What the construction is
--  ------------------------
--  Standard Dubins puts the FINAL turn circle beside a terminal POSE, at radius
--  rho. This replaces it with a circle centred on the TARGET, of radius R -- the
--  standoff ring -- so the aircraft arrives TANGENT TO THE RING rather than
--  flying at the target point. Per tick:
--
--    1. pick the aircraft's initial turn circle C1 (left or right, radius rho),
--       sense s1 = +1 right / -1 left;
--    2. pick the ring sense s2 (which tangent of the ring the path meets);
--    3. solve the common tangent between C1 (rho) and the ring (R):
--
--           sin(theta - phi) = (rho*s1 - R*s2) / D,   theta = phi + asin(k)
--
--       where O1 is C1's centre, D = |T - O1| and phi the heading O1 -> T. That
--       is the EXTERNAL tangent when the senses match and the INTERNAL tangent
--       when they oppose -- one formula covering both, which is why there is no
--       branch here;
--    4. emit the C1 arc then the straight, ENDING AT THE TANGENCY POINT. There
--       is no terminal arc and no psi_f: it is a CS path, not CSC, and the orbit
--       continues from the tangency point.
--
--  All four (s1, s2) pairs are tried and the least TURN-IN cost (rho*sweep + L)
--  wins. The open-ended orbit is deliberately not scored, so it cannot bias the
--  choice. This ranking is geometric sense selection INSIDE one plan; it is not
--  the cost-based selection among competing plans that ADR-001 superseded.
--
--  The refusals carry across
--  -------------------------
--  R < rho is refused (the orbit would exceed the curvature bound, FR-005 /
--  SR-002), as is an aircraft at the ring centre, as is a start inside the ring.
--  Each returns nil plus a reason -- the Lua equivalent of the harness raising,
--  per the interface contract's transliteration note. A refusal that did not
--  transfer would be a SAFETY divergence, not a numeric one, so the Tranche 5
--  gate requires each to trigger identically.
--
--  Frame x = East, y = North, psi from North clockwise (IR-008). Stateless.
-- =========================================================

local geom = require("harness_geom")
local dubins = require("harness_dubins")
local orbit = require("harness_orbit")

local M = {}

local PI = math.pi

-- ---------------------------------------------------------
-- One (s1, s2) candidate
-- ---------------------------------------------------------

--- Build the CS path for one sense pair.
--  Returns a table {points, reach, direction, arrival_x, arrival_y, sweep, L}
--  or nil when this pair has no tangent.
function M.reach_path(px, py, psi_i, tx, ty, R, rho, s1, s2, delta_psi, delta_d)
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
    -- Heading O1 -> T, from North clockwise: atan2(East, North) = atan(dx, dy).
    local phi = math.atan(dx, dy)
    local k = (rho * s1 - R * s2) / D
    if math.abs(k) > 1.0 then
        return nil                      -- no such tangent for this sense pair
    end
    local off = math.asin(k)
    local theta = phi + off
    local L = D * math.cos(off)         -- = D*sqrt(1 - k^2) >= 0
    if L < -1e-9 then
        return nil
    end
    if L < 0.0 then
        L = 0.0
    end

    -- Starboard(theta): the fixed perpendicular the radii are measured along.
    local nx, ny = math.cos(theta), -math.sin(theta)
    local arrival_x = tx - R * s2 * nx
    local arrival_y = ty - R * s2 * ny

    local points = {}
    local start_ph, end_ph, inc
    if s1 > 0 then                      -- right / clockwise initial arc
        start_ph, end_ph, inc = psi_i - PI / 2.0, theta - PI / 2.0, true
    else                                -- left / counter-clockwise initial arc
        start_ph, end_ph, inc = psi_i + PI / 2.0, theta + PI / 2.0, false
    end
    dubins.generate_arc_points(points, o1x, o1y, rho, start_ph, end_ph,
                               delta_psi, inc)
    local sweep1 = dubins.arc_sweep_rad(start_ph, end_ph, inc)

    local sx, sy = px, py
    if #points > 0 then
        sx, sy = points[#points].x, points[#points].y
    end
    if L > 1e-6 then
        dubins.generate_straight_points(points, sx, sy, theta, L, delta_d)
    end
    -- No terminal arc: the path flies into the tangent and ends on the ring.

    local direction = "ccw"
    if s2 > 0 then
        direction = "cw"
    end
    return {
        points = points,
        reach = rho * sweep1 + L,
        direction = direction,
        arrival_x = arrival_x,
        arrival_y = arrival_y,
    }
end

-- ---------------------------------------------------------
-- The least-cost candidate
-- ---------------------------------------------------------

--- The least turn-in cost CS path onto the ring.
--  Returns the same table shape as reach_path, or nil plus a reason.
function M.shortest_path(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                         delta_psi, delta_d)
    if orbit_radius_m < turn_radius_m - 1e-9 then
        return nil, "target circle radius < minimum turn radius: the orbit " ..
                    "would exceed the curvature bound"
    end
    local dx, dy = px - tx, py - ty
    if math.sqrt(dx * dx + dy * dy) < orbit_radius_m - 1e-6 then
        -- Inside the ring: an outward tangent may exist geometrically, but the
        -- approach-from-outside construction is out of TASK-024's envelope
        -- (continuing on the ring is TASK-025). Fail deterministically.
        return nil, "aircraft is inside the target ring; no approach tangent"
    end

    local best = nil
    for _, s1 in ipairs({ 1, -1 }) do
        for _, s2 in ipairs({ 1, -1 }) do
            local cand = M.reach_path(px, py, psi_i, tx, ty, orbit_radius_m,
                                      turn_radius_m, s1, s2, delta_psi, delta_d)
            if cand ~= nil then
                if best == nil or cand.reach < best.reach then
                    best = cand
                end
            end
        end
    end
    if best == nil then
        return nil, "no target-circle tangent solves this configuration"
    end
    return best
end

--- One guidance point a look-ahead along the shortest CS path (TASK-024 only,
--  no orbit continuation). Returns a table, or nil plus a reason.
function M.approach_guidance(px, py, psi_i, tx, ty, orbit_radius_m,
                             turn_radius_m, look_ahead_m, delta_psi, delta_d)
    local path, reason = M.shortest_path(px, py, psi_i, tx, ty, orbit_radius_m,
                                         turn_radius_m, delta_psi, delta_d)
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
-- Approach + ramp-free orbit continuation (TASK-025)
-- ---------------------------------------------------------

--- One guidance point: CS approach outside the ring, orbit continuation on it.
--
--  `phase` is "approach" (outside) or "orbit" (on/inside) -- a DISCRETE
--  geometric switch on d against R, never a blended ramp weight. It is
--  continuous without a ramp because the approach arrives tangent to the ring,
--  which is the property TASK-025 exists to exploit and the one a port could
--  most easily lose.
--
--  Returns a table {gx, gy, phase, direction, curvature, ring_angle_rad?} or
--  nil plus a reason.
function M.guidance(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                    look_ahead_m, delta_psi, delta_d, precompensate)
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
                                              delta_psi, delta_d)
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
