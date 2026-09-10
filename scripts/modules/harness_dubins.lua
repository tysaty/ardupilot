-- =========================================================
--  harness_dubins -- Dubins primitives  |  created 2026-09-03
--  TASK-006 Tranche 4 (forward port).
--
--  Circle centres, arc sweep, arc- and straight-segment point generation, and
--  path arc-length sampling. Ported from py_harness/geometry/dubins.py and the
--  path helpers of py_harness/geometry/orbit.py.
--
--  Relationship to modules/dubins_weave_full.lua
--  ---------------------------------------------
--  That module (853 lines) predates the harness and implements the full
--  six-family Dubins generator for the SUPERSEDED mid-term controller
--  (ADR-001). It was read before this was written, per Tranche 4's instruction
--  to check for reusable primitives. Its arc and straight generators are
--  equivalent in intent, but they are entangled with that controller's data
--  shapes and its own heading bookkeeping, so reusing them would have coupled a
--  live port to dead code and made a divergence attributable to either. These
--  are transliterated from the harness instead, which is what the differential
--  test compares against. dubins_weave_full.lua is left untouched.
--
--  Point representation: a path is a 1-indexed array of {x, y, psi} tables, in
--  the geometry frame x = East, y = North, psi from North clockwise (IR-008).
--  Lua arrays are 1-indexed where Python's are 0-indexed; that difference is the
--  single most likely place for a port to drop or duplicate an element, so the
--  differential test compares whole point sequences and their lengths, not just
--  endpoints.
--
--  Stateless (VR-015, A-VAL-003): the caller owns every path it is handed.
-- =========================================================

local geom = require("harness_geom")

local M = {}

local PI = math.pi

-- ---------------------------------------------------------
-- Turn circles
-- ---------------------------------------------------------

--- Centre of the turn circle to the aircraft's RIGHT, radius rho.
function M.circle_center_right(x, y, psi, rho)
    return x + rho * math.cos(psi), y - rho * math.sin(psi)
end

--- Centre of the turn circle to the aircraft's LEFT, radius rho.
function M.circle_center_left(x, y, psi, rho)
    return x - rho * math.cos(psi), y + rho * math.sin(psi)
end

--- Minimum turn radius from the coordinated-turn relation, rho = V^2/(g tan phi).
--  phi_max in radians. Present for completeness; the harness fixes airspeed and
--  derives bank instead (A-VAL-004), so nothing in the port calls this.
function M.min_turn_radius(v_t, phi_max, g)
    return (v_t * v_t) / (g * math.tan(phi_max))
end

-- ---------------------------------------------------------
-- Arcs
-- ---------------------------------------------------------

--- Swept angle of an arc from psi_start to psi_end, radians, always >= 0.
--  `increasing` selects the sense; the branch that adds or subtracts 2*pi is
--  what makes the sweep the one actually flown rather than its complement.
function M.arc_sweep_rad(psi_start, psi_end, increasing)
    if increasing then
        if psi_end < psi_start then
            psi_end = psi_end + 2.0 * PI
        end
        return psi_end - psi_start
    end
    if psi_end > psi_start then
        psi_end = psi_end - 2.0 * PI
    end
    return psi_start - psi_end
end

--- One point on a circle at phase psi_n, in (x = East, y = North).
--  Note the sin/cos order: x uses sin and y uses cos, because psi is measured
--  from North clockwise rather than from East anticlockwise.
function M.arc_point(xc, yc, rho, psi_n)
    return xc + rho * math.sin(psi_n), yc + rho * math.cos(psi_n)
end

--- One straight step of delta_d along heading theta.
function M.straight_step(x_prev, y_prev, theta, delta_d)
    return x_prev + delta_d * math.sin(theta), y_prev + delta_d * math.cos(theta)
end

--- Append the arc from psi_start to psi_end onto `points` (modified in place).
--
--  Sampled every delta_psi, then snapped to the exact endpoint when the last
--  whole step fell short. The snap matters: without it the path ends up to one
--  sampling interval short of the tangency point, and every downstream length
--  and guidance point inherits the error.
--
--  Faithful to the harness including its integer floor: `n_steps` is
--  floor(sweep / delta_psi). Lua 5.3 has an integer subtype and `//` is floor
--  division on floats, so `sweep // delta_psi` gives a float whose value is the
--  floor -- matching Python's `//`. It is converted with math.floor and used as
--  a loop bound, so no integer/float subtlety reaches the arithmetic.
function M.generate_arc_points(points, xc, yc, rho, psi_start, psi_end,
                               delta_psi, increasing)
    if math.abs(psi_end - psi_start) < 1e-9 then
        return
    end
    local sweep, sign
    if increasing then
        if psi_end < psi_start then
            psi_end = psi_end + 2.0 * PI
        end
        sweep = psi_end - psi_start
        sign = 1
    else
        if psi_end > psi_start then
            psi_end = psi_end - 2.0 * PI
        end
        sweep = psi_start - psi_end
        sign = -1
    end
    if sweep > 2.0 * PI then
        sweep = 2.0 * PI
    end
    local n_steps = math.floor(sweep / delta_psi)
    for i = 1, n_steps do
        local psi = psi_start + sign * i * delta_psi
        local x, y = M.arc_point(xc, yc, rho, psi)
        points[#points + 1] = { x = x, y = y, psi = psi }
    end
    if sweep - n_steps * delta_psi > 1e-6 then
        local psi = psi_start + sign * sweep
        local x, y = M.arc_point(xc, yc, rho, psi)
        points[#points + 1] = { x = x, y = y, psi = psi }
    end
end

--- Append a straight run of total_d along theta onto `points`.
--
--  The `while dsum <= total_d` bound is the harness's, reproduced exactly
--  including its consequence: the last emitted point can overshoot total_d by
--  up to delta_d. That is as-built behaviour, not a defect to be corrected
--  during a port -- a "fix" here would make the differential test pass while
--  hiding the real difference (TASK-006's no-behaviour-change constraint).
function M.generate_straight_points(points, x_start, y_start, theta, total_d,
                                    delta_d)
    local x, y = x_start, y_start
    local dsum = 0.0
    while dsum <= total_d do
        x, y = M.straight_step(x, y, theta, delta_d)
        points[#points + 1] = { x = x, y = y, psi = theta }
        dsum = dsum + delta_d
    end
    return x, y
end

-- ---------------------------------------------------------
-- Path arc length
-- ---------------------------------------------------------

--- Cumulative planar arc length along a path. Returns a 1-indexed array whose
--  first entry is 0.0 and whose i-th entry is the length up to points[i].
function M.cumulative_arc_length(points)
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
--
--  Interpolating rather than snapping to the nearest sample is deliberate: the
--  guidance point would otherwise jump by the sampling interval every time the
--  bracketing pair changed, and that jump would appear in PR-008's
--  discontinuity metric as if the plan had changed.
--
--  Returns nil for an empty path, which is the Lua equivalent of the harness
--  raising ValueError (the interface contract's transliteration note: raise
--  where a Lua port would return nil).
function M.point_at_arc_length(points, arc_length_m)
    local n = #points
    if n == 0 then
        return nil
    end
    if n == 1 then
        return points[1].x, points[1].y
    end
    local lengths = M.cumulative_arc_length(points)
    if arc_length_m <= 0.0 then
        return points[1].x, points[1].y
    end
    if arc_length_m >= lengths[n] then
        return points[n].x, points[n].y
    end
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

--- Arc length of the point on `points` nearest to (px, py), metres.
--
--  "How far along am I" is a projection, not an accumulation: the aircraft
--  steers toward a carrot under a turn-rate limit and does not fly the
--  committed path exactly, so accumulating speed*dt since the commit would
--  drift whenever it cut a corner.
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
