-- =========================================================
--  harness_kangaroo -- target motion models  |  created 2026-09-03
--  TASK-006 Tranche 2 (deterministic modes) and Tranche 7 (elastic).
--
--  point / straight / circle / rectangle / elastic, as PURE FUNCTIONS OF t:
--  no state, no aircraft, no estimator. Ported from py_harness/kangaroo.py
--  (TASK-009, TASK-030), which was itself ported from kangaroo_MAV.lua -- so
--  the deterministic modes are a REVERSE CHECK against work already relied on,
--  and any disagreement is a finding about that work, not only about this port.
--
--  Why these are ported early
--  --------------------------
--  The target trajectory is the INPUT to every later comparison. If the two
--  implementations disagree about where the kangaroo is, every downstream
--  tranche compares two different scenarios and no result means anything.
--
--  Frame: positions and velocities are (north, east) in metres and m/s --
--  NOT the geometry frame. The kangaroo lives in the harness's history frame,
--  and harness_geom.ne_to_xy is the documented boundary (IR-008). Headings are
--  degrees at this boundary only (IR-005), converted on entry.
--
--  Stateless (VR-015, A-VAL-003).
-- =========================================================

local geom = require("harness_geom")

local M = {}

--- The four deterministic base modes, in the harness's order.
M.MODES = { "point", "straight", "circle", "rectangle" }

--- Default fraction of the commanded speed the elastic slow phase runs at.
M.ELASTIC_SLOW_FACTOR = 0.3

--- Default seconds held at each speed before ramping to the other.
M.ELASTIC_HOLD_S = 8.0

--- Default seconds spent ramping between them.
M.ELASTIC_RAMP_S = 4.0

-- ---------------------------------------------------------
-- Placement
-- ---------------------------------------------------------

--- Point fwd_m along `heading` and disp_m to its RIGHT, as (n, e).
--  From kangaroo_MAV.lua's straight/point initial placement:
--  n = cos*fwd - sin*disp, e = sin*fwd + cos*disp.
function M.heading_frame_offset(heading_deg, fwd_m, disp_m)
    local h = geom.radians(heading_deg)
    local n = math.cos(h) * fwd_m - math.sin(h) * disp_m
    local e = math.sin(h) * fwd_m + math.cos(h) * disp_m
    return n, e
end

-- ---------------------------------------------------------
-- Deterministic modes -- Tranche 2
-- ---------------------------------------------------------

--- Stationary point. t is ignored; velocity is zero.
function M.point_state(t, heading_deg, fwd_m, disp_m)
    local n, e = M.heading_frame_offset(heading_deg, fwd_m, disp_m)
    return n, e, 0.0, 0.0
end

--- Straight line from the placed start on `heading` at speed_ms.
function M.straight_state(t, heading_deg, fwd_m, disp_m, speed_ms)
    local n0, e0 = M.heading_frame_offset(heading_deg, fwd_m, disp_m)
    local h = geom.radians(heading_deg)
    local vn = math.cos(h) * speed_ms
    local ve = math.sin(h) * speed_ms
    return n0 + vn * t, e0 + ve * t, vn, ve
end

--- Circle of radius_m about the placed centre, at speed_ms.
--  omega = speed/radius; the point starts at angle 0 (centre + (r, 0)),
--  matching integrate_circle: n = cn + r*cos(a), e = ce + r*sin(a).
--  Returns nil plus a reason for a non-positive radius (division by zero).
function M.circle_state(t, heading_deg, fwd_m, disp_m, radius_m, speed_ms)
    if radius_m <= 0.0 then
        return nil, "circle radius must be positive"
    end
    local cn, ce = M.heading_frame_offset(heading_deg, fwd_m, disp_m)
    local a = (speed_ms / radius_m) * t
    local n = cn + radius_m * math.cos(a)
    local e = ce + radius_m * math.sin(a)
    local vn = -speed_ms * math.sin(a)
    local ve = speed_ms * math.cos(a)
    return n, e, vn, ve
end

--- Constant-speed traversal of a length x width rectangle perimeter.
--
--  Corners are the rotated rectangle of integrate_rectangle, traversed
--  0 -> 1 -> 2 -> 3 -> 0. Along-perimeter distance is speed*t wrapped by the
--  perimeter, so the target loops. The corner transitions are the likely
--  divergence point and get their own differential cases.
--
--  Note the `i == 4` fallthrough (Python's `i == 3`, zero-indexed): the last
--  side accepts whatever distance remains, which is what stops a float
--  rounding at the wrap point falling out of the loop with no answer.
function M.rectangle_state(t, heading_deg, fwd_m, disp_m, length_m, width_m,
                           speed_ms)
    if length_m <= 0.0 or width_m <= 0.0 then
        return nil, "rectangle needs positive length and width"
    end
    local on, oe = M.heading_frame_offset(heading_deg, fwd_m, disp_m)
    local h = geom.radians(heading_deg)
    local ch, sh = math.cos(h), math.sin(h)
    local corners = {
        { on, oe },
        { on + length_m * ch, oe + length_m * sh },
        { on + length_m * ch - width_m * sh, oe + length_m * sh + width_m * ch },
        { on - width_m * sh, oe + width_m * ch },
    }
    local perimeter = 2.0 * (length_m + width_m)
    local d = 0.0
    if speed_ms > 0.0 then
        d = (speed_ms * t) % perimeter
    end
    for i = 1, 4 do
        local s0 = corners[i]
        local s1 = corners[(i % 4) + 1]
        local dn, de = s1[1] - s0[1], s1[2] - s0[2]
        local side_len = math.sqrt(dn * dn + de * de)
        if d <= side_len or i == 4 then
            local frac = 0.0
            if side_len > 0.0 then
                frac = math.min(d, side_len) / side_len
            end
            local ux, uy = 0.0, 0.0
            if side_len > 0.0 then
                ux, uy = dn / side_len, de / side_len
            end
            return s0[1] + dn * frac, s0[2] + de * frac,
                   ux * speed_ms, uy * speed_ms
        end
        d = d - side_len
    end
    return corners[1][1], corners[1][2], 0.0, 0.0
end

-- ---------------------------------------------------------
-- Elastic -- Tranche 7
-- ---------------------------------------------------------

--- One full slow-ramp-fast-ramp cycle, seconds.
function M.elastic_period_s(hold_s, ramp_s)
    return 2.0 * (hold_s + ramp_s)
end

--- Speed at t: hold slow, ramp up, hold fast, ramp down, repeat.
--  Returns nil plus a reason for negative speeds, a negative hold or a
--  non-positive ramp.
function M.elastic_speed(t, slow_ms, fast_ms, hold_s, ramp_s)
    if slow_ms < 0.0 or fast_ms < 0.0 then
        return nil, "elastic speeds must be >= 0"
    end
    if hold_s < 0.0 then
        return nil, "elastic hold must be >= 0"
    end
    if ramp_s <= 0.0 then
        return nil, "elastic ramp must be > 0"
    end
    local span = fast_ms - slow_ms
    local u = t % M.elastic_period_s(hold_s, ramp_s)
    if u < hold_s then
        return slow_ms
    end
    u = u - hold_s
    if u < ramp_s then
        return slow_ms + span * geom.smoothstep(u / ramp_s)
    end
    u = u - ramp_s
    if u < hold_s then
        return fast_ms
    end
    u = u - hold_s
    return fast_ms - span * geom.smoothstep(u / ramp_s)
end

--- Distance travelled by time t under elastic_speed, metres.
--  Closed-form integral of the profile, so position stays a pure function of t
--  with no accumulator (VR-015, A-VAL-003). Over a whole cycle the mean speed
--  is exactly (slow + fast)/2.
function M.elastic_distance(t, slow_ms, fast_ms, hold_s, ramp_s)
    if t <= 0.0 then
        return 0.0
    end
    local span = fast_ms - slow_ms
    local period = M.elastic_period_s(hold_s, ramp_s)
    local per_cycle = 0.5 * (slow_ms + fast_ms) * period
    local whole = math.floor(t / period)
    local u = t - whole * period
    local dist = whole * per_cycle

    local take = math.min(u, hold_s)                        -- slow hold
    dist = dist + slow_ms * take
    u = u - take
    if u <= 0.0 then return dist end

    take = math.min(u, ramp_s)                              -- ramp up
    dist = dist + slow_ms * take
             + span * ramp_s * geom.smoothstep_integral(take / ramp_s)
    u = u - take
    if u <= 0.0 then return dist end

    take = math.min(u, hold_s)                              -- fast hold
    dist = dist + fast_ms * take
    u = u - take
    if u <= 0.0 then return dist end

    take = math.min(u, ramp_s)                              -- ramp down
    dist = dist + fast_ms * take
             - span * ramp_s * geom.smoothstep_integral(take / ramp_s)
    return dist
end

--- `base_mode` travelled at an elastic pace (TASK-030).
--
--  The base mode is evaluated at UNIT SPEED, so its time argument is arc length
--  directly and its velocity is the unit tangent; the elastic distance and
--  speed are then substituted in. That is why one implementation serves
--  straight, circle and rectangle alike without any of them changing.
--
--  Returns nil plus a reason for "point" (a stationary target has no pace to
--  vary) or an unknown base mode.
function M.elastic_state(t, base_mode, heading_deg, fwd_m, disp_m, radius_m,
                         length_m, width_m, slow_ms, fast_ms, hold_s, ramp_s)
    if base_mode == "point" then
        return nil, "elastic needs a moving base mode; 'point' is stationary"
    end
    if base_mode ~= "straight" and base_mode ~= "circle"
            and base_mode ~= "rectangle" then
        return nil, "unknown elastic base mode"
    end
    local dist = M.elastic_distance(t, slow_ms, fast_ms, hold_s, ramp_s)
    local speed, reason = M.elastic_speed(t, slow_ms, fast_ms, hold_s, ramp_s)
    if speed == nil then
        return nil, reason
    end
    local n, e, vn, ve
    if base_mode == "straight" then
        n, e, vn, ve = M.straight_state(dist, heading_deg, fwd_m, disp_m, 1.0)
    elseif base_mode == "circle" then
        n, e, vn, ve = M.circle_state(dist, heading_deg, fwd_m, disp_m,
                                      radius_m, 1.0)
    else
        n, e, vn, ve = M.rectangle_state(dist, heading_deg, fwd_m, disp_m,
                                         length_m, width_m, 1.0)
    end
    if n == nil then
        return nil, e
    end
    return n, e, vn * speed, ve * speed
end

-- ---------------------------------------------------------
-- Dispatch
-- ---------------------------------------------------------

--- Evaluate any mode by name. Returns n, e, vn, ve, or nil plus a reason.
--  `opts` is a table carrying whatever the chosen mode needs: heading_deg,
--  fwd_m, disp_m, speed_ms, radius_m, length_m, width_m, and for elastic
--  base_mode, slow_ms, fast_ms, hold_s, ramp_s.
function M.state(mode, t, opts)
    local heading_deg = opts.heading_deg or 0.0
    local fwd_m = opts.fwd_m or 0.0
    local disp_m = opts.disp_m or 0.0
    local speed_ms = opts.speed_ms or 0.0
    if mode == "point" then
        return M.point_state(t, heading_deg, fwd_m, disp_m)
    elseif mode == "straight" then
        return M.straight_state(t, heading_deg, fwd_m, disp_m, speed_ms)
    elseif mode == "circle" then
        return M.circle_state(t, heading_deg, fwd_m, disp_m,
                              opts.radius_m or 150.0, speed_ms)
    elseif mode == "rectangle" then
        return M.rectangle_state(t, heading_deg, fwd_m, disp_m,
                                 opts.length_m or 300.0,
                                 opts.width_m or 150.0, speed_ms)
    elseif mode == "elastic" then
        return M.elastic_state(t, opts.base_mode or "straight", heading_deg,
                               fwd_m, disp_m, opts.radius_m or 150.0,
                               opts.length_m or 300.0, opts.width_m or 150.0,
                               opts.slow_ms or (speed_ms * M.ELASTIC_SLOW_FACTOR),
                               opts.fast_ms or speed_ms,
                               opts.hold_s or M.ELASTIC_HOLD_S,
                               opts.ramp_s or M.ELASTIC_RAMP_S)
    end
    return nil, "unknown kangaroo mode"
end

return M
