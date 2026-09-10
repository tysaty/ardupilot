-- =========================================================
--  harness_orbit -- standoff-ring geometry  |  created 2026-09-03
--  TASK-006 Tranche 5 (forward port), shared with Tranche 4.
--
--  Ring entry, orbit sense, the arc-length carrot and the TASK-027
--  pre-compensation. Ported from py_harness/geometry/orbit.py, which was itself
--  ported from py_plots/ (DEC-2026-06-25-06) -- so this is a port of a port and
--  the differential test is against the harness, which is the reference the
--  project actually uses.
--
--  Frame: x = East, y = North; psi from North clockwise (IR-008). A ring point
--  is P = T + R*(sin psi, cos psi), i.e. (East, North) -- note that sin goes to
--  East and cos to North, which is the opposite of the usual maths convention
--  and is the single most transposable line in this file.
--
--  Stateless (VR-015, A-VAL-003).
-- =========================================================

local M = {}

local PI = math.pi

--- Largest angular advance for which the pre-compensation is defined, radians.
--  At alpha -> pi/2 the factor 1/cos(alpha) diverges and beyond it the chord
--  geometry no longer settles on a circle at all, so the request is REFUSED
--  rather than answered with a huge or negative radius. 80 degrees is a
--  look-ahead of about 1.4 ring radii.
--
--  This refusal is safety-relevant, not numeric: a port in which it did not
--  trigger would silently command a guidance ring the aircraft cannot fly. The
--  Tranche 5 gate requires it to trigger identically on both sides.
M.ALPHA_MAX_RAD = 80.0 * PI / 180.0

-- ---------------------------------------------------------
-- Ring entry
-- ---------------------------------------------------------

--- Angle of the ring point P about the target T, as atan2(East, North).
function M.entry_angle(px, py, tx, ty)
    return math.atan(px - tx, py - ty)
end

--- One ring point at angle psi, returned as (x = East, y = North).
function M.orbit_point(tx, ty, R, psi)
    return tx + R * math.sin(psi), ty + R * math.cos(psi)
end

--- Pick the orbit sense (+1 / -1) whose initial velocity best continues psi_f.
--
--  With P = T + R*(sin psi, cos psi) and psi = psi0 + dir*omega*t, the velocity
--  is proportional to dir*(cos psi0, -sin psi0). The sense is chosen by the
--  larger dot product with the arrival direction, which is what makes the
--  handover from the approach to the orbit continuous without a ramp.
function M.orbit_direction(psi0, psi_f)
    local ax, ay = math.sin(psi_f), math.cos(psi_f)   -- (East, North)
    local best_dir, best_dot = 1, -math.huge
    for _, d in ipairs({ 1, -1 }) do
        local vx = d * math.cos(psi0)
        local vy = -d * math.sin(psi0)
        local dot = vx * ax + vy * ay
        if dot > best_dot then
            best_dir = d
            best_dot = dot
        end
    end
    return best_dir
end

--- Ring-entry tangent points from S to a ring of radius R about T.
--
--  Returns a 1-indexed array of {px, py, psi_f, sign}; EMPTY when the start is
--  inside the ring, which is the no-solution case and must stay distinguishable
--  from an error.
function M.tangent_points(sx, sy, tx, ty, R)
    local dx, dy = tx - sx, ty - sy
    local d = math.sqrt(dx * dx + dy * dy)
    local out = {}
    if d <= R then
        return out
    end
    local L = math.sqrt(d * d - R * R)
    local gamma = math.atan(dy, dx)          -- standard maths angle of S -> T
    local off = math.asin(R / d)
    for _, sign in ipairs({ 1, -1 }) do
        local ang = gamma + sign * off
        local px = sx + L * math.cos(ang)
        local py = sy + L * math.sin(ang)
        -- Back into the heading convention: psi = atan2(East, North).
        local psi_f = math.atan(px - sx, py - sy)
        out[#out + 1] = { px = px, py = py, psi_f = psi_f, sign = sign }
    end
    return out
end

-- ---------------------------------------------------------
-- The carrot on the ring -- A-VAL-005 and TASK-027
-- ---------------------------------------------------------

--- The ring point arc_length_m around the ring from psi0. Returns x, y, psi.
--
--  UNCOMPENSATED: the guidance point lands on the commanded ring, but the
--  aircraft flies the CHORD to it, so the circle actually flown settles inside
--  the ring at R*cos(arc/R) (A-VAL-005). Retained unchanged because the
--  as-built behaviour must stay reproducible.
function M.orbit_point_at_arc_length(tx, ty, R, psi0, direction, arc_length_m)
    local psi = psi0 + direction * (arc_length_m / R)
    local x, y = M.orbit_point(tx, ty, R, psi)
    return x, y, psi
end

--- Carrot-ring radius whose chord geometry settles on the commanded R.
--
--      Rc = R / cos(alpha),     alpha = arc_length_m / R
--
--  The open-loop correction of TASK-027 Option A: it cancels the known
--  chord-cutting offset exactly and carries no state and no gain. It does not
--  close a loop on radial error, so it corrects this offset and nothing else.
--
--  Returns nil, reason on refusal -- the Lua equivalent of the harness raising
--  ValueError. The caller MUST check: returning R unchanged here would convert
--  a refusal into a silently degraded command.
function M.precompensated_ring_radius(R, arc_length_m)
    if R <= 0.0 then
        return nil, "ring radius must be positive"
    end
    if arc_length_m < 0.0 then
        return nil, "arc length must be >= 0"
    end
    local alpha = arc_length_m / R
    if alpha >= M.ALPHA_MAX_RAD then
        return nil, "look-ahead is at or beyond the angular limit where the " ..
                    "chord correction is defined; reduce the look-ahead or " ..
                    "enlarge the ring"
    end
    return R / math.cos(alpha)
end

--- The compensated carrot: same angular advance, placed on the virtual ring.
--  Returns x, y, psi, or nil, reason.
--
--  psi is the angle about the TARGET -- the ring angle the aircraft is steering
--  toward -- and is independent of which ring the point was placed on.
function M.orbit_point_at_arc_length_compensated(tx, ty, R, psi0, direction,
                                                 arc_length_m)
    local rc, reason = M.precompensated_ring_radius(R, arc_length_m)
    if rc == nil then
        return nil, reason
    end
    local psi = psi0 + direction * (arc_length_m / R)
    local x, y = M.orbit_point(tx, ty, rc, psi)
    return x, y, psi
end

--- Rc when precompensate is true, else R. The single place the two carrot laws
--  are selected between, so the choice is a parameter rather than a branch
--  repeated at each call site.
function M.guidance_ring_radius(R, arc_length_m, precompensate)
    if precompensate then
        return M.precompensated_ring_radius(R, arc_length_m)
    end
    return R
end

--- One orbit guidance point under either carrot law. Returns x, y, psi, or
--  nil, reason when the compensation refuses.
function M.orbit_guidance_point(tx, ty, R, psi0, direction, arc_length_m,
                                precompensate)
    if precompensate then
        return M.orbit_point_at_arc_length_compensated(
            tx, ty, R, psi0, direction, arc_length_m)
    end
    return M.orbit_point_at_arc_length(tx, ty, R, psi0, direction, arc_length_m)
end

return M
