-- =========================================================
--  harness_orbit -- standoff-ring geometry  
--  Ported/created 2026-09-03
--
--  Ring entry, orbit sense, the arc-length carrot and the applied pre-compensation.
--
--  Frame: NE (x = East, y = North); psi from North clockwise. 
--  A ring point is P = T + R*(sin psi, cos psi), 
--  i.e. (East, North) -- note that sin (East), cos (North) - opposite to usual 
--  maths - transposable. 
--
--  Stateless
-- Read concurrent with harness_dubins.lua
-- =========================================================

local M = {}

local PI = math.pi

--  Generating agains tthe chord cutting for flown paths (angular advance adjustment, radians)
--  At alpha -> pi/2 the factor 1/cos(alpha) diverges and beyond it the chord
--  geometry no longer settles on a circle at all, so the request is refused
--  rather than answered with a huge or negative radius. 80 degrees is a look-ahead of about 1.4 ring radii.
--
--  This refusal is safety-relevant: a port in which it did not trigger would
--   silently command a guidance ring the aircraft cannot fly. The

M.ALPHA_MAX_RAD = 80.0 * PI / 180.0

-- ---------------------------------------------------------
-- The kangaroo ring
-- ---------------------------------------------------------

-- Entry angle
--- Angle of the ring point P about the target T as atan2(East, North).
function M.entry_angle(px, py, tx, ty)
    return math.atan(px - tx, py - ty)
end

-- orbit point
--- One ring point at angle psi returned as (x = East, y = North).
-- Used to generate the next point of the orbit
function M.orbit_point(tx, ty, R, psi)
    return tx + R * math.sin(psi), ty + R * math.cos(psi)
end

--- Pick the orbit sense (+1 / -1) whose initial velocity best continues psi_f.
--  With P = T + R*(sin psi, cos psi) and psi = psi0 + dir * omega * time, the velocity
--  is proportional to dir * (cos psi0, -sin psi0). The sense is chosen by the
--  larger dot product with the arrival direction, force continuous handover w/o ram

function M.orbit_direction(psi0, psi_f)
    -- East, North
    local ax, ay = math.sin(psi_f), math.cos(psi_f)   -- (East, North)
    -- generate a dot product
    local best_dir, best_dot = 1, -math.huge
    -- for loop to work throug hthe best orbit direction
    for _, d in ipairs({ 1, -1 }) do
        local vx = d * math.cos(psi0)
        local vy = -d * math.sin(psi0)
        local dot = vx * ax + vy * ay
        -- establish best
        if dot > best_dot then
            best_dir = d
            best_dot = dot
        end
    end
    -- return the optimal orbit direction
    return best_dir
end

-- Tangent point, relative from a point S to the ring of radius centred on the orbit point
--
--  Returns a 1-indexed array of {px, py, psi_f, sign}
--  EMPTY when the start is inside the ring, which is the no-solution case 
-- and must stay distinguishable from an error.
function M.tangent_points(sx, sy, tx, ty, R)
    -- generate points
    local dx, dy = tx - sx, ty - sy
    local d = math.sqrt(dx * dx + dy * dy)
    local out = {}
    if d <= R then
        return out
    end
    local L = math.sqrt(d * d - R * R)
    -- gamma angle from point S to Tangent T
    local gamma = math.atan(dy, dx)
    local off = math.asin(R / d)
    -- for the pair, generate the tangent points
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
-- Establishing the carrot 
-- ---------------------------------------------------------

-- Try 1 - uncompensated approach, the gudiance point lands on the command; 
-- the chord cuts it so the circle is flown inside the ring at R*cos(arc_length/R)
--- The ring point arc_length_m around the ring from psi0. Returns x, y, psi.

function M.orbit_point_at_arc_length(tx, ty, R, psi0, direction, arc_length_m)
    local psi = psi0 + direction * (arc_length_m / R)
    -- the the next point of carrot
    local x, y = M.orbit_point(tx, ty, R, psi)
    return x, y, psi
end

-- Try 2 - compensated appraoch
-- Adjustment to command the carrot ring radius, where the chord matches the required R
--  Swapped maths to: Rc = R / cos(alpha),
--  where alpha is: alpha = arc_length_m / R
--
--  The open-loop correction cancels the known chord-cutting offset exactly and carries 
--  no state and no gain. It does not close a loop on radial error, so it corrects this offset.


-- Part 1: compensated Ring Radius
function M.precompensated_ring_radius(R, arc_length_m)
    -- nil case - radius is zero
    if R <= 0.0 then
        return nil, "ring radius must be positive"
    end
    -- nil case - arc legnth of zero
    if arc_length_m < 0.0 then
        return nil, "arc length must be >= 0"
    end
    local alpha = arc_length_m / R
    -- if the arclentth_radius is grater than the maximum alpha
    --  Returns nil, reason on refusal -- the Lua equivalent of the harness raising
    --  ValueError. The caller checks: returning R unchanged here would convert
    --  a refusal into a silently degraded command.
    if alpha >= M.ALPHA_MAX_RAD then
        return nil, "look-ahead is at or beyond the angular limit where the " ..
                    "chord correction is defined; reduce the look-ahead or " ..
                    "enlarge the ring"
    end
    -- compensated carrot
    return R / math.cos(alpha)
end

-- Part 2: Comensating the carrot
--- The compensated carrot: same angular advance, but placed on the carrect3ed ring radius.
--  Returns x, y, psi, or error.
--
--  psi is the angle about the kangaroo -- the ring angle the aircraft is steering
--  toward, is independent of which ring the point was placed on.
function M.orbit_point_at_arc_length_compensated(tx, ty, R, psi0, direction, arc_length_m)
    local rc, reason = M.precompensated_ring_radius(R, arc_length_m)
    if rc == nil then
        return nil, reason
    end
    local psi = psi0 + direction * (arc_length_m / R)
    -- corrected point
    local x, y = M.orbit_point(tx, ty, rc, psi)
    return x, y, psi
end

-- Handling the compensation versus non-compesnation appraoch 
--- R_c when precompensate is true, else R. The single place the two carrot laws
--  are selected between, so the choice is a parameter rather than a branch
--  repeated at each call site.
function M.guidance_ring_radius(R, arc_length_m, precompensate)
    if precompensate then
        return M.precompensated_ring_radius(R, arc_length_m)
    end
    return R
end

--- One orbit guidance point under either carrot law. 
-- Returns x, y, psi, or nil, reason when the compensation refuses.
function M.orbit_guidance_point(tx, ty, R, psi0, direction, arc_length_m, precompensate)
    -- if pre compensation
    if precompensate then
        return M.orbit_point_at_arc_length_compensated(
            tx, ty, R, psi0, direction, arc_length_m)
    end
    -- if not compensated
    return M.orbit_point_at_arc_length(tx, ty, R, psi0, direction, arc_length_m)
end

-- return
return M
