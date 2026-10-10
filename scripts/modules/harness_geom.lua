-- =========================================================
--  harness_geom -geometric primitives  
--  created 2026-09-03
--
--  Angle wrapping, frame conversion and the small helpers every other ported
--  module depends on. Ported from the python harness
--  state.wrap_pi,
--  geometry/dubins.py's helpers,
--   (north, east) <-> (x = East, y = North) convention
--
--  Why this exists alongside modules/math_helpers.lua ...
--  Two of its functions also differ from the harness in ways that matter:
--    * math_helpers.wrap_pi uses a while-loop and is HALF-OPEN AT -pi
--      (an input of exactly +pi returns +pi). The harness wraps to [-pi, +pi)
--      with (a + pi) % (2*pi) - pi, so exactly +pi returns -pi. Both are
--      defensible; they are not the same function, and the interval must be
--      documented. wrap_pi below is the HARNESS convention.
--    * math_helpers has no (north, east) <-> (x, y) conversion at all, which is
--      where every prototype defect in this area came from.
--
--  Frame: x = East, y = North; psi measured from North, increasing clockwise
--  Distances metres, speeds m/s, angles radians.
--
--  Stateless: this module exposes functions and numeric constants only, so two
--  identical calls return identical answers.
-- =========================================================

local M = {}

local PI = math.pi
M.PI = PI

-- ---------------------------------------------------------
-- Angles
-- ---------------------------------------------------------

--- Wrap an angle to [-pi, +pi), the harness interval.
--  Half-open at +pi: an input of exactly -pi or +pi returns -pi.
--
--  Lua's `%` on floats matches Python's: the result takes the sign of the
--  divisor, so `-1.0 % 2.0` is 1.0 in both. `math.fmod` does NOT -- it returns
--  -1.0 -- so this must not be written with fmod. Measured on the target
--  runtime (Lua 5.3) by the differential tests.

function M.wrap_pi(a)
    return (a + PI) % (2.0 * PI) - PI
end

--- Wrap an angle to [0, 2*pi).
function M.wrap_2pi(a)
    return a % (2.0 * PI)
end

--- Two-argument arctangent, y first, matches Python's math.atan2(y, x).

function M.atan2(y, x)
    return math.atan(y, x)
end

--- Heading (from North, clockwise) of the vector (north, east).
--  This is atan2(east, north), NOT atan2(north, east): the argument order is
--  the whole of the convention, and swapping it is silent.

function M.heading_of(north, east)
    return math.atan(east, north)
end

-- ---------------------------------------------------------
-- Frame conversio
-- ---------------------------------------------------------
-- The harness records history as (n_m, e_m); the geometry works in
-- (x = East, y = North). Written out as named functions rather than inline
--  because a silent transpose is indistinguishable from a geometry bug
-- and every prototype error in this area was exactly that.

--- transpose to XY coordinates
function M.ne_to_xy(north, east)
    return east, north
end

--- transpose to NE coordinates
function M.xy_to_ne(x, y)
    return y, x
end

-- ---------------------------------------------------------
-- Small numeric helpers
-- ---------------------------------------------------------
--- Euclidean distance
function M.dist2d(x1, y1, x2, y2)
    local dx = x2 - x1
    local dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)
end

--- Hypotneuse
function M.hypot(x, y)
    return math.sqrt(x * x + y * y)
end

--- Clamp
function M.clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

--- Degrees to radians
function M.radians(deg)
    return deg * PI / 180.0
end

--- Radians to degrees
function M.degrees(rad)
    return rad * 180.0 / PI
end


-- SMoothing function
--- 3q^2 - 2q^3 on [0, 1], clamped outside it. The weave's ramp and
--  the elastic speed profile share this curve.
function M.smoothstep(q)
    if q <= 0.0 then return 0.0 end
    if q >= 1.0 then return 1.0 end
    return q * q * (3.0 - 2.0 * q)
end


-- Non linear smoothing
--- Integral of smoothstep from 0 to q: q^3 - q^4/2, saturating at 0.5.
--  Needed so a speed profile's position stays a closed-form function of t with
--  no accumulator and no per-tick state.
function M.smoothstep_integral(q)
    if q <= 0.0 then return 0.0 end
    if q >= 1.0 then return 0.5 end
    return q * q * q - 0.5 * q * q * q * q
end

return M
