-- =========================================================
--  harness_geom -- shared geometric primitives  |  created 2026-09-03
--  TASK-006 Tranche 1 (math primitives).
--
--  Angle wrapping, frame conversion and the small helpers every later tranche
--  depends on. Ported from py_harness (state.wrap_pi, geometry/dubins.py's
--  helpers, and the (north, east) <-> (x = East, y = North) convention of
--  IR-008).
--
--  Why this exists alongside modules/math_helpers.lua
--  --------------------------------------------------
--  math_helpers.lua is the SHIPPING controller's helper table and is not
--  touched here: TASK-006 is a transliteration and must not change flight code.
--  Two of its functions also differ from the harness in ways that matter:
--
--    * math_helpers.wrap_pi uses a while-loop and is HALF-OPEN AT -pi
--      (an input of exactly +pi returns +pi). The harness wraps to [-pi, +pi)
--      with (a + pi) % (2*pi) - pi, so exactly +pi returns -pi. Both are
--      defensible; they are not the same function, and IR-003 requires the
--      interval to be documented. wrap_pi below is the HARNESS convention.
--    * math_helpers has no (north, east) <-> (x, y) conversion at all, which is
--      where every prototype defect in this area came from.
--
--  Frame: x = East, y = North; psi measured from North, increasing clockwise
--  (IR-001, IR-002, IR-008). Distances metres, speeds m/s, angles radians.
--
--  Stateless: this module exposes functions and numeric constants only, so two
--  identical calls return identical answers (VR-015, A-VAL-003).
-- =========================================================

local M = {}

local PI = math.pi
M.PI = PI

-- ---------------------------------------------------------
-- Angles
-- ---------------------------------------------------------

--- Wrap an angle to [-pi, +pi), the harness interval (IR-003).
--  Half-open at +pi: an input of exactly -pi or +pi returns -pi.
--
--  Lua's `%` on floats matches Python's: the result takes the sign of the
--  divisor, so `-1.0 % 2.0` is 1.0 in both. `math.fmod` does NOT -- it returns
--  -1.0 -- so this must not be written with fmod. Measured on the target
--  runtime by the Tranche 0 semantics record.
function M.wrap_pi(a)
    return (a + PI) % (2.0 * PI) - PI
end

--- Wrap an angle to [0, 2*pi).
function M.wrap_2pi(a)
    return a % (2.0 * PI)
end

--- Two-argument arctangent, y first, matching Python's math.atan2(y, x).
--
--  Lua 5.3 provides this as math.atan(y, x); math.atan2 was deprecated in 5.3
--  and removed in 5.4. Wrapping it here means every call site is written once
--  and the version difference is contained (IR-004, A-SW-005 -- both
--  Provisional until the Tranche 1 differential test measures them, which it
--  does across all four quadrants and the axis cases).
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
-- Frame conversion -- IR-008
-- ---------------------------------------------------------
-- The harness records history as (n_m, e_m); the geometry works in
-- (x = East, y = North). Written out as named functions rather than done
-- inline, because a silent transpose is indistinguishable from a geometry bug
-- and every prototype error in this area was exactly that.

--- (north, east) -> (x, y) = (east, north).
function M.ne_to_xy(north, east)
    return east, north
end

--- (x, y) -> (north, east) = (y, x).
function M.xy_to_ne(x, y)
    return y, x
end

-- ---------------------------------------------------------
-- Small numeric helpers
-- ---------------------------------------------------------

--- Euclidean distance. Written as sqrt(dx^2 + dy^2) to match the harness's
--  math.hypot to the last bit on the values this project uses; Lua has no hypot.
function M.dist2d(x1, y1, x2, y2)
    local dx = x2 - x1
    local dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)
end

--- Magnitude of (x, y).
function M.hypot(x, y)
    return math.sqrt(x * x + y * y)
end

--- Clamp v into [lo, hi].
function M.clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

--- Degrees to radians.
function M.radians(deg)
    return deg * PI / 180.0
end

--- Radians to degrees.
function M.degrees(rad)
    return rad * 180.0 / PI
end

--- 3q^2 - 2q^3 on [0, 1], clamped outside it. The weave's ramp (TASK-004) and
--  the elastic speed profile (TASK-030) share this curve.
function M.smoothstep(q)
    if q <= 0.0 then return 0.0 end
    if q >= 1.0 then return 1.0 end
    return q * q * (3.0 - 2.0 * q)
end

--- Integral of smoothstep from 0 to q: q^3 - q^4/2, saturating at 0.5.
--  Needed so a speed profile's POSITION stays a closed-form function of t with
--  no accumulator and no per-tick state.
function M.smoothstep_integral(q)
    if q <= 0.0 then return 0.0 end
    if q >= 1.0 then return 0.5 end
    return q * q * q - 0.5 * q * q * q * q
end

return M
