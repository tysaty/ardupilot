-- =========================================================
--  harness_weave -- the weave families and the heading baseline
--  created 2026-09-03
--  TASK-006 Tranche 8 (reverse check).  Algorithms: amplitude /
--  continuous_weave, var_amplitude / vaw, heading_a, and the shared orbit ramp
--  that turns each into its _orbit variant.
--
--  Ported from py_harness/geometry/amplitude.py (TASK-004),
--  var_amplitude_weave.py (TASK-014), heading.py (TASK-010) and the ramp in
--  dubins_orbit.py.
--
--  Naming is PROVISIONAL, and this is Tranche 8's open precondition
--  ----------------------------------------------------------------
--  TASK-006 says not to open this tranche before TASK-034 Stage 3 settles,
--  because four naming questions are unresolved -- including what the `a` in
--  `heading_a` means, which is not recoverable from the source. That decision is
--  STILL OPEN. The port here is numeric only: the module and its functions keep
--  the harness's registry names so the differential test has something to
--  compare, and NO name is claimed as settled. Renaming after ADR-006 is
--  accepted is a mechanical change to this file and its test.
--
--  Reproduce, do not fix
--  ---------------------
--  The weave is PLANE-ANCHORED: the reference line is recomputed from the
--  aircraft's position every cycle, so the amplitude actually achieved is
--  smaller than the amplitude commanded (A-DEC-009). That gap is in the
--  SHIPPING controller and is reproduced here deliberately, on both sides. It is
--  not a porting defect and must not be corrected during a port -- a "fix" makes
--  the differential test pass while hiding a real divergence.
--
--  Frame x = East, y = North (IR-008). Stateless: the weave's arc-length phase
--  `s` is derived by the caller from speed and elapsed time, never accumulated
--  here (A-VAL-003).
-- =========================================================

local geom = require("harness_geom")
local orbit = require("harness_orbit")

local M = {}

local PI = math.pi

--- Floor on relative speed for the variable-amplitude onset, m/s. Stops a
--  near-zero closing speed collapsing the onset window to nothing.
M.V_FLOOR_MS = 0.1

-- ---------------------------------------------------------
-- Shared wave maths
-- ---------------------------------------------------------

--- 3q^2 - 2q^3. Defined in the original Lua and used unclamped here, exactly as
--  the harness does: the callers clamp q first.
function M.smoothstep(q)
    return 3.0 * q * q - 2.0 * q * q * q
end

--- Largest amplitude keeping curvature <= 1/R_min for the wave.
--  For y = A sin(2*pi*s/lambda) the peak curvature is A*(2*pi/lambda)^2, so
--  A_max = eta*lambda^2 / (4*pi^2*R_min). eta in (0, 1] is the safety factor;
--  eta = 1 sits exactly at the curvature limit.
--  Returns nil plus a reason outside the domain.
function M.curvature_limited_amplitude(lambda_m, r_min_m, eta)
    if r_min_m <= 0.0 then
        return nil, "r_min_m must be positive"
    end
    if lambda_m <= 0.0 then
        return nil, "lambda_m must be positive"
    end
    if not (eta > 0.0 and eta <= 1.0) then
        return nil, "eta must be in (0, 1]"
    end
    return eta * (lambda_m * lambda_m) / (4.0 * PI * PI * r_min_m)
end

--- (y, y', y'') of y = A sin(2*pi*s/lambda + phase) at arc length s.
function M.wave_derivatives(s_m, amplitude_m, lambda_m, phase_rad)
    if lambda_m <= 0.0 then
        return nil, "lambda_m must be positive"
    end
    local k = 2.0 * PI / lambda_m
    local arg = k * s_m + (phase_rad or 0.0)
    local y = amplitude_m * math.sin(arg)
    local yp = amplitude_m * k * math.cos(arg)
    local ypp = -amplitude_m * k * k * math.sin(arg)
    return y, yp, ypp
end

--- Planar curvature |y''| / (1 + y'^2)^(3/2).
function M.curvature(y_prime, y_double_prime)
    local denom = (1.0 + y_prime * y_prime)
    return math.abs(y_double_prime) / (denom * math.sqrt(denom))
end

-- ---------------------------------------------------------
-- amplitude / continuous_weave -- TASK-004
-- ---------------------------------------------------------

--- Distance-dependent desired amplitude.
--  q ramps 0 -> 1 as the distance falls from d_start to d_full. "smoothstep" is
--  the default; "linear" reproduces continuous_weave.lua's A_cap * q.
function M.desired_amplitude(distance_m, a_cap_m, d_start_m, d_full_m, envelope)
    if d_start_m <= d_full_m then
        return nil, "need d_start > d_full"
    end
    local q = geom.clamp((d_start_m - distance_m) / (d_start_m - d_full_m),
                         0.0, 1.0)
    local shaped = q
    if envelope == nil or envelope == "smoothstep" then
        shaped = M.smoothstep(q)
    end
    return a_cap_m * shaped
end

--- The commanded amplitude: the desired envelope, capped by curvature.
function M.effective_amplitude(distance_m, lambda_m, r_min_m, a_cap_m,
                               d_start_m, d_full_m, eta, envelope)
    local a_desired, reason = M.desired_amplitude(distance_m, a_cap_m,
                                                  d_start_m, d_full_m, envelope)
    if a_desired == nil then
        return nil, reason
    end
    local cap
    cap, reason = M.curvature_limited_amplitude(lambda_m, r_min_m, eta)
    if cap == nil then
        return nil, reason
    end
    return math.min(a_desired, cap)
end

--- One guidance point for the amplitude weave, plane-anchored.
--  Returns {gx, gy, amplitude, curvature, curvature_limit} or nil plus a reason.
function M.amplitude_guidance(px, py, psi_i, tx, ty, s_m, lambda_m, r_min_m,
                              a_cap_m, d_start_m, d_full_m, eta, look_ahead_m,
                              envelope, phase_rad)
    local dx, dy = tx - px, ty - py
    local distance_m = math.sqrt(dx * dx + dy * dy)
    if distance_m < 1e-6 then
        return nil, "aircraft is on the target; no weave direction"
    end

    -- Unit vector toward the target and the LEFT normal, in (East, North).
    local ux, uy = dx / distance_m, dy / distance_m
    local nx, ny = -uy, ux

    -- Plane-anchored straight reference: u = 0 is the aircraft (A-DEC-009).
    local u = math.min(look_ahead_m, distance_m)
    local line_x = px + u * ux
    local line_y = py + u * uy

    local amp, reason = M.effective_amplitude(distance_m, lambda_m, r_min_m,
                                              a_cap_m, d_start_m, d_full_m,
                                              eta, envelope)
    if amp == nil then
        return nil, reason
    end
    local y, yp, ypp = M.wave_derivatives(s_m, amp, lambda_m, phase_rad)
    local cap = M.curvature_limited_amplitude(lambda_m, r_min_m, eta)
    return {
        gx = line_x + nx * y,
        gy = line_y + ny * y,
        amplitude = amp,
        curvature = M.curvature(yp, ypp),
        curvature_limit = cap,
    }
end

-- ---------------------------------------------------------
-- var_amplitude / vaw -- TASK-014
-- ---------------------------------------------------------

--- Distance at which the weave begins, scaled by relative speed:
--  d_start_eff = d_full + lead_s * max(v_rel, v_floor).
function M.onset_distance_m(d_full_m, v_rel_ms, lead_s, v_floor_ms)
    v_floor_ms = v_floor_ms or M.V_FLOOR_MS
    if lead_s <= 0.0 then
        return nil, "lead_s must be positive"
    end
    local v_eff = v_rel_ms
    if not (v_rel_ms > v_floor_ms) then
        v_eff = v_floor_ms
    end
    return d_full_m + lead_s * v_eff
end

--- The speed-scaled envelope, capped by curvature.
function M.speed_scaled_amplitude(distance_m, v_rel_ms, lambda_m, r_min_m,
                                  a_cap_m, d_full_m, lead_s, eta, v_floor_ms)
    if a_cap_m < 0.0 then
        return nil, "a_cap_m must be non-negative"
    end
    if d_full_m < 0.0 then
        return nil, "d_full_m must be non-negative"
    end
    local d_start_eff, reason = M.onset_distance_m(d_full_m, v_rel_ms, lead_s,
                                                   v_floor_ms)
    if d_start_eff == nil then
        return nil, reason
    end
    local window = d_start_eff - d_full_m        -- = lead_s * v_eff > 0
    local q = geom.clamp((d_start_eff - distance_m) / window, 0.0, 1.0)
    local a_desired = a_cap_m * M.smoothstep(q)
    local cap
    cap, reason = M.curvature_limited_amplitude(lambda_m, r_min_m, eta)
    if cap == nil then
        return nil, reason
    end
    return math.min(a_desired, cap)
end

--- One guidance point for the variable-amplitude weave, plane-anchored.
function M.var_amplitude_guidance(px, py, psi_i, tx, ty, s_m, v_rel_ms,
                                  lambda_m, r_min_m, a_cap_m, d_full_m, lead_s,
                                  eta, look_ahead_m, phase_rad, v_floor_ms)
    local dx, dy = tx - px, ty - py
    local distance_m = math.sqrt(dx * dx + dy * dy)
    if distance_m < 1e-6 then
        return nil, "aircraft is on the target; no weave direction"
    end
    local ux, uy = dx / distance_m, dy / distance_m
    local nx, ny = -uy, ux
    local u = math.min(look_ahead_m, distance_m)
    local line_x = px + u * ux
    local line_y = py + u * uy

    local amp, reason = M.speed_scaled_amplitude(distance_m, v_rel_ms, lambda_m,
                                                 r_min_m, a_cap_m, d_full_m,
                                                 lead_s, eta, v_floor_ms)
    if amp == nil then
        return nil, reason
    end
    local y, yp, ypp = M.wave_derivatives(s_m, amp, lambda_m, phase_rad)
    return {
        gx = line_x + nx * y,
        gy = line_y + ny * y,
        amplitude = amp,
        curvature = M.curvature(yp, ypp),
        curvature_limit = M.curvature_limited_amplitude(lambda_m, r_min_m, eta),
        onset_m = M.onset_distance_m(d_full_m, v_rel_ms, lead_s, v_floor_ms),
    }
end

-- ---------------------------------------------------------
-- heading_a -- TASK-010
-- ---------------------------------------------------------

--- Guidance point one look-ahead along the bearing to the target.
--  The bearing convention is implicit: the point is placed along the unit vector
--  to the target, which is the same thing as forming atan2(dEast, dNorth)
--  without forming the angle.
function M.heading_guidance(px, py, tx, ty, look_ahead_m)
    local dx, dy = tx - px, ty - py
    local distance_m = math.sqrt(dx * dx + dy * dy)
    if distance_m < 1e-6 then
        return nil, "aircraft is on the target; no heading to align to"
    end
    local u = math.min(look_ahead_m, distance_m)
    return px + u * dx / distance_m, py + u * dy / distance_m
end

-- ---------------------------------------------------------
-- The shared orbit ramp -- what makes each *_orbit variant
-- ---------------------------------------------------------

--- Blend weight for the approach-to-orbit handoff, in [0, 1].
--  0 outside the transition band (range >= R + look_ahead): pure approach.
--  1 on or inside the ring: pure orbit. Linear in between. The band is one
--  look-ahead wide, so the ramp starts exactly where the old hard switch fired
--  and finishes at the ring.
--
--  NOTE this is the RAMPED handoff, which is a different mechanism from the
--  RAMP-FREE continuation of harness_cs_orbit. The `_orbit` suffix means both
--  things in the registry and the catalogue's "Approach -> hold" field is what
--  distinguishes them; nothing here resolves that naming problem.
function M.ramp_weight(range_m, orbit_radius_m, look_ahead_m)
    local band = look_ahead_m
    local outer = orbit_radius_m + band
    if band <= 0.0 then
        if range_m > orbit_radius_m then
            return 0.0
        end
        return 1.0
    end
    if range_m >= outer then
        return 0.0
    end
    if range_m <= orbit_radius_m then
        return 1.0
    end
    return (outer - range_m) / band
end

--- heading_a ramped into the orbit. Returns gx, gy, phase, or nil plus a reason.
function M.heading_orbit_guidance(px, py, psi_i, tx, ty, orbit_radius_m,
                                  look_ahead_m, precompensate)
    local dx, dy = tx - px, ty - py
    local range_m = math.sqrt(dx * dx + dy * dy)
    local w = M.ramp_weight(range_m, orbit_radius_m, look_ahead_m)

    local hx, hy = nil, nil
    if w < 1.0 then
        hx, hy = M.heading_guidance(px, py, tx, ty, look_ahead_m)
    end
    local ox, oy = nil, nil
    if w > 0.0 then
        if range_m >= 1e-6 then
            local psi0 = orbit.entry_angle(px, py, tx, ty)
            local direction = orbit.orbit_direction(psi0, psi_i)
            ox, oy = orbit.orbit_guidance_point(tx, ty, orbit_radius_m, psi0,
                                                direction, look_ahead_m,
                                                precompensate)
        end
    end

    if hx == nil and ox == nil then
        return nil, "no heading approach and no orbit point (on the target?)"
    end
    if ox == nil then
        return hx, hy, 0.0
    end
    if hx == nil then
        return ox, oy, 1.0
    end
    return (1.0 - w) * hx + w * ox, (1.0 - w) * hy + w * oy, w
end

return M
