-- Flight-test Wrapper for physical validation
-- Keep aircraft-specific limits/configuration separate from geometry.
--
-- Concept
-- Choose the flight mode 'arm' by name through sitl_arms. Every arm already meets
--     entry(snapshot, cfg) -> {guidance_n_m, guidance_e_m, algorithm_state} | error: nil, reason
--     and sitl_arms.resolve(name) maps a name to it. 
--
--     Arm       Name                            Lua                                      Needs
--     Baseline  dubins_target_orbit (or _hyst)  sitl_arms adapter over harness_cs_orbit  target or estimate
--     A         adaptive_db_circle              harness_adaptive_db.guidance_point       target_est, lookahead_steps 25
--     F         carrot_shift_cs                 harness_carrot_shift.guidance_point      target_est_raw, lookahead_steps 0
--
-- Before use:
--   * Write it from shared code, not copies: heading_from, command, release,
--     log_tick and anchor exist twice already (runner and demo). Lift them into
--     one module (say sitl_vehicle_io.lua) that all three scripts require.
--
-- SD card requireements
-- scripts/hardware_val.lua; 
-- scripts/modules/: sitl_arms.lua, 
-- the HARNESS_MODULES files (harness_geom, harness_dubins, harness_orbit,
-- harness_cs_orbit, harness_kangaroo, harness_segments, harness_estimator,
-- harness_adaptive_dbm harness_carrot_shift, sitl_adsb.lua, the config
-- module). 

-- SITL demo used: SCR_VM_I_COUNT 1000000 and SCR_HEAP_SIZE 2 MiB,
-- Arm A implementation was 252k to 846k instructions per tick range. 
--
-- Validation ladder (the same file all the way):
--   1. SITL: a KangarooFollowHardwareVal autotest modelled on KangarooFollowDemo,
--      flying this file through the 24-step matrix per arm; unit-test it under
--      stubbed bindings as test_sitl_demo.py does.
--   2. Bench: shadow mode on the flight controller, virtual target; measure
--      instructions, heap and tick time per arm.
--   3. Flight, shadow: compute and log only; replay the log through Python.
--   4. Flight, active, virtual stationary point: baseline, then A, then F.
--   5. Virtual moving target at ratio 0.5 or below.
--   6. Live target.
--   Each rung's logs go through extract_bundle.py into the same comparison as
--   the Python and SITL runs.

-- ============================================================
-- 1. Configuration 
-- ============================================================
-- PLAN: do not retype constants. demo.py --stage --campaign <CAMP-002>
-- --cell A-straight-constant-half --look-ahead-m 30 already writes
-- kangaroo_demo_cfg.lua from a campaign cell's spec (cfg, dt_s, estimate,
-- lookahead_steps, estimator noise). Copy it to modules/ (or add a
-- --stage-hardware variant) and require it.

-- PLAN: live settings as a parameter table, like SHR_ / KDEM_: HVAL_ENABLE
-- (default 0), HVAL_ARM (0 baseline, 1 A, 2 F), HVAL_OUT (0 shadow, 1 active),
-- HVAL_TGT (0 virtual point, 1 virtual moving, 2 live), HVAL_ALT_M,
-- HVAL_BOUND_M, HVAL_ACT_FN.

-- Algorithm IDs/names, test altitude and altitude reference.
-- Orbit radius, minimum turn radius, airspeed/bank limits.
-- Target-data timeout, test boundary, nominal update interval.
-- Recovery policy for each fault type.
-- Default experimental guidance to DISABLED.

local state = {
    enabled = false,
    active_algorithm = nil,
    fault_latched = false,
    path = nil,
    point_index = 1,
    last_update_ms = nil,
    run_id = 0,
}

-- Parameter table
-- ============================================================

--[[
    // @Param: HVAL_ENABLE
    // @DisplayName: Plane Follow standoff orbit enable
    // @Description: When 1, lateral guidance is the standoff-orbit behaviour: a circle-straight approach onto a ring of HVAL_RADIUS about the target, arriving tangent to it, then an orbit on the ring with a pre-compensated carrot, steered by a course command at the carrot. Needs the standoff_orbit module, SCR_VM_I_COUNT of 200000 and SCR_HEAP_SIZE of 1048576. When 0 the applet behaves exactly as before.
    // @Values: 0:Disabled,1:Enabled
--]]
HVAL_ENABLE = bind_add_param("SO_ENABLE", 26, 0)

--[[
    // @Param: HVAL_RADIUS
    // @DisplayName: Plane Follow standoff ring radius
    // @Description: Radius of the standoff ring about the target that the standoff-orbit behaviour approaches and orbits. Must be at least the turn radius in use (HVAL_RHO or the derived floor).
    // @Range: 20 500
    // @Units: m
--]]
HVAL_RADIUS = bind_add_param("SO_RADIUS", 27, 70)

--[[
    // @Param: HVAL_LKAHD
    // @DisplayName: Plane Follow standoff carrot look-ahead
    // @Description: Distance along the planned path at which the standoff-orbit guidance point (the carrot) is placed. Distinct from FOLLP_LKAHD, which is seconds of along-track projection for the speed law. Must be below 80 degrees of arc around the ring (1.4 x HVAL_RADIUS).
    // @Range: 1 200
    // @Units: m
--]]
HVAL_LKAHD = bind_add_param("SO_LKAHD", 28, 50)

--[[
    // @Param: HVAL_RHO
    // @DisplayName: Plane Follow standoff turn radius
    // @Description: Minimum turn radius used to build the standoff-orbit approach. 0 derives it each tick from the bank limit in force (the smaller of ROLL_LIMIT_DEG and the heading command's acceleration limit) and the current airspeed. A value below that floor is reported once and the floor is used instead.
    // @Range: 0 500
    // @Units: m
--]]
HVAL_RHO = bind_add_param("SO_RHO", 29, 0)

--[[
    // @Param: HVAL_MARGIN
    // @DisplayName: Plane Follow standoff sense margin
    // @Description: Hysteresis margin, in metres of turn-in path length, by which the opposite orbit sense must be cheaper before the standoff-orbit approach switches sense. 0 re-selects the sense every tick.
    // @Range: 0 100
    // @Units: m
--]]
HVAL_MARGIN = bind_add_param("SO_MARGIN", 30, 10)

--[[
    // @Param: HVAL_PRECOMP
    // @DisplayName: Plane Follow standoff ring pre-compensation
    // @Description: When 1 the orbit carrot is placed on a larger virtual ring, R / cos(L / R), so the circle actually flown is HVAL_RADIUS. When 0 the carrot is placed on the ring itself and the flown circle settles inside it at R cos(L / R).
    // @Values: 0:Off,1:On
--]]
HVAL_PRECOMP = bind_add_param("SO_PRECOMP", 18, 1)

--[[
    // @Param: HVAL_ASPD
    // @DisplayName: Plane Follow standoff airspeed
    // @Description: Airspeed to command while the standoff-orbit behaviour is selected. 0 leaves the applet's speed law in charge, which steers airspeed toward the target's and is not what a slow ground target wants. The standoff geometry assumes one airspeed.
    // @Range: 0 100
    // @Units: m/s
--]]
HVAL_ASPD = bind_add_param("SO_ASPD", 19, 0)

-- ============================================================
-- 2. INPUTS AND AIRCRAFT STATE
-- ============================================================
-- PLAN: reuse heading_from(vel) from the runner / demo (course from
-- math.atan(ve, vn) in (-pi, pi], yaw when slow); that wrapping fixed
-- TASK-058's phantom right turn. Local frame: origin:get_distance_NE(pos),
-- anchored at engagement, as anchor() does.
-- Read pilot enable and algorithm-selection inputs.
-- Read actual flight mode, arming/flying state and aircraft pose.
-- Read airspeed, ground velocity, altitude and health indicators.
-- Treat missing/invalid inputs explicitly; never substitute zero.
-- Use a consistent local North/East frame for geometry.

-- ArduPilot parameters this script reads 
local AIRSPEED_MIN = Parameter('AIRSPEED_MIN')
local ROLL_LIMIT_DEG = Parameter('ROLL_LIMIT_DEG')
local GRAVITY_MSS = 9.80665
-- 0 = derive the floor; >0 = requested radius, m
local HVAL_RHO_M = bind("RHO_M", 9, 0)


-- When the aircraft is not moving - the standoff course needs to be the 
-- ground course from the NED velocity (the direction of motion), or the yaw
-- when the aircraft is not moving
function standoff.course(vel)
   if vel ~= nil then
      local vn, ve = vel:x(), vel:y()
      if (vn * vn + ve * ve) >= (STANDOFF_MIN_COURSE_SPEED * STANDOFF_MIN_COURSE_SPEED) then
         return math.atan(ve, vn)
      end
   end
   return ahrs:get_yaw_rad()
end



-- ============================================================
-- 3. TARGET ACQUISITION AND ESTIMATION
-- ============================================================
-- PLAN: one function returning kn, ke, kvn, kve, sample_ms, three sources:
--   virtual point / moving: harness_segments.make_segments + state_at, as the
--     demo (no second vehicle); sitl_adsb.lua shows it on the GCS map;
--   live: follow:get_target_location_and_velocity() and
--     follow:get_last_update_ms() (the AP_Follow plumbing in plane_follow.lua
--     and the standoff code moved below), converted to the anchor frame.


-- PLAN: harness_estimator.new / update / predict in the runner's order. Call
-- update only on a new sample, with the measured dt since the last one. A gets
-- target_est (projected) only; F gets target_est_raw only (no double lead).


-- PLAN: with a live target the baseline has no "truth": decide whether it
-- steers on the measurement or the raw estimate, and record it.
-- Choose stationary virtual, moving virtual or live target.
-- Store sample timestamp separately from receive timestamp.
-- Reject stale, out-of-order and implausible samples.
-- Update estimator only when an accepted measurement arrives.
-- Predict to the required horizon using measured elapsed time.



-- ============================================================
-- 4. PATH RESET AND ALGORITHM SWITCHING
-- ============================================================
-- PLAN: reset = algorithm_state = {} plus estimator re-init, all the demo's
-- anchor() does; the arms carry no other state.
local function reset_path()
    state.path = nil
    state.point_index = 1
    -- Clear controller_busy and algorithm-specific cached geometry.
    -- Reset estimator only when required by the test protocol.
end

-- Debounce selection inputs.
-- Initially accept algorithm changes only while disabled.
-- On accepted change: reset path, initialise algorithm, log event.
-- Later permit airborne switching only after transition testing.

-- ============================================================
-- 5. AUTHORITY AND ENGAGEMENT GATES
-- ============================================================
-- PLAN: the runner's start conditions (arming:is_armed(), vehicle:get_mode()
-- == GUIDED) plus a fresh target and rc:get_aux_cached(HVAL_ACT_FN), the
-- FOLLP_ACT_FN pattern. On leaving GUIDED call the runner's release().


-- Experimental commands require:
--   explicit enable + eligible flight mode + established flight
--   + valid aircraft state + fresh target + no latched fault.
--
-- If the pilot leaves GUIDED:
--   stop commands immediately, clear engagement state,
--   and require deliberate re-engagement.
-- Never force GUIDED repeatedly or override a failsafe mode.

-- ============================================================
-- 6. GUIDANCE DISPATCH
-- ============================================================
-- PLAN: use the existing snapshot contract instead of a new :step() interface:
--   local snapshot = { t_s = t, plane_n_m = pn, plane_e_m = pe, plane_hdg_rad = hdg,
--     target_n_m = kn, target_e_m = ke, target_vn_ms = kvn, target_ve_ms = kve,
--     algorithm_state = algorithm_state, target_est = est_proj, target_est_raw = est_raw }
--   local result, reason = entry(snapshot, cfg)  -- entry = arms.resolve(ARM_NAMES[HVAL_ARM])
-- Give every algorithm the same input/output interface:
--
-- result = algorithms[selected]:step({
--     aircraft = aircraft,
--     target = estimated_target,
--     dt = actual_elapsed_seconds,
--     config = config,
-- })
--
-- result should contain:
--   valid, guidance_point, path metadata, diagnostic/fault reason.
--
-- Keep servo, throttle and vehicle-mode commands out of algorithms.

-- ============================================================
-- 7. OUTPUT VALIDATION
-- ============================================================
-- PLAN: turn-radius floor from standoff.turn_radius() in the moved code
-- (ROLL_LIMIT_DEG and airspeed); refuse to engage if cfg.turn_radius_m is below
-- it (PHYSICAL_FLIGHT_PROCESS.md: 45 m against 63.7 m at 45 deg). Boundary: the
-- demo's contain() / bound logic and a point-in-box check on the carrot. Reject
-- non-finite points and carrot jumps above about 2L per tick. An arm's
-- nil, reason is a fault, as the runner treats it (SHR_DONE 2).
-- Check coordinates are finite and altitude frame is explicit.
-- Check guidance point/path against the approved test boundary.
-- Check geometric feasibility and excessive command-point jumps.
-- Check measured aircraft envelope and stale calculations.
-- Reject invalid results; do not silently clip bad geometry.


-- derived turn radius
-- section 7: the smallest turn radius the bank limit allows at this airspeed.
-- The heading command is sent with acceleration g*tan(ROLL_LIMIT_DEG) (runner's
-- command()), so ROLL_LIMIT_DEG is the only bank limit; no fixed 10 m/s/s cap.
local warned_rho = false
local function turn_radius(airspeed)
    local bank_deg = ROLL_LIMIT_DEG:get()
    if bank_deg == nil or bank_deg <= 0 or bank_deg >= 89 then
        return nil, "ROLL_LIMIT_DEG unreadable or out of range"
    end
    local v = airspeed
    local v_min = AIRSPEED_MIN:get()
    if v == nil or (v_min ~= nil and v < v_min) then
        v = v_min
    end
    if v == nil or v <= 0 then
        return nil, "no usable airspeed"
    end
    local floor = (v * v) / (GRAVITY_MSS * math.tan(math.rad(bank_deg)))
    local rho = HVAL_RHO_M:get()
    if rho == nil or rho <= 0 then
        return floor
    end
    if rho < floor then
        if not warned_rho then
            gcs:send_text(4, string.format(
                "HVAL: RHO %.0f m below %.0f m floor at %.0f deg %.0f m/s; using floor",
                rho, floor, bank_deg, v))
            warned_rho = true
        end
        return floor
    end
    return rho
end


-- the turn radius the approach is built with: HVAL_RHO, or the floor the
-- bank limit in force allows at this airspeed. A requested radius below the
-- floor cannot be flown; it is reported once and the floor is used.
function standoff.turn_radius(airspeed)
   local bank_deg = math.deg(math.atan(STANDOFF_HEADING_ACCEL / GRAVITY_MSS))
   local roll_limit = ROLL_LIMIT_DEG:get()
   if roll_limit ~= nil and roll_limit > 0 and roll_limit < bank_deg then
      bank_deg = roll_limit
   end
   local v = airspeed
   if v == nil or v < airspeed_min then
      v = airspeed_min
   end
   local floor = (v * v) / (GRAVITY_MSS * math.tan(math.rad(bank_deg)))
   local rho = HVAL_RHO:get()
   if rho == nil or rho <= 0 then
      return floor
   end
   if rho < floor then
      if not standoff.warned_rho then
         gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. string.format(
            ": HVAL_RHO %.0f m is below the %.0f m floor at %.0f deg and %.0f m/s; using the floor",
            rho, floor, bank_deg, v))
         standoff.warned_rho = true
      end
      return floor
   end
   return rho
end





-- ============================================================
-- 8. COMMAND OUTPUT / SHADOW MODE
-- ============================================================
-- PLAN: reuse the runner's command() (GUIDED_CHANGE_ALTITUDE once, then
-- GUIDED_CHANGE_HEADING as COG, acceleration g * tan(ROLL_LIMIT_DEG)) and
-- release(); shadow = the same tick without command(). Needs GUIDED_P ~15000.
-- PLAN: bank limit (TASK-061): at 45 deg the 70 m ring was not held in SITL
-- (36.7 m RMS on the stationary point); at 60 deg about 6 m. Fly a larger R at
-- 45 deg or get a higher limit approved; ROLL_LIMIT_DEG is a safety parameter.
-- SHADOW: compute and log outputs, but issue no flight commands.
-- ACTIVE: send validated guidance through the chosen Plane API.
-- Check command return status and record rejected commands.
-- Recheck authority immediately before sending the command.



-- ============================================================
-- 9. FAULT HANDLING AND RECOVERY
-- ============================================================
-- PLAN: latch, release(), then one mode change to the tested recovery mode
-- (plane_follow uses FOLLP_FAIL_MODE); yield if the pilot or a failsafe already
-- changed mode.
-- Latch faults to prevent repeated engage/recover oscillation.
-- For a detected fault while this wrapper owns guidance:
--   stop experimental commands and request the tested recovery.
-- Verify recovery-mode acceptance; report failure.
-- If pilot/failsafe already took control, yield to that mode.
--
-- A crashed script cannot execute its own recovery logic.
-- Validate that case separately; this handler covers detected faults.



-- ============================================================
-- 10. LOGGING
-- ============================================================
-- PLAN: write the runner's log_tick rows (HANC once; HREC / HEST / HALG / HAIR
-- each tick) so extract_bundle.py and the metrics read a flight log unchanged
-- and it lands in the same compare.csv as Python and SITL. Replay logged poses
-- through Python to check the carrot tick for tick (TASK-058: 1 cm). Add one
-- HVAL row: run id, arm, enable, mode, target age, command accepted.
-- Log timestamp, run ID, algorithm, enable state and flight mode.
-- Log actual dt, target age, estimate, prediction and guidance point.
-- Log aircraft position, airspeed, ground velocity and wind.
-- Log radial error, bank, altitude and command acceptance.
-- Log all switching, engagement, disengagement and fault events.
-- Rate-limit ground-station text; retain detailed onboard logs.




-- ============================================================
-- 11. UPDATE CALLBACK
-- ============================================================
-- PLAN: this order already matches the runner's step().
local function update()
    -- 1. Read time, controls and aircraft state.
    -- 2. Yield immediately if pilot/failsafe has taken authority.
    -- 3. Process disengagement and permitted algorithm selection.
    -- 4. Acquire target and update estimator.
    -- 5. Check engagement gates.
    -- 6. Compute guidance using actual elapsed time.
    -- 7. Validate output and aircraft envelope.
    -- 8. Log; command only if ACTIVE and still authorised.
    -- 9. Handle detected faults and verify recovery.

    return update, 100 -- nominal callback delay in milliseconds
end



-- ============================================================
-- 12. STARTUP
-- ============================================================
-- PLAN: the runner's pcall(require, ...) / fail_load pattern; announce the arm
-- and configuration; start disabled.
-- Validate configuration and required inputs/bindings.
-- Announce script version and selected test configuration.
-- Start disabled; do not arm or change flight mode on startup.


function standoff.load()
   if standoff.module ~= nil then
      return standoff.module
   end
   if standoff.load_failed then
      return nil
   end
   local ok, mod = pcall(require, "standoff_orbit")
   if not ok or mod == nil then
      standoff.load_failed = true
      gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT .. ": standoff_orbit module not found; HVAL_ENABLE ignored")
      return nil
   end
   standoff.module = mod
   return mod
end

return update, 100



--- add the plane_follow.lua scipt changes here
--[==[
Moved 2026-10-03 from libraries/AP_Scripting/applets/plane_follow.lua, which
was restored to the upstream original (identical to upstream/master at
8a0f0c1e4c). This is the standoff-orbit behaviour added to the applet for
TASK-055 (HVAL_ENABLE): every line the session added, in order, with
where it sat and any original line it replaced. Kept inside this comment
because it runs inside plane_follow.lua (it uses the applet's bind_add_param,
MAV_SEVERITY, now_ms, wrap_360, airspeed_min, set_vehicle_heading, follow_mode
and Update()) and because statements may not follow the return above.

-- ---- [1] plane_follow.lua, hunk near original line 23
-- ---- placed after the original line: FOLLP_TURN_DEG - if the target is more than this many degrees left or right, assume it's turning
   FOLLP_SO_* - optional standoff-orbit behaviour (FOLLP_SO_ENABLE = 1): a circle-straight
   approach onto a ring about the target and an orbit on it, from the standoff_orbit module.
   With FOLLP_SO_ENABLE = 0 (the default) the applet is unchanged.

-- ---- [2] plane_follow.lua, hunk near original line 23
-- ---- placed after the original line: 
-- ---- REPLACED these original line(s):
--      SCRIPT_VERSION = "4.7.0-075"
-- ---- with:
SCRIPT_VERSION = "4.7.0-076"

-- ---- [3] plane_follow.lua, hunk near original line 276 (in: FOLLP_XT_I_MAX = bind_add_param("XT_I_MAX", 24, 100))
-- ---- placed after the original line: 



-- ---- [4] plane_follow.lua, hunk near original line 295 (in: AIRSPEED_MIN = Parameter('AIRSPEED_MIN'))
-- ---- placed after the original line: WINDSPEED_MAX = Parameter('AHRS_WIND_MAX')
ROLL_LIMIT_DEG = Parameter('ROLL_LIMIT_DEG')

-- ---- [5] plane_follow.lua, hunk near original line 537 (in: local simulate_failure = {)
-- ---- placed after the original line: 
-------------------------------------------------------------------------------
--- Standoff orbit behaviour (FOLLP_SO_ENABLE = 1)
-------------------------------------------------------------------------------
--[[
   Lateral guidance from the standoff_orbit module: one guidance point (the
   carrot) per tick from the aircraft pose and the target position, on a
   circle-straight approach onto the ring of FOLLP_SO_RADIUS about the target
   and then around the ring. The bearing to the carrot is sent through the
   applet's existing heading command as a course-over-ground command, so the
   aircraft flies AT the carrot; the cross-track PID and the heading
   heuristics do not apply while it is selected. Everything else in the
   applet (activation, target loss, exit and fail modes, altitude, the speed
   law unless FOLLP_SO_ASPD is set) is unchanged.

   The local frame is the aircraft's own position each tick (East, North
   metres); the only state carried is the previous orbit sense.
--]]
local STANDOFF_DELTA_PSI_RAD = math.rad(5.0)   -- arc sampling step of the approach path
local STANDOFF_DELTA_D_M = 0.5                 -- straight sampling step, metres
local STANDOFF_HEADING_ACCEL = 10.0            -- the acceleration set_vehicle_heading() sends, m/s/s
local STANDOFF_MIN_COURSE_SPEED = 1.0          -- below this ground speed the course is the yaw, m/s
local STANDOFF_MSG_INTERVAL_MS = 5000
local GRAVITY_MSS = 9.80665
local STANDOFF_PHASE_CODE = { approach = 0, orbit = 1 }
local STANDOFF_DIR_CODE = { cw = 1, ccw = -1 }

local standoff = {
   module = nil,
   load_failed = false,
   sense = nil,          -- "cw" / "ccw" from the previous tick, nil at activation
   t0_ms = nil,
   anchor = nil,         -- Location at activation; the frame the log rows are written in
   warned_rho = false,
   last_refusal_ms = nil,
}

local function standoff_num(v)
   if v == nil then return 0.0 end
   if v == true then return 1.0 end
   if v == false then return 0.0 end
   return v
end

function standoff.reset()
   standoff.sense = nil
   standoff.t0_ms = nil
   standoff.anchor = nil
   standoff.warned_rho = false
   standoff.last_refusal_ms = nil
end







-- one tick: the carrot and the course to it. Returns nil when the module is
-- missing or the geometry refuses, and the applet's own heading law applies.
function standoff.compute(current_location, target_location, target_velocity, airspeed)
   local mod = standoff.load()
   if mod == nil then
      return nil
   end
   local vel = ahrs:get_velocity_NED()
   local psi = standoff.course(vel)
   local yaw = ahrs:get_yaw_rad()
   if standoff.t0_ms == nil then
      standoff.t0_ms = now_ms
      standoff.anchor = current_location:copy()
      logger:write('HANC', 't0ms,Lat,Lng,Alt,Hdg,Yaw,AltCmd', 'Iiiffff',
                   now_ms, current_location:lat(), current_location:lng(),
                   current_location:alt() * 0.01, psi, yaw, current_location:alt() * 0.01)
      gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. string.format(
         ": standoff orbit R %.0f m L %.0f m", FOLLP_SO_RADIUS:get(), FOLLP_SO_LKAHD:get()))
   end
   -- the frame: aircraft at the origin, x East, y North
   local ofs = current_location:get_distance_NED(target_location)
   local tx, ty = ofs:y(), ofs:x()
   local R = FOLLP_SO_RADIUS:get()
   local L = FOLLP_SO_LKAHD:get()
   local rho = standoff.turn_radius(airspeed)
   local margin = FOLLP_SO_MARGIN:get()
   local precomp = (FOLLP_SO_PRECOMP:get() ~= 0)
   local g, reason = mod.guidance(0.0, 0.0, psi, tx, ty, R, rho, L,
                                  STANDOFF_DELTA_PSI_RAD, STANDOFF_DELTA_D_M,
                                  precomp, standoff.sense, margin)
   if g == nil then
      if standoff.last_refusal_ms == nil or (now_ms - standoff.last_refusal_ms) > STANDOFF_MSG_INTERVAL_MS then
         gcs:send_text(MAV_SEVERITY.WARNING, SCRIPT_NAME_SHORT .. ": standoff refused: " .. tostring(reason))
         standoff.last_refusal_ms = now_ms
      end
      return nil
   end
   local previous = standoff.sense
   if previous ~= nil and g.direction ~= previous then
      gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT .. ": standoff sense " .. previous .. " to " .. g.direction)
   end
   standoff.sense = g.direction
   return {
      heading_deg = wrap_360(math.deg(math.atan(g.gx, g.gy))),
      g = g,
      previous = previous,
      t_s = (now_ms - standoff.t0_ms):tofloat() * 0.001,
      tx = tx, ty = ty, psi = psi, yaw = yaw, vel = vel, rho = rho,
      target_velocity = target_velocity,
   }
end

-- the harness record, in the frame anchored at activation, so the same
-- extractor and metrics that read the research SITL runs read this log
function standoff.log(current_location, r)
   local rel = standoff.anchor:get_distance_NED(current_location)
   local pn, pe = rel:x(), rel:y()
   local tvn, tve = 0.0, 0.0
   if r.target_velocity ~= nil then
      tvn, tve = r.target_velocity:x(), r.target_velocity:y()
   end
   logger:write('HREC', 't,PN,PE,PHdg,TN,TE,TVN,TVE,GN,GE', 'ffffffffff',
                r.t_s, pn, pe, r.psi, pn + r.ty, pe + r.tx, tvn, tve,
                pn + r.g.gy, pe + r.g.gx)
   local held = (r.previous ~= nil) and (r.g.direction == r.previous)
   local switched = (r.previous ~= nil) and (r.g.direction ~= r.previous)
   logger:write('HALG', 't,Ph,Dir,Cur,Rep,Tsr,Held,Sw,K,Ring', 'ffffffffff',
                r.t_s, standoff_num(STANDOFF_PHASE_CODE[r.g.phase]),
                standoff_num(STANDOFF_DIR_CODE[r.g.direction]),
                standoff_num(r.g.curvature), 0.0, 0.0,
                standoff_num(held), standoff_num(switched), 0.0,
                standoff_num(r.g.ring_angle_rad))
   local wind = ahrs:get_wind()
   local aspd = ahrs:airspeed_estimate() or 0.0
   local gvn, gve = 0.0, 0.0
   if r.vel ~= nil then gvn, gve = r.vel:x(), r.vel:y() end
   local wn, we = 0.0, 0.0
   if wind ~= nil then wn, we = wind:x(), wind:y() end
   logger:write('HAIR', 't,AS,GVN,GVE,WN,WE,Yaw,Roll,Crs', 'fffffffff',
                r.t_s, aspd, gvn, gve, wn, we, r.yaw, ahrs:get_roll_rad(), r.psi)
end


-- ---- [6] plane_follow.lua, hunk near original line 590 (in: local follow_mode = {)
-- ---- placed after the original line: xt_pid.reset()
      standoff.reset()

-- ---- [7] plane_follow.lua, hunk near original line 908 (in: function Update())
-- ---- placed after the original line: 
   -- standoff orbit (FOLLP_SO_ENABLE = 1): the course to the carrot replaces the heading heuristics below
   local standoff_result = nil
   if FOLLP_SO_ENABLE:get() == 1 then
      standoff_result = standoff.compute(current_location, target_location, target_velocity, vehicle_airspeed)
   end


-- ---- [8] plane_follow.lua, hunk near original line 908 (in: function Update())
-- ---- placed after the original line: -- target_heading - vehicle_heading catches the circumstance where the target vehicle is heading in completely the opposite direction
-- ---- REPLACED these original line(s):
--         if (math.abs(along_track_distance) < airspeed_max * 0.75 or (math.abs(cross_track_distance) < airspeed_max * 0.25)) or
-- ---- with:
   if standoff_result ~= nil then
      desired_heading = standoff_result.heading_deg
      mechanism = 3 -- standoff carrot - for logging
   elseif (math.abs(along_track_distance) < airspeed_max * 0.75 or (math.abs(cross_track_distance) < airspeed_max * 0.25)) or

-- ---- [9] plane_follow.lua, hunk near original line 922 (in: function Update())
-- ---- placed after the original line: -- The desired heading needs a PID controller for crosstrack, but only when it gets close.
-- ---- REPLACED these original line(s):
--         if close or too_close_follow_up > 0 then
-- ---- with:
   if (close or too_close_follow_up > 0) and standoff_result == nil then

-- ---- [10] plane_follow.lua, hunk near original line 944 (in: function Update())
-- ---- placed after the original line: 
   -- standoff orbit: hold one airspeed when asked to, since the ring geometry assumes one
   if standoff_result ~= nil and FOLLP_SO_ASPD:get() > 0 then
      airspeed_new = FOLLP_SO_ASPD:get()
   end


-- ---- [11] plane_follow.lua, hunk near original line 944 (in: function Update())
-- ---- placed after the original line: -- Finally after all the calculations - send the target heading, altitude and airspeed to AP
-- ---- REPLACED these original line(s):
--         set_vehicle_heading({heading = desired_heading})
-- ---- with:
   if standoff_result ~= nil then
      -- course over ground at the carrot: the aircraft flies AT the point, as the standoff geometry assumes
      set_vehicle_heading({heading = desired_heading, type = MAV_HEADING_TYPE.COG})
   else
      set_vehicle_heading({heading = desired_heading})
   end

-- ---- [12] plane_follow.lua, hunk near original line 986 (in: function Update())
-- ---- placed after the original line: )
   if standoff_result ~= nil then
      standoff.log(current_location, standoff_result)
   end

]==]


--- drop MD additional content here
--[==[
Moved 2026-10-03 from libraries/AP_Scripting/applets/plane_follow.md (restored
to the upstream original): the standoff-orbit documentation, Markdown.

<!-- ---- [1] plane_follow.md, hunk near original line 99 (in: as the error. This is the D gain for the "V" PID controller.) -->
<!-- ---- placed after the original line:  -->
## FOLLP_SO_ENABLE

Selects the optional standoff-orbit behaviour (see "Standoff orbit behaviour"
below). 0, the default, leaves the applet exactly as it was. 1 replaces the
lateral guidance with a circle-straight approach onto a ring about the target
and an orbit on that ring.

## FOLLP_SO_RADIUS

Radius of the standoff ring about the target, metres (default 70). It must be
at least the turn radius in use.

## FOLLP_SO_LKAHD

The standoff carrot: the distance along the planned path at which the guidance
point is placed, metres (default 50). This is a distance, not the seconds of
FOLLP_LKAHD, which keeps its meaning for the speed law. It must stay below 80
degrees of arc around the ring (about 1.4 x FOLLP_SO_RADIUS).

## FOLLP_SO_RHO

Minimum turn radius used to build the approach, metres. 0 (default) derives it
each tick from the bank limit in force (the smaller of ROLL_LIMIT_DEG and the
45.6 degrees implied by the heading command's acceleration) and the current
airspeed: 63.7 m at 25 m/s and 45 degrees. A value below that floor is reported
once on the GCS and the floor is used instead.

## FOLLP_SO_MARGIN

Hysteresis margin for the orbit sense, metres of turn-in path length (default
10). Once a sense (clockwise or counter-clockwise) is chosen it is kept unless
the other sense's approach is cheaper by more than this. 0 re-selects every
tick, which on the line of sight to the target makes the sense alternate.

## FOLLP_SO_PRECOMP

1 (default) places the orbit carrot on a virtual ring of radius
R / cos(L / R) so that the circle actually flown is FOLLP_SO_RADIUS. 0 places
it on the ring itself; the aircraft then flies the chord to it and settles on
a smaller circle of radius R cos(L / R) (52.9 m for the defaults).

## FOLLP_SO_ASPD

Airspeed to command while the standoff is selected, m/s. 0 (default) leaves
the applet's speed law in charge; that law steers airspeed toward the target's,
which for a slow ground target means AIRSPEED_MIN. The standoff geometry
assumes one airspeed, so set this to the cruise speed the turn radius was
chosen for.


<!-- ---- [2] plane_follow.md, hunk near original line 108 (in: controller's microSD card on the FOLLOW plane. s) -->
<!-- ---- placed after the original line: in the `APM/scripts/modules` directory on the SD card on the FOLLOW plane. -->
If FOLLP_SO_ENABLE will be used, also install standoff_orbit.lua there and set
SCR_VM_I_COUNT = 200000 and SCR_HEAP_SIZE = 1048576 (or the largest the board
allows); the approach geometry samples four candidate paths per update. Set
GUIDED_P to about 15000 as well: the standoff steers by course commands, and
Plane's GUIDED heading controller is proportional, so at the default gain of
5000 a 30 degree course error gives only 27 degrees of bank, which cannot fly
the planned turn.

<!-- ---- [3] plane_follow.md, hunk near original line 145 (in: MAV1_POSITION = 10) -->
<!-- ---- placed after the original line: Ideally the connection is direct plane-to-plane and not routed via a Ground Control Station. This has been tested with 2x HolyBro SiK telemetry radios, one in each plane. RFD900 radios might work and LTE or other IP radio based connections will probably work well, but haven't been tested. Some users have reported using ESP32 WiFi modules configured with one of the radios set to be in station mode. Fast telemetry updates from the target to the following plane will give the best results. -->

## Standoff orbit behaviour

With FOLLP_SO_ENABLE = 1 the applet follows a slow or stationary ground target
the way a fixed-wing aircraft can: it does not try to sit on the target's
heading and speed, it approaches a ring of FOLLP_SO_RADIUS about the target
and circles it. The behaviour comes from the standoff_orbit module, which was
developed and measured as the baseline guidance law of a research programme on
fixed-wing following of wildlife (a Python geometric harness, differential
tests between the Python and the Lua, and the ArduPilot SITL campaign that
repeats the harness runs). Its defaults are that harness's; they are not
flight limits.

Each update the module takes the aircraft's position and ground course and the
target's position from AP_Follow, and returns one guidance point:

- outside the ring, the point 50 m (FOLLP_SO_LKAHD) along a circle-straight
  path: an initial turn of the turn radius followed by a straight that meets
  the ring tangentially, the cheapest of the four turn-direction and
  orbit-sense combinations, with the orbit sense held against FOLLP_SO_MARGIN
  so it does not flip while the aircraft is on the line of sight;
- on or inside the ring, a point ahead around the ring in the sense the
  aircraft arrived with, placed on the pre-compensated ring
  (FOLLP_SO_PRECOMP) so the circle actually flown is FOLLP_SO_RADIUS.

The applet sends the bearing to that point as a course-over-ground heading
command, so the aircraft flies at the point rather than loitering about it;
the cross-track PID and the overshoot and turning heuristics do not apply
while the standoff is selected. Activation, target loss, FOLLP_TIMEOUT,
FOLLP_FAIL_MODE, FOLLP_EXIT_MODE, altitude (FOLL_ALT_TYPE, FOLLP_ALT_OVR) and
the speed law are unchanged; FOLLP_SO_ASPD can hold one airspeed instead.
FOLL_OFS_X/Y/Z are ignored by the standoff, which rings the target itself.

If the module is not installed the applet reports it once and behaves as if
FOLLP_SO_ENABLE were 0. If the geometry has no solution for an update (the
ring is smaller than the turn radius, or the aircraft is at the target) the
applet reports it and uses its ordinary heading law for that update.

The standoff writes its own log messages beside PF1 and PF2: HANC once at
activation (the anchor of the local frame), and HREC, HALG and HAIR each
update (aircraft, target and guidance point in metres from the anchor; the
phase, orbit sense and curvature; airspeed, ground velocity, wind, yaw, roll
and course), so the run can be compared with the research harness's records.

]==]

