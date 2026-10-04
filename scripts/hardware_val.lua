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

-- SHARED CONFIGURATION: spec.json, read as `cfg` (ADR-011)
-- ============================================================
-- The same file, in the same format, the SITL campaign runner and the live
-- demonstration read: generated from a campaign cell's spec by
-- kangaroo_follow (demo.py --stage --cell <cell> [--look-ahead-m 30];
-- schedule.spec_cfg), never typed. One flat table: the HarnessConfig every
-- arm reads and the run fields at the same level, so it goes to the arm
-- unchanged, entry(snapshot, cfg). Copy scripts/spec.json and
-- scripts/modules/sitl_spec.lua to the SD card with this script.
--
--   cfg.algorithm, cfg.estimate, cfg.lookahead_steps, cfg.estimator
--                               the staged arm and its estimator settings
--   cfg.orbit_radius_m          ring radius R, m          (was HVAL_SO_RADIUS)
--   cfg.look_ahead_m            carrot L, m               (was HVAL_SO_LKAHD)
--   cfg.turn_radius_m           planning turn radius, m   (was HVAL_SO_RHO, HVAL_RHO_M)
--   cfg.cs_sense_margin_m       orbit-sense margin, m     (was HVAL_SO_MARGIN)
--   cfg.orbit_precompensate     R / cos(L / R) carrot ring (was HVAL_SO_PRECOMP)
--   cfg.airspeed_ms             the airspeed the geometry assumes (was HVAL_SO_ASPD)
--   cfg.dt_s                    nominal update interval, s (0.1)
--   cfg.bank_limit_deg          the bank the configuration was built for
--   cfg.roll_limit_deg          the ROLL_LIMIT_DEG it must be flown at (60)
--
-- Bank limit (ADR-011): this script never sets ROLL_LIMIT_DEG (SR-004). It
-- refuses to engage unless the live ROLL_LIMIT_DEG equals
-- cfg.roll_limit_deg, so the aircraft only ever flies the configuration
-- that SITL and the demonstration flew. Approving 60 deg on the aircraft is
-- the approver's decision, not this file's.
local spec_ok, spec_mod = pcall(require, "sitl_spec")
local cfg, cfg_where = nil, nil
if spec_ok then
    cfg, cfg_where = spec_mod.load()
else
    cfg_where = "require('sitl_spec'): " .. tostring(spec_mod)
end
if cfg == nil then
    -- start disabled and say why; never fall back to typed defaults
    state.fault_latched = true
    gcs:send_text(3, "HVAL: no configuration: " .. tostring(cfg_where))
end

--- ADR-011 gate for section 5: true only when the live ROLL_LIMIT_DEG is the
--  bank limit spec.json was generated for.
local function roll_limit_ok()
    if cfg == nil then return false, "no spec.json" end
    local ok, _live, msg = spec_mod.roll_limit_check(cfg)
    return ok, msg
end

-- PARAM table
-- ============================================================
-- Live settings only: everything that shapes the guidance is in cfg above.
-- The key must be used by only one script on the flight controller: 87 is
-- taken by an upstream applet, 141 by none; the SITL runner and demo scan
-- upward from 40 for a free key. Full names must fit in 16 characters.
local PARAM_TABLE_KEY = 141
local PARAM_TABLE_PREFIX = "HVAL_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), "HVAL: could not add param table")

local function bind(name, idx, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "HVAL: could not add " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "HVAL: could not bind " .. name)
    return p
end

--[[
    // @Param: HVAL_ENABLE
    // @DisplayName: Hardware validation enable
    // @Description: Master enable for experimental guidance. 0 (default): nothing is computed or commanded. 1: the script may engage when every gate in section 5 passes, including the HVAL_ACT_FN switch and the ROLL_LIMIT_DEG check against spec.json; both this and the switch are required.
    // @Values: 0:Disabled,1:Enabled
--]]
local HVAL_ENABLE = bind("ENABLE", 1, 0)

--[[
    // @Param: HVAL_ARM
    // @DisplayName: Hardware validation arm
    // @Description: Guidance law, resolved by name through sitl_arms (ARM_NAMES below). Must name the arm spec.json was staged for (cfg.algorithm): each arm's spec carries its own estimate and lookahead_steps. Read only while disengaged.
    // @Values: 0:Baseline (dubins_target_orbit),1:Arm A (adaptive_db_circle),2:Arm F (carrot_shift_cs)
--]]
local HVAL_ARM = bind("ARM", 2, 0)

--[[
    // @Param: HVAL_OUT
    // @DisplayName: Hardware validation output
    // @Description: 0 (default) shadow: compute and log the guidance point but send no command. 1 active: send the validated course command (section 8).
    // @Values: 0:Shadow,1:Active
--]]
local HVAL_OUT = bind("OUT", 3, 0)

--[[
    // @Param: HVAL_TGT
    // @DisplayName: Hardware validation target source
    // @Description: 0 virtual stationary point and 1 virtual moving target (harness_segments, as the demo), placed relative to the anchor at engagement; 2 live target from AP_Follow (FOLL_SYSID).
    // @Values: 0:Virtual point,1:Virtual moving,2:Live
--]]
local HVAL_TGT = bind("TGT", 4, 0)

--[[
    // @Param: HVAL_ALT_M
    // @DisplayName: Hardware validation altitude
    // @Description: Altitude commanded while engaged, metres above home (MAV_FRAME_GLOBAL_RELATIVE_ALT, as the runner's GUIDED_CHANGE_ALTITUDE). Also written as AltCmd in HANC. Default from spec.json (cfg.alt_m) when it carries one.
    // @Range: 30 400
    // @Units: m
--]]
local HVAL_ALT_M = bind("ALT_M", 5, (cfg and cfg.alt_m) or 60)

--[[
    // @Param: HVAL_ACT_FN
    // @DisplayName: Hardware validation activation switch
    // @Description: RCx_OPTION scripting function whose switch engages the experimental guidance (high) and disengages it (low), as FOLLP_ACT_FN in plane_follow.lua. 300 to 307 are the Scripting1 to Scripting8 aux functions.
    // @Range: 300 307
--]]
local HVAL_ACT_FN = bind("ACT_FN", 6, 303)

--[[
    // @Param: HVAL_BOUND_M
    // @DisplayName: Hardware validation boundary
    // @Description: Half-side of the square test box centred on the anchor, metres. A guidance point or a virtual target outside it is a fault (section 7). Must fit inside the approved test area.
    // @Range: 100 2000
    // @Units: m
--]]
local HVAL_BOUND_M = bind("BOUND_M", 7, 400)

--[[
    // @Param: HVAL_TGT_TMO
    // @DisplayName: Hardware validation target timeout
    // @Description: A live target sample older than this is stale: the script disengages (or refuses to engage) and latches a fault. Measured from follow:get_last_update_ms().
    // @Range: 0.2 10
    // @Units: s
--]]
local HVAL_TGT_TMO = bind("TGT_TMO", 8, 2)

--[[
    // @Param: HVAL_FAIL_MODE
    // @DisplayName: Hardware validation recovery mode
    // @Description: Flight mode requested once when a fault is detected while this script owns guidance (section 9). Not requested if the pilot or a failsafe has already changed mode.
    // @Values: 11:RTL,12:Loiter
--]]
local HVAL_FAIL_MODE = bind("FAIL_MODE", 9, 12)

-- Algorithm IDs/names: HVAL_ARM -> the name sitl_arms.resolve() takes. It
-- must equal cfg.algorithm (or its _hyst form) for the staged spec.
local ARM_NAMES = {
    [0] = "dubins_target_orbit",
    [1] = "adaptive_db_circle",
    [2] = "carrot_shift_cs",
}

-- Nominal update interval: cfg.dt_s (0.1 s), not a parameter: the
-- estimator and arm A's horizon count in ticks of it.

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
-- The requested turn radius is cfg.turn_radius_m from spec.json (ADR-011),
-- no longer a parameter (was HVAL_RHO_M).


local pos = ahrs:get_location()
local vel = ahrs:get_velocity_NED()
local mode = vehicle:get_mode()
local armed = arming:is_armed()
local airspeed_ms = ahrs:airspeed_estimate()

if pos == nil or vel == nil then
    -- Pass this failure to your engagement/fault handling.
    return
end

local hdg = heading_from(vel)

-- Only after anchor() has established origin:
if origin == nil then
    return
end

local ne = origin:get_distance_NE(pos)
local pn, pe = ne:x(), ne:y()

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

-- Once at test start, using the demo's existing helpers:
local s = settings()

if not rebuild(0.0, kn, ke, legs_for(s), s, 0) then
    return false, "could not build virtual target"
end

-- Each callback:
local t = (now_ms - t0_ms) * 0.001

local kn, ke, kvn, kve = segs.state_at(segments, t)

if kn == nil then
    -- state_at returns its failure reason in the second value.
    return nil, ke
end

local sample_ms = now_ms

-- ============================================================
-- 4. PATH RESET AND ALGORITHM SWITCHING
-- ============================================================



-- Debounce selection inputs.
-- Initially accept algorithm changes only while disabled.
-- On accepted change: reset path, initialise algorithm, log event.
-- Later permit airborne switching only after transition testing.

-- algorithms selection
local algorithm_state = {}
local active_arm = nil
local entry = nil

-- arm settings
local ARM_NAMES = {
    [0] = "dubins_target_orbit",
    [1] = "adaptive_db_circle",
    [2] = "carrot_shift_cs",
}



-- PLAN: reset = algorithm_state = {} plus estimator re-init, all the demo's
-- anchor() does; the arms carry no other state.
local function reset_path()
    state.path = nil
    state.point_index = 1
    -- Clear controller_busy and algorithm-specific cached geometry.
    -- Reset estimator only when required by the test protocol.
end

-- select the algorithm
local function select_algorithm()
    local requested = HVAL_ARM:get()

    if requested == active_arm then
        return true
    end

    -- Initially allow changes only while disabled.
    if HVAL_ENABLE:get() ~= 0 then
        return false, "disable before changing algorithm"
    end

    local name = ARM_NAMES[requested]
    if name == nil then
        return false, "unknown algorithm ID"
    end

    local resolved, reason = arms.resolve(name)
    if resolved == nil then
        return false, reason
    end

    entry = resolved
    active_arm = requested
    reset_path()

    gcs:send_text(6, "HVAL: selected " .. name)
    return true
end

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

local MODE_GUIDED = 15

local function can_engage(aircraft_valid, flight_ready, target_fresh)
    if HVAL_ENABLE:get() ~= 1 then
        return false, "disabled"
    end

    local act_fn = HVAL_ACT_FN:get()
    if act_fn == nil or act_fn <= 0 then
        return false, "activation switch not configured"
    end

    if not rc:has_valid_input()
        or rc:get_aux_cached(act_fn) ~= 2 then
        return false, "pilot activation unavailable/off"
    end

    if not arming:is_armed() then
        return false, "not armed"
    end

    if vehicle:get_mode() ~= MODE_GUIDED then
        return false, "not GUIDED"
    end

    if not flight_ready then
        return false, "flight not established"
    end

    if not aircraft_valid then
        return false, "invalid aircraft state"
    end

    if not target_fresh then
        return false, "target unavailable/stale"
    end

    if state.fault_latched then
        return false, "fault latched"
    end

    if entry == nil or HVAL_ARM:get() ~= active_arm then
        return false, "algorithm not ready"
    end

    return true
end


-- ============================================================
-- 6. GUIDANCE DISPATCH
-- ============================================================
-- PLAN: use the existing snapshot contract instead of a new :step() interface:
-- result should contain:
--   valid, guidance_point, path metadata, diagnostic/fault reason.
--
-- Keep servo, throttle and vehicle-mode commands out of algorithms.

   local snapshot = { t_s = t, plane_n_m = pn, plane_e_m = pe, plane_hdg_rad = hdg,
    target_n_m = kn, target_e_m = ke, target_vn_ms = kvn, target_ve_ms = kve,
     algorithm_state = algorithm_state, target_est = est_proj, target_est_raw = est_raw }
   
     -- entry = arms.resolve(ARM_NAMES[HVAL_ARM])
   local result, reason = entry(snapshot, cfg)
-- Give every algorithm the same input/output interface:
   result = algorithms[selected]:step({
      aircraft = aircraft,
      target = estimated_target,
      dt = actual_elapsed_seconds,
      config = config,
   })
--

local function compute_guidance(
    t, pn, pe, hdg, kn, ke, kvn, kve, est_proj, est_raw)

    if entry == nil then
        return nil, "no algorithm selected"
    end

    return entry({
        t_s = t,
        plane_n_m = pn,
        plane_e_m = pe,
        plane_hdg_rad = hdg,

        target_n_m = kn,
        target_e_m = ke,
        target_vn_ms = kvn,
        target_ve_ms = kve,

        algorithm_state = algorithm_state,
        target_est = est_proj,
        target_est_raw = est_raw,
    }, cfg)
end
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

local function finite(value)
    return type(value) == "number"
        and value == value
        and value > -math.huge
        and value < math.huge
end

local function validate_guidance(result, pn, pe, airspeed_ms, bound_m)
    if type(result) ~= "table" then
        return false, "no guidance result"
    end

    local gn = result.guidance_n_m
    local ge = result.guidance_e_m

    if not finite(gn) or not finite(ge) then
        return false, "invalid guidance coordinates"
    end

    if not finite(pn) or not finite(pe) then
        return false, "invalid aircraft coordinates"
    end

    -- Circular test boundary centred on the fixed origin.
    if not finite(bound_m) or bound_m <= 0 then
        return false, "invalid test boundary"
    end

    if pn * pn + pe * pe > bound_m * bound_m then
        return false, "aircraft outside test boundary"
    end

    if gn * gn + ge * ge > bound_m * bound_m then
        return false, "guidance point outside test boundary"
    end

    -- Require a usable speed; do not substitute AIRSPEED_MIN.
    -- Supply true airspeed here for the physical radius calculation.
    if not finite(airspeed_ms) or airspeed_ms <= 0 then
        return false, "invalid true airspeed"
    end

    local bank_deg = ROLL_LIMIT_DEG:get()
    if not finite(bank_deg) or bank_deg <= 0 or bank_deg >= 89 then
        return false, "invalid bank limit"
    end

    local radius_floor =
        airspeed_ms * airspeed_ms
        / (GRAVITY_MSS * math.tan(math.rad(bank_deg)))

    local requested_radius = cfg.turn_radius_m

    if not finite(requested_radius) or requested_radius <= 0 then
        return false, "invalid configured turn radius"
    end

    if requested_radius < radius_floor then
        return false, "configured turn radius below physical floor"
    end

    return true
end

-- Derived Turn Raidus Floor
-- section 7: the smallest turn radius the bank limit allows at this airspeed.
-- The heading command is sent with acceleration g*tan(ROLL_LIMIT_DEG) (runner's
-- command()), so ROLL_LIMIT_DEG is the only bank limit; no fixed 10 m/s/s cap.

local warned_rho = false

-- turn radius implementation - check
local function turn_radius(airspeed)
   -- get the rool limit
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
    -- implementing the floor as V^2/(g*bank angle)
    local floor = (v * v) / (GRAVITY_MSS * math.tan(math.rad(bank_deg)))
    local rho = cfg and cfg.turn_radius_m
    -- if rho is invalid
    if rho == nil or rho <= 0 then
        return floor
    end
    -- handling floor case
    if rho < floor then
        if not warned_rho then
            gcs:send_text(4, string.format(
                "HVAL: RHO %.0f m below %.0f m floor at %.0f deg %.0f m/s; using floor",
                rho, floor, bank_deg, v))
            warned_rho = true
        end
        return floor
    end
    -- else rturn rho
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

local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002

local MAV_FRAME_GLOBAL = 0
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
local HEADING_TYPE_COG = 0

-- Cache the last accepted altitude command.
-- Clear this cache on release or a new engagement.
local sent_alt_m = nil

-- Track whether experimental commands may be controlling the aircraft.
state.owns_guidance = false

local function send_guidance(
    result, pn, pe, aircraft_valid, flight_ready, target_fresh)

    local output_mode = HVAL_OUT:get()

    -- SHADOW: the update loop still computes, validates and logs.
    -- This function sends no flight commands in shadow mode.
    if output_mode == 0 then
        -- Changing the parameter alone does not release an old command.
        -- Return a failure so the caller performs disengagement/recovery.
        if state.owns_guidance then
            return false, "active-to-shadow transition needs disengagement"
        end
        return true, "shadow"
    end

    if output_mode ~= 1 then
        return false, "invalid output mode"
    end

    -- ACTIVE: result must already have passed section 7.
    -- Recheck pilot authority and engagement gates before commands.
    local allowed, reason =
        can_engage(aircraft_valid, flight_ready, target_fresh)

    if not allowed then
        return false, reason
    end

    local alt_m = HVAL_ALT_M:get()
    local bank_deg = ROLL_LIMIT_DEG:get()

    if not finite(alt_m) or alt_m <= 0 then
        return false, "invalid test altitude"
    end

    if not finite(bank_deg) or bank_deg <= 0 or bank_deg >= 89 then
        return false, "invalid bank limit"
    end

    -- Direction from aircraft to the validated guidance point.
    -- North/East coordinates give course clockwise from North.
    local dn = result.guidance_n_m - pn
    local de = result.guidance_e_m - pe

    if dn * dn + de * de < 0.01 then
        return false, "guidance point too close for course command"
    end

    local course_deg = math.deg(math.atan(de, dn)) % 360

    -- PLAN: heading acceleration = g * tan(ROLL_LIMIT_DEG).
    -- This avoids introducing the demo's fixed acceleration fallback.
    local acceleration = GRAVITY_MSS * math.tan(math.rad(bank_deg))

    -- Mark ownership before any command may have taken effect.
    -- If altitude succeeds but heading fails, recovery is still needed.
    state.owns_guidance = true

    -- PLAN: GUIDED_CHANGE_ALTITUDE once per engagement.
    -- Resend only if the requested altitude changes.
    if sent_alt_m ~= alt_m then
        local accepted =
            gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
                frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
                p3 = 1000.0, -- Demo's rate cap; review for the aircraft.
                z = alt_m,
            })

        -- Check acceptance; caller logs rejection and handles the fault.
        if not accepted then
            return false, "altitude command refused"
        end

        sent_alt_m = alt_m
    end

    -- TODO: repeat can_engage() here immediately before heading output.
    -- The current check precedes the whole altitude/heading sequence.

    -- PLAN: GUIDED_CHANGE_HEADING as course over ground (COG).
    -- Send the current guidance course each active callback.
    local accepted =
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            frame = MAV_FRAME_GLOBAL,
            p1 = HEADING_TYPE_COG,
            p2 = course_deg,
            p3 = acceleration,
        })

    -- Return rejection to section 9.
    -- Section 10 must record the reason before the callback exits.
    if not accepted then
        return false, "heading command refused"
    end

    return true, "active"
end


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

-- Deliberately unset here: recovery depends on your test procedure.
local RECOVERY_MODE = nil

local function handle_fault(reason)
    -- Report and attempt recovery once per latched fault.
    if state.fault_latched then
        return
    end

    local owned_guidance = state.owns_guidance

    state.fault_latched = true
    state.enabled = false
    state.fault_reason = tostring(reason)
    sent_alt_m = nil

    gcs:send_text(3, "HVAL: fault: " .. state.fault_reason)

    -- Shadow faults need no flight-mode change.
    if not owned_guidance then
        return
    end

    -- Pilot/failsafe has already changed mode: yield.
    if vehicle:get_mode() ~= MODE_GUIDED then
        state.owns_guidance = false
        return
    end

    if RECOVERY_MODE == nil or RECOVERY_MODE == MODE_GUIDED then
        gcs:send_text(3, "HVAL: recovery mode not configured")
        -- Previous commands may remain effective in GUIDED.
        return
    end

    local accepted = vehicle:set_mode(RECOVERY_MODE)

    if not accepted or vehicle:get_mode() ~= RECOVERY_MODE then
        gcs:send_text(3, "HVAL: recovery mode change failed")
        -- Retain ownership flag: release has not been confirmed.
        return
    end

    state.owns_guidance = false
    gcs:send_text(4, "HVAL: recovery mode entered")
end

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

-- section 10, written once (where anchor() runs)
logger:write('HANC', 't0ms,Lat,Lng,Alt,Hdg,Yaw,AltCmd', 'Iiiffff',
             now_ms, origin:lat(), origin:lng(),
             origin:alt() * 0.01,        -- actual altitude of the anchor, m (absolute)
             hdg, yaw,
             HVAL_ALT_M:get())           -- commanded altitude, m above home


-- ============================================================
-- 11. UPDATE CALLBACK
-- ============================================================
-- PLAN: this order already matches the runner's step().
local function update()
    -- 1. Read time, controls and aircraft state.
    local now_ms = millis():tofloat()
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()

    -- 2. Yield immediately if pilot/failsafe has taken authority.
    if mode ~= MODE_GUIDED or not armed then
        if state.owns_guidance then
            handle_fault("left GUIDED or disarmed")
        end

        -- Your re-engagement latch must require an OFF/ON cycle.
        return update, 100
    end

    -- 3. Process disengagement and permitted algorithm selection.
    if HVAL_ENABLE:get() == 0 then
        if state.owns_guidance then
            handle_fault("experimental guidance disabled")
        end

        local selected, selection_reason = select_algorithm()

        -- Section 10: record selection refusal if not selected.
        -- Do not clear a latched fault automatically.
        return update, 100
    end

    -- 4. Acquire target and update estimator.
    -- Insert your section 2–3 code here, defining:
    --   t, pn, pe, hdg
    --   kn, ke, kvn, kve
    --   est_proj, est_raw, true_airspeed_ms
    --   aircraft_valid, flight_ready, target_fresh
    --
    -- Anchor once per run before converting local positions.
    -- Update the estimator only on a new accepted target sample.


    -- 5. Compute guidance.
    local result, reason = compute_guidance(
        t, pn, pe, hdg, kn, ke, kvn, kve, est_proj, est_raw
    )

    if result == nil then
        -- Section 9: handle failure using reason.
        return update, 100
    end

    -- 6. Validate guidance.
    local valid, why = validate_guidance(
        result, pn, pe, true_airspeed_ms, HVAL_BOUND_M:get()
    )

    if not valid then
        -- Section 9: handle failure using why.
        return update, 100
    end

    algorithm_state = result.algorithm_state or {}

    local result, reason = compute_guidance(
        t, pn, pe, hdg,
        kn, ke, kvn, kve,
        est_proj, est_raw
    )

    if result == nil then
        handle_fault(reason)
        return update, 100
    end

    -- 7. Validate output and aircraft envelope.
   local valid, why = validate_guidance(
        result, pn, pe,
        true_airspeed_ms, HVAL_BOUND_M:get()
    )

    if not valid then
        handle_fault(why)
        return update, 100
    end

    algorithm_state = result.algorithm_state or {}
    
    -- 8. Log; command only if ACTIVE and still authorised.
    local accepted, output_status = send_guidance(
        result, pn, pe,
        aircraft_valid, flight_ready, target_fresh
    )

    -- 9. Handle detected faults and verify recovery.
    if not accepted then
        handle_fault(output_status)
        return update, 100
    end

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
local arms, est_mod, demo, cfg

local arms = load("sitl_arms")
local est_mod = load("harness_estimator")
local ARM_NAMES = { [0] = "dubins_target_orbit", [1] = "adaptive_db_circle", [2] = "carrot_shift_cs" }

-- Require module loading
local loaded, load_failed = {}, {}
-- load the name of th efunction
local function load(name)
   -- return loaded name
    if loaded[name] ~= nil then 
      return loaded[name] 
   end
    -- if failed to report
    if load_failed[name] then 
      return nil 
    end
    -- handle messaging and error handling
    local ok, mod = pcall(require, name)
    if not ok or mod == nil then
        load_failed[name] = true
        gcs:send_text(3, "HVAL: module " .. name .. " not found: " .. tostring(mod))
        
        return nil
    end
    -- load name
    loaded[name] = mod
    return mod
end


-- Assign the shared variables declared near the top.
arms = load("sitl_arms")
est_mod = load("harness_estimator")
demo = load("kangaroo_demo_cfg")

-- Stop startup if a required module failed to load.
if arms == nil or est_mod == nil or demo == nil then
    return
end

cfg = demo.cfg

if type(cfg) ~= "table" then
    gcs:send_text(3, "HVAL: missing configuration table")
    return
end

-- Start disabled; do not arm or change flight mode.
HVAL_ENABLE:set(0)
state.enabled = false
state.owns_guidance = false
state.fault_latched = false

-- Section 4 resolves the initial algorithm while disabled.
local selected, reason = select_algorithm()

if not selected then
    gcs:send_text(
        3, "HVAL: initial algorithm unavailable: " .. tostring(reason)
    )
    return
end

-- Announce the selected algorithm and configuration source.
gcs:send_text(
    6, "HVAL: loaded disabled; algorithm "
        .. tostring(ARM_NAMES[active_arm])
        .. "; config kangaroo_demo_cfg"
)

-- Begin scheduled callbacks.
return update, 100
