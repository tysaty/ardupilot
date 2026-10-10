-- Flight-test Wrapper for physical validation
-- Keep aircraft-specific limits/configuration separate from geometry.
--
-- ONE SCRIPT (10 October 2026): the virtual kangaroo that kangaroo_source.lua
-- ran beside this script (the two-script, Sam-Follow-03 layout) is merged in
-- as section 3b, and HVAL_TGT 3 reads it directly instead of over the KBUS_
-- parameter bus. The two-script versions are kept in working_folder_lua/
-- (hardware_val.lua and kangaroo_source.lua as of the merge).
-- Only HVAL_TGT 3 (the site kangaroo) remains: the anchor-relative virtual
-- targets HVAL_TGT 0 and 1 were removed with it (the physical validation
-- flies TGT 3 only); the archived hardware_val.lua still has them.
--
-- Concept
-- Choose the flight mode 'arm' by name through sitl_arms. Every arm already meets
--     entry(snapshot, cfg) -> {guidance_n_m, guidance_e_m, algorithm_state} | error: nil, reason
--     and sitl_arms.resolve(name) maps a name to it.
--
--     Arm       Name                            Lua                                      Needs
--     Baseline  dubins_target_orbit (or _hyst)  sitl_arms adapter over harness_cs_orbit  target or estimate
--     A         adaptive_db_circle              harness_adaptive_db.guidance_point       target_est, lookahead_steps <find>
--     F         carrot_shift_cs                 harness_carrot_shift.guidance_point      target_est_raw, lookahead_steps 1
--
-- Before use:
--   * Write it from shared code, not copies: heading_from, command, release,
--     log_tick and anchor exist twice already (runner and demo). Lift them into
--     one module (say sitl_vehicle_io.lua) that all three scripts require.
--     (4 October 2026: still copies; the copies below follow the runner's
--     sitl_harness_runner.lua line for line so the tick is the runner's tick.)
--
-- SD card requirements (APM/ on the card; HARDWARE_VAL_BOOT_REVIEW.md)
-- APM/scripts/hardware_val.lua     the only script (no kangaroo_source.lua)
-- APM/scripts/spec.json            generated, never typed (one file for every script)
-- APM/scripts/modules/: sitl_spec.lua, sitl_arms.lua, sitl_adsb.lua and the
-- HARNESS_MODULES files (harness_geom, harness_dubins, harness_orbit,
-- harness_cs_orbit, harness_kangaroo, harness_segments, harness_estimator,
-- harness_zone, harness_adaptive_db, harness_carrot_shift;
-- harness_adaptive_horizon and harness_rh_geometric only if sitl_arms is
-- asked for arms B or C). The MAVLink modules are no longer needed.
-- Nothing else may be in APM/scripts/: every .lua there runs.

-- SITL demo used: SCR_VM_I_COUNT 1000000 and SCR_HEAP_SIZE 2 MiB,
-- Arm A implementation was 252k to 846k instructions per tick range.
-- On a flight controller SCR_HEAP_SIZE is limited to 1 MiB by its range and
-- by the board's RAM; measure heap and instructions on the bench (rung 2).
-- The board's limit is 750 kB (supervisor's goal about 400 kB). As two
-- scripts (kangaroo_source.lua beside this one, HVAL_TGT 3) the heap held
-- every arm at 650000 but not 600000 in SITL (10 October 2026, heap
-- expansion off), after spec.json lost rand_legs and kangaroo_source.lua
-- loaded its MAVLink and ADS-B modules only when used. One script saves the
-- second copy of spec.json, the site reference and the modules; measure it.
-- An earlier AH run with expansion allowed peaked at 928 kB. Update time in that
-- run (PC SITL): median 12.5 ms, 95th percentile 27 ms, worst 84 ms for this
-- script, against the 100 ms tick; a board is slower, so time it on the bench.
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
-- PLAN: do not retype constants. demo.py --stage (from a campaign cell, or
-- --plan --arm <id> for the physical-validation plan) writes spec.json
-- (cfg, dt_s, estimate, lookahead_steps, estimator noise).

-- PLAN: live settings as a parameter table, like SHR_ / KDEM_: HVAL_ENABLE
-- (default 0), HVAL_ARM (0 baseline, 1 A, 2 F), HVAL_OUT (0 shadow, 1 active),
-- HVAL_TGT (3 site kangaroo; 0 and 1 removed 2026-10-10, 2 live not implemented), HVAL_ALT_M,
-- HVAL_BOUND_M, HVAL_ACT_FN.

-- Algorithm IDs/names, test altitude and altitude reference.
-- Orbit radius, minimum turn radius, airspeed/bank limits.
-- Target-data timeout, test boundary, nominal update interval.
-- Recovery policy for each fault type.
-- Default experimental guidance to DISABLED.

local MAV_SEVERITY = { ERROR = 3, WARNING = 4, INFO = 6 }

-- Modules, declared here so every function below sees them as upvalues;
-- assigned in section 12 through load().
local arms, est_mod, segs
local zone_mod              -- harness_zone, only when spec.json carries a fence

local state = {
    enabled = false,          -- engaged: anchored and computing
    active_algorithm = nil,
    fault_latched = false,
    fault_reason = nil,
    owns_guidance = false,    -- a heading/altitude command may be in effect
    switch_was_low = false,   -- engagement needs the switch seen low first
    path = nil,
    point_index = 1,
    last_update_ms = nil,
    run_id = 0,
}

-- SHARED CONFIGURATION: spec.json, read as `cfg` (the same file the SITL
-- runner and the demo read; read once here for the kangaroo as well)
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
--   cfg.af_step_ticks           arm F's carrot lead, whole ticks of dt_s
--   cfg.dt_s                    nominal update interval, s (0.1)
--   cfg.duration_s              run length, s; the run ends (released) after it
--   cfg.legs, cfg.geometry, cfg.target_n_m, cfg.target_e_m
--                               the kangaroo's schedule: section 3b plays
--                               cfg.legs from the site (HVAL_TGT 3)
--   cfg.bank_limit_deg          the bank the configuration was built for
--   cfg.roll_limit_deg          the ROLL_LIMIT_DEG it must be flown at (60)
--
-- Bank limit: this script never sets ROLL_LIMIT_DEG (it is a flight limit). It
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
    gcs:send_text(MAV_SEVERITY.ERROR, "HVAL: no configuration: " .. tostring(cfg_where))
end

--- Bank-limit gate for section 5: true only when the live ROLL_LIMIT_DEG is the
--  bank limit spec.json was generated for.
local function roll_limit_ok()
    if cfg == nil then return false, "no spec.json" end
    local ok, _live, msg = spec_mod.roll_limit_check(cfg)
    return ok, msg
end

-- ============================================================
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
    // @Description: Master enable for experimental guidance. 0 (default): nothing is computed or commanded, and a latched fault is cleared. 1: the script may engage when every gate in section 5 passes, including the HVAL_ACT_FN switch and the ROLL_LIMIT_DEG check against spec.json; both this and the switch are required. Set to 0 at every boot.
    // @Values: 0:Disabled,1:Enabled
--]]
local HVAL_ENABLE = bind("ENABLE", 1, 0)

--[[
    // @Param: HVAL_ARM
    // @DisplayName: Hardware validation arm
    // @Description: Guidance law, resolved by name through sitl_arms (ARM_NAMES below). Must name the arm spec.json was staged for (cfg.algorithm, or its _hyst form, which is then flown): each arm's spec carries its own estimate and lookahead_steps. Read only while disengaged.
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
    // @Description: 3 (default) the on-board kangaroo in this script (section 3b, formerly kangaroo_source.lua), run from boot in the site frame (the fence's vertex centroid), which becomes this script's frame, and driven by KSRC_RUN. Any other value is refused at engagement: 0 and 1 (the anchor-relative virtual point and moving target) were removed on 10 October 2026, and 2 (live target from AP_Follow, FOLL_SYSID) is not implemented.
    // @Values: 3:Site kangaroo (KSRC_)
--]]
local HVAL_TGT = bind("TGT", 4, 3)

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
    // @Description: RCx_OPTION scripting function whose switch engages the experimental guidance (high) and disengages it (low), as FOLLP_ACT_FN in plane_follow.lua. 300 to 307 are the Scripting1 to Scripting8 aux functions. The switch must be seen low before each engagement.
    // @Range: 300 307
--]]
local HVAL_ACT_FN = bind("ACT_FN", 6, 303)

--[[
    // @Param: HVAL_BOUND_M
    // @DisplayName: Hardware validation boundary
    // @Description: Radius of the circular test area centred on the anchor, metres. The aircraft or a guidance point outside it is a fault (section 7). Must fit inside the approved test area.
    // @Range: 100 2000
    // @Units: m
--]]
local HVAL_BOUND_M = bind("BOUND_M", 7, 400)

--[[
    // @Param: HVAL_TGT_TMO
    // @DisplayName: Hardware validation target timeout
    // @Description: A live target sample older than this is stale: the script disengages (or refuses to engage) and latches a fault. HVAL_TGT 2 (not implemented): from follow:get_last_update_ms(). HVAL_TGT 3 (the kangaroo in this script) is sampled every tick, so it is never stale; a stopped kangaroo is a fault at once.
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
-- Arms that read the estimate (requires_estimate in algorithms.py), and the
-- arm that owns its lead, so a state-side lookahead_steps would lead twice.
local NEEDS_ESTIMATE = { adaptive_db_circle = true, carrot_shift_cs = true }
local OWNS_HORIZON = { carrot_shift_cs = true }

-- Nominal update interval: cfg.dt_s (0.1 s), not a parameter: the
-- estimator and arm A's horizon count in ticks of it.
local dt_s = (cfg and cfg.dt_s) or 0.1
local period_ms = math.floor(dt_s * 1000 + 0.5)

-- ============================================================
-- 2. INPUTS AND AIRCRAFT STATE
-- ============================================================
-- PLAN: reuse heading_from(vel) from the runner / demo (course from
-- math.atan(ve, vn) in (-pi, pi], yaw when slow); that wrapping fixed
-- the demo's phantom right turn (a [0, 2 pi) heading gave the right-turn
-- candidate a negative sweep). Local frame: origin:get_distance_NE(pos),
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
-- The requested turn radius is cfg.turn_radius_m from spec.json,
-- no longer a parameter (was HVAL_RHO_M).

--- Below this ground speed, m/s, the course is undefined and the yaw is
--  used: the runner's and the demo's threshold (vn^2 + ve^2 < 1).
local STANDOFF_MIN_COURSE_SPEED = 1.0

-- When the aircraft is not moving - the standoff course needs to be the
-- ground course from the NED velocity (the direction of motion), or the yaw
-- when the aircraft is not moving. Same result as the runner's
-- heading_from(vel), including cfg.heading_source "yaw".
local function course(vel)
   local yaw = ahrs:get_yaw_rad()
   if vel ~= nil and (cfg == nil or cfg.heading_source ~= "yaw") then
      local vn, ve = vel:x(), vel:y()
      if (vn * vn + ve * ve) >= (STANDOFF_MIN_COURSE_SPEED * STANDOFF_MIN_COURSE_SPEED) then
         return math.atan(ve, vn), yaw
      end
   end
   return yaw, yaw
end

--- True airspeed, m/s, or nil. The radius floor (section 7) is a TAS
--  quantity; ahrs:airspeed_estimate() is EAS.
local function true_airspeed()
    local eas = ahrs:airspeed_estimate()
    if eas == nil then return nil end
    return eas * ahrs:get_EAS2TAS()
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
-- (4 October 2026: virtual targets produce a sample every tick, so the update is
-- every tick with the nominal dt_s, exactly as the runner and the Python
-- harness; the measured dt matters only for HVAL_TGT 2.)

-- PLAN: with a live target the baseline has no "truth": decide whether it
-- steers on the measurement or the raw estimate, and record it.
-- Choose stationary virtual, moving virtual or live target.
-- Store sample timestamp separately from receive timestamp.
-- Reject stale, out-of-order and implausible samples.
-- Update estimator only when an accepted measurement arrives.
-- Predict to the required horizon using measured elapsed time.

-- Run state, set by anchor() at engagement (section 10)
local origin = nil          -- Location at the anchor
local t0_ms = nil
local zone = nil            -- the fence in this script's frame
local estimator = nil

--- The fence in the anchor frame, built at engagement from
--  cfg.fence's vertices through the vehicle's own get_distance_NE. The
--  kangaroo must start inside it less the containment margin (the orbit
--  radius): otherwise the ring would start across the fence, so engagement
--  is refused. Returns true, or false and why. No fence: nothing to do.
--  start_n, start_e: where the kangaroo is at engagement, from the site
--  reference (HVAL_TGT 3).
local function build_zone(start_n, start_e)
    zone = nil
    if cfg.fence == nil then
        return true
    end
    local built, why = zone_mod.from_latlng(origin, cfg.fence.vertices_latlng)
    if built == nil then
        return false, "fence: " .. tostring(why)
    end
    local margin = cfg.fence.containment_margin_m
    if not zone_mod.contains(built, start_n, start_e, margin) then
        return false, string.format(
            "kangaroo start (%.0f N, %.0f E) is not inside the fence less %.0f m",
            start_n, start_e, margin)
    end
    zone = built
    return true
end

--- The SITE reference: the fence's vertex centroid from spec.json, the same
--  point pv_plan.py uses, so the spec's legs are metres from it. Same point
--  every sortie; never home, never the aircraft. HVAL_TGT 3 uses it as this
--  script's origin, and the kangaroo (section 3b) runs in it. alt_cm is only
--  for the display and HANC altitude.
local function site_reference(alt_cm)
    local v = cfg.fence.vertices_latlng
    local lat, lng = 0.0, 0.0
    for i = 1, #v do lat = lat + v[i][1]; lng = lng + v[i][2] end
    local site = Location()
    site:lat(math.floor(lat / #v * 1e7 + 0.5))
    site:lng(math.floor(lng / #v * 1e7 + 0.5))
    site:alt(alt_cm)
    return site
end

-- ============================================================
-- 3b. THE KANGAROO (HVAL_TGT 3)
-- ============================================================
-- Merged in from kangaroo_source.lua (10 October 2026; the two-script
-- version is kept in working_folder_lua/). Implemented based on the prior
-- physical flight architecture (Sam-Follow-03), now in one script: one heap
-- copy of spec.json, the site reference and the modules, and no KBUS_
-- parameter bus between two scripts. HVAL_TGT 3 reads the kangaroo below
-- directly, on the same tick.
--
--  Frame: north/east metres from the SITE reference, the fence's vertex
--  centroid in spec.json (site-fixed anchor; never home, never the aircraft).
--  The kangaroo runs from boot (once home is set), engaged or not.
--
--  KSRC_RUN 0 hold (where the kangaroo is), 5 plan (spec.json's legs: the
--  physical-validation plan from pv_plan.py; 1 is the same, the old case
--  mode), 2 live (KSRC_MODE 0 point 1 straight 2 circle 3 rectangle;
--  KSRC_PACE 0 constant 1 elastic 2 stopstart; KSRC_SPD, KSRC_HDG; any change
--  rebuilds from where the kangaroo is). No KSRC_ parameter is a flight limit.
--
--  The physical-validation plan: cfg.legs starts at the point run's
--  place (cfg.target_n_m, cfg.target_e_m: held at the fence's deepest
--  point), then the transit to the shared start and the nine runs (straight,
--  circle, rectangle at constant, elastic and stop-start pace),
--  chained with no rests because each ends back at the shared start.
--  cfg.suite_runs names each run. Set KSRC_RUN 5 from the ground once the
--  aircraft is engaged, so the point run is not spent before then.
--
--  Logs: HKSR every tick (t, run, n, e, vn, ve); HKSB on every rebuild
--  (t, run, speed, heading, why: 1 run change, 2 live change, 3 fence turn,
--  4 plan run start).
--
--  Removed in the merge (kangaroo_source.lua had them): the KBUS_ bus
--  (publish_bus here, read_bus / kbus_bind in the old hardware_val.lua),
--  FOLLOW_TARGET to a remote aircraft's AP_Follow (KSRC_OUT, KSRC_CHAN) and
--  KSRC_ENABLE (it only stopped the bus, to test a stale target).

-- hold and live legs never reach their end
local KSRC_LEG_S = 36000.0
-- minimum spacing of repeated warnings, ms
local KSRC_WARN_PERIOD_MS = 5000

-----------------------------------------------------------------------------
-- 3b.1 Parameters
-----------------------------------------------------------------------------
-- KSRC_ parameter table, key 142 and the same indices as kangaroo_source.lua,
-- so saved values keep their meaning. Bound in section 12 (kangaroo_bind),
-- once the modules have loaded (the pace defaults are harness_kangaroo's).
local KSRC_TABLE_KEY = 142
local KSRC_PREFIX = "KSRC_"
local KSRC_RUN, KSRC_MODE, KSRC_PACE, KSRC_SPD, KSRC_HDG
local KSRC_RAD, KSRC_LEN, KSRC_WID
local KSRC_PSLOW, KSRC_PHOLD, KSRC_PFAST, KSRC_PRAMP
local KSRC_ADSB
local KSRC_MODE_NAMES = { [0] = "point", "straight", "circle", "rectangle" }
local KSRC_PACE_NAMES = { [0] = "constant", "elastic", "stopstart" }
local RUN_HOLD, RUN_LIVE, RUN_PLAN = 0, 2, 5
local RUN_NAMES = { [0] = "hold", "plan", "live", [5] = "plan" }

local function kangaroo_bind(kang)
    assert(param:add_table(KSRC_TABLE_KEY, KSRC_PREFIX, 16), "KSRC: could not add param table")
    local function kbind(name, idx, default)
        assert(param:add_param(KSRC_TABLE_KEY, idx, name, default), "KSRC: could not add " .. name)
        local p = Parameter()
        assert(p:init(KSRC_PREFIX .. name), "KSRC: could not bind " .. name)
        return p
    end
    local geometry = cfg.geometry or {}
    -- index 1 (ENABLE) and 2 (CHAN) were the bus and FOLLOW_TARGET: unused
    -- 0 hold, 5 plan (1 is the same), 2 live
    KSRC_RUN    = kbind("RUN",    3, 0)
    KSRC_MODE   = kbind("MODE",   4, 1)
    KSRC_PACE   = kbind("PACE",   5, 0)
    KSRC_SPD    = kbind("SPD",    6, cfg.speed_ms or 6.25)
    KSRC_HDG    = kbind("HDG",    7, 0)
    KSRC_RAD    = kbind("RAD",    8, geometry.radius_m or 60.0)
    KSRC_LEN    = kbind("LEN",    9, geometry.length_m or 140.0)
    KSRC_WID    = kbind("WID",   10, geometry.width_m or 70.0)
    KSRC_PSLOW  = kbind("PSLOW", 11, kang.PACE_DEFAULTS.slow_factor)
    KSRC_PHOLD  = kbind("PHOLD", 12, kang.PACE_DEFAULTS.hold_slow_s)
    KSRC_PFAST  = kbind("PFAST", 13, kang.PACE_DEFAULTS.hold_fast_s)
    KSRC_PRAMP  = kbind("PRAMP", 14, kang.PACE_DEFAULTS.ramp_down_s)
    -- index 15 (OUT) was the bus / FOLLOW_TARGET selector: unused
    -- 0 off, 1 broadcast ADSB_VEHICLE every 200 ms (display only)
    KSRC_ADSB   = kbind("ADSB",  16, 1)
end

-----------------------------------------------------------------------------
-- 3b.2 ADS-B for the ground station
-----------------------------------------------------------------------------
-- display only (sitl_adsb, Follow-03's framing). Loaded the first time
-- KSRC_ADSB is on (about 4 kB of heap), not at start-up.
local ok_adsb, adsb = nil, nil
local ADSB_PERIOD_MS = 200
local last_adsb_ms = nil
-- true when broadcast() will send this tick (KSRC_ADSB on, module present,
-- period elapsed), so the Location is built only when it is needed
local function adsb_due(now_ms)
    if KSRC_ADSB:get() <= 0 then
        return false
    end
    if ok_adsb == nil then
        ok_adsb, adsb = pcall(require, "sitl_adsb")
    end
    if not ok_adsb then
        return false
    end
    return last_adsb_ms == nil or (now_ms - last_adsb_ms) >= ADSB_PERIOD_MS
end
local function broadcast(now_ms, loc, vn, ve)
    last_adsb_ms = now_ms
    -- every channel; missing ones dropped
    adsb.send(loc, vn, ve)
end

-----------------------------------------------------------------------------
-- 3b.3 Geometry runs and speed variations (live mode, KSRC_RUN 2)
-----------------------------------------------------------------------------
-- the live settings, as the demo's KDEM_ parameters (mode, pace, speed)
local function settings()
    local mode = math.floor(KSRC_MODE:get() + 0.5)
    if KSRC_MODE_NAMES[mode] == nil then mode = 1 end
    local pace = math.floor(KSRC_PACE:get() + 0.5)
    if KSRC_PACE_NAMES[pace] == nil then pace = 0 end
    return {
        mode = mode, pace = pace,
        speed_ms = math.max(0.0, KSRC_SPD:get()),
        heading_deg = KSRC_HDG:get() % 360.0,
        geometry = { radius_m = KSRC_RAD:get(), length_m = KSRC_LEN:get(),
                     width_m = KSRC_WID:get() },
        pace_profile = { slow_factor = KSRC_PSLOW:get(), hold_slow_s = KSRC_PHOLD:get(),
                         hold_fast_s = KSRC_PFAST:get(), ramp_down_s = KSRC_PRAMP:get(),
                         ramp_up_s = KSRC_PRAMP:get() },
    }
end

local function signature_of(s)
    local p, g = s.pace_profile, s.geometry
    return string.format("%d|%d|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f|%.3f",
                         s.mode, s.pace, s.speed_ms, s.heading_deg, g.radius_m,
                         g.length_m, g.width_m, p.slow_factor, p.hold_slow_s,
                         p.hold_fast_s, p.ramp_down_s)
end

-- point leg where the kangaroo is (hold)
local function hold_legs()
    return { { duration_s = KSRC_LEG_S, mode = "point", heading_deg = 0.0, speed_ms = 0.0 } }
end

-- the one live leg, by the demo's rules: elastic and stopstart travel over
-- the chosen base; point has no pace
local function live_legs(s)
    local name = KSRC_MODE_NAMES[s.mode]
    if name == "point" then
        return hold_legs()
    end
    if s.pace == 1 then
        return { { duration_s = KSRC_LEG_S, mode = "elastic", heading_deg = s.heading_deg,
                   speed_ms = s.speed_ms, elastic_base = name } }
    end
    if s.pace == 2 then
        return { { duration_s = KSRC_LEG_S, mode = "stopstart", heading_deg = s.heading_deg,
                   speed_ms = s.speed_ms, elastic_base = name, pace = s.pace_profile } }
    end
    return { { duration_s = KSRC_LEG_S, mode = name, heading_deg = s.heading_deg,
               speed_ms = s.speed_ms } }
end

-----------------------------------------------------------------------------
-- 3b.4 Full package
-----------------------------------------------------------------------------
-- the point run, the transit and the nine runs (no rests: each ends at the start):
-- generated once in Python (pv_plan.py) and carried in spec.json, already
-- fence-checked. This script plays them; it never builds or changes them.
local function plan_legs()
    if cfg.legs == nil or #cfg.legs == 0 then
        return nil, "spec.json has no legs: stage it with demo.py --plan --arm"
    end
    local out = {}
    for i = 1, #cfg.legs do
        local l = cfg.legs[i]
        out[i] = { duration_s = l.duration_s, mode = l.mode,
                   heading_deg = l.heading_deg, speed_ms = l.speed_ms,
                   elastic_base = l.elastic_base, pace = l.pace }
    end
    return out
end

-----------------------------------------------------------------------------
-- 3b.5 Running the suite
-----------------------------------------------------------------------------
-- ks_site: the SITE Location; ks_zone: the fence in the site frame; running:
-- the KSRC_RUN the schedule was built for; ks_legs, signature: what the
-- schedule was built from; ks_n, ks_e (and velocity): the kangaroo this tick
-- (from SITE); ks_stopped: a refused schedule stops the kangaroo, so
-- HVAL_TGT 3 has no target and faults if engaged
local ks_site, ks_zone, ks_segments, ks_t0_ms = nil, nil, nil, nil
local running, ks_legs, signature, ks_stopped = RUN_HOLD, nil, nil, false
local ks_n, ks_e, ks_vn, ks_ve, ks_t_s = nil, nil, nil, nil, nil
local plan_t0, plan_run, plan_spare, plan_spare_run, plan_done = 0.0, 0, nil, "", false
local last_breach_ms = -KSRC_WARN_PERIOD_MS

-- rebuild from where the kangaroo is (position continuous, only velocity
-- steps); why: 1 run change, 2 live change, 3 fence turn. False on refusal.
local function rebuild(t, n, e, new_legs, geom, why)
    local built, reason = segs.make_segments(new_legs, n, e, geom, t)
    if built == nil then
        ks_stopped = true
        gcs:send_text(MAV_SEVERITY.ERROR, "KSRC: stopped: schedule refused: " .. tostring(reason))
        return false
    end
    ks_segments, ks_legs = built, new_legs
    logger:write('HKSB', 't,Run,Spd,Hdg,Why', 'fffff', t, running,
                 new_legs[1].speed_ms or 0.0, new_legs[1].heading_deg or 0.0, why)
    return true
end

--- Index of the segment of `segs_tbl` active at t (the last one past the end).
local function active_index(segs_tbl, t)
    for i = 1, #segs_tbl do
        if segs_tbl[i].t_start <= t and t < segs_tbl[i].t_end then
            return i
        end
    end
    return #segs_tbl
end

-- spare to the orbit-radius limit inside the fence, m (negative: past it)
local function spare_of(n, e)
    local d = zone_mod.inward_distances(ks_zone, n, e)
    local spare = math.huge
    for i = 1, #d do
        spare = math.min(spare, d[i] - cfg.fence.containment_margin_m)
    end
    return spare
end

-- the fence turn stays off for the plan: its legs are fence-checked in
-- Python. Announce each run as it starts, track the least spare, warn past
-- it, report at the end (the demo's suite_watch, on the plan clock).
local function plan_watch(t, now_ms)
    local tp = t - plan_t0
    local runs = cfg.suite_runs or {}
    local idx = plan_run
    while idx < #runs and tp >= runs[idx + 1].t_start_s do
        idx = idx + 1
    end
    if idx ~= plan_run then
        plan_run = idx
        local r = runs[idx]
        logger:write('HKSB', 't,Run,Spd,Hdg,Why', 'fffff', t, running, idx, 0.0, 4)
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "KSRC: run %d/%d %s (%.0f s)", idx, #runs, tostring(r.name),
            r.t_end_s - r.t_start_s))
    end
    local spare = spare_of(ks_n, ks_e)
    if plan_spare == nil or spare < plan_spare then
        plan_spare = spare
        plan_spare_run = (runs[plan_run] and runs[plan_run].name) or ""
    end
    if spare < 0.0 and (now_ms - last_breach_ms) >= KSRC_WARN_PERIOD_MS then
        last_breach_ms = now_ms
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "KSRC: kangaroo %.1f m past the %.0f m limit (%.0f N, %.0f E)",
            -spare, cfg.fence.containment_margin_m, ks_n, ks_e))
    end
    if not plan_done and #runs > 0 and tp >= runs[#runs].t_end_s then
        plan_done = true
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "KSRC: plan complete; least spare %.1f m (%s)", plan_spare, plan_spare_run))
    end
end

-- the outputs, every tick, from the SITE reference: the ADS-B display and
-- the HKSR row (the kangaroo is logged whether or not the aircraft follows)
local function kangaroo_outputs(t, now_ms, n, e, vn, ve)
    if adsb_due(now_ms) then
        -- the geographic position only when ADS-B sends it
        local loc = ks_site:copy()
        loc:offset(n, e)
        broadcast(now_ms, loc, vn, ve)
    end
    logger:write('HKSR', 't,Run,N,E,VN,VE', 'ffffff', t, running, n, e, vn, ve)
end

--- One kangaroo tick (was kangaroo_source.lua's update()); called by
--  update() before step(), as the source ran before the follower each tick.
local function kangaroo_update(now_ms)
    if zone_mod == nil or KSRC_RUN == nil or ks_stopped then return end
    if ks_site == nil then
        -- home only for the display altitude; the frame is the SITE reference
        if not ahrs:home_is_set() then return end
        ks_site = site_reference(ahrs:get_home():alt())
        ks_zone = assert(zone_mod.from_latlng(ks_site, cfg.fence.vertices_latlng))
        ks_t0_ms = now_ms
        -- point leg at the point run's place until KSRC_RUN
        ks_n, ks_e = cfg.target_n_m, cfg.target_e_m
        if not rebuild(0.0, ks_n, ks_e, hold_legs(), cfg.geometry, 1) then return end
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KSRC: kangaroo at %.0f N %.0f E of the site; KSRC_RUN 5 starts the plan", ks_n, ks_e))
    end
    local t = (now_ms - ks_t0_ms) * 0.001
    local run = math.floor(KSRC_RUN:get() + 0.5)
    if RUN_NAMES[run] == nil then run = RUN_HOLD end
    if run == 1 then run = RUN_PLAN end

    -- plan mode (the old case mode): the spec's legs from the point run's place
    if run == RUN_PLAN and running ~= RUN_PLAN then
        local pl, why = plan_legs()
        if pl == nil then
            ks_stopped = true
            gcs:send_text(MAV_SEVERITY.ERROR, "KSRC: " .. why)
            return
        end
        running = RUN_PLAN
        ks_n, ks_e = cfg.target_n_m, cfg.target_e_m
        plan_t0, plan_run, plan_spare, plan_spare_run, plan_done = t, 0, nil, "", false
        if not rebuild(t, ks_n, ks_e, pl, cfg.geometry, 1) then return end
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KSRC: plan, %d runs, from %.0f N %.0f E of the site",
            #(cfg.suite_runs or {}), ks_n, ks_e))
    end

    -- live mode: on a KSRC_* change, rebuild from where the kangaroo is
    if run == RUN_LIVE then
        local s = settings()
        if running ~= RUN_LIVE or signature ~= signature_of(s) then
            local why = (running == RUN_LIVE) and 2 or 1
            running, signature = RUN_LIVE, signature_of(s)
            if not rebuild(t, ks_n, ks_e, live_legs(s), s.geometry, why) then return end
            gcs:send_text(MAV_SEVERITY.INFO, string.format("KSRC: live %s %s %.2f m/s hdg %.0f",
                KSRC_MODE_NAMES[s.mode], KSRC_PACE_NAMES[s.pace], s.speed_ms, s.heading_deg))
        end
    end

    -- hold: KSRC_RUN 0 after a run stands the kangaroo where it is
    if run == RUN_HOLD and running ~= RUN_HOLD then
        running = RUN_HOLD
        if not rebuild(t, ks_n, ks_e, hold_legs(), cfg.geometry, 1) then return end
        gcs:send_text(MAV_SEVERITY.INFO, "KSRC: hold")
    end

    local n, e, vn, ve = segs.state_at(ks_segments, t)
    ks_n, ks_e, ks_vn, ks_ve, ks_t_s = n, e, vn, ve, now_ms * 0.001

    if running == RUN_PLAN then
        plan_watch(t, now_ms)
    elseif running == RUN_LIVE then
        -- live mode only: turn back at the fence by the harness's rule (harness_zone)
        local leg = ks_legs[active_index(ks_segments, t)]
        local turned = zone_mod.contain_heading(ks_zone, n, e, leg.heading_deg or 0.0,
                                                leg.speed_ms or 0.0, dt_s,
                                                cfg.fence.containment_margin_m)
        if turned ~= nil then
            local new_leg = zone_mod.turned_leg(leg, turned)
            new_leg.duration_s = KSRC_LEG_S
            if not rebuild(t, n, e, { new_leg }, settings().geometry, 3) then
                return
            end
        end
    end

    kangaroo_outputs(t, now_ms, n, e, vn, ve)
end

--- HVAL_TGT 3's sample: the kangaroo this tick, from SITE.
--  n, e, vn, ve, t_s  |  nil, why
local function kangaroo_sample()
    if zone_mod == nil then return nil, "no fence in spec.json, so no site kangaroo" end
    if ks_stopped then return nil, "kangaroo stopped" end
    if ks_t_s == nil then return nil, "kangaroo not started (home not set)" end
    return ks_n, ks_e, ks_vn, ks_ve, ks_t_s
end


-- HVAL_TGT 3's sample interval, for the estimator's dt (as the bus's T_S)
local ks_last_t = nil

--- Each callback: the kangaroo at t, s.
--  Returns kn, ke, kvn, kve, fresh, dt  |  nil, why.
--  fresh: a new sample (the estimator updates only then); dt: the time since
--  the previous sample, s.
local function target_at(t)
    local source = math.floor(HVAL_TGT:get() + 0.5)
    if source ~= 3 then
        return nil, "HVAL_TGT " .. tostring(source) .. ": only 3 (the site kangaroo) is supported"
    end
    -- the kangaroo (section 3b), already in this frame (origin = site),
    -- updated this tick before step(): a fresh sample every tick
    local kn, ke, vn, ve, t_s = kangaroo_sample()
    if kn == nil then return nil, ke end         -- stopped / not started: fault
    -- section 3b contains the kangaroo; no turn here, but never
    -- follow it out of the fence
    if zone ~= nil and not zone_mod.contains(zone, kn, ke, 0.0) then
        return nil, string.format("kangaroo left the fence at (%.0f N, %.0f E)", kn, ke)
    end
    local dt = (ks_last_t ~= nil) and (t_s - ks_last_t) or dt_s
    ks_last_t = t_s
    return kn, ke, vn, ve, true, dt
end


--- Estimator, exactly the runner's sequence: update, then project. With
--  HVAL_TGT 3 it updates with the measured interval between kangaroo
--  samples (one per tick); a stale sample would return the last estimate
--  (framework 7.2), as on the old bus.
--  Returns est_proj, est_raw (both nil when cfg.estimate is off).
local last_est_proj, last_est_raw = nil, nil
local function estimate(kn, ke, fresh, dt)
    if estimator == nil then return nil, nil end
    if fresh == false and last_est_raw ~= nil then return last_est_proj, last_est_raw end
    local out = estimator:update(kn, ke, dt or dt_s)
    if out == nil then return nil, nil end
    local est_raw = { n_m = out.x, e_m = out.y, vn_ms = out.vx, ve_ms = out.vy }
    local est_proj = est_mod.predict(est_raw, dt_s, cfg.lookahead_steps or 0)
    last_est_proj, last_est_raw = est_proj, est_raw
    return est_proj, est_raw
end

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
local active_name = nil
local entry = nil

-- PLAN: reset = algorithm_state = {} plus estimator re-init, all the demo's
-- anchor() does; the arms carry no other state.
local function reset_path()
    state.path = nil
    state.point_index = 1
    algorithm_state = {}
    estimator = nil
    origin = nil
end

--- The name to fly for HVAL_ARM: cfg.algorithm when it is that arm or its
--  _hyst form; otherwise a refusal (the spec carries the arm's estimator
--  settings, so another arm would fly with the wrong ones).
local function spec_name_for(requested)
    local base = ARM_NAMES[requested]
    if base == nil then
        return nil, "unknown algorithm ID " .. tostring(requested)
    end
    if cfg.algorithm ~= base and cfg.algorithm ~= base .. "_hyst" then
        return nil, string.format("HVAL_ARM %s is %s but spec.json is %s",
                                  tostring(requested), base, tostring(cfg.algorithm))
    end
    if NEEDS_ESTIMATE[base] and not cfg.estimate then
        return nil, base .. " needs the estimator (spec estimate false)"
    end
    if OWNS_HORIZON[base] and (cfg.lookahead_steps or 0) ~= 0 then
        return nil, base .. " owns its lead: spec lookahead_steps must be 0"
    end
    return cfg.algorithm
end

-- select the algorithm
local function select_algorithm()
    local requested = math.floor(HVAL_ARM:get() + 0.5)

    if requested == active_arm then
        return true
    end

    -- Initially allow changes only while disabled.
    if HVAL_ENABLE:get() ~= 0 then
        return false, "disable before changing algorithm"
    end

    local name, why = spec_name_for(requested)
    if name == nil then
        return false, why
    end

    local resolved, reason = arms.resolve(name)
    if resolved == nil then
        return false, reason
    end

    entry = resolved
    active_arm = requested
    active_name = name
    reset_path()

    gcs:send_text(MAV_SEVERITY.INFO, "HVAL: selected " .. name)
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

--- Activation switch position: true high, false low or unavailable.
local function switch_high()
    local act_fn = math.floor(HVAL_ACT_FN:get() + 0.5)
    if act_fn <= 0 then
        return false, "activation switch not configured"
    end
    if not rc:has_valid_input() or rc:get_aux_cached(act_fn) ~= 2 then
        return false, "pilot activation unavailable/off"
    end
    return true
end

local function can_engage(aircraft_valid, flight_ready, target_fresh)
    if HVAL_ENABLE:get() ~= 1 then
        return false, "disabled"
    end
    local high, why = switch_high()
    if not high then
        return false, why
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
    if entry == nil or math.floor(HVAL_ARM:get() + 0.5) ~= active_arm then
        return false, "algorithm not ready"
    end
    -- bank-limit gate: ROLL_LIMIT_DEG must be the spec's
    local roll_ok, roll_msg = roll_limit_ok()
    if not roll_ok then
        return false, roll_msg
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
         -- target values
        target_n_m = kn,
        target_e_m = ke,
        target_vn_ms = kvn,
        target_ve_ms = kve,
        -- algorithm values
        algorithm_state = algorithm_state,
        -- estimates
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

-- ============================================================
-- 8. COMMAND OUTPUT / SHADOW MODE
-- ============================================================
-- PLAN: reuse the runner's command() (GUIDED_CHANGE_ALTITUDE once, then
-- GUIDED_CHANGE_HEADING as COG, acceleration g * tan(ROLL_LIMIT_DEG)) and
-- release(); shadow = the same tick without command(). Needs GUIDED_P ~15000.
-- PLAN: bank limit: at 45 deg the 70 m ring was not held in SITL
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
local HEADING_TYPE_DEFAULT = 2      -- hand heading back to GUIDED's own navigation
local ALT_CHANGE_RATE = 1000.0      -- the runner's and plane_follow.lua's rate cap

--- gcs:run_command_int returns a MAV_RESULT integer, not a boolean, and 0
--  (MAV_RESULT_ACCEPTED) is true in Lua like every other number: test it.
local MAV_RESULT_ACCEPTED = 0
local function command_accepted(result)
    return result == MAV_RESULT_ACCEPTED
end

-- Cache the last accepted altitude command.
-- Clear this cache on release or a new engagement.
local sent_alt_m = nil

local function send_guidance(
    result, pn, pe, aircraft_valid, flight_ready, target_fresh)

    local output_mode = math.floor(HVAL_OUT:get() + 0.5)

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
        local accepted = command_accepted(
            gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE, {
                frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
                p3 = ALT_CHANGE_RATE, -- Demo's rate cap; review for the aircraft.
                z = alt_m,
            }))

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
    local accepted = command_accepted(
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING, {
            frame = MAV_FRAME_GLOBAL,
            p1 = HEADING_TYPE_COG,
            p2 = course_deg,
            p3 = acceleration,
        }))

    -- Return rejection to section 9.
    -- Section 10 must record the reason before the callback exits.
    if not accepted then
        return false, "heading command refused"
    end

    return true, "active"
end

--- Hand the heading back to GUIDED's own navigation (the runner's release()),
--  or the aircraft keeps steering the last course it was sent. Only while
--  still in GUIDED: in any other mode the GUIDED heading is already gone.
local function release()
    if state.owns_guidance and vehicle:get_mode() == MODE_GUIDED then
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING,
                            { frame = MAV_FRAME_GLOBAL, p1 = HEADING_TYPE_DEFAULT })
    end
    sent_alt_m = nil
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
-- A latched fault is cleared only by HVAL_ENABLE 0 (section 11).

local function handle_fault(reason)
    -- Report and attempt recovery once per latched fault.
    if state.fault_latched then
        return
    end

    local owned_guidance = state.owns_guidance

    state.fault_latched = true
    state.enabled = false
    state.switch_was_low = false
    state.fault_reason = tostring(reason)
    sent_alt_m = nil

    gcs:send_text(MAV_SEVERITY.ERROR, "HVAL: fault: " .. state.fault_reason)

    -- Shadow faults need no flight-mode change.
    if not owned_guidance then
        return
    end

    -- Pilot/failsafe has already changed mode: yield.
    if vehicle:get_mode() ~= MODE_GUIDED then
        state.owns_guidance = false
        return
    end

    -- Read when needed, so a change on the ground takes effect.
    local recovery_mode = math.floor(HVAL_FAIL_MODE:get() + 0.5)
    if recovery_mode <= 0 or recovery_mode == MODE_GUIDED then
        gcs:send_text(MAV_SEVERITY.ERROR, "HVAL: recovery mode not configured")
        -- Previous commands may remain effective in GUIDED: hand back.
        release()
        return
    end

    local accepted = vehicle:set_mode(recovery_mode)

    if not accepted or vehicle:get_mode() ~= recovery_mode then
        gcs:send_text(MAV_SEVERITY.ERROR, "HVAL: recovery mode change failed")
        release()
        -- Retain ownership flag: release has not been confirmed.
        return
    end

    state.owns_guidance = false
    gcs:send_text(MAV_SEVERITY.WARNING, "HVAL: recovery mode entered")
end

--- End an engagement that is not a fault: switch low, HVAL_ENABLE 0, the
--  pilot or a failsafe leaving GUIDED, or the end of cfg.duration_s. Hands
--  the heading back, and the switch must be cycled to engage again.
local function disengage(reason)
    if not state.enabled then
        return
    end
    release()
    state.owns_guidance = false
    state.enabled = false
    state.switch_was_low = false
    gcs:send_text(MAV_SEVERITY.WARNING, "HVAL: disengaged: " .. tostring(reason))
end

-- ============================================================
-- 10. LOGGING
-- ============================================================
-- PLAN: write the runner's log_tick rows (HANC once; HREC / HEST / HALG / HAIR
-- each tick) so extract_bundle.py and the metrics read a flight log unchanged
-- and it lands in the same compare.csv as Python and SITL. Replay logged poses
-- through Python to check the carrot tick for tick (the demo agreed to 1 cm). Add one
-- HVAL row: run id, arm, enable, mode, target age, command accepted.
-- Log timestamp, run ID, algorithm, enable state and flight mode.
-- Log actual dt, target age, estimate, prediction and guidance point.
-- Log aircraft position, airspeed, ground velocity and wind.
-- Log radial error, bank, altitude and command acceptance.
-- Log all switching, engagement, disengagement and fault events.
-- Rate-limit ground-station text; retain detailed onboard logs.

local PHASE_CODE = { approach = 0, orbit = 1 }
local DIR_CODE = { cw = 1, ccw = -1 }

local function num(v)
    if v == nil then return 0.0 end
    if v == true then return 1.0 end
    if v == false then return 0.0 end
    return v
end

--- Anchor the run at engagement: origin, clock, estimator, target
--  schedule, then HANC once, as the runner's anchor().
local function anchor(now_ms, pos, vel)
    local source = math.floor(HVAL_TGT:get() + 0.5)
    if source ~= 3 then
        return false, "HVAL_TGT " .. tostring(source) .. ": only 3 (the site kangaroo) is supported"
    end
    if cfg.fence == nil then return false, "HVAL_TGT 3 needs the fence (site frame)" end
    ks_last_t = nil
    local start_n, start_e = kangaroo_sample()
    if start_n == nil then return false, "kangaroo: " .. tostring(start_e) end
    origin = site_reference(pos:alt())          -- the SITE frame, as the kangaroo
    local zone_ok, zone_why = build_zone(start_n, start_e)
    if not zone_ok then
        origin = nil
        return false, zone_why
    end
    t0_ms = now_ms
    algorithm_state = {}
    sent_alt_m = nil
    estimator = nil
    last_est_proj, last_est_raw = nil, nil
    if cfg.estimate then
        estimator = est_mod.new(cfg.estimator.process_noise, cfg.estimator.measurement_noise)
        estimator:init(start_n, start_e)
    end

    state.run_id = state.run_id + 1
    local hdg, yaw = course(vel)
    logger:write('HANC', 't0ms,Lat,Lng,Alt,Hdg,Yaw,AltCmd', 'Iiiffff',
                 now_ms, origin:lat(), origin:lng(),
                 origin:alt() * 0.01,        -- actual altitude of the anchor, m (absolute)
                 hdg, yaw,
                 HVAL_ALT_M:get())           -- commanded altitude, m above home
    gcs:send_text(MAV_SEVERITY.WARNING, string.format(
        "HVAL: run %d engaged, %s, %s, hdg %.0f deg", state.run_id,
        tostring(active_name), HVAL_OUT:get() == 1 and "active" or "shadow",
        math.deg(hdg)))
    return true
end

--- The runner's per-tick rows, unchanged, plus HVAL.
local function log_tick(t, pn, pe, hdg, kn, ke, kvn, kve, gn, ge,
                        est_proj, est_raw, st, vel, yaw, status)
    logger:write('HREC', 't,PN,PE,PHdg,TN,TE,TVN,TVE,GN,GE', 'ffffffffff',
                 t, pn, pe, hdg, kn, ke, kvn, kve, gn, ge)
    if est_proj ~= nil then
        logger:write('HEST', 't,EN,EE,EVN,EVE,RN,RE,RVN,RVE', 'fffffffff',
                     t, est_proj.n_m, est_proj.e_m, est_proj.vn_ms, est_proj.ve_ms,
                     est_raw.n_m, est_raw.e_m, est_raw.vn_ms, est_raw.ve_ms)
    end
    st = st or {}
    logger:write('HALG', 't,Ph,Dir,Cur,Rep,Tsr,Held,Sw,K,Ring', 'ffffffffff',
                 t, num(PHASE_CODE[st.phase]), num(DIR_CODE[st.direction]),
                 num(st.curvature), num(st.replanned), num(st.ticks_since_replan),
                 num(st.sense_held), num(st.sense_switched),
                 num(st.k_steps or st.k_horizon), num(st.ring_angle_rad))
    local wind = ahrs:wind_estimate()
    local aspd = ahrs:airspeed_estimate() or 0.0
    local roll = ahrs:get_roll_rad()
    local gvn, gve = 0.0, 0.0
    if vel ~= nil then gvn, gve = vel:x(), vel:y() end
    local wn, we = 0.0, 0.0
    if wind ~= nil then wn, we = wind:x(), wind:y() end
    logger:write('HAIR', 't,AS,GVN,GVE,WN,WE,Yaw,Roll,Crs', 'fffffffff',
                 t, aspd, gvn, gve, wn, we, yaw, roll, hdg)
    -- HVAL: run, arm, output mode, target source, status (1 command accepted
    -- or shadow, 0 refused), arm F's carrot lead in s (0 for other arms)
    logger:write('HVAL', 't,Run,Arm,Out,Tgt,Ok,Lead', 'fffffff',
                 t, state.run_id, num(active_arm), HVAL_OUT:get(), HVAL_TGT:get(),
                 status and 1.0 or 0.0, num(st.carrot_lead_s))
end

-- ============================================================
-- 11. UPDATE CALLBACK
-- ============================================================
-- PLAN: this order already matches the runner's step().
local function step(now_ms)
    -- 1. Read time, controls and aircraft state.
    local pos = ahrs:get_location()
    local vel = ahrs:get_velocity_NED()
    local mode = vehicle:get_mode()
    local armed = arming:is_armed()
    local true_airspeed_ms = true_airspeed()
    local aircraft_valid = (pos ~= nil and vel ~= nil)

    -- Switch low arms the next engagement; low while engaged ends it.
    local high = switch_high()
    if not high then
        state.switch_was_low = true
        disengage("activation switch off")
    end

    -- 2. Yield immediately if pilot/failsafe has taken authority.
    if mode ~= MODE_GUIDED or not armed then
        disengage("left GUIDED or disarmed")
        return
    end

    -- 3. Process disengagement and permitted algorithm selection.
    if HVAL_ENABLE:get() == 0 then
        disengage("experimental guidance disabled")
        -- HVAL_ENABLE 0 is the deliberate act that clears a latched fault
        -- (not at boot when spec.json is missing).
        if state.fault_latched and cfg ~= nil and entry ~= nil then
            state.fault_latched = false
            state.fault_reason = nil
            gcs:send_text(MAV_SEVERITY.INFO, "HVAL: fault cleared")
        end
        local selected, selection_reason = select_algorithm()
        if not selected and selection_reason ~= state.selection_reason then
            -- Section 10: record selection refusal, once per reason.
            gcs:send_text(MAV_SEVERITY.WARNING, "HVAL: " .. tostring(selection_reason))
        end
        state.selection_reason = (not selected) and selection_reason or nil
        return
    end

    if state.fault_latched then
        return
    end

    -- Established flight: ArduPlane's own flying estimate and an airspeed
    -- at or above AIRSPEED_MIN.
    local v_min = AIRSPEED_MIN:get() or 0
    local flight_ready = vehicle:get_likely_flying()
        and finite(true_airspeed_ms) and true_airspeed_ms >= v_min
    -- The site kangaroo (HVAL_TGT 3) is fresh while it runs (not stopped,
    -- home set); any other HVAL_TGT is refused at engagement (anchor()).
    local target_fresh = (kangaroo_sample() ~= nil)

    -- 4. Engage: anchor once per run before converting local positions.
    if not state.enabled then
        if not state.switch_was_low then
            return
        end
        local allowed, why = can_engage(aircraft_valid, flight_ready, target_fresh)
        if not allowed then
            return
        end
        local ok, reason = anchor(now_ms, pos, vel)
        if not ok then
            handle_fault("cannot anchor: " .. tostring(reason))
            return
        end
        state.enabled = true
    end

    if not aircraft_valid then
        handle_fault("no aircraft position or velocity")
        return
    end

    local t = (now_ms - t0_ms) * 0.001
    -- The plan runs on the kangaroo's clock (KSRC_RUN 5), not from
    -- engagement, so no time limit here: the switch ends the run.

    local ne = origin:get_distance_NE(pos)
    local pn, pe = ne:x(), ne:y()
    local hdg, yaw = course(vel)

    -- Acquire target and update estimator.
    local kn, ke, kvn, kve, fresh, tdt = target_at(t)
    if kn == nil then
        handle_fault("target: " .. tostring(ke))
        return
    end
    local est_proj, est_raw = estimate(kn, ke, fresh, tdt)

    -- 5. Compute guidance.
    local result, reason = compute_guidance(
        t, pn, pe, hdg,
        kn, ke, kvn, kve,
        est_proj, est_raw
    )

    if result == nil then
        log_tick(t, pn, pe, hdg, kn, ke, kvn, kve, 0.0 / 0.0, 0.0 / 0.0,
                 est_proj, est_raw, algorithm_state, vel, yaw, false)
        handle_fault("no solution: " .. tostring(reason))
        return
    end

    -- 6. Validate output and aircraft envelope.
    local valid, why = validate_guidance(
        result, pn, pe,
        true_airspeed_ms, HVAL_BOUND_M:get()
    )

    if not valid then
        log_tick(t, pn, pe, hdg, kn, ke, kvn, kve,
                 result.guidance_n_m or 0.0 / 0.0, result.guidance_e_m or 0.0 / 0.0,
                 est_proj, est_raw, result.algorithm_state, vel, yaw, false)
        handle_fault(why)
        return
    end

    algorithm_state = result.algorithm_state or {}

    -- 7. Command only if ACTIVE and still authorised.
    local accepted, output_status = send_guidance(
        result, pn, pe,
        aircraft_valid, flight_ready, target_fresh
    )

    -- 8. Log.
    log_tick(t, pn, pe, hdg, kn, ke, kvn, kve,
             result.guidance_n_m, result.guidance_e_m,
             est_proj, est_raw, algorithm_state, vel, yaw, accepted)

    -- 9. Handle detected faults and verify recovery.
    if not accepted then
        handle_fault(output_status)
    end
end

--- One tick inside pcall, as the runner: an error is reported and
--  latched as a fault instead of silently stopping the script while a
--  heading command is still in effect.
local function tick(now_ms)
    -- the kangaroo first (it ran before the follower each tick as a
    -- separate script), then the follower reads it
    kangaroo_update(now_ms)
    step(now_ms)
end

local function update()
    local ok, err = pcall(tick, millis():toint())
    if not ok then
        gcs:send_text(MAV_SEVERITY.ERROR, "HVAL: " .. tostring(err))
        pcall(handle_fault, "script error")
        return update, 1000
    end
    return update, period_ms -- nominal callback delay in milliseconds
end

-- ============================================================
-- 12. STARTUP
-- ============================================================
-- PLAN: the runner's pcall(require, ...) / fail_load pattern; announce the arm
-- and configuration; start disabled.
-- Validate configuration and required inputs/bindings.
-- Announce script version and selected test configuration.
-- Start disabled; do not arm or change flight mode on startup.

-- Require module loading: protected, and the failure reported (require
-- itself caches what loaded)
local function load(name)
    local ok, mod = pcall(require, name)
    if not ok or mod == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "HVAL: module " .. name .. " not found: " .. tostring(mod))
        return nil
    end
    return mod
end

-- Start disabled; do not arm or change flight mode.
HVAL_ENABLE:set(0)
state.enabled = false
state.owns_guidance = false

-- No spec.json: section 1 latched a fault and said why. Stop here; nothing
-- below may run on typed defaults.
if cfg == nil then
    return
end

-- Assign the shared variables declared near the top (the
-- configuration is spec.json, not kangaroo_demo_cfg).
arms = load("sitl_arms")
est_mod = load("harness_estimator")
segs = load("harness_segments")
if cfg.fence ~= nil then
    zone_mod = load("harness_zone")
    if zone_mod == nil then
        return
    end
else
    gcs:send_text(MAV_SEVERITY.WARNING,
                  "HVAL: spec.json has no fence; the kangaroo is not contained")
end

-- Stop startup if a required module failed to load.
if arms == nil or est_mod == nil or segs == nil then
    return
end

-- The kangaroo (section 3b) needs the fence: it is the site reference.
if zone_mod ~= nil then
    kangaroo_bind(load("harness_kangaroo"))
end

-- Section 4 resolves the initial algorithm while disabled.
local selected, reason = select_algorithm()

if not selected then
    -- Keep running: the operator can correct HVAL_ARM on the ground, and
    -- can_engage() refuses until an arm is selected.
    gcs:send_text(
        MAV_SEVERITY.ERROR, "HVAL: initial algorithm unavailable: " .. tostring(reason)
    )
end

-- Announce the selected algorithm and configuration source.
gcs:send_text(
    MAV_SEVERITY.INFO, "HVAL: loaded disabled; algorithm "
        .. tostring(active_name)
        .. "; config " .. tostring(cfg_where)
)
do
    local ok_roll, roll_msg = roll_limit_ok()
    gcs:send_text(ok_roll and MAV_SEVERITY.INFO or MAV_SEVERITY.WARNING,
                  "HVAL: " .. tostring(roll_msg))
end

-- Begin scheduled callbacks.
return update, period_ms
