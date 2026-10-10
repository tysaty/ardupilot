-- =========================================================
--  sitl_harness_runner -- fly one harness cell in ArduPlane SITL (TASK-052)
--  created 2026-09-16
--
--  The vehicle side of the SITL campaign. It is the "caller" TASK-006 left
--  unwritten: it wires a ported harness_* module (through sitl_arms) into a
--  flight script, replays the cell's kangaroo schedule from a generated leg
--  table (TASK-046 D4, TASK-052 D2) and logs the harness record quantities
--  into the DataFlash log (TASK-046 D5, TASK-052 D3) so extract_bundle.py is
--  a column read.
--
--  It replaces control_cont.lua AND kangaroo_MAV.lua for the duration of a
--  cell (stage_scripts.py moves them aside and restores them): there is no
--  bus, no ADS-B and no KF_ parameter table here. The target is evaluated
--  from the schedule at the tick's time, exactly as the Python harness does,
--  and the estimator is the ported harness_estimator with the spec's noise.
--
--  What is deliberately NOT here: any guidance geometry, any parameter that
--  is a flight limit, any decision the task files reserve. Airspeed and L1
--  tuning are in kangaroo-follow.parm (SR-004); the bank limit is the
--  spec's roll_limit_deg, which the driver sets as ROLL_LIMIT_DEG and this
--  script only checks (ADR-011).
--
--  Inputs
--  ------
--    scripts/spec.json       the shared configuration (ADR-011), generated
--                            per cell by kangaroo_follow/schedule.py
--                            (spec_cfg) from the Python cell's spec.json and
--                            read through modules/sitl_spec.lua as `cfg`,
--                            one flat table: the flattened HarnessConfig the
--                            arm reads, and at the same level cell_id,
--                            algorithm, legs, geometry {radius_m, length_m,
--                            width_m}, target_n_m, target_e_m, duration_s,
--                            dt_s, estimate, lookahead_steps, estimator
--                            {process_noise, measurement_noise},
--                            heading_source ("course" | "yaw"),
--                            roll_limit_deg. The demonstration and the
--                            hardware validation script read the same file.
--                            Until 2026-10-04: modules/sitl_cfg.lua.
--    SHR_START (param)       the AutoTest driver sets 1 when the aircraft is
--                            at the D6 pose in GUIDED; the window starts on
--                            the next tick and the local frame is anchored
--                            at the aircraft's position at that instant.
--
--  Outputs
--  -------
--    HANC  once at anchor: origin lat/lon/alt, heading, sim time.
--    HREC  per tick: t, plane N/E/hdg, target N/E/vN/vE, guidance N/E.
--    HEST  per tick when estimating: projected and raw estimate.
--    HALG  per tick: phase, direction, curvature, replan fields, sense fields.
--    HAIR  per tick: airspeed, ground velocity, EKF wind, yaw, roll, course.
--    SHR_DONE = 1 when the window has elapsed, 2 on a refusal (no solution),
--    3 on a load error. SHR_T_S mirrors elapsed time for the driver.
--
--  Frame: north/east metres about the anchor, heading radians clockwise from
--  North, as the harness.
--
--  Command channel (SHR_CHAN; TASK-061, 2026-10-01). 1, the default: the
--  course from the aircraft to the guidance point is sent as
--  GUIDED_CHANGE_HEADING (type COG, bank capped at ROLL_LIMIT_DEG), with the
--  altitude held at SHR_ALT_M by GUIDED_CHANGE_ALTITUDE -- the channel
--  kangaroo_demo.lua flies and TASK-058 certified, so the aircraft steers at
--  the point as the harness aircraft does. Needs GUIDED_P raised
--  (kangaroo-follow.parm). 0: vehicle:set_target_location() at the point at
--  SHR_ALT_M above home, the runner's only channel until 2026-10-01 and the
--  same single call as control_cont.lua:370. GUIDED loiters about that
--  location at WP_LOITER_RAD, so with a carrot shorter than the loiter
--  radius it flies a bank-limited circle, not the law (TASK-058 thesis
--  point 1). Kept selectable for that comparison only.
-- =========================================================

local MAV_SEVERITY = { ERROR = 3, WARNING = 4, INFO = 6 }
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- GUIDED commands for the heading channel (MAVLink common, as kangaroo_demo.lua)
local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002
local MAV_FRAME_GLOBAL = 0
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
local HEADING_TYPE_COG = 0          -- course over ground: the harness heading
local HEADING_TYPE_DEFAULT = 2      -- hand heading back to GUIDED's own navigation
--- GUIDED caps a heading command's bank at atan(accel / g) as well as
--  ROLL_LIMIT_DEG. Sending g * tan(ROLL_LIMIT_DEG) makes the roll limit the
--  one that binds (kangaroo_demo.lua, 2026-09-24). Fallback if the parameter
--  cannot be read: the follow applet's 10 m/s/s (45.6 deg).
local GRAVITY_MSS = 9.80665
local HEADING_ACCEL_FALLBACK_MSS = 10.0
local function heading_accel_mss()
    local roll_deg = param:get("ROLL_LIMIT_DEG")
    if roll_deg == nil or roll_deg <= 0 or roll_deg >= 89 then
        return HEADING_ACCEL_FALLBACK_MSS
    end
    return GRAVITY_MSS * math.tan(math.rad(roll_deg))
end
--- Climb/descent rate cap sent with the altitude, as plane_follow.lua (max).
local ALT_CHANGE_RATE = 1000.0

-- ---------------------------------------------------------------------
-- Parameters (SHR_ = SITL harness runner). None is a flight limit.
-- ---------------------------------------------------------------------
local PARAM_TABLE_KEY = nil
local PARAM_TABLE_PREFIX = "SHR_"
for key = 40, 200 do
    if param:add_table(key, PARAM_TABLE_PREFIX, 7) then
        PARAM_TABLE_KEY = key
        break
    end
end
assert(PARAM_TABLE_KEY ~= nil, "SHR: no free param table key")

local function bind(name, idx, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default),
           "SHR: could not add " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "SHR: could not bind " .. name)
    return p
end

local SHR_START = bind("START", 1, 0)   -- driver sets 1 to open the window
local SHR_DONE  = bind("DONE",  2, 0)   -- runner: 1 done, 2 refused, 3 load error
local SHR_ALT_M = bind("ALT_M", 3, 100) -- commanded altitude above home, m
local SHR_T_S   = bind("T_S",   4, 0)   -- elapsed window time, s (mirror)
local SHR_TICK  = bind("TICK",  5, 0)   -- tick counter (mirror)
local SHR_REPORT = bind("REPORT", 6, 5) -- status text period, s (0 = quiet)
local SHR_CHAN  = bind("CHAN",  7, 1)   -- command channel: 1 heading (COG), 0 location

-- ---------------------------------------------------------------------
-- Load the cell and the ported modules. A failure here is reported and the
-- script stops scheduling itself: an unflyable cell is a finding, not a loop.
-- ---------------------------------------------------------------------
local function fail_load(why)
    gcs:send_text(MAV_SEVERITY.ERROR, "SHR: load failed: " .. tostring(why))
    SHR_DONE:set(3)
    return nil
end

local ok_spec, spec_mod = pcall(require, "sitl_spec")
if not ok_spec then
    return fail_load("require('sitl_spec'): " .. tostring(spec_mod))
end
local cfg, cfg_where = spec_mod.load()
if cfg == nil then
    return fail_load("spec.json: " .. tostring(cfg_where))
end
local ok_seg, segs = pcall(require, "harness_segments")
if not ok_seg then
    return fail_load("require('harness_segments'): " .. tostring(segs))
end
local ok_arms, arms = pcall(require, "sitl_arms")
if not ok_arms then
    return fail_load("require('sitl_arms'): " .. tostring(arms))
end

local entry, entry_source = arms.resolve(cfg.algorithm)
if entry == nil then
    return fail_load("algorithm '" .. tostring(cfg.algorithm) .. "': " ..
                     tostring(entry_source))
end

local segments, seg_reason = segs.make_segments(
    cfg.legs, cfg.target_n_m, cfg.target_e_m, cfg.geometry, 0.0)
if segments == nil then
    return fail_load("make_segments: " .. tostring(seg_reason))
end

local estimator = nil
local est_mod = nil
if cfg.estimate then
    local ok_est, mod = pcall(require, "harness_estimator")
    if not ok_est then
        return fail_load("require('harness_estimator'): " .. tostring(mod))
    end
    est_mod = mod
end

local dt_s = cfg.dt_s or 0.1
local period_ms = math.floor(dt_s * 1000 + 0.5)
local duration_s = cfg.duration_s
local lookahead_steps = cfg.lookahead_steps or 0
local heading_source = cfg.heading_source or "course"

gcs:send_text(MAV_SEVERITY.WARNING, string.format(
    "SHR: loaded cell %s arm %s via %s", tostring(cfg.cell_id),
    tostring(cfg.algorithm), tostring(entry_source)))
-- ADR-011: the driver sets ROLL_LIMIT_DEG to cfg.roll_limit_deg; a mismatch
-- means the cell is not flying the configuration it was generated for.
do
    local ok_roll, _live, roll_msg = spec_mod.roll_limit_check(cfg)
    gcs:send_text(ok_roll and MAV_SEVERITY.INFO or MAV_SEVERITY.WARNING,
                  "SHR: " .. roll_msg)
end

-- ---------------------------------------------------------------------
-- Run state
-- ---------------------------------------------------------------------
local started = false
local finished = false
local origin = nil          -- Location at anchor
local home_alt_cm = nil
local t0_ms = nil
local tick = 0
local algorithm_state = {}
local last_report_ms = 0
local sent_alt_m = nil      -- altitude last sent on the heading channel

local function channel()
    return (math.floor(SHR_CHAN:get() + 0.5) == 0) and 0 or 1
end

local PHASE_CODE = { approach = 0, orbit = 1 }
local DIR_CODE = { cw = 1, ccw = -1 }

local function num(v)
    if v == nil then return 0.0 end
    if v == true then return 1.0 end
    if v == false then return 0.0 end
    return v
end

local function heading_from(vel)
    -- Ground course from the NED velocity; yaw when asked for or when the
    -- aircraft is not moving. Recorded in HAIR so the choice is inspectable.
    local yaw = ahrs:get_yaw_rad()
    if heading_source == "yaw" or vel == nil then
        return yaw, yaw
    end
    local vn, ve = vel:x(), vel:y()
    if (vn * vn + ve * ve) < 1.0 then
        return yaw, yaw
    end
    -- math.atan(ve, vn) is in (-pi, pi], the harness's heading range
    -- (state.py wrap_pi), and is passed on as it is. It was shifted into
    -- [0, 2 pi) until 2026-09-24, which the Dubins arc sweep could not take
    -- and which produced a phantom right turn west of north (TASK-058
    -- finding 3). ahrs:get_yaw_rad() is in the same range.
    local course = math.atan(ve, vn)
    return course, yaw
end

local function anchor(now_ms, pos, vel)
    origin = pos:copy()
    local home = ahrs:get_home()
    if home == nil then
        return false, "no home"
    end
    home_alt_cm = home:alt()
    t0_ms = now_ms
    tick = 0
    algorithm_state = {}
    if cfg.estimate then
        estimator = est_mod.new(cfg.estimator.process_noise,
                                cfg.estimator.measurement_noise)
        estimator:init(cfg.target_n_m, cfg.target_e_m)
    end
    sent_alt_m = nil
    local hdg, yaw = heading_from(vel)
    logger:write('HANC', 't0ms,Lat,Lng,Alt,Hdg,Yaw,AltCmd', 'Iiiffff',
                 now_ms, origin:lat(), origin:lng(), origin:alt() * 0.01,
                 hdg, yaw, SHR_ALT_M:get())
    gcs:send_text(MAV_SEVERITY.WARNING, string.format(
        "SHR: started at hdg %.0f deg, channel %s", math.deg(hdg),
        channel() == 1 and "heading" or "location"))
    return true
end

--- Send the guidance point on the SHR_CHAN channel. Returns true on success.
local function command(pn, pe, gn, ge)
    if channel() == 0 then
        local loc = origin:copy()
        loc:offset(gn, ge)
        loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
        loc:alt(math.floor(home_alt_cm + SHR_ALT_M:get() * 100 + 0.5))
        return vehicle:set_target_location(loc)
    end
    local alt_m = SHR_ALT_M:get()
    if sent_alt_m ~= alt_m then
        if not gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                                   { frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     p3 = ALT_CHANGE_RATE, z = alt_m }) then
            return false
        end
        sent_alt_m = alt_m
    end
    local course_deg = math.deg(math.atan(ge - pe, gn - pn)) % 360.0
    return gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING,
                               { frame = MAV_FRAME_GLOBAL, p1 = HEADING_TYPE_COG,
                                 p2 = course_deg, p3 = heading_accel_mss() })
end

--- At the end of the window, hand the heading back to GUIDED's own
--  navigation, or the aircraft keeps steering the last course it was sent.
local function release()
    if channel() == 1 and sent_alt_m ~= nil then
        gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING,
                            { frame = MAV_FRAME_GLOBAL, p1 = HEADING_TYPE_DEFAULT })
    end
end

local function log_tick(t, pn, pe, hdg, kn, ke, kvn, kve, gn, ge,
                        est_proj, est_raw, state, vel, yaw)
    logger:write('HREC', 't,PN,PE,PHdg,TN,TE,TVN,TVE,GN,GE', 'ffffffffff',
                 t, pn, pe, hdg, kn, ke, kvn, kve, gn, ge)
    if est_proj ~= nil then
        logger:write('HEST', 't,EN,EE,EVN,EVE,RN,RE,RVN,RVE', 'fffffffff',
                     t, est_proj.n_m, est_proj.e_m, est_proj.vn_ms, est_proj.ve_ms,
                     est_raw.n_m, est_raw.e_m, est_raw.vn_ms, est_raw.ve_ms)
    end
    local st = state or {}
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
end

local function step(now_ms)
    local pos = ahrs:get_location()
    local vel = ahrs:get_velocity_NED()
    if pos == nil then
        return
    end

    if not started then
        if SHR_START:get() ~= 1 or not arming:is_armed()
           or vehicle:get_mode() ~= MODE_GUIDED then
            return
        end
        local ok, why = anchor(now_ms, pos, vel)
        if not ok then
            gcs:send_text(MAV_SEVERITY.ERROR, "SHR: cannot anchor: " .. why)
            return
        end
        started = true
    end

    local t = (now_ms - t0_ms) * 0.001
    if t > duration_s then
        if not finished then
            finished = true
            release()
            SHR_DONE:set(1)
            gcs:send_text(MAV_SEVERITY.WARNING, string.format(
                "SHR: done, %d ticks in %.1f s", tick, t))
        end
        return
    end

    -- Plane in the anchor frame
    local ne = origin:get_distance_NE(pos)
    if ne == nil then return end
    local pn, pe = ne:x(), ne:y()
    local hdg, yaw = heading_from(vel)

    -- Kangaroo from the replayed schedule, as the harness evaluates it
    local kn, ke, kvn, kve = segs.state_at(segments, t)
    if kn == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "SHR: schedule: " .. tostring(ke))
        return
    end

    -- Estimator, exactly the harness sequence: update, then project
    local est_proj, est_raw = nil, nil
    if estimator ~= nil then
        local out = estimator:update(kn, ke, dt_s)
        if out ~= nil then
            est_raw = { n_m = out.x, e_m = out.y, vn_ms = out.vx, ve_ms = out.vy }
            est_proj = est_mod.predict(est_raw, dt_s, lookahead_steps)
        end
    end

    local snapshot = {
        t_s = t,
        plane_n_m = pn, plane_e_m = pe, plane_hdg_rad = hdg,
        target_n_m = kn, target_e_m = ke, target_vn_ms = kvn, target_ve_ms = kve,
        algorithm_state = algorithm_state,
        target_est = est_proj,
        target_est_raw = est_raw,
    }

    local result, reason = entry(snapshot, cfg)
    if result == nil then
        -- The harness's "records the failure and stops" behaviour.
        finished = true
        release()
        SHR_DONE:set(2)
        gcs:send_text(MAV_SEVERITY.ERROR, "SHR: no solution: " .. tostring(reason))
        log_tick(t, pn, pe, hdg, kn, ke, kvn, kve, 0.0 / 0.0, 0.0 / 0.0,
                 est_proj, est_raw, algorithm_state, vel, yaw)
        return
    end
    algorithm_state = result.algorithm_state or {}

    if not command(pn, pe, result.guidance_n_m, result.guidance_e_m) then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "SHR: command failed (channel %d)", channel()))
    end

    tick = tick + 1
    log_tick(t, pn, pe, hdg, kn, ke, kvn, kve,
             result.guidance_n_m, result.guidance_e_m,
             est_proj, est_raw, algorithm_state, vel, yaw)
    SHR_T_S:set(t)
    SHR_TICK:set(tick)

    local report_s = SHR_REPORT:get()
    if report_s > 0 and (now_ms - last_report_ms) >= report_s * 1000 then
        last_report_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "SHR: t=%.1f d=%.0fm %s", t, math.sqrt((kn - pn)^2 + (ke - pe)^2),
            tostring(algorithm_state.phase or "")))
    end
end

local function update()
    local ok, err = pcall(step, millis():toint())
    if not ok then
        gcs:send_text(MAV_SEVERITY.ERROR, "SHR: " .. tostring(err))
        return update, 1000
    end
    return update, period_ms
end

return update()
