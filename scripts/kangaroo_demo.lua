-- =========================================================
--  kangaroo_demo -- fly the baseline against a kangaroo whose mode and
--  speed are changed live from the ground station (TASK-058)
--  created 2026-09-24
--
--  A demonstration, not an experiment. The campaign runner
--  (sitl_harness_runner.lua) replays one Python cell's fixed schedule so its
--  log can be compared with the harness; this script instead lets an
--  operator change the kangaroo's mode, pace, speed and heading from
--  MAVProxy while the aircraft follows it, so the baseline can be shown
--  against every harness mode and across the swept speeds in one flight.
--
--  What it shares with the runner, unchanged:
--    * the guidance law: sitl_arms resolves the algorithm named by the
--      generated kangaroo_demo_cfg.lua (the baseline, from a CAMP-003 cell's
--      spec) and its flattened HarnessConfig -- nothing is typed here;
--    * the kangaroo: harness_segments / harness_kangaroo, the same ported
--      mode geometry the campaign replays;
--    * the height: KDEM_ALT_M above home.
--  What it adds: the kangaroo is broadcast as ADSB_VEHICLE (sitl_adsb) so the
--  map shows it, and every parameter change rebuilds the schedule from the
--  kangaroo's CURRENT position, so the target never jumps (velocity may step,
--  as at a harness leg switch).
--
--  Command channel (KDEM_CHAN). 1, the default: the course from the aircraft
--  to the guidance point is sent as GUIDED_CHANGE_HEADING (type COG), with
--  the altitude held by GUIDED_CHANGE_ALTITUDE -- the channel the follow
--  applet's standoff behaviour uses (TASK-055 D1 = a). GUIDED then steers
--  at the point under the heading controller, as the harness aircraft does.
--  0: vehicle:set_target_location() at the point, as the campaign runner
--  does; GUIDED loiters about it at WP_LOITER_RAD, and with the point about
--  50 m ahead that is a permanent circle (2026-09-24 demonstration flight).
--  Channel 1 needs GUIDED_P raised (kangaroo-demo.parm).
--
--  Operating it (MAVProxy, after `mode TAKEOFF`, `arm throttle`, climb):
--    mode GUIDED                    start: kangaroo placed KDEM_RANGE ahead
--    param set KDEM_MODE 2          0 point, 1 straight, 2 circle,
--                                   3 rectangle, 4 rand (random legs)
--    param set KDEM_PACE 1          0 constant, 1 elastic (surge and hold)
--    param set KDEM_SPD 18.75       target speed, m/s
--    param set KDEM_HDG 90          heading, deg clockwise from North
--                                   (-1: the aircraft's heading at start)
--    param set KDEM_RESET 1         re-place the kangaroo ahead of the aircraft
--    param set KDEM_CHAN 0          command as the campaign runner (loiter)
--    param set KDEM_LOOK 5          carrot look-ahead, m: 50 the thesis
--                                   default, 5 the short-carrot baseline
--                                   (sec:carrot-optimisation). The ported
--                                   geometry takes it on every call and
--                                   re-derives the pre-compensated ring
--                                   from it, so a change is the same law at
--                                   another carrot, not a different law.
--
--  Units: metres, m/s, degrees clockwise from North. None of the KDEM_
--  parameters is a flight limit (SR-004); bank, airspeed and L1 tuning come
--  from kangaroo-follow.parm as for a campaign cell.
--
--  Log: HREC and HALG in the runner's formats (so the log reads like a cell),
--  plus HDEM once per schedule rebuild: t, mode, pace, speed, heading, and
--  why (0 start/reset, 1 parameter change, 2 containment turn).
-- =========================================================

local MAV_SEVERITY = { ERROR = 3, WARNING = 4, INFO = 6 }
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0

-- GUIDED commands for the heading channel (MAVLink common, as plane_follow.lua)
local MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
local MAV_CMD_GUIDED_CHANGE_HEADING = 43002
local MAV_FRAME_GLOBAL = 0
local MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
local HEADING_TYPE_COG = 0          -- course over ground: the harness heading
local HEADING_TYPE_DEFAULT = 2      -- hand heading back to GUIDED's own navigation
--- GUIDED limits the bank of a heading command to atan(accel / g) as well as
--  ROLL_LIMIT_DEG. The acceleration sent is g * tan(ROLL_LIMIT_DEG), so the
--  roll limit is the bank limit that binds: 45 deg in kangaroo-follow.parm
--  (the flight code), 60 deg when set to the harness's BANK_LIMIT_DEG. Until
--  2026-09-24 a fixed 10 m/s/s (the follow applet's, 45.6 deg) was sent,
--  which would have capped a 60 deg run at 45.6 deg. Fallback if the
--  parameter cannot be read: that same 10 m/s/s.
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

--- Mode codes for KDEM_MODE, in the harness's order (kangaroo.py MODES,
--  then kangaroo_rand).
local MODE_NAMES = { [0] = "point", "straight", "circle", "rectangle", "rand" }
local MODE_RAND = 4
local PACE_NAMES = { [0] = "constant", "elastic" }

--- Duration of a single-mode leg, s. Long enough that a demonstration never
--  reaches the end, where harness_segments would freeze the target.
local LEG_S = 36000.0
--- ADS-B broadcast period, ms (kangaroo_MAV.lua sent at 200 ms).
local ADSB_PERIOD_MS = 200
--- Minimum spacing of repeated guidance-failure messages, ms.
local FAIL_TEXT_PERIOD_MS = 5000

-- ---------------------------------------------------------------------
-- Generated configuration and the ported modules
-- ---------------------------------------------------------------------
local function fail_load(why)
    gcs:send_text(MAV_SEVERITY.ERROR, "KDEM: load failed: " .. tostring(why))
    return nil
end

local ok_cfg, demo = pcall(require, "kangaroo_demo_cfg")
if not ok_cfg then
    return fail_load("require('kangaroo_demo_cfg'): " .. tostring(demo))
end
local ok_seg, segs = pcall(require, "harness_segments")
if not ok_seg then
    return fail_load("require('harness_segments'): " .. tostring(segs))
end
local ok_arms, arms = pcall(require, "sitl_arms")
if not ok_arms then
    return fail_load("require('sitl_arms'): " .. tostring(arms))
end
local ok_adsb, adsb = pcall(require, "sitl_adsb")
if not ok_adsb then
    return fail_load("require('sitl_adsb'): " .. tostring(adsb))
end
local entry, entry_source = arms.resolve(demo.algorithm)
if entry == nil then
    return fail_load("algorithm '" .. tostring(demo.algorithm) .. "': " ..
                     tostring(entry_source))
end
local est_mod = nil
if demo.estimate then
    local ok_est, mod = pcall(require, "harness_estimator")
    if not ok_est then
        return fail_load("require('harness_estimator'): " .. tostring(mod))
    end
    est_mod = mod
end

local cfg = demo.cfg
local dt_s = demo.dt_s or 0.1
local period_ms = math.floor(dt_s * 1000 + 0.5)
local lookahead_steps = demo.lookahead_steps or 0

-- ---------------------------------------------------------------------
-- Parameters (KDEM_ = kangaroo demonstration). Defaults from the cell.
-- ---------------------------------------------------------------------
local PARAM_TABLE_PREFIX = "KDEM_"
local PARAM_TABLE_KEY = nil
for key = 40, 200 do
    if param:add_table(key, PARAM_TABLE_PREFIX, 15) then
        PARAM_TABLE_KEY = key
        break
    end
end
assert(PARAM_TABLE_KEY ~= nil, "KDEM: no free param table key")

local function bind(name, idx, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default),
           "KDEM: could not add " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "KDEM: could not bind " .. name)
    return p
end

local geometry = demo.geometry or {}
local KDEM_MODE   = bind("MODE",    1, demo.mode_code or 1)
local KDEM_PACE   = bind("PACE",    2, demo.pace_code or 0)
local KDEM_SPD    = bind("SPD",     3, demo.speed_ms or 12.5)
local KDEM_HDG    = bind("HDG",     4, -1)
local KDEM_RAD    = bind("RAD",     5, geometry.radius_m or 150.0)
local KDEM_LEN    = bind("LEN",     6, geometry.length_m or 300.0)
local KDEM_WID    = bind("WID",     7, geometry.width_m or 150.0)
local KDEM_RANGE  = bind("RANGE",   8, demo.start_range_m or 300.0)
local KDEM_ALT_M  = bind("ALT_M",   9, demo.alt_m or 60.0)
local KDEM_BOUND  = bind("BOUND_M", 10, demo.bound_m or 1000.0)
local KDEM_ADSB   = bind("ADSB",    11, 1)
local KDEM_RESET  = bind("RESET",   12, 0)
local KDEM_REPORT = bind("REPORT",  13, 5)
local KDEM_CHAN   = bind("CHAN",    14, 1)
local KDEM_LOOK   = bind("LOOK",    15, cfg.look_ahead_m)

gcs:send_text(MAV_SEVERITY.WARNING, string.format(
    "KDEM: loaded %s via %s (from %s)", tostring(demo.algorithm),
    tostring(entry_source), tostring(demo.source_cell)))
gcs:send_text(MAV_SEVERITY.WARNING, string.format(
    "KDEM: carrot look-ahead %.1f m", cfg.look_ahead_m))

--- Carry KDEM_LOOK into the configuration the arm reads. A non-positive
--  value is refused and the parameter put back.
local function apply_look_ahead()
    local want = KDEM_LOOK:get()
    if want == cfg.look_ahead_m then
        return
    end
    if want ~= want or want <= 0 then   -- NaN or non-positive
        gcs:send_text(MAV_SEVERITY.ERROR, string.format(
            "KDEM: look-ahead must be > 0 m, kept %.1f m", cfg.look_ahead_m))
        KDEM_LOOK:set(cfg.look_ahead_m)
        return
    end
    cfg.look_ahead_m = want
    gcs:send_text(MAV_SEVERITY.WARNING, string.format(
        "KDEM: carrot look-ahead %.1f m", want))
end

-- ---------------------------------------------------------------------
-- State
-- ---------------------------------------------------------------------
local started = false
local origin = nil          -- Location at the anchor
local home_alt_cm = nil
local t0_ms = nil
local anchor_hdg_deg = 0.0
local segments = nil
local legs = nil            -- the named legs `segments` was built from
local signature = nil       -- settings the schedule was built for
local estimator = nil
local algorithm_state = {}
local kn, ke, kvn, kve = 0.0, 0.0, 0.0, 0.0
local last_report_ms = 0
local last_adsb_ms = 0
local last_fail_ms = -FAIL_TEXT_PERIOD_MS
local sent_chan = nil       -- command channel last used
local sent_alt_m = nil      -- altitude last sent on the heading channel

local PHASE_CODE = { approach = 0, orbit = 1 }
local DIR_CODE = { cw = 1, ccw = -1 }

local function num(v)
    if v == nil then return 0.0 end
    if v == true then return 1.0 end
    if v == false then return 0.0 end
    return v
end

local function wrap360(deg)
    return deg % 360.0
end

local function settings()
    local mode = math.floor(KDEM_MODE:get() + 0.5)
    if MODE_NAMES[mode] == nil then mode = 1 end
    local pace = (math.floor(KDEM_PACE:get() + 0.5) == 1) and 1 or 0
    local hdg = KDEM_HDG:get()
    if hdg < 0 then hdg = anchor_hdg_deg end
    return {
        mode = mode, pace = pace,
        speed_ms = math.max(0.0, KDEM_SPD:get()),
        heading_deg = wrap360(hdg),
        radius_m = KDEM_RAD:get(), length_m = KDEM_LEN:get(), width_m = KDEM_WID:get(),
    }
end

local function signature_of(s)
    return string.format("%d|%d|%.3f|%.3f|%.3f|%.3f|%.3f", s.mode, s.pace,
                         s.speed_ms, s.heading_deg, s.radius_m, s.length_m, s.width_m)
end

--- The named legs for the settings: one long leg, or the recorded rand legs
--  at the chosen speed. Elastic pace applies to the single-mode legs only
--  (point has no pace to vary; rand is constant pace, as in CAMP-003).
local function legs_for(s)
    local name = MODE_NAMES[s.mode]
    if s.mode == MODE_RAND then
        local out = {}
        for i = 1, #demo.rand_legs do
            local r = demo.rand_legs[i]
            out[i] = { duration_s = r.duration_s, mode = r.mode,
                       heading_deg = r.heading_deg, speed_ms = s.speed_ms }
        end
        return out
    end
    if name == "point" then
        return { { duration_s = LEG_S, mode = "point", heading_deg = s.heading_deg,
                   speed_ms = 0.0 } }
    end
    if s.pace == 1 then
        return { { duration_s = LEG_S, mode = "elastic", heading_deg = s.heading_deg,
                   speed_ms = s.speed_ms, elastic_base = name } }
    end
    return { { duration_s = LEG_S, mode = name, heading_deg = s.heading_deg,
               speed_ms = s.speed_ms } }
end

local function geometry_of(s)
    return { radius_m = s.radius_m, length_m = s.length_m, width_m = s.width_m }
end

--- Rebuild the schedule from (n, e) at time t. Returns true on success.
local function rebuild(t, n, e, new_legs, s, why)
    local built, reason = segs.make_segments(new_legs, n, e, geometry_of(s), t)
    if built == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "KDEM: schedule refused: " .. tostring(reason))
        return false
    end
    segments = built
    legs = new_legs
    logger:write('HDEM', 't,Mode,Pace,Spd,Hdg,Why', 'ffffff',
                 t, s.mode, s.pace, s.speed_ms, s.heading_deg, why)
    return true
end

local function announce(s)
    local name = MODE_NAMES[s.mode]
    local pace = (s.mode == MODE_RAND or name == "point") and "" or (" " .. PACE_NAMES[s.pace])
    gcs:send_text(MAV_SEVERITY.WARNING, string.format(
        "KDEM: kangaroo %s%s %.2f m/s hdg %.0f", name, pace, s.speed_ms, s.heading_deg))
end

--- Index of the segment active at t (the last one past the end).
local function active_index(t)
    for i = 1, #segments do
        if segments[i].t_start <= t and t < segments[i].t_end then
            return i
        end
    end
    return #segments
end

--- Keep the kangaroo within KDEM_BOUND_M of the anchor: a straight leg that
--  is heading out past the bound is turned back towards the anchor, as the
--  harness's zone turns the target (a demonstration aid, not the zone's
--  exact geometry). Circle and rectangle legs are closed and left alone.
local function contain(t, s)
    local bound = KDEM_BOUND:get()
    if bound <= 0 or (kn * kn + ke * ke) <= bound * bound
       or (kn * kvn + ke * kve) <= 0 then
        return
    end
    local i = active_index(t)
    local leg = legs[i]
    local base = leg.mode == "elastic" and (leg.elastic_base or "straight") or leg.mode
    if base ~= "straight" then
        return
    end
    local home_hdg = wrap360(math.deg(math.atan(-ke, -kn)))
    local rest = {}
    local turned = {}
    for k, v in pairs(leg) do turned[k] = v end
    turned.heading_deg = home_hdg
    turned.duration_s = math.max(segments[i].t_end - t, dt_s)
    rest[1] = turned
    for j = i + 1, #legs do
        rest[#rest + 1] = legs[j]
    end
    if rebuild(t, kn, ke, rest, s, 2) then
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KDEM: kangaroo turned at the %.0f m bound, hdg %.0f", bound, home_hdg))
    end
end

local function heading_from(vel)
    local yaw = ahrs:get_yaw_rad()
    if vel == nil then return yaw end
    local vn, ve = vel:x(), vel:y()
    if (vn * vn + ve * ve) < 1.0 then return yaw end
    -- math.atan(ve, vn) is in (-pi, pi], the harness's heading range
    -- (state.py wrap_pi), and is passed on as it is. It was shifted into
    -- [0, 2 pi) until 2026-09-24, which the Dubins arc sweep could not take
    -- and which produced a phantom right turn west of north (TASK-058
    -- finding 3). ahrs:get_yaw_rad() is in the same range.
    local course = math.atan(ve, vn)
    return course
end

local function anchor(now_ms, pos, vel)
    local home = ahrs:get_home()
    if home == nil then
        return false, "no home"
    end
    origin = pos:copy()
    home_alt_cm = home:alt()
    t0_ms = now_ms
    algorithm_state = {}
    sent_chan, sent_alt_m = nil, nil
    anchor_hdg_deg = wrap360(math.deg(heading_from(vel)))
    local range = KDEM_RANGE:get()
    local h = math.rad(anchor_hdg_deg)
    kn, ke, kvn, kve = range * math.cos(h), range * math.sin(h), 0.0, 0.0
    if demo.estimate then
        estimator = est_mod.new(demo.estimator.process_noise,
                                demo.estimator.measurement_noise)
        estimator:init(kn, ke)
    end
    local s = settings()
    if not rebuild(0.0, kn, ke, legs_for(s), s, 0) then
        return false, "schedule"
    end
    signature = signature_of(s)
    gcs:send_text(MAV_SEVERITY.WARNING, string.format(
        "KDEM: started, kangaroo %.0f m ahead on hdg %.0f", range, anchor_hdg_deg))
    announce(s)
    return true
end

local function to_location(n, e, alt_cm)
    local loc = origin:copy()
    loc:offset(n, e)
    loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
    loc:alt(alt_cm)
    return loc
end

--- Send the guidance point on the selected channel. Returns true on success.
local function command(pn, pe, gn, ge)
    local chan = (math.floor(KDEM_CHAN:get() + 0.5) == 0) and 0 or 1
    if chan ~= sent_chan then
        if chan == 0 and sent_chan == 1 then
            -- clear the heading type, or GUIDED keeps steering the old course
            gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING,
                                { frame = MAV_FRAME_GLOBAL, p1 = HEADING_TYPE_DEFAULT })
        end
        sent_chan, sent_alt_m = chan, nil
        gcs:send_text(MAV_SEVERITY.WARNING, chan == 1 and
                      "KDEM: command channel heading (COG) at the guidance point" or
                      "KDEM: command channel location (GUIDED loiters about it)")
    end
    if chan == 0 then
        return vehicle:set_target_location(to_location(gn, ge,
            math.floor(home_alt_cm + KDEM_ALT_M:get() * 100 + 0.5)))
    end
    local alt_m = KDEM_ALT_M:get()
    if sent_alt_m ~= alt_m then
        if not gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                                   { frame = MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     p3 = ALT_CHANGE_RATE, z = alt_m }) then
            return false
        end
        sent_alt_m = alt_m
    end
    local course_deg = wrap360(math.deg(math.atan(ge - pe, gn - pn)))
    return gcs:run_command_int(MAV_CMD_GUIDED_CHANGE_HEADING,
                               { frame = MAV_FRAME_GLOBAL, p1 = HEADING_TYPE_COG,
                                 p2 = course_deg, p3 = heading_accel_mss() })
end

local function log_tick(t, pn, pe, hdg, gn, ge, state)
    logger:write('HREC', 't,PN,PE,PHdg,TN,TE,TVN,TVE,GN,GE', 'ffffffffff',
                 t, pn, pe, hdg, kn, ke, kvn, kve, gn, ge)
    local st = state or {}
    logger:write('HALG', 't,Ph,Dir,Cur,Rep,Tsr,Held,Sw,K,Ring', 'ffffffffff',
                 t, num(PHASE_CODE[st.phase]), num(DIR_CODE[st.direction]),
                 num(st.curvature), num(st.replanned), num(st.ticks_since_replan),
                 num(st.sense_held), num(st.sense_switched),
                 num(st.k_steps or st.k_horizon), num(st.ring_angle_rad))
end

local function step(now_ms)
    local pos = ahrs:get_location()
    local vel = ahrs:get_velocity_NED()
    if pos == nil then
        return
    end

    if KDEM_RESET:get() ~= 0 then
        KDEM_RESET:set(0)
        started = false
        gcs:send_text(MAV_SEVERITY.WARNING, "KDEM: reset")
    end

    if not started then
        if not arming:is_armed() or vehicle:get_mode() ~= MODE_GUIDED then
            return
        end
        local ok, why = anchor(now_ms, pos, vel)
        if not ok then
            gcs:send_text(MAV_SEVERITY.ERROR, "KDEM: cannot start: " .. tostring(why))
            return
        end
        started = true
    end

    local t = (now_ms - t0_ms) * 0.001

    local n, e, vn, ve = segs.state_at(segments, t)
    if n == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "KDEM: schedule: " .. tostring(e))
        return
    end

    -- A parameter change rebuilds the schedule from where the kangaroo is
    -- now, so position is continuous and only velocity steps.
    local s = settings()
    local sig = signature_of(s)
    if sig ~= signature then
        if rebuild(t, n, e, legs_for(s), s, 1) then
            announce(s)
            n, e, vn, ve = segs.state_at(segments, t)
        end
        signature = sig
    end
    kn, ke, kvn, kve = n, e, vn, ve
    contain(t, s)
    apply_look_ahead()

    if KDEM_ADSB:get() ~= 0 and (now_ms - last_adsb_ms) >= ADSB_PERIOD_MS then
        last_adsb_ms = now_ms
        adsb.send(to_location(kn, ke, home_alt_cm), kvn, kve)
    end

    -- The aircraft follows only while in GUIDED; the kangaroo keeps moving.
    if not arming:is_armed() or vehicle:get_mode() ~= MODE_GUIDED then
        return
    end

    local ne = origin:get_distance_NE(pos)
    if ne == nil then return end
    local pn, pe = ne:x(), ne:y()
    local hdg = heading_from(vel)

    local est_proj, est_raw = nil, nil
    if estimator ~= nil then
        local out = estimator:update(kn, ke, dt_s)
        if out ~= nil then
            est_raw = { n_m = out.x, e_m = out.y, vn_ms = out.vx, ve_ms = out.vy }
            est_proj = est_mod.predict(est_raw, dt_s, lookahead_steps)
        end
    end

    local result, reason = entry({
        t_s = t,
        plane_n_m = pn, plane_e_m = pe, plane_hdg_rad = hdg,
        target_n_m = kn, target_e_m = ke, target_vn_ms = kvn, target_ve_ms = kve,
        algorithm_state = algorithm_state,
        target_est = est_proj,
        target_est_raw = est_raw,
    }, cfg)
    if result == nil then
        -- Keep the last command and try again next tick: a demonstration
        -- carries on where a campaign cell would stop and record it.
        if (now_ms - last_fail_ms) >= FAIL_TEXT_PERIOD_MS then
            last_fail_ms = now_ms
            gcs:send_text(MAV_SEVERITY.ERROR, "KDEM: no solution: " .. tostring(reason))
        end
        log_tick(t, pn, pe, hdg, 0.0 / 0.0, 0.0 / 0.0, algorithm_state)
        return
    end
    algorithm_state = result.algorithm_state or {}

    if not command(pn, pe, result.guidance_n_m, result.guidance_e_m) then
        gcs:send_text(MAV_SEVERITY.WARNING, "KDEM: guidance command refused")
    end
    log_tick(t, pn, pe, hdg, result.guidance_n_m, result.guidance_e_m, algorithm_state)

    local report_s = KDEM_REPORT:get()
    if report_s > 0 and (now_ms - last_report_ms) >= report_s * 1000 then
        last_report_ms = now_ms
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KDEM: %s %.1fm/s d=%.0fm %s", MODE_NAMES[s.mode],
            math.sqrt(kvn * kvn + kve * kve),
            math.sqrt((kn - pn) ^ 2 + (ke - pe) ^ 2),
            tostring(algorithm_state.phase or "")))
    end
end

local function update()
    local ok, err = pcall(step, millis():toint())
    if not ok then
        gcs:send_text(MAV_SEVERITY.ERROR, "KDEM: " .. tostring(err))
        return update, 1000
    end
    return update, period_ms
end

return update()
