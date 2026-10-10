--  kangaroo_source.lua -- live streaming the kangaroo position (bus, ADS-B, AP_Follow)
--  created 7 October 2026; two-script layout on the aircraft 10 October 2026
-- implmeneted based on the prior physcial flight architecture (Sam-Follow-03)
--
--  Kanagaroo position read directly from the bus into hardware_val.lua
--  (HVAL_TGT 3), both scripts on the aircraft's flight controller.
--  Frame: north/east metres from the SITE reference, the fence's vertex
--  centroid in spec.json (site-fixed anchor; never home, never the aircraft).
--
--  KSRC_RUN 0 hold (where the kangaroo is), 5 plan (spec.json's legs: the
--  physical-validation plan from pv_plan.py; 1 is the same, the old case
--  mode), 2 live (KSRC_MODE 0 point 1 straight 2 circle 3 rectangle;
--  KSRC_PACE 0 constant 1 elastic 2 stopstart; KSRC_SPD, KSRC_HDG; any change
--  rebuilds from where the kangaroo is). KSRC_ENABLE 0 stops sending. No
--  KSRC_ or KBUS_ parameter is a flight limit.
--
--  The physical-validation plan: cfg.legs starts at the point run's
--  place (cfg.target_n_m, cfg.target_e_m: held 45 s at the fence's deepest
--  point), then the transit to the shared start and the nine runs (straight,
--  circle, rectangle at constant, elastic and stop-start pace, one lap each),
--  chained with no rests because each ends back at the shared start.
--  cfg.suite_runs names each run. Set KSRC_RUN 5 from the ground once the
--  aircraft is engaged, so the point run is not spent before then.
--
--  Host scripts directory (the aircraft's APM/scripts): kangaroo_source.lua,
--  hardware_val.lua, spec.json (the same file for both); modules/ sitl_spec,
--  harness_geom, harness_kangaroo, harness_segments, harness_zone, sitl_adsb,
--  MAVLink/mavlink_msgs and MAVLink/mavlink_msg_FOLLOW_TARGET
--  (libraries/AP_Scripting/modules/MAVLink).
--  Aircraft: ADSB_TYPE 0 and avoidance off (it must ignore its own kangaroo);
--  SCR_HEAP_SIZE: set the maximum, 1048576 (1 MiB). The two scripts share one
--  heap: flying arm AH through the whole plan in SITL (10 October 2026) peaked at
--  928 kB, about 120 kB below the maximum. Confirm on the board with
--  SCR_DEBUG_OPTS 2 before flight.
--
--  Logs: HKSR every tick (t, run, n, e, vn, ve, sent); HKSB on every rebuild
--  (t, run, speed, heading, why: 1 run change, 2 live change, 3 fence turn,
--  4 plan run start).

-- requires
local mavlink_msgs = require("MAVLink/mavlink_msgs")
local spec_mod = require("sitl_spec")
local segs = require("harness_segments")
local kang = require("harness_kangaroo")
-- projecting the fence
local zone_mod = require("harness_zone")
-- same spec.json as the aircraft
local cfg = assert(spec_mod.load())
-- the site reference is the fence: no fence, no frame
assert(cfg.fence ~= nil and type(cfg.fence.vertices_latlng) == "table"
       and #cfg.fence.vertices_latlng >= 3,
       "KSRC: spec.json has no fence, so no site reference")
-- taking configuration of the floor offset
local PERIOD_MS = math.floor((cfg.dt_s or 0.1) * 1000 + 0.5)

local MAV_SEVERITY = { ERROR = 3, WARNING = 4, INFO = 6 }
-- hold and live legs never reach their end
local LEG_S = 36000.0
-- minimum spacing of repeated warnings, ms
local WARN_PERIOD_MS = 5000

-----------------------------------------------------------------------------
-- 1. Parameters
-----------------------------------------------------------------------------

-- KSRC_ parameter table (own key), KSRC_RUN, KSRC_CHAN, KSRC_MODE ... as KDEM_
-- add in the parameter table
-- (key 142: hardware_val.lua uses 141 and the bus 143, on the aircraft)
local PARAM_TABLE_KEY = 142
local PARAM_TABLE_PREFIX = "KSRC_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), "KSRC: could not add param table")
local function bind(name, idx, default)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), "KSRC: could not add " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "KSRC: could not bind " .. name)
    return p
end
local geometry = cfg.geometry or {}

-- 0: stop sending (stale-target test)
local KSRC_ENABLE = bind("ENABLE", 1, 1)
-- MAVLink channel of the link to the aircraft (FOLLOW_TARGET, KSRC_OUT bit 1)
local KSRC_CHAN   = bind("CHAN",   2, 0)
-- 0 hold, 5 plan (1 is the same), 2 live
local KSRC_RUN    = bind("RUN",    3, 0)
local KSRC_MODE   = bind("MODE",   4, 1)
local KSRC_PACE   = bind("PACE",   5, 0)
local KSRC_SPD    = bind("SPD",    6, cfg.speed_ms or 6.25)
local KSRC_HDG    = bind("HDG",    7, 0)
local KSRC_RAD    = bind("RAD",    8, geometry.radius_m or 60.0)
local KSRC_LEN    = bind("LEN",    9, geometry.length_m or 140.0)
local KSRC_WID    = bind("WID",   10, geometry.width_m or 70.0)
local KSRC_PSLOW  = bind("PSLOW", 11, kang.PACE_DEFAULTS.slow_factor)
local KSRC_PHOLD  = bind("PHOLD", 12, kang.PACE_DEFAULTS.hold_slow_s)
local KSRC_PFAST  = bind("PFAST", 13, kang.PACE_DEFAULTS.hold_fast_s)
local KSRC_PRAMP  = bind("PRAMP", 14, kang.PACE_DEFAULTS.ramp_down_s)
local MODE_NAMES = { [0] = "point", "straight", "circle", "rectangle" }
local PACE_NAMES = { [0] = "constant", "elastic", "stopstart" }
local RUN_HOLD, RUN_LIVE, RUN_PLAN = 0, 2, 5
local RUN_NAMES = { [0] = "hold", "plan", "live", [5] = "plan" }

-----------------------------------------------------------------------------
-- 1.1  New KSRC_ parameters (next free indices in the KSRC_ table, size 16)
-----------------------------------------------------------------------------
-- bit 0: on-board bus (KBUS_)
--  bit 1: FOLLOW_TARGET on KSRC_CHAN (to a remote aircraft's AP_Follow)
-- 1 = bus only (Follow-03), 2 = remote only,
-- 3 = both
local KSRC_OUT  = bind("OUT",  15, 1)
-- 0 off, 1 broadcast ADSB_VEHICLE every 200 ms (display only)
local KSRC_ADSB = bind("ADSB", 16, 1)

-----------------------------------------------------------------------------
-- 1.2. The bus: its own parameter table
-----------------------------------------------------------------------------
-- KBUS_ table, key 143 (hardware_val 141, KSRC_ 142), 8 entries, written
-- only by this script and read by hardware_val.lua (read_bus, HVAL_TGT 3).
-- use :set(), never :set_and_save(): nothing is written to storage
local KBUS_TABLE_KEY = 143
local KBUS_PREFIX = "KBUS_"
assert(param:add_table(KBUS_TABLE_KEY, KBUS_PREFIX, 8), "KSRC: could not add KBUS table")
local function bus_bind(name, idx)
    assert(param:add_param(KBUS_TABLE_KEY, idx, name, 0), "KSRC: could not add KBUS_" .. name)
    local p = Parameter()
    assert(p:init(KBUS_PREFIX .. name), "KSRC: could not bind KBUS_" .. name)
    return p
end
local KBUS = {
    -- odd while writing, even when stable
    SEQ = bus_bind("SEQ", 1),
    -- sample time, s since boot (float32: about 0.25 ms resolution after an
    -- hour, enough for a 100 ms tick)
    T_S = bus_bind("T_S", 2),
    -- kangaroo north of SITE, m   -- metres, NOT degrees: float32 degrees were
    N_M = bus_bind("N_M", 3),      --  0.4 m N / 1.4 m E at the site (Follow-03);
    -- kangaroo east of SITE, m    --  metres are < 0.1 mm at 1 km
    E_M = bus_bind("E_M", 4),
    -- m/s
    VN  = bus_bind("VN",  5),
    VE  = bus_bind("VE",  6),
    -- KSRC_RUN in force (0 hold, 2 live, 5 plan)
    RUN = bus_bind("RUN", 7),
    -- rebuild count (a jump in it marks a run change, mode change or fence turn)
    REB = bus_bind("REB", 8),
}
local bus_seq = 0
-- publish bus
local function publish_bus(t_s, n, e, vn, ve, run, rebuilds)
    bus_seq = bus_seq + 1
    -- odd: writing
    KBUS.SEQ:set(2 * bus_seq - 1)
    KBUS.T_S:set(t_s)
    -- in this script's frame: from SITE
    KBUS.N_M:set(n)
    KBUS.E_M:set(e)
    KBUS.VN:set(vn)
    KBUS.VE:set(ve)
    KBUS.RUN:set(run)
    -- count kept in rebuild()
    KBUS.REB:set(rebuilds)
    -- even: stable
    KBUS.SEQ:set(2 * bus_seq)
end

-----------------------------------------------------------------------------
-- 2. ADS-B for the ground station
-----------------------------------------------------------------------------
-- display only (sitl_adsb, Follow-03's framing)
local ok_adsb, adsb = pcall(require, "sitl_adsb")
local ADSB_PERIOD_MS = 200
local last_adsb_ms = nil
local function broadcast(now_ms, loc, vn, ve)
    if not ok_adsb or KSRC_ADSB:get() <= 0 then
        return
    end
    if last_adsb_ms ~= nil and (now_ms - last_adsb_ms) < ADSB_PERIOD_MS then
        return
    end
    last_adsb_ms = now_ms
    -- every channel; missing ones dropped
    adsb.send(loc, vn, ve)
end

-----------------------------------------------------------------------------
-- 3. Fence gemoetry
-----------------------------------------------------------------------------
-- Site reference: the fence's vertex centroid from spec.json. Same point
--     every sortie; never home, never the aircraft. pv_plan.py uses the same
--     point, so the spec's legs are metres from it. alt_cm (home's ground
--     AMSL) is only for the ADS-B and FOLLOW_TARGET altitude.
local function site_reference(alt_cm)
    -- addiing site reference
    local v = cfg.fence.vertices_latlng
    local lat, lng = 0.0, 0.0
    for i = 1, #v do
        lat = lat + v[i][1]
        lng = lng + v[i][2]
    end
    local site = Location()
    site:lat(math.floor(lat / #v * 1e7 + 0.5))
    site:lng(math.floor(lng / #v * 1e7 + 0.5))
    -- ground AMSL, cm (display and FOLLOW_TARGET only)
    site:alt(alt_cm)
    return site
end

-----------------------------------------------------------------------------
-- 4. Geometry runs and speed variations (live mode, KSRC_RUN 2)
-----------------------------------------------------------------------------
-- the live settings, as the demo's KDEM_ parameters (mode, pace, speed)
local function settings()
    local mode = math.floor(KSRC_MODE:get() + 0.5)
    if MODE_NAMES[mode] == nil then mode = 1 end
    local pace = math.floor(KSRC_PACE:get() + 0.5)
    if PACE_NAMES[pace] == nil then pace = 0 end
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
    return { { duration_s = LEG_S, mode = "point", heading_deg = 0.0, speed_ms = 0.0 } }
end

-- the one live leg, by the demo's rules: elastic and stopstart travel over
-- the chosen base; point has no pace
local function live_legs(s)
    local name = MODE_NAMES[s.mode]
    if name == "point" then
        return hold_legs()
    end
    if s.pace == 1 then
        return { { duration_s = LEG_S, mode = "elastic", heading_deg = s.heading_deg,
                   speed_ms = s.speed_ms, elastic_base = name } }
    end
    if s.pace == 2 then
        return { { duration_s = LEG_S, mode = "stopstart", heading_deg = s.heading_deg,
                   speed_ms = s.speed_ms, elastic_base = name, pace = s.pace_profile } }
    end
    return { { duration_s = LEG_S, mode = name, heading_deg = s.heading_deg,
               speed_ms = s.speed_ms } }
end

-----------------------------------------------------------------------------
-- 5. Full package
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
-- 6. Running the suite
-----------------------------------------------------------------------------
-- site: the SITE Location; zone: the fence in the site frame; running: the
-- KSRC_RUN the schedule was built for; legs, signature: what the schedule was
-- built from; kn, ke: the kangaroo last tick (from SITE); stopped: a refused
-- schedule ends the stream
local site, zone, segments, t0_ms = nil, nil, nil, nil
local running, legs, signature, kn, ke, stopped = RUN_HOLD, nil, nil, nil, nil, false
local rebuild_count = 0
local plan_t0, plan_run, plan_spare, plan_spare_run, plan_done = 0.0, 0, nil, "", false
local last_breach_ms = -WARN_PERIOD_MS
-- section 7, defined below
local outputs

-- rebuild from where the kangaroo is (position continuous, only velocity
-- steps); why: 1 run change, 2 live change, 3 fence turn. False on refusal.
local function rebuild(t, n, e, new_legs, geom, why)
    local built, reason = segs.make_segments(new_legs, n, e, geom, t)
    if built == nil then
        stopped = true
        gcs:send_text(MAV_SEVERITY.ERROR, "KSRC: stopped: schedule refused: " .. tostring(reason))
        return false
    end
    segments, legs = built, new_legs
    rebuild_count = rebuild_count + 1
    logger:write('HKSB', 't,Run,Spd,Hdg,Why', 'fffff', t, running,
                 new_legs[1].speed_ms or 0.0, new_legs[1].heading_deg or 0.0, why)
    return true
end

local function active_index(t)
    for i = 1, #segments do
        if segments[i].t_start <= t and t < segments[i].t_end then
            return i
        end
    end
    return #segments
end

-- spare to the orbit-radius limit inside the fence, m (negative: past it)
local function spare_of(n, e)
    local d = zone_mod.inward_distances(zone, n, e)
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
    local spare = spare_of(kn, ke)
    if plan_spare == nil or spare < plan_spare then
        plan_spare = spare
        plan_spare_run = (runs[plan_run] and runs[plan_run].name) or ""
    end
    if spare < 0.0 and (now_ms - last_breach_ms) >= WARN_PERIOD_MS then
        last_breach_ms = now_ms
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "KSRC: kangaroo %.1f m past the %.0f m limit (%.0f N, %.0f E)",
            -spare, cfg.fence.containment_margin_m, kn, ke))
    end
    if not plan_done and #runs > 0 and tp >= runs[#runs].t_end_s then
        plan_done = true
        gcs:send_text(MAV_SEVERITY.WARNING, string.format(
            "KSRC: plan complete; least spare %.1f m (%s)", plan_spare, plan_spare_run))
    end
end

local function update()
    if stopped then return update, 1000 end
    if site == nil then
        -- home only for the display altitude; the frame is the SITE reference
        if not ahrs:home_is_set() then return update, 1000 end
        site = site_reference(ahrs:get_home():alt())
        zone = assert(zone_mod.from_latlng(site, cfg.fence.vertices_latlng))
        t0_ms = millis()
        -- point leg at the point run's place until KSRC_RUN
        kn, ke = cfg.target_n_m, cfg.target_e_m
        if not rebuild(0.0, kn, ke, hold_legs(), cfg.geometry, 1) then return update, 1000 end
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KSRC: kangaroo at %.0f N %.0f E of the site; KSRC_RUN 5 starts the plan", kn, ke))
    end
    local t = (millis() - t0_ms):tofloat() * 0.001
    local run = math.floor(KSRC_RUN:get() + 0.5)
    if RUN_NAMES[run] == nil then run = RUN_HOLD end
    if run == 1 then run = RUN_PLAN end

    -- plan mode (the old case mode): the spec's legs from the point run's place
    if run == RUN_PLAN and running ~= RUN_PLAN then
        local pl, why = plan_legs()
        if pl == nil then
            stopped = true
            gcs:send_text(MAV_SEVERITY.ERROR, "KSRC: " .. why)
            return update, 1000
        end
        running = RUN_PLAN
        kn, ke = cfg.target_n_m, cfg.target_e_m
        plan_t0, plan_run, plan_spare, plan_spare_run, plan_done = t, 0, nil, "", false
        if not rebuild(t, kn, ke, pl, cfg.geometry, 1) then return update, 1000 end
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KSRC: plan, %d runs, from %.0f N %.0f E of the site",
            #(cfg.suite_runs or {}), kn, ke))
    end

    -- live mode: on a KSRC_* change, rebuild from where the kangaroo is
    -- (the demo's rules carried here; a shared kangaroo_live.lua module is a
    -- possible follow-up)
    if run == RUN_LIVE then
        local s = settings()
        if running ~= RUN_LIVE or signature ~= signature_of(s) then
            local why = (running == RUN_LIVE) and 2 or 1
            running, signature = RUN_LIVE, signature_of(s)
            if not rebuild(t, kn, ke, live_legs(s), s.geometry, why) then return update, 1000 end
            gcs:send_text(MAV_SEVERITY.INFO, string.format("KSRC: live %s %s %.2f m/s hdg %.0f",
                MODE_NAMES[s.mode], PACE_NAMES[s.pace], s.speed_ms, s.heading_deg))
        end
    end

    -- hold: KSRC_RUN 0 after a run stands the kangaroo where it is
    if run == RUN_HOLD and running ~= RUN_HOLD then
        running = RUN_HOLD
        if not rebuild(t, kn, ke, hold_legs(), cfg.geometry, 1) then return update, 1000 end
        gcs:send_text(MAV_SEVERITY.INFO, "KSRC: hold")
    end

    local n, e, vn, ve = segs.state_at(segments, t)
    kn, ke = n, e
    local now_ms = millis():toint()

    if running == RUN_PLAN then
        plan_watch(t, now_ms)
    elseif running == RUN_LIVE then
        -- live mode only: turn back at the fence by the harness's rule (harness_zone)
        local leg = legs[active_index(t)]
        local turned = zone_mod.contain_heading(zone, n, e, leg.heading_deg or 0.0,
                                                leg.speed_ms or 0.0, PERIOD_MS * 0.001,
                                                cfg.fence.containment_margin_m)
        if turned ~= nil then
            local new_leg = zone_mod.turned_leg(leg, turned)
            new_leg.duration_s = LEG_S
            if not rebuild(t, n, e, { new_leg }, settings().geometry, 3) then
                return update, 1000
            end
        end
    end

    outputs(t, now_ms, n, e, vn, ve)
    return update, PERIOD_MS
end

-----------------------------------------------------------------------------
-- 7. Adjusting outputs for site reference offset
-----------------------------------------------------------------------------
-- the outputs, every tick, from the SITE reference: the bus (KSRC_OUT bit 0),
-- FOLLOW_TARGET (bit 1) and the ADS-B display
outputs = function(t, now_ms, n, e, vn, ve)
    if KSRC_ENABLE:get() <= 0 then return end
    local out = math.floor(KSRC_OUT:get() + 0.5)
    local loc = site:copy()
    loc:offset(n, e)
    if (out & 1) ~= 0 then
        publish_bus(now_ms * 0.001, n, e, vn, ve, running, rebuild_count)
    end
    local sent = false
    if (out & 2) ~= 0 then
        local msg = { timestamp = now_ms, est_capabilities = 3,
                      lat = loc:lat(), lon = loc:lng(), alt = site:alt() * 0.01,
                      vel = { vn, ve, 0 }, acc = { 0, 0, 0 },
                      attitude_q = { 1, 0, 0, 0 }, rates = { 0, 0, 0 },
                      position_cov = { 0, 0, 0 }, custom_state = 0 }
        sent = mavlink:send_chan(KSRC_CHAN:get(), mavlink_msgs.encode("FOLLOW_TARGET", msg))
    end
    broadcast(now_ms, loc, vn, ve)
    logger:write('HKSR', 't,Run,N,E,VN,VE,Sent', 'fffffff',
                 t, running, n, e, vn, ve, sent and 1 or 0)
end

return update, 1000
