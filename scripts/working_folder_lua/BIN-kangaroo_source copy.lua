--  kangaroo_source.lua -- live streaming the kangaroo position via AP_Follow
--  created 7 October 2026
-- implmeneted based on the prior physcial flight architecture
--
--  Kanagaroo position read directly from the bus into hardware_val.lua
--  Frame: north/east metres from the host's home (site-fixed anchor).
--
--  KSRC_RUN 0 hold (where the kangaroo is), 1 case (spec.json's legs),
--  2 live (KSRC_MODE 0 point 1 straight 2 circle 3 rectangle; KSRC_PACE
--  0 constant 1 elastic 2 stopstart; KSRC_SPD, KSRC_HDG; any change rebuilds
--  from where the kangaroo is). KSRC_ENABLE 0 stops sending. No KSRC_
--  parameter is a flight limit (SR-004).
--
--  Host scripts directory: kangaroo_source.lua, spec.json (the aircraft's),
--  modules/ sitl_spec, harness_geom, harness_kangaroo, harness_segments,
--  harness_zone (only with a fence), MAVLink/mavlink_msgs and
--  MAVLink/mavlink_msg_FOLLOW_TARGET (libraries/AP_Scripting/modules/MAVLink).

-- requires
local mavlink_msgs = require("MAVLink/mavlink_msgs")
local spec_mod = require("sitl_spec")
local segs = require("harness_segments")
local kang = require("harness_kangaroo")
-- same spec.json as the aircraft
local cfg = assert(spec_mod.load())
-- taking configuration of the floor offset 
local PERIOD_MS = math.floor((cfg.dt_s or 0.1) * 1000 + 0.5)
-- projecting the fence
local zone_mod = cfg.fence and require("harness_zone") or nil

local MAV_SEVERITY = { ERROR = 3, WARNING = 4, INFO = 6 }
-- hold and live legs never reach their end
local LEG_S = 36000.0        

-----------------------------------------------------------------------------
-- 1. Parameters
----------------------------------------------------------------------------

-- KSRC_ parameter table (own key), KSRC_RUN, KSRC_CHAN, KSRC_MODE ... as KDEM_
-- add in the parameter table
-- (key 142: hardware_val.lua uses 141, on the aircraft)
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
-- MAVLink channel of the link to the aircraft
local KSRC_CHAN   = bind("CHAN",   2, 0)  
-- 0 hold, 1 case, 2 live
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
local RUN_NAMES = { [0] = "hold", "case", "live" }

-----------------------------------------------------------------------------
-- 1.1  New KSRC_ parameters (next free indices in the KSRC_ table, size 16)
-----------------------------------------------------------------------------
    KSRC_OUT   (idx 15, default 1)  
    -- bit 0: on-board bus (KBUS_)
    --  bit 1: FOLLOW_TARGET on KSRC_CHAN (TASK-065)
    -- 1 = bus only (Follow-03), 2 = remote only,
    -- 3 = both
    KSRC_ADSB  (idx 16, default 1)  0 off, 1 broadcast ADSB_VEHICLE every 200 ms

-----------------------------------------------------------------------------
-- 1. 2. The bus: its own parameter table
-----------------------------------------------------------------------------
    KBUS_ table, key 143 (hardware_val 141, KSRC_ 142), 8 entries:
      KBUS_SEQ   sequence: odd while writing, even when stable
      KBUS_T_S   sample time, s since boot (float32: about 0.25 ms resolution
                 after an hour, enough for a 100 ms tick)
      KBUS_N_M   kangaroo north of SITE, m      -- metres, NOT degrees:
      KBUS_E_M   kangaroo east of SITE, m       -- float32 degrees were 0.4 m N /
                                                --  1.4 m E at the site (Follow-03);
                                                --  metres are < 0.1 mm at 1 km
      KBUS_VN    velocity north, m/s
      KBUS_VE    velocity east, m/s
      KBUS_RUN   KSRC_RUN in force (0 hold, 1 case, 2 live)
      KBUS_REB   rebuild count (a jump in it marks a mode change or fence turn)

    local bus_seq = 0
    function publish_bus(t_s, n, e, vn, ve)
        bus_seq = bus_seq + 1
        -- odd: writing
        KBUS_SEQ:set(2 * bus_seq - 1)          
        KBUS_T_S:set(t_s)
        -- in this script's frame: from SITE
        KBUS_N_M:set(n);  KBUS_E_M:set(e)      
        KBUS_VN:set(vn);  KBUS_VE:set(ve)
        KBUS_RUN:set(running or 0)
        -- count kept in rebuild()
        KBUS_REB:set(rebuild_count)        
         -- even: stable    
        KBUS_SEQ:set(2 * bus_seq)             
    end
    -- use :set(), never :set_and_save(): nothing is written to storage


----------------------------------------------------------------------------
-- 2. ADS-B for the ground station 
-----------------------------------------------------------------------------
    local adsb = require("sitl_adsb")
    local ADSB_PERIOD_MS = 200
    local last_adsb_ms = 0
    function broadcast(now_ms, loc, vn, ve)
        if KSRC_ADSB:get() == 0 then return end
        if now_ms - last_adsb_ms < ADSB_PERIOD_MS then return end
        last_adsb_ms = now_ms
        -- every channel; missing ones dropped
        adsb.send(loc, vn, ve)                 
    end

----------------------------------------------------------------------------
-- 2. Fence gemoetry
-----------------------------------------------------------------------------

    -- Site reference (section 13a): the fence's vertex centroid from
    --     spec.json. Same point every sortie; never home, never the aircraft.
    local function site_reference(cfg, alt_cm)
        local fence = cfg.fence
        if fence == nil or type(fence.vertices_latlng) ~= "table"
                or #fence.vertices_latlng < 3 then
            return nil, "spec.json has no fence: no site reference"
        end
        -- addiing site reference
        local v = fence.vertices_latlng
        local lat, lng = 0.0, 0.0
        for i = 1, #v do
            lat = lat + v[i][1]
            lng = lng + v[i][2]
        end
        local site = Location()
        site:lat(math.floor(lat / #v * 1e7 + 0.5))
        site:lng(math.floor(lng / #v * 1e7 + 0.5))
        site:alt(alt_cm)                  -- ground AMSL, cm (display and FOLLOW_TARGET only)
        return site
    end

    -- [Deconflict] New KSRC_ parameters (the table already has 16 slots; 15 and 16 free)
    local KSRC_OUT  = bind("OUT",  15, 1)  -- bit 0: on-board bus, bit 1: FOLLOW_TARGET
    local KSRC_ADSB = bind("ADSB", 16, 1)  -- 1: ADSB_VEHICLE every 200 ms (display only)

    -- [Deconflict - if this is simpler] The on-board bus: its own table, written only by this script
    -- hardware_val 141, KSRC_ 142
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
        -- sample time, s since boot
        T_S = bus_bind("T_S", 2),   
        -- kangaroo north of SITE, m
        N_M = bus_bind("N_M", 3),   
        -- kangaroo east of SITE, m
        E_M = bus_bind("E_M", 4),  
        -- m/s 
        VN  = bus_bind("VN",  5),
        VE  = bus_bind("VE",  6), 
        -- KSRC_RUN in force
        RUN = bus_bind("RUN", 7),  
        -- rebuild count 
        REB = bus_bind("REB", 8),
    }
    local bus_seq = 0
    -- publish bus
    local function publish_bus(t_s, n, e, vn, ve, run, rebuilds)
        bus_seq = bus_seq + 1
        KBUS.SEQ:set(2 * bus_seq - 1)
        KBUS.T_S:set(t_s)
        KBUS.N_M:set(n)
        KBUS.E_M:set(e)
        KBUS.VN:set(vn)
        KBUS.VE:set(ve)
        KBUS.RUN:set(run)
        KBUS.REB:set(rebuilds)
        -- :set, never :set_and_save
        KBUS.SEQ:set(2 * bus_seq)         
    end

----------------------------------------------------------------------------
-- 2. ADBS for ground station
-----------------------------------------------------------------------------
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
        adsb.send(loc, vn, ve)
    end

----------------------------------------------------------------------------
-- 3. Geometry runs
-----------------------------------------------------------------------------
    -- The geometry suite, shared-start cycle (sections 12 and 13). The spec.json carries a separate config
    local SUITE_RUNS = 3
    -- add point
    local SUITE_PACES = { "constant", "elastic", "stopstart" }
    local SUITE_GEOMETRIES = { "straight", "circle", "rectangle" }
    local SUITE_PLAN_DEFAULT = {
        -- from SITE
        start_n_m = 90.0, start_e_m = 10.0,   
        -- circle and rectangle
        closed_hdg = 180.0,         
        -- out; back is +180          
        straight_hdg = 200.0,              
        -- to 10 m short of the 70 m limit   
        straight_m = 194.4,                 
        -- point leg between runs  
        rest_s = 10.0,                        
    }

    -- time for a monotone closed form distance to reach d (bisection)
    local function time_for_distance(dist_fn, d)
        local lo, hi = 0.0, 1.0
        while dist_fn(hi) < d do
            hi = hi * 2.0
        end
        for _ = 1, 60 do
            local mid = 0.5 * (lo + hi)
            if dist_fn(mid) < d then lo = mid else hi = mid end
        end
        return hi
    end

----------------------------------------------------------------------------
-- 4. Speed variations
-----------------------------------------------------------------------------
    local function paced_leg(base, pace, hdg, V, dist_m)
        if pace == "constant" then
            return { duration_s = dist_m / V, mode = base, heading_deg = hdg,
                     speed_ms = V }
        elseif pace == "elastic" then
            local slow = V * kang.ELASTIC_SLOW_FACTOR
            local dur = time_for_distance(function(t)
                return kang.elastic_distance(t, slow, V, kang.ELASTIC_HOLD_S, kang.ELASTIC_RAMP_S)
            end, dist_m)
            return { duration_s = dur, mode = "elastic", heading_deg = hdg,
                     speed_ms = V, elastic_base = base }
        end
        local p = assert(kang.stopstart_profile(nil))
        local dur = time_for_distance(function(t)
            return kang.stopstart_distance(t, V, p)
        end, dist_m)
        return { duration_s = dur, mode = "stopstart", heading_deg = hdg,
                 speed_ms = V, elastic_base = base, pace = {} }
    end


 ----------------------------------------------------------------------------
-- 4. Combining geometries
-----------------------------------------------------------------------------
    -- one geometry at one pace: three runs, ending where it started
    local function geometry_run_legs(geometry, pace, V, geom, plan)
        -- <add point per the plan>
        if geometry == "straight" then
            local legs = {}
            for i = 1, 2 * SUITE_RUNS do      -- out, back, out, back, out, back
                local hdg = (plan.straight_hdg + ((i % 2 == 0) and 180.0 or 0.0)) % 360.0
                legs[#legs + 1] = paced_leg("straight", pace, hdg, V, plan.straight_m)
            end
            return legs
        end
        local per
        if geometry == "circle" then
            per = 2.0 * math.pi * geom.radius_m
        else
            per = 2.0 * (geom.length_m + geom.width_m)
        end
        return { paced_leg(geometry, pace, plan.closed_hdg, V, SUITE_RUNS * per) }
    end

----------------------------------------------------------------------------
-- 5. Full package
-----------------------------------------------------------------------------
    -- the nine runs, with a rest between them
    local function suite_legs(V, geom, plan)
        local legs = {}
        for _, geometry in ipairs(SUITE_GEOMETRIES) do
            for _, pace in ipairs(SUITE_PACES) do
                for _, leg in ipairs(geometry_run_legs(geometry, pace, V, geom, plan)) do
                    legs[#legs + 1] = leg
                end
                if plan.rest_s > 0 then
                    legs[#legs + 1] = { duration_s = plan.rest_s, mode = "point",
                                        heading_deg = 0.0, speed_ms = 0.0 }
                end
            end
        end
        return legs
    end


----------------------------------------------------------------------------
-- 6. Running the suite
-----------------------------------------------------------------------------

    -- [F] KSRC_RUN 5: the geometry suite from the shared start. The kangaroo
    --     is placed at the start once (logged), then the nine runs chain;
    --     each ends back at the start. In update(), after the live block:
    
        if run == 5 and running ~= 5 then
            running = 5
            local plan = cfg.suite or SUITE_PLAN_DEFAULT
            kn, ke = plan.start_n_m, plan.start_e_m          -- from SITE
            if not rebuild(t, suite_legs(KSRC_SPD:get(), settings().geometry, plan),
                           settings().geometry, 5) then
                return update, 1000
            end
            gcs:send_text(MAV_SEVERITY.INFO, "KSRC: suite")
        end
        -- and RUN_NAMES gains [5] = "suite"; the fence turn stays off for
        -- case (1) and suite (5): their legs are fence-checked in Python

----------------------------------------------------------------------------
-- 7. Adjusting outputs for site reference offset
-----------------------------------------------------------------------------
    -- the outputs, replacing the single FOLLOW_TARGET send at the end of
    --     update(); `anchor` is now the SITE reference ([A]), set once:
    --         anchor = assert(site_reference(cfg, ahrs:get_home():alt()))
    
        if KSRC_ENABLE:get() <= 0 then return update, PERIOD_MS end
        local now_ms = millis():toint()
        local loc = anchor:copy(); loc:offset(n, e)
        local out = math.floor(KSRC_OUT:get() + 0.5)
        local sent = false
        if (out & 1) ~= 0 then
            publish_bus(now_ms * 0.001, n, e, vn, ve, running or 0, rebuild_count)
        end
        if (out & 2) ~= 0 then
            (the existing FOLLOW_TARGET msg and mavlink:send_chan, unchanged;
             sent = its result)
        end
        broadcast(now_ms, loc, vn, ve)
        logger:write('HKSR', 't,Run,N,E,VN,VE,Sent', 'fffffff',
                     t, running and 1 or 0, n, e, vn, ve, sent and 1 or 0)
        return update, PERIOD_MS
    
        (rebuild_count: add `rebuild_count = rebuild_count + 1` in rebuild())



    
    -- at anchor(): the SITE reference (the same function as [A], from the
    -- same spec.json) and its offset to the engagement origin, once
    --     site = assert(site_reference(cfg, origin:alt()))
    --     site_to_origin = site:get_distance_NE(origin)
    -- each tick, in target_at(t) for HVAL_TGT 3:
    --     local n, e, vn, ve, t_s, fresh_or_why = read_bus(millis():toint() * 0.001)
    --     if n == nil then return nil, fresh_or_why end     -- existing fault path
    --     local age = millis():toint() * 0.001 - t_s        -- sample up to one period old
    --     local kn = n - site_to_origin:x() + vn * age
    --     local ke = e - site_to_origin:y() + ve * age
    --     return kn, ke, vn, ve
    -- and the estimator updates only on `fresh` samples, dt = sample-time
    -- interval (KANGAROO_AP_FOLLOW_FRAMEWORK section 7.2)




--[==[
=============================================================================
PSEUDO-CODE: Option B, the Follow-03 two-script layout (TASK-068, 2026-10-10)
Not code: a long comment after the return, so nothing here runs. Plan only,
for the author to review and implement.
=============================================================================

Layout on the AIRCRAFT's flight controller (for no ground host, no companion):
    APM/scripts/kangaroo_source.lua   computes the kangaroo from boot, broadcasts ADS-B, writes the bus
    APM/scripts/hardware_val.lua      reads the bus (HVAL_TGT 3), flies the arm
    APM/scripts/modules/              sitl_spec, harness_*, sitl_adsb, MAVLink/mavlink_msgs (+ FOLLOW_TARGET def)

    kangaroo_source.lua --(KBUS_ parameters, same board)--> hardware_val.lua
           +--(ADSB_VEHICLE, every MAVLink channel)--> ground station map
           +--(FOLLOW_TARGET, optional)--> a remote aircraft's AP_Follow

-- ArduPilot runs every script in ONE scripting thread, one update() at a
time, so a read cannot interleave with a write inside one update(). The
sequence counter below is kept anyway (Follow-03 had it): it still catches a
ground station writing a KBUS_ parameter, and a reader that runs before the
writer has published anything.

-----------------------------------------------------------------------------
1. New KSRC_ parameters (next free indices in the KSRC_ table, size 16)
-----------------------------------------------------------------------------
    KSRC_OUT   (idx 15, default 1)  bit 0: on-board bus (KBUS_)
                                    bit 1: FOLLOW_TARGET on KSRC_CHAN (TASK-065)
                                    1 = bus only (Follow-03), 2 = remote only,
                                    3 = both
    KSRC_ADSB  (idx 16, default 1)  0 off, 1 broadcast ADSB_VEHICLE every 200 ms

-----------------------------------------------------------------------------
2. The bus: its own parameter table, written only by this script
-----------------------------------------------------------------------------
    KBUS_ table, key 143 (hardware_val 141, KSRC_ 142), 8 entries:
      KBUS_SEQ   sequence: odd while writing, even when stable
      KBUS_T_S   sample time, s since boot (float32: about 0.25 ms resolution
                 after an hour, enough for a 100 ms tick)
      KBUS_N_M   kangaroo north of SITE, m      -- metres, NOT degrees:
      KBUS_E_M   kangaroo east of SITE, m       -- float32 degrees were 0.4 m N /
                                                --  1.4 m E at the site (Follow-03);
                                                --  metres are < 0.1 mm at 1 km
      KBUS_VN    velocity north, m/s
      KBUS_VE    velocity east, m/s
      KBUS_RUN   KSRC_RUN in force (0 hold, 1 case, 2 live)
      KBUS_REB   rebuild count (a jump in it marks a mode change or fence turn)

    local bus_seq = 0
    function publish_bus(t_s, n, e, vn, ve)
        bus_seq = bus_seq + 1
        KBUS_SEQ:set(2 * bus_seq - 1)          -- odd: writing
        KBUS_T_S:set(t_s)
        KBUS_N_M:set(n);  KBUS_E_M:set(e)      -- in this script's frame: from SITE
        KBUS_VN:set(vn);  KBUS_VE:set(ve)
        KBUS_RUN:set(running or 0)
        KBUS_REB:set(rebuild_count)            -- count kept in rebuild()
        KBUS_SEQ:set(2 * bus_seq)              -- even: stable
    end
    -- use :set(), never :set_and_save(): nothing is written to storage

-----------------------------------------------------------------------------
3. ADS-B for the ground station (display only, as Follow-03 and the demo)
-----------------------------------------------------------------------------
    local adsb = require("sitl_adsb")          -- the module the demo uses
    local ADSB_PERIOD_MS = 200
    local last_adsb_ms = 0
    function broadcast(now_ms, loc, vn, ve)
        if KSRC_ADSB:get() == 0 then return end
        if now_ms - last_adsb_ms < ADSB_PERIOD_MS then return end
        last_adsb_ms = now_ms
        adsb.send(loc, vn, ve)                 -- every channel; missing ones dropped
    end
    -- flight card: ADSB_TYPE 0 and avoidance off, so the aircraft never treats
    -- its own kangaroo as traffic (TASK-068 D5)

-----------------------------------------------------------------------------
4. Where these go in update(), keeping the existing structure
-----------------------------------------------------------------------------
    ... unchanged down to:
        local n, e, vn, ve = segs.state_at(segments, t)
        kn, ke = n, e
        (fence turn, unchanged)
    then, replacing the single FOLLOW_TARGET send at the end:
        if KSRC_ENABLE:get() <= 0 then return update, PERIOD_MS end
        local out = math.floor(KSRC_OUT:get() + 0.5)
        local loc = anchor:copy(); loc:offset(n, e)
        if (out & 1) ~= 0 then publish_bus(t, n, e, vn, ve) end
        if (out & 2) ~= 0 then
            (existing FOLLOW_TARGET msg and mavlink:send_chan, unchanged)
        end
        broadcast(millis():toint(), loc, vn, ve)
        (existing HKSR logger row, unchanged; add Out to it if wanted)
        return update, PERIOD_MS

    Start-up on the aircraft: anchor = SITE (section 13a), not home: replace
    the author's `anchor = ahrs:get_home()` with the site reference built from
    spec.json. KSRC_RUN 1 (case) starts spec.json's legs; for an experiment
    the run should start when hardware_val engages (see 6, "start").

-----------------------------------------------------------------------------
5. Reader in hardware_val.lua: HVAL_TGT 3 "on-board bus" (for reference;
   that file is where it is implemented)
-----------------------------------------------------------------------------
    -- bound lazily: kangaroo_source.lua may load after hardware_val.lua
    local kbus = { seq = Parameter(), t_s = Parameter(), n = Parameter(),
                   e = Parameter(), vn = Parameter(), ve = Parameter() }
    function kbus_ready()
        return kbus.seq:init("KBUS_SEQ") and kbus.t_s:init("KBUS_T_S") and ...
    end

    local last_seq = nil
    function read_bus(now_s)                   -- returns n, e, vn, ve, t_s | nil, why
        if not kbus_ready() then return nil, "no kangaroo bus" end
        local s1 = kbus.seq:get()
        if s1 % 2 ~= 0 then return nil, "bus being written" end
        local t_s, n, e, vn, ve = kbus.t_s:get(), kbus.n:get(), kbus.e:get(),
                                  kbus.vn:get(), kbus.ve:get()
        if kbus.seq:get() ~= s1 then return nil, "torn read" end
        if now_s - t_s > HVAL_TGT_TMO:get() then return nil, "bus stale" end
        local fresh = (s1 ~= last_seq); last_seq = s1
        return n, e, vn, ve, t_s, fresh
    end

    -- frame: the bus is north/east of the SITE reference (section 13a), the
    -- same fixed point on every sortie. hardware_val's own working frame is
    -- anchored at the aircraft at engagement (only a choice of origin for the
    -- arithmetic: the kangaroo's ground position does not depend on it).
    -- Once, at anchor():
    --     site_to_origin = site:get_distance_NE(origin)
    -- then each tick:
    --     kn = bus_n - site_to_origin:x();  ke = bus_e - site_to_origin:y()
    -- and project the sample to this tick's time (scripts are scheduled
    -- separately, so the sample is up to one period old):
    --     age = now_s - t_s;  kn = kn + vn * age;  ke = ke + ve * age
    -- estimator: update only when `fresh`, with dt = the interval between
    -- sample times (as designed for HVAL_TGT 2, KANGAROO_AP_FOLLOW_FRAMEWORK
    -- section 7.2)

    -- target_at(t) for HVAL_TGT 3: read_bus(); nil -> the existing fault path
    -- (handle_fault("target stale"), latch, HVAL_FAIL_MODE); a stale or missing
    -- bus also refuses engagement through can_engage()'s target_fresh
    -- fence: hardware_val keeps its own fence check on the received position;
    -- the turn itself happens in kangaroo_source (live mode) or is already in
    -- the case legs (legs_flown)

-----------------------------------------------------------------------------
6. Decisions to settle before implementing (TASK-068)
-----------------------------------------------------------------------------
    start   - case mode: who starts the legs at engagement? Proposed: the
              reader writes nothing (one writer per bus); the operator sets
              KSRC_RUN 1 at the same call as raising the switch, as on the
              ground host, and the log records both times.
    rate    - both scripts at 100 ms; publish every tick, ADS-B every 200 ms.
    stale   - HVAL_TGT_TMO 0.5 s for the bus (Follow-03 used 1.0 s).
    budget  - two scripts on one board: measure heap and instructions with
              both loaded (hardware_val alone needed > 692 kB in SITL;
              Follow-03 flew both its scripts in 350 kB).
    parity  - the guidance then sees a time-shifted kangaroo, so parity with
              the Python becomes replay of the logged bus samples (HKSR on
              this side, HREC/HTGT on hardware_val's).

-----------------------------------------------------------------------------
7. Tests to write with it
-----------------------------------------------------------------------------
    - writer: KBUS_ values equal the kangaroo each tick; SEQ even after a
      tick, odd only inside publish_bus; KSRC_OUT bits select bus / remote.
    - ADS-B: sent every 200 ms with KSRC_ADSB 1, never with 0; payload decodes
      (pymavlink ADSB_VEHICLE) to the kangaroo.
    - reader: stale, torn and unchanged samples refused; site-to-anchor
      conversion; projection to the tick; fault after HVAL_TGT_TMO.
    - both scripts under stubs together, in either load order, against the
      Python run (replay of the bus samples).
    - SITL: both scripts on one aircraft instance; heap and instruction count.

-----------------------------------------------------------------------------
8. What the Follow-03 kangaroo actually did (data/Sam_follow-03.bin)
-----------------------------------------------------------------------------
    - ONE mode for the whole flight: kangaroo_MAV.lua's "mission" mode
      ("KANG: virtual target initialised (mission)" at 11 s), at a constant
      10 m/s, from 11 s to 706 s. It did NOT cycle modes.
    - Mission mode walked the AIRCRAFT's own loaded mission, re-read live
      (mission:get_item, item:x() lat / item:y() lng), as straight legs
      between waypoints, looping back to the first (the mission ended in a
      DO_JUMP). Four waypoints, a quadrilateral, relative to the EKF origin:
          WP1 446 N  61 E   WP2 425 N 338 E   WP3 164 N 318 E   WP4 186 N  50 E
    - Window: about 280 m north-south by 290 m east-west, from 165 to 446 m
      north and 46 to 337 m east of the origin; at most 543 m from it.
      One lap about 1,070 m, about 107 s at 10 m/s. (The track jumps from
      near WP3 back to WP1 at about 70 s, which looks like the live mission
      re-read restarting the loop; inferred from the positions, not logged.)
    - (No site coordinates here: offsets only, AGENTS.md.)

-----------------------------------------------------------------------------
9. Mission loop: Follow-03's kangaroo, on the ported code (KSRC_RUN 3)
-----------------------------------------------------------------------------
    -- the kangaroo follows the loaded mission's NAV waypoints in a loop, as
    -- legs harness_segments can evaluate (straight legs, KSRC_SPD), so the
    -- window and lap are Follow-03's but the motion is the gated kangaroo
    function mission_legs(speed_ms, pace)
        local pts = {}
        for i = 1, mission:num_commands() - 1 do          -- 0 is home (skipped)
            local item = mission:get_item(i)
            if item and item:command() == 16 then          -- NAV_WAYPOINT only
                local wp = anchor:copy(); wp:lat(item:x()); wp:lng(item:y())
                local d = anchor:get_distance_NE(wp)       -- metres from SITE
                pts[#pts + 1] = { n = d:x(), e = d:y() }
            end
        end
        if #pts < 2 then return nil, "mission has fewer than 2 waypoints" end
        local legs = {}
        for i = 1, #pts do                                  -- closed loop
            local a, b = pts[i], pts[i % #pts + 1]
            local dn, de = b.n - a.n, b.e - a.e
            local hdg = math.deg(math.atan(de, dn)) % 360
            legs[#legs + 1] = { duration_s = math.sqrt(dn*dn + de*de) / speed_ms,
                                mode = (pace == 2) and "stopstart" or "straight",
                                heading_deg = hdg, speed_ms = speed_ms,
                                pace = (pace == 2) and settings().pace_profile or nil }
        end
        return legs, pts[1]
    end
    -- on KSRC_RUN 3: travel to WP1 first is not modelled; the kangaroo jumps
    -- there once (logged), then rebuild(t, legs) starting at pts[1]; when the
    -- last leg ends, rebuild again from where it is (it is back at WP1), so
    -- it loops for ever. Re-read the mission only when KSRC_RUN goes to 3
    -- (not live, unlike Follow-03), so the loop never restarts mid-lap.
    -- A stopstart pace on the legs gives a kangaroo that stops partway along
    -- each side; elastic is not offered here (its slow phase starts each leg).

-----------------------------------------------------------------------------
10. Mode cycle: step through several modes (KSRC_RUN 4)
-----------------------------------------------------------------------------
    -- Follow-03 did not do this; it is the requested addition. One cycle is a
    -- list of (mode, pace, seconds); after the last entry it starts again.
    -- Each change rebuilds from where the kangaroo is (position continuous,
    -- only velocity steps), exactly as a live KSRC_* change does.
    --
    -- new parameters (the KSRC_ table grows from 16 to 20 entries):
    --   KSRC_CYC     (idx 17)  bitmask of modes in the cycle, in this order:
    --                          1 point, 2 straight, 4 circle, 8 rectangle,
    --                          16 straight stopstart, 32 circle elastic,
    --                          64 mission loop (section 9)
    --                          default 2 + 4 + 8 + 16 + 1 = 31
    --   KSRC_CYC_S   (idx 18)  seconds in each mode (default 40)
    --   KSRC_CYC_HDG (idx 19)  heading change between straight entries, deg
    --                          (default 90, so straights do not leave the box)
    --   KSRC_CYC_IDX (idx 20)  read-only: the cycle entry in force (logged)
    --
    -- or, for an experiment, the cycle comes from spec.json ("cycle": a leg
    -- list, repeated), so it is fixed and replayable in the Python
    local CYCLE_TABLE = {
        [1]  = { mode = 0, pace = 0 },   -- point
        [2]  = { mode = 1, pace = 0 },   -- straight
        [4]  = { mode = 2, pace = 0 },   -- circle
        [8]  = { mode = 3, pace = 0 },   -- rectangle
        [16] = { mode = 1, pace = 2 },   -- straight, stopstart
        [32] = { mode = 2, pace = 1 },   -- circle, elastic
        [64] = { mission = true },       -- mission loop
    }
    local cycle, cycle_i, cycle_t0 = nil, 0, nil

    function build_cycle()
        cycle = {}
        local mask = math.floor(KSRC_CYC:get() + 0.5)
        for _, bit in ipairs({ 2, 4, 8, 16, 32, 64, 1 }) do   -- point last: a rest
            if (mask & bit) ~= 0 then cycle[#cycle + 1] = CYCLE_TABLE[bit] end
        end
        return #cycle > 0
    end

    function cycle_step(t)
        local entry_s = KSRC_CYC_S:get()
        if cycle_t0 ~= nil and (t - cycle_t0) < entry_s then return true end
        cycle_i = cycle_i % #cycle + 1
        cycle_t0 = t
        KSRC_CYC_IDX:set(cycle_i)
        local c = cycle[cycle_i]
        local s = settings()
        local new_legs
        if c.mission then
            new_legs = mission_legs(s.speed_ms, 0)
            if new_legs == nil then return cycle_step_skip(t) end   -- no mission: next entry
        else
            s.mode, s.pace = c.mode, c.pace
            -- turn the heading a little each straight entry, so successive
            -- straights cover the box instead of running out of it
            s.heading_deg = (s.heading_deg + KSRC_CYC_HDG:get() * (cycle_i - 1)) % 360
            new_legs = live_legs(s)
        end
        -- legs long enough for the entry; the fence turn (section 4, live
        -- mode only, extended to modes 3 and 4) still applies
        for _, leg in ipairs(new_legs) do leg.duration_s = math.max(leg.duration_s, entry_s) end
        gcs:send_text(MAV_SEVERITY.INFO, string.format("KSRC: cycle %d/%d", cycle_i, #cycle))
        return rebuild(t, new_legs, s.geometry, 4)          -- why 4: cycle step
    end

    -- in update(), beside the case and live blocks:
    --   if run == 3 and running ~= 3 then running = 3; (mission loop, section 9) end
    --   if run == 4 then
    --       if running ~= 4 then running = 4; build_cycle(); cycle_i = 0; cycle_t0 = nil end
    --       if not cycle_step(t) then return update, 1000 end
    --   end
    --   and the fence turn condition becomes: running == 2 or running == 3
    --   or running == 4
    -- HKSB already logs every rebuild (Why 4 marks a cycle step); add the
    -- cycle index to HKSR if wanted

-----------------------------------------------------------------------------
11. Cycle decisions (TASK-068)
-----------------------------------------------------------------------------
    - which modes, in which order, and how long each (default above: straight,
      circle, rectangle, straight stopstart, point; 40 s each, about 200 s a
      cycle, roughly two Follow-03 laps);
    - whether experiment runs use the parameter cycle (live, flexible) or a
      cycle in spec.json (fixed, and replayable in the Python harness);
    - the speed for the mission loop (Follow-03 used 10 m/s) and whether a
      mission loop should honour a fence that the mission itself crosses;
    - the window: a 60 m circle and 140 x 70 m rectangle (ADR-012 defaults)
      fit easily in Follow-03's 280 x 290 m window; straights at 6.25 m/s
      cover 250 m in 40 s, so a 40 s straight can reach the fence and turn.

-----------------------------------------------------------------------------
12. Geometry suite: each geometry, two full runs, at each of the three paces
-----------------------------------------------------------------------------
    Requested 2026-10-10. Order (9 runs):

        straight  x { constant, elastic, stopstart }
        circle    x { constant, elastic, stopstart }
        rectangle x { constant, elastic, stopstart }

    "Three full runs of the geometry":
        circle     three laps        3 x 2 pi r          (r 60 m:   1,131 m)
        rectangle  three laps        3 x 2 (L + W)       (140 x 70: 1,260 m)
        straight   three out-and-back runs of length L   (L 140 m:    840 m)
                   (straight has no lap; out and back keeps it in the box)

    The three paces are the ones already implemented (harness_kangaroo):
        constant   the base mode at KSRC_SPD
        elastic    slow 0.3 x V, 8 s holds, 4 s ramps, starts slow  (TASK-030)
        stopstart  starts at V, slows to 0.1 x V, holds, restarts   (TASK-064)

    Each run lasts until the kangaroo has covered the run's distance, so an
    elastic or stopstart run takes longer than a constant one. Times (Python,
    kangaroo.py distance functions, default geometry and profiles):

        V 6.25 m/s       constant   elastic   stopstart
          straight        134 s      209 s      240 s
          circle          181 s      280 s      321 s
          rectangle       202 s      310 s      365 s    total 2,242 s, 37 min

        V 12.5 m/s       constant   elastic   stopstart
          straight         67 s      108 s      114 s
          circle           90 s      139 s      161 s
          rectangle       101 s      158 s      183 s    total 1,121 s, 19 min

    -> longer than one sortie at 6.25 m/s: split it (one geometry per
       sortie, about 10 to 15 min each) or fly it at 12.5 m/s.

    PREFERRED: build the suite in Python and fly it in case mode (KSRC_RUN 1).
    The suite is then ordinary spec.json legs, fixed, replayable tick for tick
    in the Python harness, fence-checked before flight, and needs no new Lua:

        # Python, beside kangaroo.composite_legs (pseudo-code)
        def suite_legs(V, geometry, runs=3, paces=("constant", "elastic", "stopstart"),
                       rest_s=10.0):
            legs = []
            for base in ("straight", "circle", "rectangle"):
                dist = run_distance(base, geometry) * runs
                for pace in paces:
                    if base == "straight":
                        # out and back, `runs` times: 2 x runs legs of length L.
                        # A pace restarts at every leg start (elastic slow,
                        # stopstart fast), so each straight leg gets the pace
                        # over its own length.
                        for i in range(2 * runs):
                            hdg = suite_heading + (180.0 if i % 2 else 0.0)
                            legs.append(paced_leg(base, pace, hdg, V,
                                                  distance=geometry["length_m"]))
                    else:
                        legs.append(paced_leg(base, pace, suite_heading, V,
                                              distance=dist))
                    legs.append(point_leg(rest_s))        # a short rest between runs
            return legs

        def paced_leg(base, pace, hdg, V, distance):
            if pace == "constant":
                dur = distance / V
                return leg(dur, base, hdg, V)
            if pace == "elastic":
                dur = time_for_distance(lambda t: elastic_distance(t, 0.3 * V, V), distance)
                return leg(dur, "elastic", hdg, V, elastic_base=base)
            dur = time_for_distance(lambda t: stopstart_distance(t, V), distance)
            return leg(dur, "stopstart", hdg, V, elastic_base=base, pace={})

        # time_for_distance: bisection on the closed-form distance (both are
        # monotone in t); kangaroo.elastic_time_for_distance already exists for
        # elastic. Then: schedule_fits(legs, start, zone=PolygonZone(fence))
        # refuses a suite that leaves the fence, before anything is staged.

    ALTERNATIVE (on the vehicle, KSRC_RUN 5): the same loop in Lua, run by
    kangaroo_source.lua when the run starts, using harness_kangaroo's
    elastic_distance / stopstart_distance and a bisection for each duration:

        function time_for_distance(dist_fn, d)
            local lo, hi = 0.0, 1.0
            while dist_fn(hi) < d do hi = hi * 2 end
            for _ = 1, 60 do
                local mid = 0.5 * (lo + hi)
                if dist_fn(mid) < d then lo = mid else hi = mid end
            end
            return hi
        end
        function suite_legs(V, geom)
            (as the Python above, building named legs:
             { duration_s, mode, heading_deg, speed_ms, elastic_base, pace })
        end
        -- on KSRC_RUN 5: rebuild(t, suite_legs(KSRC_SPD:get(), settings().geometry), ...)
        -- log the run index (HKSB Why 5 with the leg number) so each of the 9
        -- runs can be cut out of the log afterwards
        -- costs: about 60 distance evaluations per leg at start-up, once; no
        -- fence check before flight (the live fence turn would bend the runs)

    Notes:
    - Legs chain from where the previous one ended, so the circle and
      rectangle are placed from the kangaroo's position at that time, not at
      a chosen centre. The Python route lets the fence fit be checked and the
      start chosen so all nine runs stay inside it; the vehicle route cannot.
    - A paced run may end part-way through a slow phase (it ends on distance,
      not on a whole cycle); the next run starts its own pace, so speed can
      step at a run boundary. The rest leg between runs makes that a stop.
    - This suite and the mode cycle (section 10) answer different needs: the
      suite is a fixed experiment (one run per geometry and pace, comparable
      across arms); the cycle is a live demonstration.

-----------------------------------------------------------------------------
13. The runs placed in the test fence, and the experiment set-up (2026-10-10)
-----------------------------------------------------------------------------
    Fence: sitl-runs/SV2_fence_line2.txt (4 vertices; about 300 m a side,
    93,000 m^2; deepest point 138 m from the nearest wall). Rule (ADR-012):
    the kangaroo stays >= 70 m (the ring R) inside every wall. Overlay:
    docs/figures/physical_validation_runs.png. Offsets below are metres from
    the fence's vertex centroid (no site coordinates in the repository).

    Placement of each geometry (best start found by a 10 m / 15 deg search,
    the exact kangaroo path, 3 runs; same path at every pace):

        geometry    start N, E (m)   heading   clearance to the 70 m limit
        straight    -60, -30          30 deg    34 m
        circle      +70,   0           0 deg     8 m
        rectangle   +70, +30         180 deg    24 m
        all three from one start:  +90, +10, 180 deg, 1 m

    ADOPTED (author, 2026-10-10): the SHARED-START CYCLE (overlay panel b).
    Every run starts and ends at +90 N, +10 E, so the nine runs chain with no
    repositioning and the kangaroo is back at the start between runs:
        circle     heading 180, 3 laps                    (1,131 m)
        rectangle  heading 180, 3 laps                    (1,260 m)
        straight   heading 200 out, 180 deg back, 3 times; each leg runs to
                   10 m short of the 70 m limit: 194.4 m  (1,166 m)
                   (200 deg is the longest heading from the start: 204 m to
                   the limit; 180 deg would give 186 m)
    Whole cycle checked on the exact path at 0.1 s: fits, 0.7 m spare (set by
    the circle; the straight keeps its 10 m), and ends 0.00 m from the start.
    The straight replaces section 12's 3 x 140 m out-and-back.

        cycle time per arm   constant  elastic  stopstart   total
          6.25 m/s straight    187       286       327
                   circle      181       280       321
                   rectangle   202       310       365      2,459 s, 41 min
          12.5 m/s straight     93       143       164
                   circle       90       139       161
                   rectangle   101       158       183      1,232 s, 20.5 min

    Straight legs on the vehicle (pseudo):
        function straight_legs(V, pace, hdg_out, length_m)
            local legs = {}
            for i = 1, 6 do                                  -- 3 out-and-back
                local hdg = (hdg_out + ((i % 2 == 0) and 180 or 0)) % 360
                legs[i] = paced_leg("straight", pace, hdg, V, length_m)
            end
            return legs                                      -- ends at the start
        end
        -- a paced leg restarts its pace at each turn (elastic slow, stopstart
        -- fast), so each 194 m leg carries the whole pace over its length
        -- the 10 m margin covers the kangaroo moving one tick past the planned
        -- turn point and any rounding of the leg duration
        -- 0.7 m on the circle is thin: the fence turn (live) or the harness
        -- zone check (Python) is the backstop; hardware_val.lua faults if the
        -- kangaroo actually leaves the fence

    The mode cycle's 40 s straights do NOT fit (250 m at 6.25 m/s, 90 m over);
    the Follow-03 mission loop does NOT fit (its waypoints are on this fence).

    13a. SITE-FIXED FRAME (author, 2026-10-10): the kangaroo is placed in
    the Spring Valley 2 site frame, NOT relative to the aircraft and NOT
    relative to home (home is wherever the aircraft is armed, so it moves
    between sorties; the cycle has 0.7 m spare, so it cannot absorb that).

      SITE = the vertex centroid of the test fence (SV2_fence_line2), read
      from spec.json (cfg.fence.vertices_latlng, already carried for ADR-012):

        function site_reference(cfg)
            local v = cfg.fence.vertices_latlng      -- {{lat_deg, lng_deg}, ...}
            local lat, lng = 0.0, 0.0
            for i = 1, #v do lat = lat + v[i][1]; lng = lng + v[i][2] end
            local site = Location()
            site:lat(math.floor(lat / #v * 1e7 + 0.5))
            site:lng(math.floor(lng / #v * 1e7 + 0.5))
            site:alt(<ground altitude AMSL, cm, from spec.json or home>)
            return site
        end
        -- no fence in spec.json -> refuse to start (no site, no kangaroo)
        -- the offsets in this section (shared start +90 N, +10 E; headings)
        -- are metres from SITE, so spec.json's target_n_m/e_m = 90, 10

      - kangaroo_source.lua: anchor = site_reference(cfg) instead of
        ahrs:get_home(); KBUS_N_M/E_M are from SITE; ADS-B and FOLLOW_TARGET
        positions are site + (n, e). The kangaroo is on the same ground track
        every sortie, whoever arms where.
      - hardware_val.lua (HVAL_TGT 3) converts SITE -> its engagement origin
        (section 5); the arm flies the same ground geometry wherever the pilot
        engages. Engagement is refused if the aircraft is not inside the
        fence (as now).
      - hardware_val.lua HVAL_TGT 1 as it stands places the kangaroo at
        cfg.target_n_m/e_m from the AIRCRAFT at engagement: NOT VIABLE for
        this experiment (the author, 2026-10-10). If HVAL_TGT 1 is kept as a
        single-script fallback, it needs the same change: build the legs from
        site_reference(cfg) + (target_n_m, target_e_m) and convert to the
        engagement origin, instead of placing them from the aircraft.

    Experiment matrix (each arm flies the same nine runs, same placements):

        for arm in { 0H, AH, FH }:
            for geometry in { straight, circle, rectangle }:      -- shared start
                for pace in { constant, elastic, stopstart }:
                    case run: 3 runs of the geometry at the pace (section 12)
                    shadow first (HVAL_OUT 0), then active (HVAL_OUT 1)

        arm   registry name             estimate  lookahead_steps  other          HVAL_ARM
        0H    dubins_target_orbit_hyst  no        0                margin 10 m    0 (_hyst spec)
        AH    adaptive_db_circle_hyst   yes       25 (2.5 s)       replan 1, hold plan
                                                                    NOT PORTED TO LUA (TASK-066 D2):
                                                                    port the held-sense adapter,
                                                                    or fly plain A (HVAL_ARM 1)
                                                                    and record the substitution
        FH    carrot_shift_cs_hyst      yes       0                af_step_ticks 8 (0.8 s,
                                                                    TASK-067 optimum; default 1)
                                                                                    2 (_hyst spec)

        common: airspeed 25 m/s, bank 60 deg (ROLL_LIMIT_DEG 60, ADR-011),
        R 70 m, carrot L 50 m, turn radius 45 m (above the 27.6 m/s TAS floor
        fault: see HARDWARE_VAL_BOOT_REVIEW third pass), dt 0.1 s.

    Kangaroo speed: ratio 0.25 (6.25 m/s) or 0.5 (12.5 m/s) of airspeed.
    Flight time per arm (9 runs, shared-start cycle, no rests): 41 min at
    6.25 m/s, 20.5 min at 12.5 m/s; three arms: about 123 or 62 min of runs,
    before transits, shadow passes and repeats. Split by geometry across
    sorties if needed: each geometry still starts and ends at the shared start. Ratio 0.5 is close to the 0.56 holdability
    limit (R/rho - 1).

    Generating it (Python, before flight; the preferred route of section 12):
        for arm in (0H, AH, FH):
            for geometry in (straight, circle, rectangle):
                spec = base_spec(arm)                  # the arm's row above
                spec.legs = suite_legs(V, geometry_only=geometry)   # 3 paces
                spec.target_n_m, target_e_m = shared_start          # from SITE
                spec.fence = SV2_fence_line2, containment 70 m
                assert schedule_fits(spec.legs, start, zone=PolygonZone(fence))
                run the Python harness on it (the comparison run)
                write spec.json for the card (schedule.spec_cfg)

-----------------------------------------------------------------------------
14. COMMENTED CODE: Option B, first draft (2026-10-10)
-----------------------------------------------------------------------------
    Real Lua, kept inside this comment until the author moves it into the
    script. [A] to [E] were run under stubs against the Python on 2026-10-10:
    the suite takes 2,459 s at 6.25 m/s and 1,232 s at 12.5 m/s (the Python
    figures), every run ends 0.0000 m from the shared start through
    harness_segments, the site reference is built from the fence (a missing
    fence is refused), and the bus writes as designed. [F] to [H] are NOT yet
    run. Where each block goes:
      [A] to [E]  after the KSRC_ parameter block, before `local anchor, ...`
      [F]         in update(), after the live-mode block
      [G]         replaces the end of update(), from `if KSRC_ENABLE:get() <= 0`
      [H]         hardware_val.lua, section 3 (target source HVAL_TGT 3)
    The kangaroo modes are the PORTED ones (harness_segments /
    harness_kangaroo), not kangaroo_MAV.lua's: see docs/Physical_validation.md.

    -- [A] Site reference (section 13a): the fence's vertex centroid from
    --     spec.json. Same point every sortie; never home, never the aircraft.
    local function site_reference(cfg, alt_cm)
        local fence = cfg.fence
        if fence == nil or type(fence.vertices_latlng) ~= "table"
                or #fence.vertices_latlng < 3 then
            return nil, "spec.json has no fence: no site reference"
        end
        local v = fence.vertices_latlng
        local lat, lng = 0.0, 0.0
        for i = 1, #v do
            lat = lat + v[i][1]
            lng = lng + v[i][2]
        end
        local site = Location()
        site:lat(math.floor(lat / #v * 1e7 + 0.5))
        site:lng(math.floor(lng / #v * 1e7 + 0.5))
        site:alt(alt_cm)                  -- ground AMSL, cm (display and FOLLOW_TARGET only)
        return site
    end

    -- [B] New KSRC_ parameters (the table already has 16 slots; 15 and 16 free)
    local KSRC_OUT  = bind("OUT",  15, 1)  -- bit 0: on-board bus, bit 1: FOLLOW_TARGET
    local KSRC_ADSB = bind("ADSB", 16, 1)  -- 1: ADSB_VEHICLE every 200 ms (display only)

    -- [C] The on-board bus: its own table, written only by this script
    local KBUS_TABLE_KEY = 143            -- hardware_val 141, KSRC_ 142
    local KBUS_PREFIX = "KBUS_"
    assert(param:add_table(KBUS_TABLE_KEY, KBUS_PREFIX, 8), "KSRC: could not add KBUS table")
    local function bus_bind(name, idx)
        assert(param:add_param(KBUS_TABLE_KEY, idx, name, 0), "KSRC: could not add KBUS_" .. name)
        local p = Parameter()
        assert(p:init(KBUS_PREFIX .. name), "KSRC: could not bind KBUS_" .. name)
        return p
    end
    local KBUS = {
        SEQ = bus_bind("SEQ", 1),   -- odd while writing, even when stable
        T_S = bus_bind("T_S", 2),   -- sample time, s since boot
        N_M = bus_bind("N_M", 3),   -- kangaroo north of SITE, m
        E_M = bus_bind("E_M", 4),   -- kangaroo east of SITE, m
        VN  = bus_bind("VN",  5),   -- m/s
        VE  = bus_bind("VE",  6),   -- m/s
        RUN = bus_bind("RUN", 7),   -- KSRC_RUN in force
        REB = bus_bind("REB", 8),   -- rebuild count
    }
    local bus_seq = 0
    local function publish_bus(t_s, n, e, vn, ve, run, rebuilds)
        bus_seq = bus_seq + 1
        KBUS.SEQ:set(2 * bus_seq - 1)
        KBUS.T_S:set(t_s)
        KBUS.N_M:set(n)
        KBUS.E_M:set(e)
        KBUS.VN:set(vn)
        KBUS.VE:set(ve)
        KBUS.RUN:set(run)
        KBUS.REB:set(rebuilds)
        KBUS.SEQ:set(2 * bus_seq)         -- :set, never :set_and_save
    end

    -- [D] ADS-B for the ground station (display only; sitl_adsb, Follow-03 framing)
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
        adsb.send(loc, vn, ve)
    end

    -- [E] The geometry suite, shared-start cycle (sections 12 and 13). The
    --     plan values are SV2_fence_line2's (re-fit for any other fence);
    --     spec.json may carry its own as cfg.suite.
    local SUITE_RUNS = 3
    local SUITE_PACES = { "constant", "elastic", "stopstart" }
    local SUITE_GEOMETRIES = { "straight", "circle", "rectangle" }
    local SUITE_PLAN_DEFAULT = {
        start_n_m = 90.0, start_e_m = 10.0,   -- from SITE
        closed_hdg = 180.0,                   -- circle and rectangle
        straight_hdg = 200.0,                 -- out; back is +180
        straight_m = 194.4,                   -- to 10 m short of the 70 m limit
        rest_s = 10.0,                        -- point leg between runs
    }

    -- time for a monotone closed-form distance to reach d (bisection)
    local function time_for_distance(dist_fn, d)
        local lo, hi = 0.0, 1.0
        while dist_fn(hi) < d do
            hi = hi * 2.0
        end
        for _ = 1, 60 do
            local mid = 0.5 * (lo + hi)
            if dist_fn(mid) < d then lo = mid else hi = mid end
        end
        return hi
    end

    local function paced_leg(base, pace, hdg, V, dist_m)
        if pace == "constant" then
            return { duration_s = dist_m / V, mode = base, heading_deg = hdg,
                     speed_ms = V }
        elseif pace == "elastic" then
            local slow = V * kang.ELASTIC_SLOW_FACTOR
            local dur = time_for_distance(function(t)
                return kang.elastic_distance(t, slow, V, kang.ELASTIC_HOLD_S,
                                             kang.ELASTIC_RAMP_S)
            end, dist_m)
            return { duration_s = dur, mode = "elastic", heading_deg = hdg,
                     speed_ms = V, elastic_base = base }
        end
        local p = assert(kang.stopstart_profile(nil))
        local dur = time_for_distance(function(t)
            return kang.stopstart_distance(t, V, p)
        end, dist_m)
        return { duration_s = dur, mode = "stopstart", heading_deg = hdg,
                 speed_ms = V, elastic_base = base, pace = {} }
    end

    -- one geometry at one pace: three runs, ending where it started
    local function geometry_run_legs(geometry, pace, V, geom, plan)
        if geometry == "straight" then
            local legs = {}
            for i = 1, 2 * SUITE_RUNS do      -- out, back, out, back, out, back
                local hdg = (plan.straight_hdg + ((i % 2 == 0) and 180.0 or 0.0)) % 360.0
                legs[#legs + 1] = paced_leg("straight", pace, hdg, V, plan.straight_m)
            end
            return legs
        end
        local per
        if geometry == "circle" then
            per = 2.0 * math.pi * geom.radius_m
        else
            per = 2.0 * (geom.length_m + geom.width_m)
        end
        return { paced_leg(geometry, pace, plan.closed_hdg, V, SUITE_RUNS * per) }
    end

    -- the nine runs, with a rest between them
    local function suite_legs(V, geom, plan)
        local legs = {}
        for _, geometry in ipairs(SUITE_GEOMETRIES) do
            for _, pace in ipairs(SUITE_PACES) do
                for _, leg in ipairs(geometry_run_legs(geometry, pace, V, geom, plan)) do
                    legs[#legs + 1] = leg
                end
                if plan.rest_s > 0 then
                    legs[#legs + 1] = { duration_s = plan.rest_s, mode = "point",
                                        heading_deg = 0.0, speed_ms = 0.0 }
                end
            end
        end
        return legs
    end

    -- [F] KSRC_RUN 5: the geometry suite from the shared start. The kangaroo
    --     is placed at the start once (logged), then the nine runs chain;
    --     each ends back at the start. In update(), after the live block:
    --
    --     if run == 5 and running ~= 5 then
    --         running = 5
    --         local plan = cfg.suite or SUITE_PLAN_DEFAULT
    --         kn, ke = plan.start_n_m, plan.start_e_m          -- from SITE
    --         if not rebuild(t, suite_legs(KSRC_SPD:get(), settings().geometry, plan),
    --                        settings().geometry, 5) then
    --             return update, 1000
    --         end
    --         gcs:send_text(MAV_SEVERITY.INFO, "KSRC: suite")
    --     end
    --     -- and RUN_NAMES gains [5] = "suite"; the fence turn stays off for
    --     -- case (1) and suite (5): their legs are fence-checked in Python

    -- [G] the outputs, replacing the single FOLLOW_TARGET send at the end of
    --     update(); `anchor` is now the SITE reference ([A]), set once:
    --         anchor = assert(site_reference(cfg, ahrs:get_home():alt()))
    --
    --     if KSRC_ENABLE:get() <= 0 then return update, PERIOD_MS end
    --     local now_ms = millis():toint()
    --     local loc = anchor:copy(); loc:offset(n, e)
    --     local out = math.floor(KSRC_OUT:get() + 0.5)
    --     local sent = false
    --     if (out & 1) ~= 0 then
    --         publish_bus(now_ms * 0.001, n, e, vn, ve, running or 0, rebuild_count)
    --     end
    --     if (out & 2) ~= 0 then
    --         (the existing FOLLOW_TARGET msg and mavlink:send_chan, unchanged;
    --          sent = its result)
    --     end
    --     broadcast(now_ms, loc, vn, ve)
    --     logger:write('HKSR', 't,Run,N,E,VN,VE,Sent', 'fffffff',
    --                  t, running and 1 or 0, n, e, vn, ve, sent and 1 or 0)
    --     return update, PERIOD_MS
    --
    --     (rebuild_count: add `rebuild_count = rebuild_count + 1` in rebuild())

    -- [H] hardware_val.lua, HVAL_TGT 3: read the bus. Bound lazily, since
    --     kangaroo_source.lua may load after hardware_val.lua.
    --
    -- local kbus = nil
    -- local function kbus_bind()
    --     if kbus ~= nil then return true end
    --     local t = {}
    --     for _, name in ipairs({ "SEQ", "T_S", "N_M", "E_M", "VN", "VE" }) do
    --         local p = Parameter()
    --         if not p:init("KBUS_" .. name) then return false end
    --         t[name] = p
    --     end
    --     kbus = t
    --     return true
    -- end
    --
    -- local last_seq = nil
    -- --- n, e (from SITE), vn, ve, t_s, fresh  |  nil, why
    -- local function read_bus(now_s)
    --     if not kbus_bind() then return nil, "no kangaroo bus (kangaroo_source.lua not loaded)" end
    --     local s1 = kbus.SEQ:get()
    --     if s1 % 2 ~= 0 then return nil, "kangaroo bus being written" end
    --     local t_s, n, e = kbus.T_S:get(), kbus.N_M:get(), kbus.E_M:get()
    --     local vn, ve = kbus.VN:get(), kbus.VE:get()
    --     if kbus.SEQ:get() ~= s1 then return nil, "kangaroo bus torn read" end
    --     if now_s - t_s > HVAL_TGT_TMO:get() then return nil, "kangaroo bus stale" end
    --     local fresh = (s1 ~= last_seq)
    --     last_seq = s1
    --     return n, e, vn, ve, t_s, fresh
    -- end
    --
    -- -- at anchor(): the SITE reference (the same function as [A], from the
    -- -- same spec.json) and its offset to the engagement origin, once
    -- --     site = assert(site_reference(cfg, origin:alt()))
    -- --     site_to_origin = site:get_distance_NE(origin)
    -- -- each tick, in target_at(t) for HVAL_TGT 3:
    -- --     local n, e, vn, ve, t_s, fresh_or_why = read_bus(millis():toint() * 0.001)
    -- --     if n == nil then return nil, fresh_or_why end     -- existing fault path
    -- --     local age = millis():toint() * 0.001 - t_s        -- sample up to one period old
    -- --     local kn = n - site_to_origin:x() + vn * age
    -- --     local ke = e - site_to_origin:y() + ve * age
    -- --     return kn, ke, vn, ve
    -- -- and the estimator updates only on `fresh` samples, dt = sample-time
    -- -- interval (KANGAROO_AP_FOLLOW_FRAMEWORK section 7.2)
=============================================================================
15. DECISIONS FROM THE OPEN ITEMS (author, 2026-10-10) AND WHAT THEY CHANGE
=============================================================================
docs/Physical_validation.md "Open items before flying this", as answered:

  * AH: ported (harness_adaptive_db.guidance_point_hyst, both sitl_arms
    copies); validated in the SITL demo from the plan config (below).
  * Python suite builder: NOT a separate tool; the plan is checked in SITL.
    The legs are still generated in Python, once, by
    Tools/autotest/kangaroo_follow/pv_plan.py from pv_plan.json, and carried
    in spec.json -- so [E] (an on-board suite builder) is SUPERSEDED: the
    aircraft plays the spec's legs, it does not build them. pv_plan.py also
    refuses a plan that leaves the fence less the orbit radius (the spec's
    own composite check), so a spec that exists has been checked.
  * Turn radius: bank 60 deg in the configuration (pv_plan.json
    aircraft.bank_limit_deg -> spec bank_limit_deg and roll_limit_deg 60;
    flight card ROLL_LIMIT_DEG 60). At 25 m/s that is a 36.8 m turn, inside
    the 45 m the arms plan with; hardware_val's floor faults above about
    27.6 m/s true airspeed.
  * Script heap: SCR_HEAP_SIZE at least 800 kB for hardware_val.lua alone;
    the two-script total is measured on the board (flight card).
  * Sortie plan: arm-major. One arm flies the whole plan (point, transit,
    nine runs) before the next arm: 0H, then FH, then AH. Active only
    (HVAL_OUT 1). One spec.json per arm: demo.py --plan --arm <id> in SITL,
    the same file hardware_val.lua reads (ADR-011), with HVAL_ARM to match
    (0H -> 0, FH -> 2, AH -> 1).
  * Geometry sizes: the defaults stand (circle 60 m, rectangle 140 x 70 m).
  * Point mode added: flown FIRST, held 120 s at 30 m N, 0 m E of the site
    reference (the fence's deepest point, 68 m spare), away from the shared
    start; then an unscored transit at the plan speed to the shared start
    (90 m N, 10 m E), then the nine runs (two laps / two out-and-backs,
    12.5 m/s), each followed by a 10 s rest. 17.3 min per arm.
  * Carrot look-ahead 25 m (Physical_validation.md common set-up).

The tested reference for [I] below is kangaroo_demo.lua's suite mode
(KDEM_MODE 5; anchor() and suite_watch()), gated against the Python schedule
in tests/unit/test_pv_plan.py. Port it here rather than re-deriving it.

-- [I] KSRC_RUN 5 from the spec's legs (replaces [E]). In the SITE frame the
--     schedule's (n, e) ARE metres from the site reference, so the bus
--     ([C]) is written with them directly and ADS-B places them with
--     site:offset(n, e).
--
-- local function plan_legs(cfg)
--     if cfg.legs == nil or #cfg.legs == 0 then
--         return nil, "spec.json has no legs: stage it with demo.py --plan --arm"
--     end
--     local out = {}
--     for i = 1, #cfg.legs do
--         local l = cfg.legs[i]
--         out[i] = { duration_s = l.duration_s, mode = l.mode,
--                    heading_deg = l.heading_deg, speed_ms = l.speed_ms,
--                    elastic_base = l.elastic_base, pace = l.pace }
--     end
--     return out
-- end
--
-- -- at start (boot, or KSRC_RUN set to 5):
-- --     site = assert(site_reference(cfg, home_alt_cm))         -- [A]
-- --     local legs = assert(plan_legs(cfg))
-- --     segments = assert(segs.make_segments(legs, cfg.target_n_m, cfg.target_e_m,
-- --                                          cfg.geometry, 0.0))
-- --     t0_ms = millis():toint(); suite_run = 0
-- -- each tick:
-- --     local n, e, vn, ve = segs.state_at(segments, (millis():toint() - t0_ms) * 0.001)
-- --     publish_bus(t_s, n, e, vn, ve, 5, 0)                    -- [C]
-- --     local loc = site:copy(); loc:offset(n, e)
-- --     broadcast(now_ms, loc, vn, ve)                          -- [D]
-- --     announce each run from cfg.suite_runs (name, t_start_s, t_end_s)
-- --     NO containment turn (it would change the checked plan); track the
-- --     least spare to the orbit radius and warn past it, as suite_watch().
--
-- Start time: the demo starts the plan when the aircraft enters GUIDED; on
-- the aircraft the kangaroo runs from boot (Option B). D-open: start the
-- plan on a pilot switch (KSRC_RUN 5 set from the ground) so the point run
-- is not spent before the aircraft is engaged. Recommended.
=============================================================================
]==]



-- old code - before 10 October






local anchor, segments, t0_ms, running = nil, nil, nil, false
-- running: the KSRC_RUN the schedule was built for (false until the first
-- run); legs, signature: what the schedule was built from; kn, ke: the
-- kangaroo last tick; stopped: a refused schedule ends the stream
local legs, signature, kn, ke, zone, stopped = nil, nil, nil, nil, nil, false

-- point leg at the spec's start until KSRC_RUN
local function hold_start(t)
    segments = segs.make_segments({ { duration_s = 1e6, mode = "point",
        heading_deg = 0.0, speed_ms = 0.0 } }, cfg.target_n_m, cfg.target_e_m,
        cfg.geometry, t)
end

-- the live settings, as the demo's KDEM_ (TASK-058, TASK-064)
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

-- the one live leg, by the demo's rules: elastic and stopstart travel over
-- the chosen base; point has no pace
local function live_legs(s)
    local name = MODE_NAMES[s.mode]
    if name == "point" then
        return { { duration_s = LEG_S, mode = "point", heading_deg = s.heading_deg,
                   speed_ms = 0.0 } }
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

-- rebuild from where the kangaroo is (position continuous, only velocity
-- steps); why: 1 run change, 2 live change, 3 fence turn. False on refusal.
local function rebuild(t, new_legs, geom, why)
    local built, reason = segs.make_segments(new_legs, kn, ke, geom, t)
    if built == nil then
        stopped = true
        gcs:send_text(MAV_SEVERITY.ERROR, "KSRC: stopped streaming: schedule refused: "
                      .. tostring(reason))
        return false
    end
    segments, legs = built, new_legs
    logger:write('HKSB', 't,Run,Spd,Hdg,Why', 'fffff', t, running or 0,
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

local function update()
    if stopped then return update, 1000 end
    if anchor == nil then
        if not ahrs:home_is_set() then return update, 1000 end
        anchor = ahrs:get_home(); t0_ms = millis(); hold_start(0.0)
        kn, ke = cfg.target_n_m, cfg.target_e_m
        if zone_mod ~= nil then
            zone = assert(zone_mod.from_latlng(anchor, cfg.fence.vertices_latlng))
        end
        gcs:send_text(MAV_SEVERITY.INFO, string.format(
            "KSRC: streaming FOLLOW_TARGET from sysid %d on chan %d",
            param:get("MAV_SYSID") or 0, KSRC_CHAN:get()))
    end
    local t = (millis() - t0_ms):tofloat() * 0.001
    local run = math.floor(KSRC_RUN:get() + 0.5)
    if RUN_NAMES[run] == nil then run = 0 end
    -- case mode
    if run == 1 and running ~= 1 then
        running = 1
        if not rebuild(t, cfg.legs, cfg.geometry, 1) then return update, 1000 end
        gcs:send_text(MAV_SEVERITY.INFO, "KSRC: case")
    end
    -- live mode: on a KSRC_* change, kangaroo_live.rebuild(t, n, e, ...)
    -- (the demo's rules carried here; kangaroo_live.lua is a follow-up, K9)
    if run == 2 then
        local s = settings()
        if running ~= 2 or signature ~= signature_of(s) then
            local why = (running == 2) and 2 or 1
            running, signature = 2, signature_of(s)
            if not rebuild(t, live_legs(s), s.geometry, why) then return update, 1000 end
            gcs:send_text(MAV_SEVERITY.INFO, string.format("KSRC: live %s %s %.2f m/s hdg %.0f",
                MODE_NAMES[s.mode], PACE_NAMES[s.pace], s.speed_ms, s.heading_deg))
        end
    end
    -- hold: KSRC_RUN 0 after a run stands the kangaroo where it is
    if run == 0 and running then
        running = false
        if not rebuild(t, { { duration_s = LEG_S, mode = "point", heading_deg = 0.0,
                              speed_ms = 0.0 } }, cfg.geometry, 1) then
            return update, 1000
        end
        gcs:send_text(MAV_SEVERITY.INFO, "KSRC: hold")
    end
    local n, e, vn, ve = segs.state_at(segments, t)
    kn, ke = n, e
    -- live mode only: turn back at the fence by the harness's rule (harness_zone)
    if zone ~= nil and running == 2 then
        local leg = legs[active_index(t)]
        local turned = zone_mod.contain_heading(zone, n, e, leg.heading_deg or 0.0,
                                                leg.speed_ms or 0.0, PERIOD_MS * 0.001,
                                                cfg.fence.containment_margin_m)
        if turned ~= nil then
            local new_leg = zone_mod.turned_leg(leg, turned)
            new_leg.duration_s = LEG_S
            if not rebuild(t, { new_leg }, settings().geometry, 3) then return update, 1000 end
        end
    end
    if KSRC_ENABLE:get() <= 0 then return update, PERIOD_MS end
    local loc = anchor:copy(); loc:offset(n, e)
    local msg = { timestamp = millis():toint(), est_capabilities = 3,
                  lat = loc:lat(), lon = loc:lng(), alt = anchor:alt() * 0.01,
                  vel = { vn, ve, 0 }, acc = { 0, 0, 0 },
                  attitude_q = { 1, 0, 0, 0 }, rates = { 0, 0, 0 },
                  position_cov = { 0, 0, 0 }, custom_state = 0 }
    local sent = mavlink:send_chan(KSRC_CHAN:get(),
                                   mavlink_msgs.encode("FOLLOW_TARGET", msg))
    logger:write('HKSR', 't,Run,N,E,VN,VE,Sent', 'fffffff',
                 t, running and 1 or 0, n, e, vn, ve, sent and 1 or 0)
    return update, PERIOD_MS
end
return update, 1000
