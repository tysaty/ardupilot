--  kangaroo_source.lua -- live streaming the kangaroo position via AP_Follow
--  created 2026-10-07
-- implmeneted from KANGAROO_AP_FOLLOW_FRAMEWORK.md


-- requireis
local mavlink_msgs = require("MAVLink/mavlink_msgs")
local spec_mod = require("sitl_spec")
local segs = require("harness_segments")
-- same spec.json as the aircraft
local cfg = assert(spec_mod.load())
local PERIOD_MS = math.floor((cfg.dt_s or 0.1) * 1000 + 0.5)

-- KSRC_ parameter table (own key), KSRC_RUN, KSRC_CHAN, KSRC_MODE ... as KDEM_
-- add in the parameter table

local anchor, segments, t0_ms, running = nil, nil, nil, false

-- point leg at the spec's start until KSRC_RUN
local function hold_start(t)
    segments = segs.make_segments({ { duration_s = 1e6, mode = "point",
        heading_deg = 0.0, speed_ms = 0.0 } }, cfg.target_n_m, cfg.target_e_m,
        cfg.geometry, t)
end

local function update()
    if anchor == nil then
        if not ahrs:home_is_set() then return update, 1000 end
        anchor = ahrs:get_home(); t0_ms = millis(); hold_start(0.0)
    end
    local t = (millis() - t0_ms):tofloat() * 0.001
    -- case mode
    if KSRC_RUN:get() == 1 and not running then          
        segments = segs.make_segments(cfg.legs, cfg.target_n_m, cfg.target_e_m,
                                      cfg.geometry, t)
        running = true
    end
    -- live mode: on a KSRC_* change, kangaroo_live.rebuild(t, n, e, ...)
    local n, e, vn, ve = segs.state_at(segments, t)
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