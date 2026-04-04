-- Kangaroo 2
-- Plane SITL helper that creates a virtual target starting near home and
-- moves it in random hop-like bursts. 
-- Control logic is controlled in control.lua

-- initial vlaues
local ALT_FRAME_ABSOLUTE = 0
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}

local PARAM_TABLE_KEY = 89
local PARAM_TABLE_PREFIX = "KANG_"

-- use the kangaroo_bus for standardised passing of messages
local kangaroo_bus = require("kangaroo_bus")

local param_helpers = require("param_helpers")
local math_helpers = require("math_helpers")

-- error handling
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 13), "could not add KANG_ parameter table")

-- parameter tables
--[[
  // @Param: KANG_ENABLE
  // @DisplayName: Kangaroo enable
  // @Description: Enable the virtual hopping target for Plane Guided mode
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_ENABLE = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "ENABLE", 1, 1)

--[[
  // @Param: KANG_ALT_M
  // @DisplayName: Kangaroo altitude
  // @Description: Altitude of the virtual target above home in meters
  // @Range: 20 300
  // @Units: m
  // @User: Standard
--]]
local KANG_ALT_M = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "ALT_M", 2, 80)

--[[
  // @Param: KANG_SPD_MIN
  // @DisplayName: Kangaroo minimum speed
  // @Description: Minimum target speed during a hop burst
  // @Range: 1 40
  // @Units: m/s
  // @User: Standard
--]]
local KANG_SPD_MIN = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "SPD_MIN", 3, 8)

--[[
  // @Param: KANG_SPD_MAX
  // @DisplayName: Kangaroo maximum speed
  // @Description: Maximum target speed during a hop burst
  // @Range: 1 40
  // @Units: m/s
  // @User: Standard
--]]
local KANG_SPD_MAX = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "SPD_MAX", 4, 20)

--[[
  // @Param: KANG_HOP_MIN
  // @DisplayName: Kangaroo minimum hop
  // @Description: Minimum duration of a hop segment
  // @Range: 1 20
  // @Units: s
  // @User: Standard
--]]
local KANG_HOP_MIN = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "HOP_MIN", 5, 2)

--[[
  // @Param: KANG_HOP_MAX
  // @DisplayName: Kangaroo maximum hop
  // @Description: Maximum duration of a hop segment
  // @Range: 1 20
  // @Units: s
  // @User: Standard
--]]
local KANG_HOP_MAX = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "HOP_MAX", 6, 6)

--[[
  // @Param: KANG_BOUND_M
  // @DisplayName: Kangaroo boundary
  // @Description: Maximum distance from home before the target turns back inward
  // @Range: 50 3000
  // @Units: m
  // @User: Standard
--]]
local KANG_BOUND_M = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "BOUND_M", 7, 500)

--[[
  // @Param: KANG_TURN_DEG
  // @DisplayName: Kangaroo turn angle
  // @Description: Typical heading change limit for each random hop
  // @Range: 10 180
  // @Units: deg
  // @User: Standard
--]]
local KANG_TURN_DEG = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "TURN_DEG", 8, 110)

--[[
  // @Param: KANG_PAUSE_P
  // @DisplayName: Kangaroo pause chance
  // @Description: Chance in percent that the target pauses between hops
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local KANG_PAUSE_P = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PAUSE_P", 9, 20)

--[[
  // @Param: KANG_PAUSE_S
  // @DisplayName: Kangaroo pause time
  // @Description: Maximum pause duration when the target stops between hops
  // @Range: 0 10
  // @Units: s
  // @User: Standard
--]]
local KANG_PAUSE_S = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PAUSE_S", 10, 2)

--[[
  // @Param: KANG_PRINT
  // @DisplayName: Kangaroo print target
  // @Description: Print the virtual target location to the terminal
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_PRINT = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PRINT", 11, 1)

--[[
  // @Param: KANG_PRT_S
  // @DisplayName: Kangaroo print period
  // @Description: Period between target location prints
  // @Range: 1 30
  // @Units: s
  // @User: Standard
--]]
local KANG_PRT_S = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PRT_S", 12, 1)

--[[
  // @Param: KANG_OFS_M
  // @DisplayName: Kangaroo start offset
  // @Description: Initial offset from home to place the target
  // @Range: 0 200
  // @Units: m
  // @User: Standard
--]]
local KANG_OFS_M = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "OFS_M", 13, 200)

-- local variables initisalising
local anchor_loc = nil
local target_loc = nil
local target_north = 0
local target_east = 0
local heading_deg = 0
local speed_mps = 0
-- velocity north and east components
local target_vn = 0
local target_ve = 0
local last_update_ms = 0
local segment_end_ms = 0
local last_report_ms = 0
local target_ready = false

local function get_radius_m()
    return math.sqrt(target_north * target_north + target_east * target_east)
end

local function update_target_location()
    if anchor_loc == nil then
        return
    end
    target_loc = anchor_loc:copy()
    target_loc:offset(target_north, target_east)
    target_loc:set_alt_m((anchor_loc:alt() * 0.01) + math_helpers.clamp(KANG_ALT_M:get(), 20, 300), ALT_FRAME_ABSOLUTE)
end

local function choose_heading(bound_m, turn_deg)
    local radius_m = get_radius_m()
    local center_heading_deg = math_helpers.wrap_360(math.deg(math.atan(-target_east, -target_north)))

    if radius_m > bound_m * 0.85 then
        return math_helpers.wrap_360(center_heading_deg + math_helpers.random_between(-40, 40))
    end

    if math.random() < 0.25 then
        return math_helpers.wrap_360(center_heading_deg + math_helpers.random_between(-90, 90))
    end

    return math_helpers.wrap_360(heading_deg + math_helpers.random_between(-turn_deg, turn_deg))
end

-- generate next point
local function choose_next_segment(now_ms)
    local speed_min, speed_max = math_helpers.ordered_range(KANG_SPD_MIN:get(), KANG_SPD_MAX:get(), 1)
    local hop_min, hop_max = math_helpers.ordered_range(KANG_HOP_MIN:get(), KANG_HOP_MAX:get(), 1)
    local bound_m = math_helpers.clamp(KANG_BOUND_M:get(), 50, 3000)
    local turn_deg = math_helpers.clamp(KANG_TURN_DEG:get(), 10, 180)
    local pause_chance = math_helpers.clamp(KANG_PAUSE_P:get(), 0, 100) * 0.01
    local pause_max_s = math_helpers.clamp(KANG_PAUSE_S:get(), 0, 10)

    heading_deg = choose_heading(bound_m, turn_deg)

    local burst = math.random() < 0.35
    if burst then
        speed_mps = math_helpers.random_between(math.max(speed_min, speed_max * 0.65), speed_max)
    else
        speed_mps = math_helpers.random_between(speed_min, speed_max)
    end

    local duration_ms = math_helpers.random_between(hop_min * 1000, hop_max * 1000)
    if math.random() < pause_chance then
        speed_mps = 0
        duration_ms = math_helpers.random_between(500, math.max(500, pause_max_s * 1000))
    end

    segment_end_ms = now_ms + duration_ms
end

-- starting point for kangaroo random walk
local function ensure_anchor(now_ms)
    if anchor_loc ~= nil then
        return true
    end

    local home = ahrs:get_home()
    if home == nil then
        return false
    end

    anchor_loc = home:copy()
    anchor_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
    local start_offset = math_helpers.clamp(KANG_OFS_M:get(), 0, 200)
    heading_deg = math_helpers.random_between(0, 360)
    target_north = math.cos(math.rad(heading_deg)) * start_offset
    target_east = math.sin(math.rad(heading_deg)) * start_offset
    speed_mps = 0
    last_update_ms = now_ms
    choose_next_segment(now_ms)
    update_target_location()
    target_ready = true
    gcs:send_text(MAV_SEVERITY.INFO, "KANG: virtual target initialised")
    return true
end

-- move the randomisation function forward in time
local function integrate_target(now_ms)
    if not target_ready then
        return
    end
    local dt = (now_ms - last_update_ms) * 0.001
    if dt < 0 then
        dt = 0
    end
    if dt > 0.5 then
        dt = 0.5
    end
    last_update_ms = now_ms

    if now_ms >= segment_end_ms then
        choose_next_segment(now_ms)
    end

    local vn = math.cos(math.rad(heading_deg)) * speed_mps
    local ve = math.sin(math.rad(heading_deg)) * speed_mps

    target_north = target_north + (vn * dt)
    target_east = target_east + (ve * dt)

    local bound_m = math_helpers.clamp(KANG_BOUND_M:get(), 50, 3000)
    local radius_m = get_radius_m()
    if radius_m > bound_m then
        local scale = bound_m / radius_m
        target_north = target_north * scale
        target_east = target_east * scale
        heading_deg = math_helpers.wrap_360(math.deg(math.atan(-target_east, -target_north)) + math_helpers.random_between(-25, 25))
        segment_end_ms = now_ms + 1000
        vn = math.cos(math.rad(heading_deg)) * speed_mps
        ve = math.sin(math.rad(heading_deg)) * speed_mps
    end
    target_vn = vn
    target_ve = ve
    update_target_location()
end

-- print message to mavlink console
local function report_target(now_ms)
    if KANG_PRINT:get() < 1 or target_loc == nil then
        return
    end

    local report_period_ms = math_helpers.clamp(KANG_PRT_S:get(), 1, 30) * 1000

    if now_ms - last_report_ms < report_period_ms then
        return
    end

    last_report_ms = now_ms

    gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
            "KANG: lat=%.7f lon=%.7f alt=%.1f spd=%.1f hdg=%.0f vn=%.1f ve=%.1f r=%.1f",
            target_loc:lat() * 1.0e-7,
            target_loc:lng() * 1.0e-7,
            target_loc:alt() * 0.01,
            speed_mps,
            heading_deg,
            target_vn or 0,
            target_ve or 0,
            get_radius_m()
        )
    )
end

local function update()
    
    if KANG_ENABLE:get() < 1 then
        return
    end

    local now_ms = millis():toint()
    if not ensure_anchor(now_ms) then
        return
    end
    integrate_target(now_ms)
    kangaroo_bus.publish(target_loc, target_vn, target_ve, heading_deg, speed_mps, now_ms)
    report_target(now_ms)
end

math.randomseed(millis():toint())
math.random()
math.random()

gcs:send_text(MAV_SEVERITY.INFO, "KANG: loaded, switch Plane to GUIDED after takeoff")

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "KANG: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 100
end

return protected_wrapper()
