-- Kangaroo 2
-- Plane SITL helper that creates a virtual target starting near home and
-- moves it in random hop-like bursts. When Plane is switched to GUIDED the
-- script continuously updates the Guided target so the aircraft chases the
-- virtual kangaroo.

-- initial
local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}

local PARAM_TABLE_KEY = 89
local PARAM_TABLE_PREFIX = "KANG2_"


-- binding aparameter
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 13), "could not add KANG2_ parameter table")

--[[
  // @Param: KANG2_ENABLE
  // @DisplayName: Kangaroo2 enable
  // @Description: Enable the virtual hopping target for Plane Guided mode
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG2_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: KANG2_ALT_M
  // @DisplayName: Kangaroo2 altitude
  // @Description: Altitude of the virtual target above home in meters
  // @Range: 20 300
  // @Units: m
  // @User: Standard
--]]
local KANG2_ALT_M = bind_add_param("ALT_M", 2, 80)

--[[
  // @Param: KANG2_SPD_MIN
  // @DisplayName: Kangaroo2 minimum speed
  // @Description: Minimum target speed during a hop burst
  // @Range: 1 40
  // @Units: m/s
  // @User: Standard
--]]
local KANG2_SPD_MIN = bind_add_param("SPD_MIN", 3, 8)

--[[
  // @Param: KANG2_SPD_MAX
  // @DisplayName: Kangaroo2 maximum speed
  // @Description: Maximum target speed during a hop burst
  // @Range: 1 40
  // @Units: m/s
  // @User: Standard
--]]
local KANG2_SPD_MAX = bind_add_param("SPD_MAX", 4, 20)

--[[
  // @Param: KANG2_HOP_MIN
  // @DisplayName: Kangaroo2 minimum hop
  // @Description: Minimum duration of a hop segment
  // @Range: 1 20
  // @Units: s
  // @User: Standard
--]]
local KANG2_HOP_MIN = bind_add_param("HOP_MIN", 5, 2)

--[[
  // @Param: KANG2_HOP_MAX
  // @DisplayName: Kangaroo2 maximum hop
  // @Description: Maximum duration of a hop segment
  // @Range: 1 20
  // @Units: s
  // @User: Standard
--]]
local KANG2_HOP_MAX = bind_add_param("HOP_MAX", 6, 6)

--[[
  // @Param: KANG2_BOUND_M
  // @DisplayName: Kangaroo2 boundary
  // @Description: Maximum distance from home before the target turns back inward
  // @Range: 50 3000
  // @Units: m
  // @User: Standard
--]]
local KANG2_BOUND_M = bind_add_param("BOUND_M", 7, 500)

--[[
  // @Param: KANG2_TURN_DEG
  // @DisplayName: Kangaroo2 turn angle
  // @Description: Typical heading change limit for each random hop
  // @Range: 10 180
  // @Units: deg
  // @User: Standard
--]]
local KANG2_TURN_DEG = bind_add_param("TURN_DEG", 8, 110)

--[[
  // @Param: KANG2_PAUSE_P
  // @DisplayName: Kangaroo2 pause chance
  // @Description: Chance in percent that the target pauses between hops
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local KANG2_PAUSE_P = bind_add_param("PAUSE_P", 9, 20)

--[[
  // @Param: KANG2_PAUSE_S
  // @DisplayName: Kangaroo2 pause time
  // @Description: Maximum pause duration when the target stops between hops
  // @Range: 0 10
  // @Units: s
  // @User: Standard
--]]
local KANG2_PAUSE_S = bind_add_param("PAUSE_S", 10, 2)

--[[
  // @Param: KANG2_PRINT
  // @DisplayName: Kangaroo2 print target
  // @Description: Print the virtual target location to the terminal
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG2_PRINT = bind_add_param("PRINT", 11, 1)

--[[
  // @Param: KANG2_PRT_S
  // @DisplayName: Kangaroo2 print period
  // @Description: Period between target location prints
  // @Range: 1 30
  // @Units: s
  // @User: Standard
--]]
local KANG2_PRT_S = bind_add_param("PRT_S", 12, 1)

--[[
  // @Param: KANG2_OFS_M
  // @DisplayName: Kangaroo2 start offset
  // @Description: Initial offset from home to place the target
  // @Range: 0 200
  // @Units: m
  // @User: Standard
--]]
local KANG2_OFS_M = bind_add_param("OFS_M", 13, 200)
local anchor_loc = nil
local target_loc = nil
local target_vel = Vector2f()
local target_north = 0
local target_east = 0
local heading_deg = 0
local speed_mps = 0
local last_update_ms = 0
local segment_end_ms = 0
local last_report_ms = 0
local guided_started = false
local target_ready = false

local function clamp(value, min_value, max_value)
    if value < min_value then
        return min_value
    end
    if value > max_value then
        return max_value
    end
    return value
end

local function ordered_range(v1, v2, floor_value)
    local a = math.max(v1, floor_value)
    local b = math.max(v2, floor_value)
    if a > b then
        a, b = b, a
    end
    return a, b
end

local function random_between(min_value, max_value)
    if max_value <= min_value then
        return min_value
    end
    return min_value + (max_value - min_value) * math.random()
end

local function wrap_360(angle)
    local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

local function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
        res = res - 360
    end
    return res
end

local function get_radius_m()
    return math.sqrt(target_north * target_north + target_east * target_east)
end

local function update_target_location()
    if anchor_loc == nil then
        return
    end
    target_loc = anchor_loc:copy()
    target_loc:offset(target_north, target_east)
    target_loc:set_alt_m((anchor_loc:alt() * 0.01) + clamp(KANG2_ALT_M:get(), 20, 300), ALT_FRAME_ABSOLUTE)
end

local function choose_heading(bound_m, turn_deg)
    local radius_m = get_radius_m()
    local center_heading_deg = wrap_360(math.deg(math.atan(-target_east, -target_north)))

    if radius_m > bound_m * 0.85 then
        return wrap_360(center_heading_deg + random_between(-40, 40))
    end

    if math.random() < 0.25 then
        return wrap_360(center_heading_deg + random_between(-90, 90))
    end

    return wrap_360(heading_deg + random_between(-turn_deg, turn_deg))
end

local function choose_next_segment(now_ms)
    local speed_min, speed_max = ordered_range(KANG2_SPD_MIN:get(), KANG2_SPD_MAX:get(), 1)
    local hop_min, hop_max = ordered_range(KANG2_HOP_MIN:get(), KANG2_HOP_MAX:get(), 1)
    local bound_m = clamp(KANG2_BOUND_M:get(), 50, 3000)
    local turn_deg = clamp(KANG2_TURN_DEG:get(), 10, 180)
    local pause_chance = clamp(KANG2_PAUSE_P:get(), 0, 100) * 0.01
    local pause_max_s = clamp(KANG2_PAUSE_S:get(), 0, 10)

    heading_deg = choose_heading(bound_m, turn_deg)

    local burst = math.random() < 0.35
    if burst then
        speed_mps = random_between(math.max(speed_min, speed_max * 0.65), speed_max)
    else
        speed_mps = random_between(speed_min, speed_max)
    end

    local duration_ms = random_between(hop_min * 1000, hop_max * 1000)
    if math.random() < pause_chance then
        speed_mps = 0
        duration_ms = random_between(500, math.max(500, pause_max_s * 1000))
    end

    segment_end_ms = now_ms + duration_ms
end

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

    local start_offset = clamp(KANG2_OFS_M:get(), 0, 200)
    heading_deg = random_between(0, 360)
    target_north = math.cos(math.rad(heading_deg)) * start_offset
    target_east = math.sin(math.rad(heading_deg)) * start_offset
    speed_mps = 0
    last_update_ms = now_ms
    choose_next_segment(now_ms)
    update_target_location()
    target_ready = true

    gcs:send_text(MAV_SEVERITY.INFO, "KANG2: virtual target initialised")
    return true
end

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

    local bound_m = clamp(KANG2_BOUND_M:get(), 50, 3000)
    local radius_m = get_radius_m()
    if radius_m > bound_m then
        local scale = bound_m / radius_m
        target_north = target_north * scale
        target_east = target_east * scale
        heading_deg = wrap_360(math.deg(math.atan(-target_east, -target_north)) + random_between(-25, 25))
        segment_end_ms = now_ms + 1000
        vn = math.cos(math.rad(heading_deg)) * speed_mps
        ve = math.sin(math.rad(heading_deg)) * speed_mps
    end

    target_vel:x(vn)
    target_vel:y(ve)
    update_target_location()
end

local function maybe_report_target(now_ms)
    if KANG2_PRINT:get() < 1 or target_loc == nil then
        return
    end

    local report_period_ms = clamp(KANG2_PRT_S:get(), 1, 30) * 1000
    if now_ms - last_report_ms < report_period_ms then
        return
    end

    last_report_ms = now_ms
    gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
            "KANG2: lat=%.7f lon=%.7f alt=%.1f spd=%.1f hdg=%.0f",
            target_loc:lat() * 1.0e-7,
            target_loc:lng() * 1.0e-7,
            target_loc:alt() * 0.01,
            speed_mps,
            heading_deg
        )
    )
end

local function update_guided_follow()
    if vehicle:get_mode() ~= MODE_GUIDED then
        guided_started = false
        return
    end

    if target_loc == nil then
        return
    end

    local current_target = vehicle:get_target_location()
    if current_target == nil then
        if vehicle:set_target_location(target_loc) then
            vehicle:set_velocity_match(target_vel)
            guided_started = true
            gcs:send_text(MAV_SEVERITY.INFO, "KANG2: Guided chase engaged")
        end
        return
    end

    if not guided_started then
        guided_started = true
        gcs:send_text(MAV_SEVERITY.INFO, "KANG2: Guided chase tracking")
    end

    vehicle:set_velocity_match(target_vel)
    vehicle:update_target_location(current_target, target_loc)
end

local function update()
    if KANG2_ENABLE:get() < 1 then
        guided_started = false
        return
    end

    local now_ms = millis():toint()
    if not ensure_anchor(now_ms) then
        return
    end

    integrate_target(now_ms)
    maybe_report_target(now_ms)
    update_guided_follow()
end

math.randomseed(millis():toint())
math.random()
math.random()

gcs:send_text(MAV_SEVERITY.INFO, "KANG2: loaded, switch Plane to GUIDED after takeoff")

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "KANG2: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 100
end

return protected_wrapper()
