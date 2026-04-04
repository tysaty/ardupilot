-- Kangaroo 3
--
-- Plane SITL helper that uses the built-in SIM_SHIP target but chooses more
-- sensible defaults so Plane does not immediately settle into a small loiter
-- when switched to GUIDED.

local MODE_GUIDED = 15
local ALT_FRAME_ABSOLUTE = 0
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}

local PARAM_TABLE_KEY = 90
local PARAM_TABLE_PREFIX = "KANG3_"

local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format("could not find %s", name))
    return p
end

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format("could not add param %s", name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), "could not add KANG3_ parameter table")

--[[
  // @Param: KANG3_ENABLE
  // @DisplayName: Kangaroo3 enable
  // @Description: Enable the sim ship based kangaroo target and Guided follow helper
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG3_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: KANG3_SYSID
  // @DisplayName: Kangaroo3 ship SYSID
  // @Description: MAVLink system ID used by the simulated ship target
  // @Range: 1 255
  // @User: Standard
--]]
local KANG3_SYSID = bind_add_param("SYSID", 2, 17)

--[[
  // @Param: KANG3_DIST_M
  // @DisplayName: Kangaroo3 follow distance
  // @Description: Distance behind the simulated ship for the plane to hold in Guided mode
  // @Range: 50 600
  // @Units: m
  // @User: Standard
--]]
local KANG3_DIST_M = bind_add_param("DIST_M", 3, 220)

--[[
  // @Param: KANG3_ALT_M
  // @DisplayName: Kangaroo3 follow altitude
  // @Description: Height above the simulated ship for the plane to hold in Guided mode
  // @Range: 20 300
  // @Units: m
  // @User: Standard
--]]
local KANG3_ALT_M = bind_add_param("ALT_M", 4, 80)

--[[
  // @Param: KANG3_SPD_MIN
  // @DisplayName: Kangaroo3 minimum speed
  // @Description: Minimum random ship speed
  // @Range: 4 35
  // @Units: m/s
  // @User: Standard
--]]
local KANG3_SPD_MIN = bind_add_param("SPD_MIN", 5, 14)

--[[
  // @Param: KANG3_SPD_MAX
  // @DisplayName: Kangaroo3 maximum speed
  // @Description: Maximum random ship speed
  // @Range: 4 35
  // @Units: m/s
  // @User: Standard
--]]
local KANG3_SPD_MAX = bind_add_param("SPD_MAX", 6, 22)

--[[
  // @Param: KANG3_PATH_MIN
  // @DisplayName: Kangaroo3 minimum path
  // @Description: Minimum simulated ship path diameter
  // @Range: 100 5000
  // @Units: m
  // @User: Standard
--]]
local KANG3_PATH_MIN = bind_add_param("PATH_MIN", 7, 500)

--[[
  // @Param: KANG3_PATH_MAX
  // @DisplayName: Kangaroo3 maximum path
  // @Description: Maximum simulated ship path diameter
  // @Range: 100 5000
  // @Units: m
  // @User: Standard
--]]
local KANG3_PATH_MAX = bind_add_param("PATH_MAX", 8, 1800)

--[[
  // @Param: KANG3_HOP_MIN
  // @DisplayName: Kangaroo3 minimum hop
  // @Description: Minimum time between random ship motion updates
  // @Range: 1 60
  // @Units: s
  // @User: Standard
--]]
local KANG3_HOP_MIN = bind_add_param("HOP_MIN", 9, 6)

--[[
  // @Param: KANG3_HOP_MAX
  // @DisplayName: Kangaroo3 maximum hop
  // @Description: Maximum time between random ship motion updates
  // @Range: 1 60
  // @Units: s
  // @User: Standard
--]]
local KANG3_HOP_MAX = bind_add_param("HOP_MAX", 10, 15)

--[[
  // @Param: KANG3_SHIP_N
  // @DisplayName: Kangaroo3 ship north offset
  // @Description: Initial north offset of the sim ship from Plane home
  // @Range: -500 500
  // @Units: m
  // @User: Standard
--]]
local KANG3_SHIP_N = bind_add_param("SHIP_N", 11, 220)

--[[
  // @Param: KANG3_SHIP_E
  // @DisplayName: Kangaroo3 ship east offset
  // @Description: Initial east offset of the sim ship from Plane home
  // @Range: -500 500
  // @Units: m
  // @User: Standard
--]]
local KANG3_SHIP_E = bind_add_param("SHIP_E", 12, 120)

--[[
  // @Param: KANG3_PRINT
  // @DisplayName: Kangaroo3 print ship
  // @Description: Print the ship location to the terminal
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG3_PRINT = bind_add_param("PRINT", 13, 1)

--[[
  // @Param: KANG3_PRT_S
  // @DisplayName: Kangaroo3 print period
  // @Description: Period between ship location prints
  // @Range: 1 30
  // @Units: s
  // @User: Standard
--]]
local KANG3_PRT_S = bind_add_param("PRT_S", 14, 1)

--[[
  // @Param: KANG3_DECK_M
  // @DisplayName: Kangaroo3 deck size
  // @Description: Sim ship deck size passed to SIM_SHIP_DSIZE
  // @Range: 10 200
  // @Units: m
  // @User: Standard
--]]
local KANG3_DECK_M = bind_add_param("DECK_M", 15, 40)

local FOLL_ENABLE = bind_param("FOLL_ENABLE")
local FOLL_SYSID = bind_param("FOLL_SYSID")
local FOLL_OFS_TYPE = bind_param("FOLL_OFS_TYPE")
local FOLL_ALT_TYPE = bind_param("FOLL_ALT_TYPE")
local FOLL_OFS_X = bind_param("FOLL_OFS_X")
local FOLL_OFS_Y = bind_param("FOLL_OFS_Y")
local FOLL_OFS_Z = bind_param("FOLL_OFS_Z")

local SIM_SHIP_ENABLE = bind_param("SIM_SHIP_ENABLE")
local SIM_SHIP_SPEED = bind_param("SIM_SHIP_SPEED")
local SIM_SHIP_PSIZE = bind_param("SIM_SHIP_PSIZE")
local SIM_SHIP_SYSID = bind_param("SIM_SHIP_SYSID")
local SIM_SHIP_DSIZE = bind_param("SIM_SHIP_DSIZE")
local SIM_SHIP_OFS_X = bind_param("SIM_SHIP_OFS_X")
local SIM_SHIP_OFS_Y = bind_param("SIM_SHIP_OFS_Y")
local SIM_SHIP_OFS_Z = bind_param("SIM_SHIP_OFS_Z")

local next_hop_ms = 0
local last_ship_report_ms = 0
local ship_started = false
local ship_warned = false
local have_target = false
local guided_follow_started = false

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

local function set_param_if_needed(p, value, tolerance)
    tolerance = tolerance or 0.001
    local current = p:get()
    if current == nil or math.abs(current - value) > tolerance then
        return p:set(value)
    end
    return true
end

local function configure_follow_params()
    local follow_distance = clamp(KANG3_DIST_M:get(), 50, 600)
    local follow_altitude = clamp(KANG3_ALT_M:get(), 20, 300)
    local sysid = clamp(KANG3_SYSID:get(), 1, 255)

    set_param_if_needed(FOLL_ENABLE, 1)
    set_param_if_needed(FOLL_SYSID, sysid)
    set_param_if_needed(FOLL_OFS_TYPE, 1)
    set_param_if_needed(FOLL_ALT_TYPE, 0)
    set_param_if_needed(FOLL_OFS_X, -follow_distance, 0.1)
    set_param_if_needed(FOLL_OFS_Y, 0, 0.1)
    set_param_if_needed(FOLL_OFS_Z, -follow_altitude, 0.1)
end

local function choose_next_hop(now_ms)
    local speed_min, speed_max = ordered_range(KANG3_SPD_MIN:get(), KANG3_SPD_MAX:get(), 4)
    local path_min, path_max = ordered_range(KANG3_PATH_MIN:get(), KANG3_PATH_MAX:get(), 100)
    local hop_min, hop_max = ordered_range(KANG3_HOP_MIN:get(), KANG3_HOP_MAX:get(), 1)

    local burst = math.random() < 0.30
    local target_speed = random_between(speed_min, speed_max)
    local target_path = random_between(path_min, path_max)
    if burst then
        target_speed = random_between(math.max(speed_min, speed_max * 0.75), speed_max)
        target_path = random_between(path_min, path_min + (path_max - path_min) * 0.45)
    end

    set_param_if_needed(SIM_SHIP_SPEED, target_speed, 0.01)
    set_param_if_needed(SIM_SHIP_PSIZE, target_path, 0.1)
    next_hop_ms = now_ms + random_between(hop_min * 1000, hop_max * 1000)
end

local function ensure_ship_started(now_ms)
    local home = ahrs:get_home()
    if home == nil then
        return false
    end

    local sysid = clamp(KANG3_SYSID:get(), 1, 255)
    set_param_if_needed(SIM_SHIP_SYSID, sysid)
    set_param_if_needed(SIM_SHIP_DSIZE, clamp(KANG3_DECK_M:get(), 10, 200), 0.1)

    if SIM_SHIP_ENABLE:get() > 0 then
        ship_started = true
        if not ship_warned then
            gcs:send_text(MAV_SEVERITY.INFO, "KANG3: using active SIM_SHIP target")
            ship_warned = true
        end
        return true
    end

    set_param_if_needed(SIM_SHIP_OFS_X, clamp(KANG3_SHIP_N:get(), -500, 500), 0.1)
    set_param_if_needed(SIM_SHIP_OFS_Y, clamp(KANG3_SHIP_E:get(), -500, 500), 0.1)
    set_param_if_needed(SIM_SHIP_OFS_Z, 0, 0.01)
    choose_next_hop(now_ms)
    if SIM_SHIP_ENABLE:set(1) then
        ship_started = true
        gcs:send_text(MAV_SEVERITY.INFO, "KANG3: SIM_SHIP enabled with stand-off offsets")
        return true
    end

    gcs:send_text(MAV_SEVERITY.ERROR, "KANG3: failed to enable SIM_SHIP")
    return false
end

local function maybe_report_ship_location(now_ms, ship_loc, ship_vel)
    if KANG3_PRINT:get() < 1 or ship_loc == nil then
        return
    end

    local report_period_ms = clamp(KANG3_PRT_S:get(), 1, 30) * 1000
    if now_ms - last_ship_report_ms < report_period_ms then
        return
    end

    local groundspeed = 0
    if ship_vel ~= nil then
        groundspeed = math.sqrt((ship_vel:x() * ship_vel:x()) + (ship_vel:y() * ship_vel:y()))
    end

    ship_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
    last_ship_report_ms = now_ms
    gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
            "KANG3: ship lat=%.7f lon=%.7f alt=%.1f spd=%.1f",
            ship_loc:lat() * 1.0e-7,
            ship_loc:lng() * 1.0e-7,
            ship_loc:alt() * 0.01,
            groundspeed
        )
    )
end

local function update_guided_follow(now_ms)
    if not follow:have_target() then
        if have_target then
            gcs:send_text(MAV_SEVERITY.WARNING, "KANG3: lost simulated ship target")
        end
        have_target = false
        guided_follow_started = false
        return
    end

    local ship_loc, ship_vel = follow:get_target_location_and_velocity()
    local target_pos, target_vel = follow:get_target_location_and_velocity_ofs()
    if ship_loc == nil or target_pos == nil or target_vel == nil then
        return
    end

    have_target = true
    maybe_report_ship_location(now_ms, ship_loc, ship_vel)
    target_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)

    if vehicle:get_mode() ~= MODE_GUIDED then
        guided_follow_started = false
        return
    end

    local current_target = vehicle:get_target_location()
    if current_target == nil then
        if vehicle:set_target_location(target_pos) then
            vehicle:set_velocity_match(target_vel:xy())
            guided_follow_started = true
            gcs:send_text(MAV_SEVERITY.INFO, "KANG3: Guided follow engaged")
        end
        return
    end

    if not guided_follow_started then
        guided_follow_started = true
        gcs:send_text(MAV_SEVERITY.INFO, "KANG3: Guided follow tracking")
    end

    vehicle:set_velocity_match(target_vel:xy())
    vehicle:update_target_location(current_target, target_pos)
end

local function update()
    if KANG3_ENABLE:get() < 1 then
        guided_follow_started = false
        have_target = false
        return
    end

    local now_ms = millis():toint()
    configure_follow_params()

    if not ensure_ship_started(now_ms) then
        return
    end

    if ship_started and now_ms >= next_hop_ms then
        choose_next_hop(now_ms)
    end

    update_guided_follow(now_ms)
end

math.randomseed(millis():toint())
math.random()
math.random()

gcs:send_text(MAV_SEVERITY.INFO, "KANG3: loaded, take off then switch Plane to GUIDED")

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "KANG3: " .. err)
        return protected_wrapper, 1000
    end
    return protected_wrapper, 100
end

return protected_wrapper()
