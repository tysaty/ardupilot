-- ---------------------------------------------------------
-- Kangaroo.lua
-- Plane SITL helper that creates a virtual target starting near home and
-- moves it in random hop-like bursts. 

-- Control logic is controlled in control.lua
-- ---------------------------------------------------------
-- ---------------------------------------------------------
-- Section 1. Initial values
-- ---------------------------------------------------------

-- setting alt_frame to zero
local ALT_FRAME_ABSOLUTE = 0
local MAV_SEVERITY = {EMERGENCY = 0, ALERT = 1, CRITICAL = 2, ERROR = 3, WARNING = 4, NOTICE = 5, INFO = 6, DEBUG = 7}

-- for delcared parameters
local PARAM_TABLE_PREFIX = "KANG_"
local PARAM_TABLE_KEY = nil

gcs:send_text(MAV_SEVERITY.WARNING, "KANG: loaded at boot")

-- use the kangaroo_bus for standardised passing of messages
--local kangaroo_bus = require("kangaroo_bus")
local param_helpers = require("param_helpers")
local math_helpers = require("math_helpers")


-- initialise mavlink (no rx needed, only tx)
mavlink:init(1, 0)

-- ADSB flags bitmask values
local ADSB_FLAGS_VALID_COORDS            = 1
local ADSB_FLAGS_VALID_ALTITUDE          = 2
local ADSB_FLAGS_VALID_HEADING           = 4
local ADSB_FLAGS_VALID_VELOCITY          = 8
local ADSB_FLAGS_VALID_CALLSIGN          = 16
local ADSB_FLAGS_SIMULATED               = 64
local ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128

local ADSB_VEHICLE_MSGID = 246
local KANG_ICAO_ADDR     = 0xCAFE00
local KANG_CALLSIGN      = "KANGAROO"

local KANG_FLAGS = ADSB_FLAGS_VALID_COORDS + ADSB_FLAGS_VALID_ALTITUDE
                 + ADSB_FLAGS_VALID_HEADING + ADSB_FLAGS_VALID_VELOCITY
                 + ADSB_FLAGS_VALID_CALLSIGN + ADSB_FLAGS_SIMULATED
                 + ADSB_FLAGS_VERTICAL_VELOCITY_VALID

local function send_ADSB_VEHICLE(lat_deg, lng_deg, alt_amsl_m, heading_deg_v,
                                 speed_mps_v, vspeed_mps)
    local cs = KANG_CALLSIGN .. string.rep("\0", 9)
    cs = string.sub(cs, 1, 9)
    local payload = string.pack("<I4i4i4i4 I2I2i2I2I2 Bc9BB",
        KANG_ICAO_ADDR,
        math.floor(lat_deg  * 1e7),
        math.floor(lng_deg  * 1e7),
        math.floor(alt_amsl_m * 1000),
        math.floor(heading_deg_v * 100),
        math.floor(speed_mps_v   * 100),
        math.floor(vspeed_mps    * 100),
        KANG_FLAGS,
        1200,
        0,
        cs,
        1,
        0)
    for chan = 0, 5 do
        mavlink:send_chan(chan, ADSB_VEHICLE_MSGID, payload)
    end
end



-- ---------------------------------------------------------
-- Section 1b. KBUS_ parameter table
-- Inter-script bus: passes target state to control.lua each update cycle.
-- Owned here; control.lua binds lazily after kangaroo boots.
-- ---------------------------------------------------------
local KBUS_TABLE_PREFIX = "KBUS_"
local KBUS_TABLE_KEY = nil
for key = 0, 200 do
    if param:add_table(key, KBUS_TABLE_PREFIX, 6) then
        KBUS_TABLE_KEY = key
        break
    end
end
assert(KBUS_TABLE_KEY ~= nil, "KANG: no free KBUS param table key")

local KBUS_SEQ = param_helpers.bind_add_param(KBUS_TABLE_KEY, KBUS_TABLE_PREFIX, "SEQ", 1, 0)  -- sequence counter (odd = write in progress, even = stable)
local KBUS_T_S = param_helpers.bind_add_param(KBUS_TABLE_KEY, KBUS_TABLE_PREFIX, "T_S", 2, 0)  -- timestamp (s)
local KBUS_LAT = param_helpers.bind_add_param(KBUS_TABLE_KEY, KBUS_TABLE_PREFIX, "LAT", 3, 0)  -- target latitude (deg)
local KBUS_LON = param_helpers.bind_add_param(KBUS_TABLE_KEY, KBUS_TABLE_PREFIX, "LON", 4, 0)  -- target longitude (deg)
local KBUS_VN  = param_helpers.bind_add_param(KBUS_TABLE_KEY, KBUS_TABLE_PREFIX, "VN",  5, 0)  -- velocity north (m/s)
local KBUS_VE  = param_helpers.bind_add_param(KBUS_TABLE_KEY, KBUS_TABLE_PREFIX, "VE",  6, 0)  -- velocity east (m/s)

-- ---------------------------------------------------------
-- Section 2. Parameters
-- Parameters to support quick debugging of Kangaroo motion
-- ---------------------------------------------------------
-- establish key for aparemeter table
for key = 0, 200 do
    if param:add_table(key, PARAM_TABLE_PREFIX, 24) then
        PARAM_TABLE_KEY = key
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("KANG: using key %d", key))
        break
    end
end

assert(PARAM_TABLE_KEY ~= nil, "KANG: no free param table key")

--[[
  // @Param: KANG_RAND
  // @DisplayName: Kangaroo random
  // @Description: Enable the virtual hopping target - randomised
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_RANDOM = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "RANDOM", 1, 1)

--[[
  // @Param: KANG_STRAIGHT
  // @DisplayName: Kangaroo straight
  // @Description: Enable the virtual hopping target - straight line
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_STRAIGHT = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "STRAIGHT", 2, 0)

--[[
  // @Param: KANG_CIRCLE
  // @DisplayName: Kangaroo circle
  // @Description: Enable the virtual hopping target - circling
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_CIRCLE = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "CIRCLE", 3, 0)

--[[
  // @Param: KANG_RECTANGLE
  // @DisplayName: Kangaroo rectangle
  // @Description: Enable the virtual hopping target - rectangle
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_RECTANGLE = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "RECTANGLE", 4, 0)

--[[
  // @Param: KANG_ALT_M
  // @DisplayName: Kangaroo altitude
  // @Description: Altitude of the virtual target above home in meters
  // @Range: 20 300
  // @Units: m
  // @User: Standard
--]]
local KANG_ALT_M = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "ALT_M", 5, 80)

--[[
  // @Param: KANG_SPD_MIN
  // @DisplayName: Kangaroo minimum speed
  // @Description: Minimum target speed during a hop burst
  // @Range: 1 40
  // @Units: m/s
  // @User: Standard
--]]
local KANG_SPD_MIN = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "SPD_MIN", 6, 8)

--[[
  // @Param: KANG_SPD_MAX
  // @DisplayName: Kangaroo maximum speed
  // @Description: Maximum target speed during a hop burst
  // @Range: 1 40
  // @Units: m/s
  // @User: Standard
--]]
local KANG_SPD_MAX = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "SPD_MAX", 7, 20)

--[[
  // @Param: KANG_HOP_MIN
  // @DisplayName: Kangaroo minimum hop
  // @Description: Minimum duration of a hop segment
  // @Range: 1 20
  // @Units: s
  // @User: Standard
--]]
local KANG_HOP_MIN = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "HOP_MIN", 8, 2)

--[[
  // @Param: KANG_HOP_MAX
  // @DisplayName: Kangaroo maximum hop
  // @Description: Maximum duration of a hop segment
  // @Range: 1 20
  // @Units: s
  // @User: Standard
--]]
local KANG_HOP_MAX = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "HOP_MAX", 9, 6)

--[[
  // @Param: KANG_BOUND_M
  // @DisplayName: Kangaroo boundary
  // @Description: Maximum distance from home before the target turns back inward
  // @Range: 50 3000
  // @Units: m
  // @User: Standard
--]]
local KANG_BOUND_M = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "BOUND_M", 10, 500)

--[[
  // @Param: KANG_TURN_DEG
  // @DisplayName: Kangaroo turn angle
  // @Description: Typical heading change limit for each random hop
  // @Range: 10 180
  // @Units: deg
  // @User: Standard
--]]
local KANG_TURN_DEG = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "TURN_DEG", 11, 110)

--[[
  // @Param: KANG_PAUSE_P
  // @DisplayName: Kangaroo pause chance
  // @Description: Chance in percent that the target pauses between hops
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local KANG_PAUSE_P = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PAUSE_P", 12, 20)

--[[
  // @Param: KANG_PAUSE_S
  // @DisplayName: Kangaroo pause time
  // @Description: Maximum pause duration when the target stops between hops
  // @Range: 0 10
  // @Units: s
  // @User: Standard
--]]
local KANG_PAUSE_S = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PAUSE_S", 13, 2)

--[[
  // @Param: KANG_PRINT
  // @DisplayName: Kangaroo print target
  // @Description: Print the virtual target location to the terminal
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_PRINT = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PRINT", 14, 1)

--[[
  // @Param: KANG_PRT_S
  // @DisplayName: Kangaroo print period
  // @Description: Period between target location prints
  // @Range: 1 30
  // @Units: s
  // @User: Standard
--]]
local KANG_PRT_S = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "PRT_S", 15, 1)

--[[
  // @Param: KANG_OFS_M
  // @DisplayName: Kangaroo start offset
  // @Description: Initial offset from home to place the target
  // @Range: 0 20000
  // @Units: m
  // @User: Standard
--]]
local KANG_OFS_M = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "OFS_M", 16, 500)

--[[
  // @Param: KANG_SPD_REL
  // @DisplayName: Kangaroo speed ratio
  // @Description: Maximum kangaroo speed as a fraction of plane groundspeed. Set 0 to disable ratio limiting.
  // @Range: 0 1
  // @Increment: 0.05
  // @User: Standard
--]]
local KANG_SPD_REL = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "SPD_REL", 17, 0.6)

--[[
  // @Param: KANG_STR_HDG
  // @DisplayName: Kangaroo straight heading
  // @Description: Heading for straight-line mode (clockwise from north)
  // @Range: 0 360
  // @Units: deg
  // @User: Standard
--]]
local KANG_STR_HDG = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "STR_HDG", 18, 0)

--[[
  // @Param: KANG_LAT_DISP
  // @DisplayName: Kangaroo straight displacement (x axis)
  // @Description: Straight line dispalcement in the x axis (- left of plane, + right of plane)
  // @Range: -20000 +20000
  // @Units: m
  // @User: Standard
--]]
local KANG_LAT_DISP = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "LAT_DISP", 19, 200)

--[[
  // @Param: KANG_CIR_R
  // @DisplayName: Kangaroo circle radius
  // @Description: Orbit radius for circle mode
  // @Range: 50 3000
  // @Units: m
  // @User: Standard
--]]
local KANG_CIR_R = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "CIR_R", 20, 300)

--[[
  // @Param: KANG_REC_W
  // @DisplayName: Kangaroo rectangle width
  // @Description: Rectangle width (cross-track dimension) for rectangle mode
  // @Range: 50 3000
  // @Units: m
  // @User: Standard
--]]
local KANG_REC_W = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "REC_W", 21, 400)

--[[
  // @Param: KANG_REC_L
  // @DisplayName: Kangaroo rectangle length
  // @Description: Rectangle length (along-track dimension) for rectangle mode
  // @Range: 50 3000
  // @Units: m
  // @User: Standard
--]]
local KANG_REC_L = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "REC_L", 22, 600)

--[[
  // @Param: KANG_REC_HDG
  // @DisplayName: Kangaroo rectangle heading
  // @Description: Rectangle orientation (clockwise from north); length axis points in this direction
  // @Range: 0 360
  // @Units: deg
  // @User: Standard
--]]
local KANG_REC_HDG = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "REC_HDG", 23, 0)

--[[
  // @Param: KANG_POINT
  // @DisplayName: Kangaroo point
  // @Description: Enable the virtual target as a static point - position set via KANG_STR_HDG, KANG_OFS_M, KANG_LAT_DISP
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local KANG_POINT = param_helpers.bind_add_param(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, "POINT", 24, 0)

-- local variables initisalising
local anchor_loc = nil
local target_loc = nil
local target_north = 0
local target_east = 0
local heading_deg = 0
local speed_mps = 0
local bus_seq_counter = 0
-- velocity north and east components
local target_vn = 0
local target_ve = 0
-- last update initialisation
local last_update_ms = 0
local segment_end_ms = 0
local last_report_ms = 0
local last_home_wait_ms = 0
local target_ready = false

-- mode-specific state
local orbit_north          = 0       -- OFS_M start offset shared by all modes
local orbit_east           = 0
local straight_heading_deg = 0
local circle_angle_rad     = 0
local rect_corners         = nil
local rect_side            = 0
local rect_t               = 0.0

local function get_radius_m()
    return math.sqrt(target_north * target_north + target_east * target_east)
end

local function get_active_mode()
    if KANG_POINT:get() >= 1 then
        return "point"
    end
    if KANG_STRAIGHT:get() >= 1 then
        return "straight"
    end
    if KANG_CIRCLE:get() >= 1 then
        return "circle"
    end
    if KANG_RECTANGLE:get() >= 1 then
        return "rectangle"
    end
    if KANG_RANDOM:get() >= 1 then
        return "random"
    end
    return nil
end

local function update_target_location()
    if anchor_loc == nil then
        return
    end
    target_loc = anchor_loc:copy()
    target_loc:offset(target_north, target_east)
    -- target_loc:set_alt_m((anchor_loc:alt() * 0.01) + math_helpers.clamp(KANG_ALT_M:get(), 20, 300), ALT_FRAME_ABSOLUTE)
    target_loc:set_alt_m(anchor_loc:alt() * 0.01, ALT_FRAME_ABSOLUTE)
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

local function get_plane_groundspeed_mps()
    local groundspeed = ahrs:groundspeed_vector()
    if groundspeed == nil then
        return nil
    end
    local speed_mps_now = groundspeed:length()
    if speed_mps_now == nil then
        return nil
    end
    return math.max(0, speed_mps_now)
end

-- fix speed - bounding -- call somewhere...
-- integrate 17 May
local function get_capped_speed()
    local spd   = math_helpers.clamp(KANG_SPD_MAX:get(), 1, 40)
    local ratio = math_helpers.clamp(KANG_SPD_REL:get(), 0, 1)
    if ratio > 0 then
        local plane_spd = get_plane_groundspeed_mps()
        if plane_spd ~= nil and spd > plane_spd * ratio then
            spd = plane_spd * ratio
        end
    end
    return spd
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
    -- to do 18 May
    local speed_ratio = math_helpers.clamp(KANG_SPD_REL:get(), 0, 1)
    -- fix speed ratio
    if speed_ratio > 0 then
        local plane_speed = get_plane_groundspeed_mps()
        if plane_speed ~= nil then
            local ratio_cap_mps = plane_speed * speed_ratio
            if speed_mps > ratio_cap_mps then
                speed_mps = ratio_cap_mps
            end
        end
    end

    segment_end_ms = now_ms + duration_ms
end

-- Helper to adjust offset of the heading with the laterial projection 
local function heading_frame_offset(heading_deg_v, forward_m, lateral_m)
-- heading
    local h = math.rad(heading_deg_v)
        return {
            n = math.cos(h) * forward_m - math.sin(h) * lateral_m,
            e = math.sin(h) * forward_m + math.cos(h) * lateral_m,
        }
end

-- starting point for kangaroo random walk
local function ensure_anchor(now_ms)
    if anchor_loc ~= nil then
        return true
    end

    local base = nil

    local anchor_source = "unknown"

    -- set at home if it's available
    if ahrs:home_is_set() then
        base = ahrs:get_home()
        if base ~= nil then
            anchor_source = "home"
        end
    end

    -- if get_home doesn't load the current EKF/GPS location
    if base == nil then
        base = ahrs:get_location()
        if base ~= nil then
            anchor_source = "current"
        end
    end

    -- waiting for loc
    if base == nil then
        if now_ms - last_home_wait_ms >= 5000 then
            last_home_wait_ms = now_ms
            gcs:send_text(MAV_SEVERITY.WARNING, "KANG: waiting for home/location" .. anchor_source)
        end
        return false
    end

    anchor_loc = base:copy()
    anchor_loc:change_alt_frame(ALT_FRAME_ABSOLUTE)

    speed_mps = 0
    last_update_ms = now_ms

    -- all modes share the same random start offset from home
    -- offset for random walk
    local start_offset = math_helpers.clamp(KANG_OFS_M:get(), 0, 20000)
    -- heading angle - randomly generated
    local start_hdg = math_helpers.random_between(0, 360)
    -- orbit values
    orbit_north = math.cos(math.rad(start_hdg)) * start_offset
    orbit_east  = math.sin(math.rad(start_hdg)) * start_offset
    -- lateral displacement
    local disp_m = math_helpers.clamp(KANG_LAT_DISP:get(), -20000, 20000)
    -- forward offset
    local fwd_m = math_helpers.clamp(KANG_OFS_M:get(), 0, 20000)
    local mode = get_active_mode()

    -- static point
    if mode == "point" then
        local pt = heading_frame_offset(math_helpers.clamp(KANG_STR_HDG:get(), 0, 360), fwd_m, disp_m)
        target_north = pt.n
        target_east  = pt.e
    -- random walk
    elseif mode == "random" then
        heading_deg  = start_hdg
        target_north = orbit_north
        target_east  = orbit_east
        choose_next_segment(now_ms)
    -- straight mode
    elseif mode == "straight" then
        straight_heading_deg = math_helpers.clamp(KANG_STR_HDG:get(), 0, 360)
        local hdg_rad = math.rad(straight_heading_deg)
        target_north  = math.cos(hdg_rad) * fwd_m - math.sin(hdg_rad) * disp_m
        target_east  = math.sin(hdg_rad) * fwd_m + math.cos(hdg_rad) * disp_m
    -- circle mode
    elseif mode == "circle" then
        circle_angle_rad = 0
        -- local r = math_helpers.clamp(KANG_CIR_R:get(), 50, 3000)
        -- target_north = orbit_north + r
        -- target_east  = orbit_east

        local r = math_helpers.clamp(KANG_CIR_R:get(), 50, 3000)
        -- uses heading from the straight 
        local center = heading_frame_offset(KANG_STR_HDG:get(), fwd_m, disp_m)
        orbit_north = center.n
        orbit_east = center.e

        target_north = orbit_north + r
        target_east = orbit_east
    -- generate work
    elseif mode == "rectangle" then
        rect_side = 0
        rect_t    = 0.0
        local origin = heading_frame_offset(math_helpers.clamp(KANG_REC_HDG:get(), 0, 360), fwd_m, disp_m)
        orbit_north = origin.n
        orbit_east = origin.e
        local L  = math_helpers.clamp(KANG_REC_L:get(), 50, 3000)
        local W  = math_helpers.clamp(KANG_REC_W:get(), 50, 3000)
        local H  = math.rad(math_helpers.clamp(KANG_REC_HDG:get(), 0, 360))
        local ch = math.cos(H)
        local sh = math.sin(H)
        rect_corners = {
            { n = orbit_north, e = orbit_east},
            { n = orbit_north + L*ch, e = orbit_east + L*sh},
            { n = orbit_north + L*ch - W*sh, e = orbit_east + L*sh + W*ch},
            { n = orbit_north - W*sh, e = orbit_east + W*ch},
        }
        target_north = orbit_north
        target_east  = orbit_east
    end

    update_target_location()
    target_ready = true
    gcs:send_text(MAV_SEVERITY.WARNING, "KANG: virtual target initialised (" .. (mode or "none") .. ")")
    return true
end

local function integrate_random(now_ms, dt)
    if now_ms >= segment_end_ms then
        choose_next_segment(now_ms)
    end

    local vn = math.cos(math.rad(heading_deg)) * speed_mps
    local ve = math.sin(math.rad(heading_deg)) * speed_mps

    target_north = target_north + (vn * dt)
    target_east  = target_east  + (ve * dt)

    local bound_m  = math_helpers.clamp(KANG_BOUND_M:get(), 50, 3000)
    local radius_m = get_radius_m()

    if radius_m > bound_m then
        local scale = bound_m / radius_m
        target_north = target_north * scale
        target_east  = target_east  * scale
        heading_deg  = math_helpers.wrap_360(math.deg(math.atan(-target_east, -target_north)) + math_helpers.random_between(-25, 25))
        segment_end_ms = now_ms + 1000
        vn = math.cos(math.rad(heading_deg)) * speed_mps
        ve = math.sin(math.rad(heading_deg)) * speed_mps
    end

    target_vn = vn
    target_ve = ve
end

local function integrate_straight(dt)
    speed_mps   = math_helpers.clamp(KANG_SPD_MAX:get(), 1, 40)
    heading_deg = straight_heading_deg
    local vn = math.cos(math.rad(heading_deg)) * speed_mps
    local ve = math.sin(math.rad(heading_deg)) * speed_mps
    target_north = target_north + vn * dt
    target_east  = target_east  + ve * dt
    target_vn = vn
    target_ve = ve
end

local function integrate_circle(dt)
    local r   = math_helpers.clamp(KANG_CIR_R:get(), 50, 3000)
    speed_mps = math_helpers.clamp(KANG_SPD_MAX:get(), 1, 40)
    local omega   = speed_mps / r
    circle_angle_rad = circle_angle_rad + omega * dt
    target_north = orbit_north + r * math.cos(circle_angle_rad)
    target_east  = orbit_east  + r * math.sin(circle_angle_rad)
    target_vn    = -speed_mps * math.sin(circle_angle_rad)
    target_ve    =  speed_mps * math.cos(circle_angle_rad)
    heading_deg  = math_helpers.wrap_360(math.deg(circle_angle_rad) + 90)
end

local function integrate_rectangle(dt)
    if rect_corners == nil then return end
    speed_mps = math_helpers.clamp(KANG_SPD_MAX:get(), 1, 40)
    local function corner(i) return rect_corners[i % 4 + 1] end
    local s0 = corner(rect_side)
    local s1 = corner(rect_side + 1)
    local dn = s1.n - s0.n
    local de = s1.e - s0.e
    local side_len = math.sqrt(dn * dn + de * de)
    if side_len < 0.01 then return end

    rect_t = rect_t + (speed_mps * dt) / side_len
    if rect_t >= 1.0 then
        rect_side = (rect_side + 1) % 4
        rect_t    = rect_t - 1.0
        s0 = corner(rect_side)
        s1 = corner(rect_side + 1)
        dn = s1.n - s0.n
        de = s1.e - s0.e
        side_len = math.sqrt(dn * dn + de * de)
        if side_len < 0.01 then return end
    end

    target_north = s0.n + dn * rect_t
    target_east  = s0.e + de * rect_t
    target_vn    = (dn / side_len) * speed_mps
    target_ve    = (de / side_len) * speed_mps
    heading_deg  = math_helpers.wrap_360(math.deg(math.atan(de, dn)))
end

local function integrate_point()
    speed_mps  = 0
    target_vn  = 0
    target_ve  = 0
    heading_deg = 0
end

-- move the function forward in time
local function integrate_target(now_ms)
    if not target_ready then
        return
    end
    local dt = (now_ms - last_update_ms) * 0.001
    if dt < 0 then dt = 0 end
    if dt > 0.5 then dt = 0.5 end
    last_update_ms = now_ms

    local mode = get_active_mode()
    if     mode == "point"     then integrate_point()
    elseif mode == "random"    then integrate_random(now_ms, dt)
    elseif mode == "straight"  then integrate_straight(dt)
    elseif mode == "circle"    then integrate_circle(dt)
    elseif mode == "rectangle" then integrate_rectangle(dt)
    end

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
        MAV_SEVERITY.WARNING,
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

-- helper to handle bus function
local function publish_bus(now_ms)
    -- negative case
    if target_loc == nil then
        return false
    end

    --sequence counter
    bus_seq_counter = bus_seq_counter + 1
    local seq_even = bus_seq_counter * 2
    local seq_odd = seq_even - 1

    ---store timeframe
    local t_s = now_ms * 0.001

    -- lat lon in degrees
    local lat_deg = target_loc:lat() * 1.0e-7
    local lon_deg = target_loc:lng() * 1.0e-7
    --
    -- update write
    local ok = true
    ok = KBUS_SEQ:set(seq_odd) and ok
    ok = KBUS_T_S:set(t_s) and ok
    ok = KBUS_LAT:set(lat_deg) and ok
    ok = KBUS_LON:set(lon_deg) and ok
    ok = KBUS_VN:set(target_vn or 0) and ok
    ok = KBUS_VE:set(target_ve or 0) and ok
    ok = KBUS_SEQ:set(seq_even) and ok
    return ok
end

local function update()
    if get_active_mode() == nil then
        return
    end

    local now_ms = millis():toint()
    if not ensure_anchor(now_ms) then
        return
    end
    integrate_target(now_ms)
    publish_bus(now_ms)
    if target_loc ~= nil then
        send_ADSB_VEHICLE(
            target_loc:lat() * 1.0e-7,
            target_loc:lng() * 1.0e-7,
            target_loc:alt() * 0.01,
            heading_deg,
            speed_mps,
            0)
    end
    report_target(now_ms)
end

math.randomseed(millis():toint())
math.random()
math.random()

-- 
gcs:send_text(MAV_SEVERITY.WARNING, "KANG: loaded, switch Plane to GUIDED after takeoff")

-- return wrapped
local function protected_return()
    local success, err = pcall(update)
    if not success then
        -- error message
        gcs:send_text(MAV_SEVERITY.ERROR, "KANG: " .. err)
        -- return after a second
        return protected_return, 1000
    end
    return protected_return, 200
end

return protected_return()
