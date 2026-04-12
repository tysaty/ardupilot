-- kangaroo_first_dubins_logger.lua
-- Logs the first Dubins curve generated from the same inputs used by control.lua.

local SCRIPT_NAME = "kangaroo_first_dubins_logger.lua"
local ALT_FRAME_ABSOLUTE = 0
local MODE_AUTO = 10
local MODE_GUIDED = 15
local STALE_BUS_MS = 1000
local KANG_ALT_FALLBACK_M = 80.0
local PLANE_ABOVE_TARGET_M = 150.0

local MAV_SEVERITY = {
    EMERGENCY = 0,
    ALERT = 1,
    CRITICAL = 2,
    ERROR = 3,
    WARNING = 4,
    NOTICE = 5,
    INFO = 6,
    DEBUG = 7
}

local math_helpers = require("math_helpers")
local dubins_points = require("dubins_weave")

local bus_seq_param = Parameter()
local bus_seq_ready = bus_seq_param:init("SCR_USER1")
local bus_t_s_param = Parameter()
local bus_t_s_ready = bus_t_s_param:init("SCR_USER2")
local bus_lat_param = Parameter()
local bus_lat_ready = bus_lat_param:init("SCR_USER3")
local bus_lon_param = Parameter()
local bus_lon_ready = bus_lon_param:init("SCR_USER4")
local bus_vn_param = Parameter()
local bus_vn_ready = bus_vn_param:init("SCR_USER5")
local bus_ve_param = Parameter()
local bus_ve_ready = bus_ve_param:init("SCR_USER6")

local kang_alt_param = Parameter()
local kang_alt_ready = kang_alt_param:init("KANG_ALT_M")

local warned_bus_not_ready = false
local last_bus_seq_seen = -1
local logged_curve_once = false
local sample_file = nil
local sample_path = nil

local function as_number(value)
    if value == nil then
        return nil
    end
    if type(value) == "number" then
        return value
    end
    local ok, converted = pcall(function()
        return value:toint()
    end)
    if ok and converted ~= nil then
        return converted
    end
    ok, converted = pcall(function()
        return value:tofloat()
    end)
    if ok and converted ~= nil then
        return converted
    end
    return tonumber(tostring(value))
end

local function now_ms()
    return as_number(millis()) or 0
end

local function fmt(value, decimals)
    if value == nil then
        return ""
    end
    return string.format("%." .. tostring(decimals) .. "f", value)
end

local function all_bus_ready()
    return bus_seq_ready and bus_t_s_ready and bus_lat_ready and bus_lon_ready and bus_vn_ready and bus_ve_ready
end

local function get_log_dir()
    local stat = fs:stat("APM/logs")
    if stat and stat:is_directory() then
        return "APM/logs"
    end
    stat = fs:stat("logs")
    if stat and stat:is_directory() then
        return "logs"
    end
    return "scripts"
end

local function ensure_sample_file()
    if sample_file ~= nil then
        return true
    end

    local log_dir = get_log_dir()
    sample_path = string.format("%s/kangaroo_first_dubins_boot%u.csv", log_dir, now_ms())
    sample_file = io.open(sample_path, "w")
    if sample_file == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "KFDL: failed to open " .. sample_path)
        return false
    end

    sample_file:write(
        "time_ms,mode,bus_seq,bus_timestamp_ms,build_rho_m,build_target_distance_m,build_point_count,point_index,point_lat_deg,point_lon_deg,point_alt_m,point_rel_n_m,point_rel_e_m,point_heading_rad,plane_lat_deg,plane_lon_deg,plane_alt_m,kang_lat_deg,kang_lon_deg,kang_alt_m,kang_vn_mps,kang_ve_mps\n"
    )
    gcs:send_text(MAV_SEVERITY.INFO, "KFDL: logging to " .. sample_path)
    return true
end

local function get_home_alt_m()
    if ahrs:home_is_set() then
        local home = ahrs:get_home()
        if home ~= nil and home:alt() ~= nil then
            return home:alt() * 0.01
        end
    end

    local pos = ahrs:get_position()
    if pos ~= nil then
        pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
        local pos_alt_cm = pos:alt()
        if pos_alt_cm ~= nil then
            return pos_alt_cm * 0.01
        end
    end

    return nil
end

local function get_kangaroo_alt_m()
    if not kang_alt_ready then
        kang_alt_ready = kang_alt_param:init("KANG_ALT_M")
    end
    if kang_alt_ready then
        local alt_m = kang_alt_param:get()
        if alt_m ~= nil then
            return alt_m
        end
    end
    return KANG_ALT_FALLBACK_M
end

local function get_plane_sample()
    local pos = ahrs:get_position()
    if pos == nil then
        return nil, nil, nil
    end
    pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
    local lat = pos:lat()
    local lng = pos:lng()
    local alt = pos:alt()
    if lat == nil or lng == nil or alt == nil then
        return nil, nil, nil
    end
    return lat * 1.0e-7, lng * 1.0e-7, alt * 0.01
end

local function read_bus_target(now)
    if not all_bus_ready() then
        if not warned_bus_not_ready then
            warned_bus_not_ready = true
            gcs:send_text(MAV_SEVERITY.WARNING, "KFDL: bus params not ready")
        end
        return nil
    end

    local seq_1 = bus_seq_param:get()
    if seq_1 == nil or (seq_1 % 2) ~= 0 then
        return nil
    end

    local t_s = bus_t_s_param:get()
    local lat_deg = bus_lat_param:get()
    local lon_deg = bus_lon_param:get()
    local vn = bus_vn_param:get()
    local ve = bus_ve_param:get()
    if t_s == nil or lat_deg == nil or lon_deg == nil or vn == nil or ve == nil then
        return nil
    end

    local seq_2 = bus_seq_param:get()
    if seq_2 == nil or seq_1 ~= seq_2 or (seq_2 % 2) ~= 0 then
        return nil
    end
    if seq_2 == last_bus_seq_seen then
        return nil
    end

    local ts_ms = math.floor(t_s * 1000.0 + 0.5)
    local age_ms = now - ts_ms
    if age_ms < 0 or age_ms > STALE_BUS_MS then
        return nil
    end

    local loc = Location()
    loc:lat(math_helpers.deg_to_e7(lat_deg))
    loc:lng(math_helpers.deg_to_e7(lon_deg))

    last_bus_seq_seen = seq_2
    return {
        loc = loc,
        lat_deg = lat_deg,
        lon_deg = lon_deg,
        vn = vn,
        ve = ve,
        seq = seq_2,
        timestamp_ms = ts_ms
    }
end

local function write_curve(now, mode, kang, curve_points, build_info)
    local point_count = #curve_points
    local plane_lat, plane_lon, plane_alt = get_plane_sample()

    local kang_alt_m = nil
    local home_alt_m = get_home_alt_m()
    local kang_alt_above_home = get_kangaroo_alt_m()
    local commanded_curve_alt_m = nil
    if home_alt_m ~= nil then
        kang_alt_m = home_alt_m + kang_alt_above_home
        commanded_curve_alt_m = kang_alt_m + PLANE_ABOVE_TARGET_M
    end

    for i = 1, point_count do
        local point = curve_points[i]
        local loc = point.loc and point.loc:copy() or nil

        local point_lat_deg, point_lon_deg, point_alt_m = nil, nil, nil
        if loc ~= nil then
            loc:change_alt_frame(ALT_FRAME_ABSOLUTE)
            if commanded_curve_alt_m ~= nil then
                loc:set_alt_m(commanded_curve_alt_m, ALT_FRAME_ABSOLUTE)
            end

            local lat = loc:lat()
            local lon = loc:lng()
            local alt = loc:alt()

            if lat ~= nil then
                point_lat_deg = lat * 1.0e-7
            end
            if lon ~= nil then
                point_lon_deg = lon * 1.0e-7
            end
            if commanded_curve_alt_m ~= nil then
                point_alt_m = commanded_curve_alt_m
            elseif alt ~= nil then
                point_alt_m = alt * 0.01
            end
        end

        sample_file:write(
            table.concat(
                {
                    tostring(now),
                    tostring(mode or ""),
                    tostring(kang.seq or ""),
                    tostring(kang.timestamp_ms or ""),
                    fmt(build_info and build_info.rho_m or nil, 3),
                    fmt(build_info and build_info.target_distance_m or nil, 3),
                    tostring(build_info and build_info.point_count or point_count),
                    tostring(i),
                    fmt(point_lat_deg, 7),
                    fmt(point_lon_deg, 7),
                    fmt(point_alt_m, 2),
                    fmt(point and point.x or nil, 3),
                    fmt(point and point.y or nil, 3),
                    fmt(point and point.psi or nil, 6),
                    fmt(plane_lat, 7),
                    fmt(plane_lon, 7),
                    fmt(plane_alt, 2),
                    fmt(kang.lat_deg, 7),
                    fmt(kang.lon_deg, 7),
                    fmt(kang_alt_m, 2),
                    fmt(kang.vn, 3),
                    fmt(kang.ve, 3)
                },
                ","
            ) .. "\n"
        )
    end

    sample_file:flush()
    sample_file:close()
    sample_file = nil
end

local function update()
    if logged_curve_once then
        return update, 1000
    end

    local mode = vehicle:get_mode()
    if mode ~= MODE_AUTO and mode ~= MODE_GUIDED then
        return update, 100
    end

    local now = now_ms()
    local kang = read_bus_target(now)
    if kang == nil then
        return update, 100
    end

    if not ensure_sample_file() then
        return update, 1000
    end

    local curve_points, build_info = dubins_points.build_path(kang)
    if curve_points == nil or #curve_points == 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, "KFDL: build_path failed")
        return update, 200
    end

    write_curve(now, mode, kang, curve_points, build_info)
    logged_curve_once = true
    gcs:send_text(MAV_SEVERITY.INFO, string.format("KFDL: wrote first curve (%u pts) %s", #curve_points, sample_path))
    return update, 1000
end

local function protected_wrapper()
    local ok, fn, delay = pcall(update)
    if not ok then
        gcs:send_text(MAV_SEVERITY.ERROR, "KFDL: " .. tostring(fn))
        return protected_wrapper, 1000
    end
    return fn, delay
end

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME .. " loaded")
return protected_wrapper()
