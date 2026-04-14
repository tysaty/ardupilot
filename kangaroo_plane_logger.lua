-- kangaroo_plane_logger.lua
-- Logs plane and kangaroo target positions over time for offline 3D analysis.

local SCRIPT_NAME = "kangaroo_plane_logger.lua"
local ALT_FRAME_ABSOLUTE = 0
local SAMPLE_INTERVAL_MS = 500
local CONSOLE_INTERVAL_MS = 5000
local STALE_BUS_MS = 1000
local KANG_ALT_FALLBACK_M = 80.0
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
local sample_file = nil
local sample_path = nil
local last_sample_ms = 0
local last_console_ms = 0

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
    sample_path = string.format("%s/kangaroo_plane_trace_boot%u.csv", log_dir, now_ms())
    sample_file = io.open(sample_path, "w")
    if sample_file == nil then
        gcs:send_text(MAV_SEVERITY.ERROR, "KPL: failed to open " .. sample_path)
        return false
    end
    sample_file:write(
        "time_ms,mode,plane_lat_deg,plane_lon_deg,plane_alt_m,kang_lat_deg,kang_lon_deg,kang_alt_m,kang_vn_mps,kang_ve_mps,bus_seq,bus_timestamp_ms,bus_age_ms,bus_fresh\n"
    )
    gcs:send_text(MAV_SEVERITY.INFO, "KPL: logging to " .. sample_path)
    return true
end

local function get_home_alt_m()
    if ahrs:home_is_set() then
        local home = ahrs:get_home()
        if home ~= nil and home:alt() ~= nil then
            return home:alt() * 0.01
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
            gcs:send_text(MAV_SEVERITY.WARNING, "KPL: bus params not ready")
        end
        return nil
    end

    local seq_1 = bus_seq_param:get()
    if seq_1 == nil then
        return nil
    end
    if (seq_1 % 2) ~= 0 then
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
    if seq_2 == nil then
        return nil
    end
    if seq_1 ~= seq_2 or (seq_2 % 2) ~= 0 then
        return nil
    end

    local ts_ms = math.floor(t_s * 1000.0 + 0.5)
    local age_ms = now - ts_ms
    local fresh = (age_ms >= 0 and age_ms <= STALE_BUS_MS) and 1 or 0

    local home_alt = get_home_alt_m()
    local kang_alt = nil
    if home_alt ~= nil then
        kang_alt = home_alt + get_kangaroo_alt_m()
    end

    return {
        lat_deg = lat_deg,
        lon_deg = lon_deg,
        alt_m = kang_alt,
        vn = vn,
        ve = ve,
        seq = seq_2,
        timestamp_ms = ts_ms,
        age_ms = age_ms,
        fresh = fresh
    }
end

local function write_sample()
    local now = now_ms()
    if now - last_sample_ms < SAMPLE_INTERVAL_MS then
        return
    end
    last_sample_ms = now

    if not ensure_sample_file() then
        return
    end

    local mode = vehicle:get_mode()
    local plane_lat, plane_lon, plane_alt = get_plane_sample()
    local kang = read_bus_target(now)

    sample_file:write(
        table.concat(
            {
                tostring(now),
                tostring(mode or ""),
                fmt(plane_lat, 7),
                fmt(plane_lon, 7),
                fmt(plane_alt, 2),
                fmt(kang and kang.lat_deg or nil, 7),
                fmt(kang and kang.lon_deg or nil, 7),
                fmt(kang and kang.alt_m or nil, 2),
                fmt(kang and kang.vn or nil, 3),
                fmt(kang and kang.ve or nil, 3),
                tostring(kang and kang.seq or ""),
                tostring(kang and kang.timestamp_ms or ""),
                tostring(kang and kang.age_ms or ""),
                tostring(kang and kang.fresh or 0)
            },
            ","
        ) .. "\n"
    )

    if now - last_console_ms >= CONSOLE_INTERVAL_MS then
        last_console_ms = now
        gcs:send_text(
            MAV_SEVERITY.INFO,
            string.format("KPL: sample t=%u mode=%s bus=%s", now, tostring(mode or "nil"), tostring(kang ~= nil))
        )
    end
end

local function update()
    write_sample()
    return update, 100
end

local function protected_wrapper()
    local ok, err = pcall(update)
    if not ok then
        gcs:send_text(MAV_SEVERITY.ERROR, "KPL: " .. tostring(err))
        return protected_wrapper, 1000
    end
    return protected_wrapper, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "KPL: loaded at boot")
return protected_wrapper()
