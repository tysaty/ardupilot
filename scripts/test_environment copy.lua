-- Test environment logger for ArduPilot SITL debugging

local SCRIPT_NAME = "test_environment.lua"
local SCRIPT_VERSION = "0.1.0"
local SAMPLE_INTERVAL_MS = 2000
local CONSOLE_INTERVAL_MS = 10000
local SCRIPTS_SCAN_INTERVAL_MS = 0
local ENABLE_DATAFLASH = false
local EVENT_LOG_FILE = "test_environment_events.log"
local SAMPLE_LOG_FILE = "test_environment_samples.csv"
local NO_DATA = -9999.0
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

-- Parameters captured at boot
local CONFIG_PARAM_NAMES = {
    "SYSID_THISMAV",
    "SCR_ENABLE",
    "SCR_HEAP_SIZE",
    "SIM_SPEEDUP",
    "SIM_WIND_SPD",
    "SIM_WIND_DIR",
    "SIM_WIND_TURB",
    "SIM_GPS1_ENABLE",
    "SIM_GPS_NUMSAT",
    "SIM_RC_FAIL",
    "SIM_ENGINE_FAIL",
    "SIM_BATT_VOLTAGE"
}

-- treats number as number or converts number to string
-- pcall handles errors
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
    converted = tonumber(tostring(value))
    return converted
end

-- establishes millisecond numebrs
local function now_ms()
    return as_number(millis()) or 0
end

-- runtime identifier
local RUN_ID = string.format("boot_%u", now_ms())

-- boolean to integer
local function bool_to_int(value)
    if value then
        return 1
    end
    return 0
end

-- numeric value
local function numeric_or_default(value)
    if value == nil then
        return NO_DATA
    end
    return value
end

-- handling date time
local function fmt_number(value, decimals)
    if value == nil then
        return ""
    end
    return string.format("%." .. tostring(decimals) .. "f", value)
end

-- handling parameter vlaue
local function fmt_param_value(value)
    if value == nil then
        return "nil"
    end
    if math.abs(value - math.floor(value)) < 0.000001 then
        return string.format("%d", value)
    end
    return string.format("%.6f", value)
end

-- csv escape value
local function csv_escape(value)
    local text = tostring(value or "")
    if string.find(text, "[,\"]") then
        text = string.gsub(text, "\"", "\"\"")
        return "\"" .. text .. "\""
    end
    return text
end

-- get scripts directory
local function get_scripts_dir()
    local stat = fs:stat("APM/scripts")
    if stat and stat:is_directory() then
        return "APM/scripts"
    end
    return "scripts"
end

-- get log directory
local function get_log_dir()
    local stat = fs:stat("APM/logs")
    if stat and stat:is_directory() then
        return "APM/logs"
    end
    stat = fs:stat("logs")
    if stat and stat:is_directory() then
        return "logs"
    end
    return get_scripts_dir()
end

-- establishing local values
local SCRIPTS_DIR = get_scripts_dir()
local LOG_DIR = get_log_dir()
local IS_SITL = (SCRIPTS_DIR == "scripts")
local EVENT_LOG_PATH = LOG_DIR .. "/" .. EVENT_LOG_FILE
local SAMPLE_LOG_PATH = LOG_DIR .. "/" .. SAMPLE_LOG_FILE
-- establishing file values
local event_file = nil
local sample_file = nil
local warned_event_file = false
local warned_sample_file = false
local warned_dataflash = false
-- initialising valoues
local last_sample_ms = 0
local last_console_ms = 0
local last_scripts_scan_ms = 0
local last_mode = nil
local last_armed = nil
local last_nav_index = nil
local last_scripts_signature = ""

-- error handling for 
local function ensure_event_file()
    if event_file then
        return true
    end
    event_file = io.open(EVENT_LOG_PATH, "a")
    if event_file then
        return true
    end
    if not warned_event_file then
        warned_event_file = true
        gcs:send_text(MAV_SEVERITY.ERROR, "TENV: failed to open " .. EVENT_LOG_PATH)
    end
    return false
end

local function ensure_sample_file()
    if sample_file then
        return true
    end
    local existing = fs:stat(SAMPLE_LOG_PATH)
    sample_file = io.open(SAMPLE_LOG_PATH, "a")
    if not sample_file then
        if not warned_sample_file then
            warned_sample_file = true
            gcs:send_text(MAV_SEVERITY.ERROR, "TENV: failed to open " .. SAMPLE_LOG_PATH)
        end
        return false
    end
    if existing == nil or as_number(existing:size()) == 0 then
        sample_file:write(
            "run_id,boot_ms,frame_type,mode,armed,likely_flying,flight_time_s,mission_count,mission_nav_index,"
                .. "mission_nav_id,lat_deg,lon_deg,alt_m,alt_frame,hagl_m,home_dist_m,roll_deg,pitch_deg,yaw_deg,"
                .. "groundspeed_ms,airspeed_ms,vel_n_ms,vel_e_ms,vel_d_ms,wind_n_ms,wind_e_ms,wind_d_ms,"
                .. "hud_throttle_pct,batt_v,batt_a,batt_pct,gps_status,gps_sats,gps_hdop,gps_speed_ms,"
                .. "target_dist_m,target_bearing_deg,target_lat_deg,target_lon_deg,target_alt_m,"
                .. "follow_sysid,follow_lat_deg,follow_lon_deg,follow_alt_m,follow_vn_ms,follow_ve_ms,follow_vd_ms,"
                .. "precland_lat_deg,precland_lon_deg,precland_alt_m,precland_vn_ms,precland_ve_ms\n"
        )
        sample_file:flush()
    end
    return true
end

local function write_event(message)
    if not ensure_event_file() then
        return
    end
    event_file:write(string.format("[%s][%u] %s\n", RUN_ID, now_ms(), message))
    event_file:flush()
end

local function list_lua_scripts()
    local entries = dirlist(SCRIPTS_DIR)
    local scripts = {}
    local signature_parts = {}
    if not entries then
        return scripts, ""
    end
    table.sort(entries, function(a, b)
        return string.lower(a) < string.lower(b)
    end)

    for _, name in ipairs(entries) do
        if string.match(string.lower(name), "%.lua$") then
            local path = SCRIPTS_DIR .. "/" .. name
            local stat = fs:stat(path)
            local crc = fs:crc32(path)
            local script_info = {
                name = name,
                size = stat and as_number(stat:size()) or 0,
                mtime = stat and as_number(stat:mtime()) or 0,
                crc32 = as_number(crc) or 0
            }
            table.insert(scripts, script_info)
            table.insert(signature_parts, string.format("%s:%u:%u", name, script_info.size, script_info.crc32))
        end
    end

    return scripts, table.concat(signature_parts, "|")
end

local function log_script_list(reason)
    local scripts, signature = list_lua_scripts()
    write_event(string.format("scripts reason=%s count=%u", reason, #scripts))

    for _, script_info in ipairs(scripts) do
        write_event(
            string.format(
                "script name=%s size=%u crc32=%08X mtime=%u",
                script_info.name,
                script_info.size,
                script_info.crc32,
                script_info.mtime
            )
        )
    end

    return signature
end

local function get_location_fields(location)
    if location == nil then
        return nil, nil, nil, nil
    end

    return location:lat() * 1.0e-7,
           location:lng() * 1.0e-7,
           location:alt() * 0.01,
           location:get_alt_frame()
end

local function get_home_distance_m()
    local home_vector = ahrs:get_relative_position_NED_home()
    if home_vector == nil then
        return nil
    end
    return home_vector:xy():length()
end

local function collect_sample()
    local sample = {}

    sample.boot_ms = now_ms()
    sample.frame_type = gcs:frame_type()
    sample.mode = vehicle:get_mode()
    sample.armed = bool_to_int(arming:is_armed())
    sample.likely_flying = bool_to_int(vehicle:get_likely_flying())
    sample.flight_time_s = (as_number(vehicle:get_time_flying_ms()) or 0) * 0.001
    sample.mission_count = mission:num_commands()
    sample.mission_nav_index = mission:get_current_nav_index()
    sample.mission_nav_id = mission:get_current_nav_id()

    local location = ahrs:get_location()
    sample.lat_deg, sample.lon_deg, sample.alt_m, sample.alt_frame = get_location_fields(location)
    sample.hagl_m = ahrs:get_hagl()
    sample.home_dist_m = get_home_distance_m()
    sample.roll_deg = math.deg(ahrs:get_roll_rad())
    sample.pitch_deg = math.deg(ahrs:get_pitch_rad())
    sample.yaw_deg = math.deg(ahrs:get_yaw_rad())
    sample.airspeed_ms = ahrs:airspeed_EAS()
    sample.hud_throttle_pct = gcs:get_hud_throttle()

    local groundspeed = ahrs:groundspeed_vector()
    if groundspeed then
        sample.groundspeed_ms = groundspeed:length()
    end

    local velocity = ahrs:get_velocity_NED()
    if velocity then
        sample.vel_n_ms = velocity:x()
        sample.vel_e_ms = velocity:y()
        sample.vel_d_ms = velocity:z()
    end

    local wind = ahrs:wind_estimate()
    if wind then
        sample.wind_n_ms = wind:x()
        sample.wind_e_ms = wind:y()
        sample.wind_d_ms = wind:z()
    end

    if battery:healthy(0) then
        sample.batt_v = battery:voltage(0)
        sample.batt_a = battery:current_amps(0)
        sample.batt_pct = battery:capacity_remaining_pct(0)
    end

    local gps_instance = gps:primary_sensor()
    if gps_instance == nil then
        gps_instance = 0
    end
    sample.gps_status = gps:status(gps_instance)
    sample.gps_sats = gps:num_sats(gps_instance)
    sample.gps_hdop = gps:get_hdop(gps_instance)
    sample.gps_speed_ms = gps:ground_speed(gps_instance)

    local target_location = vehicle:get_target_location()
    sample.target_dist_m = vehicle:get_wp_distance_m()
    sample.target_bearing_deg = vehicle:get_wp_bearing_deg()
    sample.target_lat_deg, sample.target_lon_deg, sample.target_alt_m = get_location_fields(target_location)

    if follow:have_target() then
        local follow_location, follow_velocity = follow:get_target_location_and_velocity()
        sample.follow_sysid = as_number(follow:get_target_sysid())
        sample.follow_lat_deg, sample.follow_lon_deg, sample.follow_alt_m = get_location_fields(follow_location)
        if follow_velocity then
            sample.follow_vn_ms = follow_velocity:x()
            sample.follow_ve_ms = follow_velocity:y()
            sample.follow_vd_ms = follow_velocity:z()
        end
    end

    if precland:target_acquired() then
        local precland_location = precland:get_target_location()
        local precland_velocity = precland:get_target_velocity()
        sample.precland_lat_deg, sample.precland_lon_deg, sample.precland_alt_m = get_location_fields(precland_location)
        if precland_velocity then
            sample.precland_vn_ms = precland_velocity:x()
            sample.precland_ve_ms = precland_velocity:y()
        end
    end

    return sample
end

local function write_sample_csv(sample)
    if not ensure_sample_file() then
        return
    end

    local fields = {
        RUN_ID,
        sample.boot_ms,
        sample.frame_type,
        sample.mode,
        sample.armed,
        sample.likely_flying,
        fmt_number(sample.flight_time_s, 3),
        sample.mission_count,
        sample.mission_nav_index,
        sample.mission_nav_id,
        fmt_number(sample.lat_deg, 7),
        fmt_number(sample.lon_deg, 7),
        fmt_number(sample.alt_m, 2),
        sample.alt_frame,
        fmt_number(sample.hagl_m, 2),
        fmt_number(sample.home_dist_m, 2),
        fmt_number(sample.roll_deg, 2),
        fmt_number(sample.pitch_deg, 2),
        fmt_number(sample.yaw_deg, 2),
        fmt_number(sample.groundspeed_ms, 2),
        fmt_number(sample.airspeed_ms, 2),
        fmt_number(sample.vel_n_ms, 2),
        fmt_number(sample.vel_e_ms, 2),
        fmt_number(sample.vel_d_ms, 2),
        fmt_number(sample.wind_n_ms, 2),
        fmt_number(sample.wind_e_ms, 2),
        fmt_number(sample.wind_d_ms, 2),
        fmt_number(sample.hud_throttle_pct, 0),
        fmt_number(sample.batt_v, 2),
        fmt_number(sample.batt_a, 2),
        fmt_number(sample.batt_pct, 0),
        sample.gps_status,
        sample.gps_sats,
        fmt_number(sample.gps_hdop, 2),
        fmt_number(sample.gps_speed_ms, 2),
        fmt_number(sample.target_dist_m, 2),
        fmt_number(sample.target_bearing_deg, 2),
        fmt_number(sample.target_lat_deg, 7),
        fmt_number(sample.target_lon_deg, 7),
        fmt_number(sample.target_alt_m, 2),
        sample.follow_sysid,
        fmt_number(sample.follow_lat_deg, 7),
        fmt_number(sample.follow_lon_deg, 7),
        fmt_number(sample.follow_alt_m, 2),
        fmt_number(sample.follow_vn_ms, 2),
        fmt_number(sample.follow_ve_ms, 2),
        fmt_number(sample.follow_vd_ms, 2),
        fmt_number(sample.precland_lat_deg, 7),
        fmt_number(sample.precland_lon_deg, 7),
        fmt_number(sample.precland_alt_m, 2),
        fmt_number(sample.precland_vn_ms, 2),
        fmt_number(sample.precland_ve_ms, 2)
    }

    for i = 1, #fields do
        fields[i] = csv_escape(fields[i])
    end

    sample_file:write(table.concat(fields, ",") .. "\n")
    sample_file:flush()
end

local function write_dataflash(sample)
    if not ENABLE_DATAFLASH then
        return
    end

    local ok = pcall(function()
        logger:write(
            "TENV",
            "frame,mode,arm,fly,gs,air,alt,hagl,thr,batv,bata,sats",
            "ffffffffffff",
            numeric_or_default(sample.frame_type),
            numeric_or_default(sample.mode),
            numeric_or_default(sample.armed),
            numeric_or_default(sample.likely_flying),
            numeric_or_default(sample.groundspeed_ms),
            numeric_or_default(sample.airspeed_ms),
            numeric_or_default(sample.alt_m),
            numeric_or_default(sample.hagl_m),
            numeric_or_default(sample.hud_throttle_pct),
            numeric_or_default(sample.batt_v),
            numeric_or_default(sample.batt_a),
            numeric_or_default(sample.gps_sats)
        )

        logger:write(
            "TEN2",
            "tgt_d,tgt_b,wind_n,wind_e,wind_d,vn,ve,vd,hdop",
            "fffffffff",
            numeric_or_default(sample.target_dist_m),
            numeric_or_default(sample.target_bearing_deg),
            numeric_or_default(sample.wind_n_ms),
            numeric_or_default(sample.wind_e_ms),
            numeric_or_default(sample.wind_d_ms),
            numeric_or_default(sample.vel_n_ms),
            numeric_or_default(sample.vel_e_ms),
            numeric_or_default(sample.vel_d_ms),
            numeric_or_default(sample.gps_hdop)
        )
    end)

    if not ok and not warned_dataflash then
        warned_dataflash = true
        gcs:send_text(MAV_SEVERITY.WARNING, "TENV: DataFlash custom logging disabled")
    end
end

local function send_console_summary(sample)
    if CONSOLE_INTERVAL_MS <= 0 then
        return
    end

    gcs:send_text(
        MAV_SEVERITY.INFO,
        string.format(
            "TENV mode=%d arm=%d fly=%d alt=%.1f gs=%.1f bat=%.1fV sats=%d",
            sample.mode or -1,
            sample.armed or 0,
            sample.likely_flying or 0,
            sample.alt_m or NO_DATA,
            sample.groundspeed_ms or NO_DATA,
            sample.batt_v or NO_DATA,
            sample.gps_sats or -1
        )
    )

    local target_desc = nil
    if sample.follow_sysid ~= nil then
        target_desc = string.format(
            "TENV follow=%d vn=%.1f ve=%.1f vd=%.1f",
            sample.follow_sysid,
            sample.follow_vn_ms or NO_DATA,
            sample.follow_ve_ms or NO_DATA,
            sample.follow_vd_ms or NO_DATA
        )
    elseif sample.precland_lat_deg ~= nil then
        target_desc = string.format(
            "TENV precland lat=%.5f lon=%.5f vn=%.1f ve=%.1f",
            sample.precland_lat_deg,
            sample.precland_lon_deg,
            sample.precland_vn_ms or NO_DATA,
            sample.precland_ve_ms or NO_DATA
        )
    elseif sample.target_dist_m ~= nil or sample.target_bearing_deg ~= nil then
        target_desc = string.format(
            "TENV tgt_d=%.1f tgt_b=%.1f wind=%.1f/%.1f",
            sample.target_dist_m or NO_DATA,
            sample.target_bearing_deg or NO_DATA,
            sample.wind_n_ms or NO_DATA,
            sample.wind_e_ms or NO_DATA
        )
    end

    if target_desc then
        gcs:send_text(MAV_SEVERITY.INFO, target_desc)
    end
end

local function log_state_changes(sample)
    if last_mode == nil or sample.mode ~= last_mode then
        write_event(string.format("state mode=%s", tostring(sample.mode)))
        last_mode = sample.mode
    end

    if last_armed == nil or sample.armed ~= last_armed then
        write_event(string.format("state armed=%d flying=%d", sample.armed or 0, sample.likely_flying or 0))
        last_armed = sample.armed
    end

    if last_nav_index == nil or sample.mission_nav_index ~= last_nav_index then
        write_event(
            string.format(
                "state nav_index=%s nav_id=%s tgt_dist=%.2f",
                tostring(sample.mission_nav_index),
                tostring(sample.mission_nav_id),
                sample.target_dist_m or NO_DATA
            )
        )
        last_nav_index = sample.mission_nav_index
    end
end

-- configuration of snapshot
local function log_config_snapshot()
    local config_values = {}

    for _, param_name in ipairs(CONFIG_PARAM_NAMES) do
        local value = param:get(param_name)
        if value ~= nil then
            table.insert(config_values, string.format("%s=%s", param_name, fmt_param_value(value)))
        end
    end

    write_event(
        string.format(
            "startup version=%s scripts_dir=%s log_dir=%s sample_ms=%u console_ms=%u scan_ms=%u dataflash=%d",
            SCRIPT_VERSION,
            SCRIPTS_DIR,
            LOG_DIR,
            SAMPLE_INTERVAL_MS,
            CONSOLE_INTERVAL_MS,
            SCRIPTS_SCAN_INTERVAL_MS,
            bool_to_int(ENABLE_DATAFLASH)
        )
    )

    write_event(
        string.format(
            "startup frame=%s mode=%s armed=%d mission_count=%s sample_log=%s",
            tostring(gcs:frame_type()),
            tostring(vehicle:get_mode()),
            bool_to_int(arming:is_armed()),
            tostring(mission:num_commands()),
            SAMPLE_LOG_PATH
        )
    )

    if #config_values > 0 then
        write_event("config " .. table.concat(config_values, " "))
    end

    local home = ahrs:get_home()
    local home_lat, home_lon, home_alt = get_location_fields(home)
    if home_lat ~= nil then
        write_event(
            string.format("home lat=%.7f lon=%.7f alt=%.2f", home_lat, home_lon, home_alt or NO_DATA)
        )
    end
end

-- at start sending - initialising, sending text to MAVLINK
local function startup()
    ensure_event_file()
    ensure_sample_file()
    write_event("run_start")
    log_config_snapshot()
    last_scripts_signature = log_script_list("startup")
    gcs:send_text(MAV_SEVERITY.INFO, "TENV: logging to " .. SAMPLE_LOG_PATH)
end

-- update function
local function update()
    local current_ms = now_ms()
    -- change in sample
    if SCRIPTS_SCAN_INTERVAL_MS > 0 and current_ms - last_scripts_scan_ms >= SCRIPTS_SCAN_INTERVAL_MS then
        local _, signature = list_lua_scripts()
        if signature ~= last_scripts_signature then
            last_scripts_signature = log_script_list("changed")
        end
        last_scripts_scan_ms = current_ms
    end
    -- collect sample
    local sample = collect_sample()
    log_state_changes(sample)
    -- write to sample log
    if current_ms - last_sample_ms >= SAMPLE_INTERVAL_MS then
        write_sample_csv(sample)
        write_dataflash(sample)
        last_sample_ms = current_ms
    end
    --
    if current_ms - last_console_ms >= CONSOLE_INTERVAL_MS then
        send_console_summary(sample)
        last_console_ms = current_ms
    end

    return update, 200
end

-- if not IS_SITL then
--     gcs:send_text(MAV_SEVERITY.INFO, "TENV: SITL-only, inactive on hardware")
--     local function inactive()
--         return inactive, 60000
--     end
--     return inactive, 60000
-- end

startup()
return update, 200
