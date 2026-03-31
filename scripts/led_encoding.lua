-- Encoding script
-- this code recieves data from the current ship
-- converts the code to binary, appends the binary
-- adds additional hamming encoding to the update

-- Configuration
local field_bits = 32
-- hamming code bits
local hc_data_bits = 4
-- update every 100 ms
local UPDATE_PERIOD_MS = 100
local LOG_WRITE_PERIOD_MS = 1000
local LOG_FILE_NAME = "led_encoding.txt"

local last_log_ms = 0
local log_file_initialised = false
local log_file_error_reported = false

-- print text
local function rcv_info()
    local loc = ahrs:get_location()
    -- nil case
    if loc == nil then
        return nil
    end
    -- lat lon units degE7
    -- alt units cm
    return {
        lat_degE7 = loc:lat(),
        lon_degE7 = loc:lng(),
        alt_cm = loc:alt(),
    }
end

-- convert a signed integer to a fixed-width bitstring
local function numberToBinary(n, numberbits)
    local res = ""
    for i = numberbits - 1, 0, -1 do
        local bit = (n >> i) & 1
        res = res .. tostring(bit)
    end
    return res
end

-- add Hamming(7,4) parity to a 4-bit data chunk
local function addHammingCode(data)
    local d = {}
    for i = 1, hc_data_bits do
        d[i] = tonumber(data:sub(i, i))
    end

    local p1 = (d[1] + d[2] + d[4]) % 2
    local p2 = (d[1] + d[3] + d[4]) % 2
    local p3 = (d[2] + d[3] + d[4]) % 2

    return tostring(p1) .. tostring(p2) .. tostring(d[1]) ..
           tostring(p3) .. tostring(d[2]) .. tostring(d[3]) .. tostring(d[4])
end

-- apply Hamming(7,4) across the entire concatenated bitstring
local function hamming74_encode_bits(bits)
    local out = {}
    for i = 1, #bits, hc_data_bits do
        local nibble = bits:sub(i, i + hc_data_bits - 1)
        if #nibble < hc_data_bits then
            nibble = nibble .. string.rep("0", hc_data_bits - #nibble)
        end
        out[#out + 1] = addHammingCode(nibble)
    end
    return table.concat(out)
end

local function ensure_log_file()
    if log_file_initialised then
        return true
    end

    local file = io.open(LOG_FILE_NAME, "a")
    if file == nil then
        if not log_file_error_reported then
            gcs:send_text(4, "led_encoding: unable to open log file")
            log_file_error_reported = true
        end
        return false
    end

    file:write(string.format("# led encoding session start %u\n", millis()))
    file:write("time_ms,lat_degE7,lon_degE7,alt_cm,raw_bits,hamming74_bits\n")
    file:close()
    log_file_initialised = true
    return true
end

local function append_log(time_ms, lat, lon, alt, raw_bits, encoded_bits)
    if not ensure_log_file() then
        return
    end

    local file = io.open(LOG_FILE_NAME, "a")
    if file == nil then
        if not log_file_error_reported then
            gcs:send_text(4, "led_encoding: unable to append log file")
            log_file_error_reported = true
        end
        return
    end

    file:write(string.format("%u,%d,%d,%d,%s,%s\n", time_ms, lat, lon, alt, raw_bits, encoded_bits))
    file:close()
end

-- voltage on and off

local finalstring = ""
local rawstring = ""

local function update()
    -- recieve info
    local info = rcv_info()
    if info == nil then
        return update, UPDATE_PERIOD_MS
    end
    local lat = info.lat_degE7
    local lon = info.lon_degE7
    local alt = info.alt_cm

    -- convert to binary and append
    local LLA_binary = numberToBinary(lat, field_bits) ..
                       numberToBinary(lon, field_bits) ..
                       numberToBinary(alt, field_bits)

    -- convert to Hamming encoding
    rawstring = LLA_binary
    finalstring = hamming74_encode_bits(LLA_binary)

    local now_ms = millis()
    if now_ms - last_log_ms >= LOG_WRITE_PERIOD_MS then
        append_log(now_ms, lat, lon, alt, rawstring, finalstring)
        last_log_ms = now_ms
    end

    return update, UPDATE_PERIOD_MS
end

return update()
