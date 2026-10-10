-- =========================================================
--  sitl_adsb -- show the virtual kangaroo on a ground station map
--  created 2026-09-24
--
--  The SITL runner and the demonstration script evaluate the kangaroo inside
--  the vehicle's Lua, so nothing outside the log knows where it is. This
--  module broadcasts it as an ADSB_VEHICLE message (id 246), which MAVProxy's
--  map and other ground stations draw as a traffic icon. The framing is the
--  one kangaroo_MAV.lua used (send_ADSB_VEHICLE), lifted here unchanged so
--  the target looks the same as it did under the mid-term scripts.
--
--  Display only: nothing reads the message back, and no guidance quantity
--  depends on it.
-- =========================================================

local M = {}

local ADSB_VEHICLE_MSGID = 246

-- ADSB_FLAGS bitmask values
local FLAGS = 1      -- VALID_COORDS
            + 2      -- VALID_ALTITUDE
            + 4      -- VALID_HEADING
            + 8      -- VALID_VELOCITY
            + 16     -- VALID_CALLSIGN
            + 64     -- SIMULATED
            + 128    -- VERTICAL_VELOCITY_VALID

--- ICAO address and callsign as kangaroo_MAV.lua set them.
M.ICAO_ADDR = 0xCAFE00
M.CALLSIGN = "KANGAROO"

--- Squawk code and emitter type fields, as kangaroo_MAV.lua sent them.
local SQUAWK = 1200
local ALTITUDE_TYPE = 0      -- ADSB_ALTITUDE_TYPE_PRESSURE_QNH
local EMITTER_TYPE = 1       -- ADSB_EMITTER_TYPE_LIGHT

--- MAVLink channels the message is sent on (SITL exposes up to six).
local CHANNELS = 6

local initialised = false

--- Pack one ADSB_VEHICLE payload (MAVLink wire order: largest fields first).
function M.pack(lat_deg, lng_deg, alt_amsl_m, heading_deg, speed_ms, vspeed_ms)
    local cs = string.sub(M.CALLSIGN .. string.rep("\0", 9), 1, 9)
    return string.pack("<I4i4i4i4 I2I2i2I2I2 Bc9BB",
        M.ICAO_ADDR,
        math.floor(lat_deg * 1e7),
        math.floor(lng_deg * 1e7),
        math.floor(alt_amsl_m * 1000),
        math.floor((heading_deg % 360.0) * 100),
        math.floor(speed_ms * 100),
        math.floor(vspeed_ms * 100),
        FLAGS,
        SQUAWK,
        ALTITUDE_TYPE,
        cs,
        EMITTER_TYPE,
        0)
end

--- Send the kangaroo at `loc` (an ArduPilot Location) moving at (vn, ve) m/s.
--  Heading is the direction of motion, clockwise from North; a stationary
--  target reports heading 0.
function M.send(loc, vn_ms, ve_ms)
    if not initialised then
        mavlink:init(1, 0)          -- transmit only, as kangaroo_MAV.lua
        initialised = true
    end
    local speed = math.sqrt(vn_ms * vn_ms + ve_ms * ve_ms)
    local heading = 0.0
    if speed > 0.01 then
        heading = math.deg(math.atan(ve_ms, vn_ms))
    end
    local payload = M.pack(loc:lat() * 1.0e-7, loc:lng() * 1.0e-7,
                           loc:alt() * 0.01, heading, speed, 0.0)
    for chan = 0, CHANNELS - 1 do
        mavlink:send_chan(chan, ADSB_VEHICLE_MSGID, payload)
    end
end

return M
