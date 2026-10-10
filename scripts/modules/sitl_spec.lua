-- =========================================================
--  sitl_spec -- read spec.json, the one configuration the campaign runner,
--  the live demonstration and the hardware validation script share
--  created 2026-10-04
--
--  spec.json is GENERATED, never typed: kangaroo_follow/schedule.py
--  (spec_cfg / write_spec) flattens a Python campaign cell's spec.json into
--  it -- the HarnessConfig every arm reads (orbit_radius_m, look_ahead_m,
--  turn_radius_m, bank_limit_deg, dt_s, ...) and, at the same level, the run
--  fields (algorithm, estimate, lookahead_steps, estimator, legs, geometry,
--  roll_limit_deg, and the demonstration's alt_m, bound_m, ... when staged
--  by demo.py). One flat table: every script calls it `cfg` and passes the
--  same table to the arm, entry(snapshot, cfg).
--
--  Where it is read from: the scripting directory, ./scripts in SITL and
--  /APM/scripts on a ChibiOS flight controller (AP_Scripting
--  lua_common_defs.h). M.paths lists the candidates in order; a test may
--  replace it before the first load.
--
--  roll_limit_deg is the bank limit the geometry was generated
--  for. M.roll_limit_check compares it with the live ROLL_LIMIT_DEG; SITL
--  staging sets the parameter to it, the hardware script refuses to engage
--  on a mismatch and never sets the parameter itself (it is a flight limit).
--
--  The JSON reader below covers what json.dump writes for a spec: objects,
--  arrays, strings (with the standard escapes), numbers, true, false and
--  null (null reads as absent). It is not a general JSON library.
-- =========================================================

local M = {}

M.FILE = "spec.json"
M.paths = { "./scripts/" .. M.FILE, "/APM/scripts/" .. M.FILE, "APM/scripts/" .. M.FILE }

-- ---------------------------------------------------------
-- JSON (decode only)
-- ---------------------------------------------------------
local ESCAPES = { ['"'] = '"', ["\\"] = "\\", ["/"] = "/", b = "\b",
                  f = "\f", n = "\n", r = "\r", t = "\t" }

local decode_value

local function skip_ws(s, i)
    local _, e = string.find(s, "^[ \t\r\n]*", i)
    return e + 1
end

local function decode_string(s, i)
    -- s:sub(i, i) == '"'
    local out = {}
    local j = i + 1
    while true do
        local c = string.sub(s, j, j)
        if c == "" then
            error("unterminated string at " .. i)
        elseif c == '"' then
            return table.concat(out), j + 1
        elseif c == "\\" then
            local e = string.sub(s, j + 1, j + 1)
            if e == "u" then
                local hex = string.sub(s, j + 2, j + 5)
                local code = tonumber(hex, 16)
                if code == nil then error("bad \\u escape at " .. j) end
                if code < 128 then
                    out[#out + 1] = string.char(code)
                elseif utf8 ~= nil then
                    out[#out + 1] = utf8.char(code)
                else
                    error("non-ASCII \\u escape at " .. j .. " and no utf8 library")
                end
                j = j + 6
            elseif ESCAPES[e] ~= nil then
                out[#out + 1] = ESCAPES[e]
                j = j + 2
            else
                error("bad escape at " .. j)
            end
        else
            out[#out + 1] = c
            j = j + 1
        end
    end
end

local function decode_number(s, i)
    local num = string.match(s, "^-?%d+%.?%d*[eE]?[-+]?%d*", i)
    local v = num and tonumber(num)
    if v == nil then error("bad number at " .. i) end
    return v, i + #num
end

local function decode_array(s, i)
    local out = {}
    i = skip_ws(s, i + 1)
    if string.sub(s, i, i) == "]" then return out, i + 1 end
    while true do
        local v
        v, i = decode_value(s, i)
        out[#out + 1] = v
        i = skip_ws(s, i)
        local c = string.sub(s, i, i)
        if c == "]" then return out, i + 1 end
        if c ~= "," then error("expected , or ] at " .. i) end
        i = skip_ws(s, i + 1)
    end
end

local function decode_object(s, i)
    local out = {}
    i = skip_ws(s, i + 1)
    if string.sub(s, i, i) == "}" then return out, i + 1 end
    while true do
        if string.sub(s, i, i) ~= '"' then error("expected key at " .. i) end
        local k
        k, i = decode_string(s, i)
        i = skip_ws(s, i)
        if string.sub(s, i, i) ~= ":" then error("expected : at " .. i) end
        local v
        v, i = decode_value(s, skip_ws(s, i + 1))
        out[k] = v                  -- null decodes to nil: the key is absent
        i = skip_ws(s, i)
        local c = string.sub(s, i, i)
        if c == "}" then return out, i + 1 end
        if c ~= "," then error("expected , or } at " .. i) end
        i = skip_ws(s, i + 1)
    end
end

decode_value = function(s, i)
    i = skip_ws(s, i)
    local c = string.sub(s, i, i)
    if c == "{" then return decode_object(s, i) end
    if c == "[" then return decode_array(s, i) end
    if c == '"' then return decode_string(s, i) end
    if string.sub(s, i, i + 3) == "true" then return true, i + 4 end
    if string.sub(s, i, i + 4) == "false" then return false, i + 5 end
    if string.sub(s, i, i + 3) == "null" then return nil, i + 4 end
    return decode_number(s, i)
end

--- Decode a JSON document. Returns the value, or nil and a reason.
function M.decode(text)
    local ok, v, i = pcall(decode_value, text, 1)
    if not ok then return nil, tostring(v) end
    if skip_ws(text, i) <= #text then return nil, "trailing text at " .. i end
    return v
end

-- ---------------------------------------------------------
-- spec.json
-- ---------------------------------------------------------
local cached = nil

--- The shared configuration. Returns cfg, path; or nil and a reason. Read
--  once and cached: every script in the same scripting VM gets one table.
function M.load()
    if cached ~= nil then return cached, cached._path end
    local tried = {}
    for _, path in ipairs(M.paths) do
        local f = io.open(path, "r")
        if f ~= nil then
            local text = f:read("a")
            f:close()
            local cfg, why = M.decode(text or "")
            if cfg == nil then
                return nil, path .. ": " .. tostring(why)
            end
            if type(cfg) ~= "table" or cfg.algorithm == nil or cfg.dt_s == nil then
                return nil, path .. ": not a spec (no algorithm or dt_s)"
            end
            cfg._path = path
            cached = cfg
            return cfg, path
        end
        tried[#tried + 1] = path
    end
    return nil, "no " .. M.FILE .. " in " .. table.concat(tried, ", ")
end

--- Forget the cached table (tests only).
function M.reset()
    cached = nil
end

--- Whether ROLL_LIMIT_DEG matches the bank limit the spec was generated
--  for. Returns ok, live_deg, message.
M.ROLL_TOLERANCE_DEG = 0.5
function M.roll_limit_check(cfg)
    local want = cfg and cfg.roll_limit_deg
    local live = param:get("ROLL_LIMIT_DEG")
    if want == nil then
        return false, live, "spec has no roll_limit_deg"
    end
    if live == nil then
        return false, nil, "ROLL_LIMIT_DEG unreadable"
    end
    if math.abs(live - want) > M.ROLL_TOLERANCE_DEG then
        return false, live, string.format(
            "ROLL_LIMIT_DEG %.1f, spec %.1f: set it to the spec's", live, want)
    end
    return true, live, string.format("ROLL_LIMIT_DEG %.1f matches spec", live)
end

return M
