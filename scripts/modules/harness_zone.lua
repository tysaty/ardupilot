-- =========================================================
--  harness_zone -- the kangaroo's containment in the flight-test fence
--  created 2026-10-07 (ADR-012)
--
--  Port of py_harness/zone.py PolygonZone and its containment rule
--  (InclusionZone.containment_heading_deg, as ScenarioSession._contain_target
--  applies it). The demo (kangaroo_demo.lua) and the hardware validation
--  script (hardware_val.lua) call it every tick, so the kangaroo is turned
--  back at the fence by the same rule the Python harness applies; the SITL
--  campaign replays the Python run's turns (legs_flown) instead.
--
--  The rule: every wall is moved `margin_m` inward (the orbit radius by
--  default, so the ring stays inside the fence); when the next step along the
--  leg's heading would leave that inset region, the heading is reflected off
--  every inset wall the kangaroo is on or beyond while moving outward, and the
--  rest of the schedule is replaced by one leg on the new heading
--  (turned_leg). Convex fences only: polygon() refuses a concave one.
--
--  Gated against the Python by tests/unit/test_lua_differential.py
--  (Tranche 10). Frame: (north, east) metres. Stateless.
-- =========================================================

local M = {}

--- Duration of the leg a containment turn starts, s: ScenarioSession's
--  apply_change default.
M.TURN_LEG_S = 3600.0

--- A convex polygon from {{n, e}, ...} (either winding, not closed).
--  Returns the zone table, or nil and a reason.
function M.polygon(vertices)
    local verts = {}
    for i = 1, #vertices do
        local v = vertices[i]
        local n, e = v[1], v[2]
        if type(n) ~= "number" or type(e) ~= "number"
           or n ~= n or e ~= e or n == math.huge or n == -math.huge
           or e == math.huge or e == -math.huge then
            return nil, "polygon vertex " .. i .. " is not a finite number pair"
        end
        verts[#verts + 1] = { n, e }
    end
    local count = #verts
    if count >= 2 and verts[1][1] == verts[count][1] and verts[1][2] == verts[count][2] then
        verts[count] = nil
        count = count - 1
    end
    if count < 3 then
        return nil, "a polygon zone needs at least 3 vertices, got " .. count
    end
    local area2 = 0.0
    for i = 1, count do
        local a, b = verts[i], verts[i % count + 1]
        area2 = area2 + a[2] * b[1] - b[2] * a[1]
    end
    if math.abs(area2) < 1e-6 then
        return nil, "polygon zone has no area"
    end
    if area2 < 0.0 then
        local rev = {}
        for i = count, 1, -1 do rev[#rev + 1] = verts[i] end
        verts = rev
    end
    local normals, offsets = {}, {}
    for i = 1, count do
        local a, b, c = verts[i], verts[i % count + 1], verts[(i + 1) % count + 1]
        local dn, de = b[1] - a[1], b[2] - a[2]
        local len = math.sqrt(dn * dn + de * de)
        if len < 1e-6 then
            return nil, "polygon zone has a repeated vertex"
        end
        local cross = de * (c[1] - b[1]) - dn * (c[2] - b[2])
        local next_len = math.sqrt((c[1] - b[1]) ^ 2 + (c[2] - b[2]) ^ 2)
        if cross <= 1e-9 * len * math.max(next_len, 1.0) then
            return nil, "polygon zone must be convex with no collinear vertices"
        end
        local nn, ne = de / len, -dn / len
        normals[i] = { nn, ne }
        offsets[i] = nn * a[1] + ne * a[2]
    end
    return { vertices = verts, normals = normals, offsets = offsets }
end

--- Signed distance inside each wall, m (negative = outside it).
function M.inward_distances(z, n, e)
    local out = {}
    for i = 1, #z.normals do
        local w = z.normals[i]
        out[i] = w[1] * n + w[2] * e - z.offsets[i]
    end
    return out
end

--- True when (n, e) is inside with every wall moved margin_m in.
function M.contains(z, n, e, margin_m)
    local d = M.inward_distances(z, n, e)
    for i = 1, #d do
        if d[i] < margin_m then return false end
    end
    return true
end

--- True when one step along heading_deg would leave the inset region.
function M.would_exit(z, n, e, heading_deg, speed_ms, dt_s, margin_m)
    local step = speed_ms * dt_s
    local h = math.rad(heading_deg)
    return not M.contains(z, n + step * math.cos(h), e + step * math.sin(h), margin_m)
end

--- Heading reflected off each inset wall the point is on or beyond while
--  moving outward, in wall order, degrees [0, 360).
function M.reflect_heading_deg(z, n, e, heading_deg, margin_m)
    local h = math.rad(heading_deg)
    local vn, ve = math.cos(h), math.sin(h)
    local d = M.inward_distances(z, n, e)
    for i = 1, #d do
        local w = z.normals[i]
        local along = vn * w[1] + ve * w[2]
        if d[i] <= margin_m and along < 0.0 then
            vn = vn - 2.0 * along * w[1]
            ve = ve - 2.0 * along * w[2]
        end
    end
    return math.deg(math.atan(ve, vn)) % 360.0
end

--- The heading a containment turn commands now, or nil for no turn: the
--  whole of InclusionZone.containment_heading_deg.
function M.contain_heading(z, n, e, heading_deg, speed_ms, dt_s, margin_m)
    if speed_ms <= 0.0 then
        return nil
    end
    if not M.would_exit(z, n, e, heading_deg, speed_ms, dt_s, margin_m) then
        return nil
    end
    local turned = M.reflect_heading_deg(z, n, e, heading_deg, margin_m)
    if math.abs((turned - heading_deg + 180.0) % 360.0 - 180.0) < 1e-9 then
        return nil
    end
    return turned
end

--- The single leg a turn replaces the rest of the schedule with: the leg in
--  force on the new heading, same mode, speed and elastic base, for
--  TURN_LEG_S (ScenarioSession.apply_change).
function M.turned_leg(leg, heading_deg)
    return { duration_s = M.TURN_LEG_S, mode = leg.mode, heading_deg = heading_deg,
             speed_ms = leg.speed_ms, elastic_base = leg.elastic_base,
             pace = leg.pace }
end

--- The polygon's area centroid, n, e (PolygonZone.centroid).
function M.centroid(z)
    local a, cn, ce = 0.0, 0.0, 0.0
    local v = z.vertices
    for i = 1, #v do
        local p, q = v[i], v[i % #v + 1]
        local w = p[2] * q[1] - q[2] * p[1]
        a = a + w
        ce = ce + (p[2] + q[2]) * w
        cn = cn + (p[1] + q[1]) * w
    end
    a = a * 0.5
    return cn / (6.0 * a), ce / (6.0 * a)
end

--- Vehicle only: fence vertices {{lat_deg, lng_deg}, ...} as a zone in the
--  frame anchored at `origin` (a Location), through the vehicle's own
--  Location:get_distance_NE. Returns the zone, or nil and a reason.
function M.from_latlng(origin, latlng)
    if type(latlng) ~= "table" then
        return nil, "no fence vertices"
    end
    local verts = {}
    for i = 1, #latlng do
        local v = latlng[i]
        local loc = Location()
        loc:lat(math.floor(v[1] * 1e7 + 0.5))
        loc:lng(math.floor(v[2] * 1e7 + 0.5))
        local ne = origin:get_distance_NE(loc)
        verts[i] = { ne:x(), ne:y() }
    end
    return M.polygon(verts)
end

return M
