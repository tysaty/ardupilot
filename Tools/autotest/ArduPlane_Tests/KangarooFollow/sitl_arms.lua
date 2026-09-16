-- =========================================================
--  sitl_arms -- which ported guidance law flies a SITL cell (TASK-052)
--  created 2026-09-16
--
--  Maps a Python registry name (spec.json "algorithm.name") to the Lua
--  entry point that produces one guidance point from one snapshot, in the
--  shape the differential harness already gates:
--
--      entry(snapshot, cfg) -> {guidance_n_m, guidance_e_m, algorithm_state}
--                            | nil, reason
--
--  Three arms ship their own `guidance_point` (harness_adaptive_db,
--  harness_adaptive_horizon, harness_rh_geometric). The baseline and its
--  hysteresis variant only have the geometry-level `harness_cs_orbit.guidance`,
--  so the snapshot adapter for them lives HERE -- outside modules/ -- as a
--  transliteration of algorithms.py's DubinsTargetOrbitAlgorithm and
--  DubinsTargetOrbitHystAlgorithm, and is gated tick for tick against the
--  Python by tests/unit/test_sitl_campaign.py. It is an adapter, not a law:
--  it reads the snapshot, chooses the estimate over the truth when present,
--  calls the ported geometry and repackages the result.
--
--  Everything without an entry point is REFUSED with a reason, and the
--  campaign records the cell as a finding (VR-014): no law is edited here to
--  make it fly. Arm D and the A/B/D hysteresis adapters are the known gaps
--  (TASK-006 status; TASK-046 D3).
--
--  Stateless. Frame as the harness: snapshot in (north, east) metres, the
--  geometry in (x = east, y = north); heading identity (algorithms.py
--  `_heading_to_geometry`).
-- =========================================================

local M = {}

local function target_ea(snapshot)
    local est = snapshot.target_est
    if est ~= nil then
        return est.e_m, est.n_m
    end
    return snapshot.target_e_m, snapshot.target_n_m
end

--- The baseline CS-onto-orbit, with or without the TASK-047 held sense.
local function cs_orbit_entry(hyst)
    return function(snapshot, cfg)
        local cs = require("harness_cs_orbit")
        local px, py = snapshot.plane_e_m, snapshot.plane_n_m
        local tx, ty = target_ea(snapshot)
        local psi_i = snapshot.plane_hdg_rad
        local previous, margin = nil, nil
        if hyst then
            local st = snapshot.algorithm_state or {}
            if st.direction == "cw" or st.direction == "ccw" then
                previous = st.direction
            end
            margin = cfg.cs_sense_margin_m
        end
        local g, reason = cs.guidance(px, py, psi_i, tx, ty,
                                      cfg.orbit_radius_m, cfg.turn_radius_m,
                                      cfg.look_ahead_m, cfg.delta_psi_rad,
                                      cfg.delta_d_m, cfg.orbit_precompensate,
                                      previous, margin)
        if g == nil then
            return nil, reason
        end
        local state = {
            phase = g.phase,
            direction = g.direction,
            curvature = g.curvature,
        }
        if g.ring_angle_rad ~= nil then
            state.ring_angle_rad = g.ring_angle_rad
        end
        if hyst then
            state.sense_held = (previous ~= nil) and (g.direction == previous)
            state.sense_switched = (previous ~= nil) and (g.direction ~= previous)
            -- cost_cw_m / cost_ccw_m are diagnostic in the Python and are not
            -- returned by the ported geometry; omitted, not faked.
        end
        return {
            guidance_n_m = g.gy,
            guidance_e_m = g.gx,
            algorithm_state = state,
        }
    end
end

--- Registry: Python algorithm name -> how to fly it.
--  `module` names a ported module whose `guidance_point` is the entry;
--  `entry` is an adapter defined here.
M.REGISTRY = {
    dubins_target_orbit      = { entry = cs_orbit_entry(false), source = "sitl_arms adapter over harness_cs_orbit.guidance" },
    dubins_target_orbit_hyst = { entry = cs_orbit_entry(true),  source = "sitl_arms adapter over harness_cs_orbit.guidance (held sense)" },
    adaptive_db_circle       = { module = "harness_adaptive_db" },
    adaptive_horizon_cs      = { module = "harness_adaptive_horizon" },
    rh_geometric             = { module = "harness_rh_geometric" },
}

--- Known gaps, each with the reason the campaign will record.
M.NOT_PORTED = {
    velocity_db_circle       = "arm D has no Lua module (TASK-043; TASK-046 D3)",
    velocity_db_circle_hyst  = "arm D has no Lua module (TASK-043; TASK-046 D3)",
    adaptive_db_circle_hyst  = "held-sense adapter for harness_adaptive_db not ported (TASK-006 Tranche 9 note)",
    adaptive_horizon_cs_hyst = "held-sense adapter for harness_adaptive_horizon not ported (TASK-006 Tranche 9 note)",
}

--- Resolve a name to `entry, source` or `nil, reason`.
function M.resolve(name)
    local row = M.REGISTRY[name]
    if row == nil then
        local why = M.NOT_PORTED[name]
        if why == nil then
            why = "no Lua entry point registered for algorithm '" .. tostring(name) .. "'"
        end
        return nil, why
    end
    if row.entry ~= nil then
        return row.entry, row.source
    end
    local ok, mod = pcall(require, row.module)
    if not ok then
        return nil, "require('" .. row.module .. "') failed: " .. tostring(mod)
    end
    if type(mod.guidance_point) ~= "function" then
        return nil, row.module .. " has no guidance_point"
    end
    return mod.guidance_point, row.module .. ".guidance_point"
end

--- Names that can fly, for the planner's refusal-before-run check.
function M.flyable()
    local out = {}
    for name, _ in pairs(M.REGISTRY) do
        out[#out + 1] = name
    end
    table.sort(out)
    return out
end

return M
