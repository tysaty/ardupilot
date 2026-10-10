-- =========================================================
--  ARM F - the carrot shift CS-orbit algorithm
--  created 2026-10-01 
--  This algorithm implements the kalman filter to provide state esitamtion
--  to offset the predicted path by 
---      g' = g + v_est * dt_s * af_step_ticks

--    1. build hte baseline (harness_cs_orbit
--    2. take the guidance point it returns, g = (gx, gy) - the carrot, the
--       point actually being flown;
--    3. shift tp that point by the Kalman filter's velocity over one
--       control tick
--    4. command flight to the new point (g'.) The path, ring, phase, 
--     orbit sense and curvature stay.

--
--  What is reused, unchanged:
--    harness_cs_orbit.guidance(px, py, psi_i, tx, ty, R, rho, L, dpsi, dd,
--                              precompensate, preferred_direction, margin)
--        -> {gx, gy, phase, direction, curvature, ring_angle_rad?} | nil, why
--     The baseline law: CS approach outside the ring, ramp-free orbit, pre-comp carrot
--     The harness_estimator (the Kalman filter) is called in the run harness
--        script (kangaroo_demo.lua / sitl_harness_runner.lua)  every tick and included as 
--        a snapshot:
--          snapshot.target_est_raw = {n_m, e_m, vn_ms, ve_ms}   un-projected
--          snapshot.target_est  = target_est_raw projected lookahead_steps

--        Arm F reads target_est_raw and owns its own lead, so a
--        state-side lookahead_steps would lead twice (see "Double lead").
--    sitl_arms.lua cs_orbit_entry -- the existing baseline adapter. Steps 1
--        to 4 of guidance_point below follow it line for line; read it first.
--
--  The Python harness is normative: the Python arm was built
--  first; this Lua module must then match it tick for tick under
--  tests/unit/test_lua_differential.py before it is registered anywhere.
--
--  Frame: snapshot in (north, east) metres; geometry in (x = East, y = North);
--  psi from North clockwise; heading passed through unchanged (as
--  sitl_arms.lua does). Distances m, speeds m/s, time.

--  Stateless at module level: anything carried between ticks travels through
--  algorithm_state.
-- =========================================================

local cs = require("harness_cs_orbit")

local M = {}

-- ---------------------------------------------------------
-- The carrot shift (the only new maths in arm F)
-- ---------------------------------------------------------

--- The lead to add to the carrot, metres, as (shift_e_m, shift_n_m).
--
--  est        -- snapshot.target_est_raw: {n_m, e_m, vn_ms, ve_ms}
--  dt_s       -- control interval, s (cfg.dt_s; 0.1 at 10 Hz)
--  step_ticks -- whole control ticks to lead (cfg.af_step_ticks; 1 = the arm
--                as specified). Must be >= 1; 0 would make this arm 0.
--
--  Returns shift_e_m, shift_n_m, or nil plus a reason.
function M.carrot_shift(est, dt_s, step_ticks)
    -- error handling
    -- setting up estimator
    if est == nil then
        return nil, "carrot_shift_cs requires the state estimator"
    end
    -- type checks first: a missing cfg key arrives as nil, and comparing nil
    -- with a number is a Lua error, not a refusal
    if type(dt_s) ~= "number" or dt_s <= 0 then
        return nil, "dt_s must be a positive number"
    end
    if type(step_ticks) ~= "number" or step_ticks < 1 then
        return nil, "af_step_ticks must be a number >= 1 (is it in config_dict?)"
    end
    local lead_s = dt_s * step_ticks
    return est.ve_ms * lead_s, est.vn_ms * lead_s
end

-- ---------------------------------------------------------
-- The adapter: one tick against the harness snapshot contract
-- ---------------------------------------------------------

--- One guidance point. Same contract as every arm in sitl_arms.lua
--      guidance_point(snapshot, cfg)
--          -> {guidance_n_m, guidance_e_m, algorithm_state} | nil, reason
--
--  `snapshot` carries plane_n_m, plane_e_m, plane_hdg_rad, target_n_m,
--  target_e_m, target_vn_ms, target_ve_ms, algorithm_state, target_est,
--  target_est_raw.
--  `cfg` is the flattened HarnessConfig (py_harness/algorithms.config_dict):
--  orbit_radius_m, turn_radius_m, look_ahead_m, delta_psi_rad, delta_d_m,
--  orbit_precompensate, cs_sense_margin_m, dt_s -- plus af_step_ticks, which
--  does NOT exist yet and must be added to config.py / config_dict first (follow
--  vd_step_ticks, arm D's equivalent).

--
--  `hyst` selects the held orbit sense, the way sitl_arms'
--  cs_orbit_entry(hyst) and the Python _SenseHysteresisMixin do: it is a
--  property of the arm, not a config value, so it is passed in, not read from
--  cfg. M.guidance_point and M.guidance_point_hyst below are the two entries.
local function guidance(snapshot, cfg, hyst)
    -- Step 1: aircraft pose (copy from sitl_arms cs_orbit_entry)
    local px, py = snapshot.plane_e_m, snapshot.plane_n_m
    local psi_i = snapshot.plane_hdg_rad
    -- Step 2: Establish the estimate to offset
    local est = snapshot.target_est_raw
    -- error handling
    if est == nil then
        return nil, "carrot_shift_cs requires the state estimator: ..."
    end
    --
    -- path about the estimate
    local tx, ty = est.e_m, est.n_m

    --  Whichever is chosen, record it in algorithm_state (e.g.
    --  state.path_about = "estimate" | "truth") so a bundle says which ran.

    -- Step 3: held orbit sense, for the _hyst variant only
    -- Same rule as sitl_arms - cs_orbit_entry(true)  - i.e. read the last direction
    -- from state, pass it with the margin, if the previous is nil than it's an argmin
    -- (the baseline without hysteresis).

    local previous, margin = nil, nil
    -- if the input sense has hysteriss - flag
    if hyst then
        -- entry into sitl_arms
        local st = snapshot.algorithm_state or {} 
            if st.direction == "cw" or st.direction == "ccw" then
                previous = st.direction
            end
        margin = cfg.cs_sense_margin_m
    end

    -- Step 4: Implmeneting the BASELINE guidance
    -- Exactly the call sitl_arms makes for the baseline
    local g, reason = cs.guidance(px, py, psi_i, tx, ty,
                                      cfg.orbit_radius_m, cfg.turn_radius_m,
                                      cfg.look_ahead_m, cfg.delta_psi_rad,
                                      cfg.delta_d_m, cfg.orbit_precompensate,
                                      previous, margin)
        if g == nil then
            return nil, reason
        end

    --  g.gx, g.gy is the carrot g (geometry frame: x = East, y = North).

    -- ---- Step 5: shift the carrot based on the state estimate 

        local shift_e, shift_n = 0.0, 0.0
        local shifted = false
    --  handle both the orbit and the Dubins path 
        local apply = true
        -- applying the shift
        if apply then
            local se, sn_or_reason = M.carrot_shift(est, cfg.dt_s, cfg.af_step_ticks)
            if se == nil then
                return nil, sn_or_reason
            end
            shift_e, shift_n, shifted = se, sn_or_reason, true
        end
        -- shift the values
        local gx2, gy2 = g.gx + shift_e, g.gy + shift_n

    -- Step 6: Report on the shift
    -- Report the unshifted point too, so the off-path offset is measurable
    -- and the lead in seconds.
        local state = {
            phase = g.phase,
            -- unchanged curvature and direction
            direction = g.direction,
            curvature = g.curvature,
            -- orbit phase
            ring_angle_rad = g.ring_angle_rad, -- orbit phase only (nil otherwise)
            guidance_raw_n_m = g.gy,
            guidance_raw_e_m = g.gx,
            carrot_shift_n_m = shift_n,
            carrot_shift_e_m = shift_e,
            carrot_lead_s = shifted and (cfg.dt_s * cfg.af_step_ticks) or 0.0,
            carrot_shifted = shifted,
            -- the open decisions, recorded so every bundle says which ran
            path_about = "estimate",        -- the path about the estimate (alternative: "truth")
            carrot_shift_phases = "both",   -- shift in both phases (alternative: "approach")
        }
        -- managing hysteresis: set on EVERY tick of the _hyst variant, false
        -- when there is no previous sense yet -- as sitl_arms and the Python
        -- _sense_state do, so the first tick matches field for field
        if hyst then
            state.sense_held = (previous ~= nil) and (g.direction == previous)
            state.sense_switched = (previous ~= nil) and (g.direction ~= previous)
        end

    -- Step 7: return, snapshot frame (NE)

        return {
            -- return shfited points
            guidance_n_m = gy2,
            guidance_e_m = gx2,
            algorithm_state = state,
    }
end

--- The two entries, as sitl_arms' contract expects: (snapshot, cfg).
--  carrot_shift_cs      -> M.guidance_point       (no held sense)
--  carrot_shift_cs_hyst -> M.guidance_point_hyst  (held sense)
function M.guidance_point(snapshot, cfg)
    return guidance(snapshot, cfg, false)
end

function M.guidance_point_hyst(snapshot, cfg)
    return guidance(snapshot, cfg, true)
end

-- ---------------------------------------------------------
-- After it is implemented (checklist; nothing here runs)
-- ---------------------------------------------------------
--
--  Tests (Lua, through tests/unit/test_lua_differential.py's pattern):
--    * v_est = 0: guidance_point equals harness_cs_orbit.guidance about the
--      same target, field for field (the shift is exactly zero).
--    * moving target: (g' - g) == v_est * dt_s * af_step_ticks to 1e-9 m, and
--      phase, direction, curvature equal the baseline's.
--    * refusals: no estimate; af_step_ticks < 1; dt_s <= 0.
--    * tick for tick against the Python carrot_shift_cs once it exists.
--
--  Registration (only after the tests pass; do not do this in the scaffold):
--    * add "harness_carrot_shift.lua" to HARNESS_MODULES in
--      Tools/autotest/kangaroo_follow/paths.py, so staging hashes it;
--    * add carrot_shift_cs (and carrot_shift_cs_hyst) to sitl_arms.REGISTRY,
--      in BOTH copies (Tools/autotest/ArduPlane_Tests/KangarooFollow/ and
--      scripts/modules/);
--    * the cell's spec must set estimate = true and lookahead_steps = 0
--      ("Double lead": a state-side projection would lead the path as well).

return M
