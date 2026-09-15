-- =========================================================
--  harness_adaptive_db -- CS-orbit about a PREDICTED target  |  created 2026-09-03
--  TASK-006 Tranche 6 (forward port).  Algorithm: adaptive_db_circle (TASK-033).
--
--  THE FIRST STATEFUL ALGORITHM PORTED. It carries algorithm_state across ticks
--  -- the committed pose, the held centre, the replan clock -- so the
--  differential test must compare THE STATE AS WELL AS THE OUTPUT, at every
--  tick. A state divergence that has not yet reached the output is still a
--  divergence, and it surfaces later as an unexplained step.
--
--  Two differences from harness_cs_orbit, and nothing else:
--
--    1. The ring is centred on a PREDICTION -- where the estimator says the
--       target will be k_horizon ticks ahead -- not on where it is now. The
--       prediction itself is state-side and pre-existing (harness_estimator
--       .predict); nothing here estimates anything.
--    2. The CS curve is COMMITTED for n_replan ticks and then replaced,
--       UNCONDITIONALLY. There is no candidate set, no cost function, no
--       scoring of the new plan against the active one, no hysteresis and no
--       cooldown. The tick counter reaching n_replan is the only thing that
--       changes the path. This is a COMMITMENT INTERVAL, not the replanning
--       DECISION that ADR-001 removed, and the distinction is the reason this
--       algorithm was allowed to exist (ADR-004).
--
--  Hold policies (TASK-033 D5)
--  ---------------------------
--  "plan"        -- the predicted centre AND the committed CS curve are both
--                   held. Progress along the committed curve is found by
--                   nearest-point projection and the carrot is placed a
--                   look-ahead further along it.
--  "centre_only" -- only the centre is held; the curve is re-solved from the
--                   live pose every tick. This isolates the prediction variable
--                   from the commitment variable.
--
--  At k_horizon = 0, n_replan = 1, centre_only the construction reduces term for
--  term to harness_cs_orbit reading the estimate -- asserted by test, not assumed.
--
--  The committed plan is rebuilt each tick from the COMMIT POSE and HELD CENTRE
--  rather than stored as a sampled point list: shortest_path is deterministic,
--  so re-running it on the stored inputs returns the identical path (PR-004),
--  and algorithm_state stays a handful of numbers instead of a few hundred
--  points. A vehicle port would cache the sampled path instead; this
--  re-derives it because the harness is measuring GEOMETRY, not planning cost.
--  The replan interval therefore changes WHICH plan is flown, not how much
--  computation happens.
--
--  Frame x = East, y = North, psi from North clockwise (IR-008).
--  Stateless at module level: everything carried between ticks travels through
--  algorithm_state (VR-015, A-VAL-003).
-- =========================================================

local dubins = require("harness_dubins")
local orbit = require("harness_orbit")
local cs = require("harness_cs_orbit")

local M = {}

--- Hold the predicted centre and the committed CS curve for the whole interval.
M.HOLD_PLAN = "plan"

--- Hold only the predicted centre; re-solve the curve from the live pose.
M.HOLD_CENTRE_ONLY = "centre_only"

-- ---------------------------------------------------------
-- Geometry
-- ---------------------------------------------------------

--- Closed-form orbit continuation about a HELD centre. Shared by both policies.
--  Identical in form to harness_cs_orbit's orbit branch -- the sense is
--  re-derived from the heading each tick rather than stored -- except that the
--  centre is the held PREDICTED centre, not the target's present position.
local function orbit_hold(px, py, cx, cy, psi_i, R, look_ahead_m, precompensate)
    local psi0 = orbit.entry_angle(px, py, cx, cy)
    local direction = orbit.orbit_direction(psi0, psi_i)
    local gx, gy, psi = orbit.orbit_guidance_point(cx, cy, R, psi0, direction,
                                                   look_ahead_m, precompensate)
    if gx == nil then
        return nil, gy
    end
    local sense = "ccw"
    if direction > 0 then
        sense = "cw"
    end
    return {
        gx = gx,
        gy = gy,
        phase = "orbit",
        direction = sense,
        curvature = 1.0 / R,
        ring_angle_rad = psi,
    }
end

M.orbit_hold = orbit_hold

--- One guidance point about the held predicted ring centre (cx, cy).
--
--  The CALLER owns the replan clock and decides whether this tick is a replan
--  instant; this function is told the answer through `plan` and holds no clock.
--
--  `plan` is {px, py, psi} -- the aircraft pose at the last replan instant -- or
--  nil to commit the live pose now. It is ignored entirely under centre_only.
--
--  Returns a table {gx, gy, phase, direction, curvature, ring_angle_rad?,
--  plan?} or nil plus a reason. `plan` is present in the approach phase only:
--  the orbit phase commits no curve.
--  preferred_direction / sense_margin_m (optional; TASK-048): orbit-sense
--  hysteresis on the CS solve, passed through to harness_cs_orbit. Both nil
--  reproduces the pre-2026-09-14 behaviour exactly.
function M.guidance(px, py, psi_i, cx, cy, plan, orbit_radius_m, turn_radius_m,
                    look_ahead_m, delta_psi, delta_d, hold_policy, precompensate,
                    preferred_direction, sense_margin_m)
    if hold_policy ~= M.HOLD_PLAN and hold_policy ~= M.HOLD_CENTRE_ONLY then
        return nil, "unknown hold policy"
    end
    local R = orbit_radius_m
    if R < turn_radius_m - 1e-9 then
        return nil, "target circle radius < minimum turn radius: the orbit " ..
                    "would exceed the curvature bound"
    end

    local dx, dy = px - cx, py - cy
    local d = math.sqrt(dx * dx + dy * dy)
    if d < 1e-6 then
        return nil, "aircraft is at the ring centre; no orbit angle"
    end

    if d <= R then
        -- On / inside the ring: continue around the held centre. No ramp -- the
        -- tangent arrival of the TASK-024 approach makes the switch continuous.
        return orbit_hold(px, py, cx, cy, psi_i, R, look_ahead_m, precompensate)
    end

    if hold_policy == M.HOLD_CENTRE_ONLY then
        -- Re-solve the CS path from the live pose against the frozen centre.
        local g, reason = cs.approach_guidance(px, py, psi_i, cx, cy, R,
                                               turn_radius_m, look_ahead_m,
                                               delta_psi, delta_d,
                                               preferred_direction, sense_margin_m)
        if g == nil then
            return nil, reason
        end
        return {
            gx = g.gx,
            gy = g.gy,
            phase = "approach",
            direction = g.direction,
            curvature = g.curvature,
        }
    end

    -- HOLD_PLAN: fly the curve committed at the last replan instant.
    local commit_x, commit_y, commit_psi
    if plan == nil then
        commit_x, commit_y, commit_psi = px, py, psi_i
    else
        commit_x, commit_y, commit_psi = plan.px, plan.py, plan.psi
    end

    local path, reason = cs.shortest_path(commit_x, commit_y, commit_psi,
                                          cx, cy, R, turn_radius_m,
                                          delta_psi, delta_d,
                                          preferred_direction, sense_margin_m)
    if path == nil then
        return nil, reason
    end

    -- Where the aircraft actually is along that committed curve, then a
    -- look-ahead further on. point_at_arc_length clamps at the path end, which
    -- IS the tangency point on the ring -- so a carrot that runs off the end
    -- sits at the tangency until the phase test above switches to the orbit.
    local s_now = dubins.progress_along(path.points, px, py)
    local gx, gy = dubins.point_at_arc_length(path.points, s_now + look_ahead_m)
    return {
        gx = gx,
        gy = gy,
        phase = "approach",
        direction = path.direction,
        curvature = 1.0 / R,
        plan = { px = commit_x, py = commit_y, psi = commit_psi },
    }
end

-- ---------------------------------------------------------
-- The adapter: the replan clock and algorithm_state
-- ---------------------------------------------------------

--- One tick of the algorithm, against the harness's snapshot contract.
--
--  `snapshot` carries plane_n_m, plane_e_m, plane_hdg_rad, target_n_m,
--  target_e_m, algorithm_state and target_est (already projected k_horizon ahead
--  by harness_estimator.predict, state-side).
--  `cfg` carries orbit_radius_m, turn_radius_m, look_ahead_m, delta_psi_rad,
--  delta_d_m, replan_every, hold_policy, orbit_precompensate.
--
--  Returns {guidance_n_m, guidance_e_m, algorithm_state} or nil plus a reason.
--
--  THE ESTIMATOR IS REQUIRED. Unlike every other algorithm this does not fall
--  back to the true target: centring the ring on a prediction is the whole
--  algorithm, and without one the run would quietly become plain CS-orbit under
--  a different name and be mistaken for evidence.
function M.guidance_point(snapshot, cfg)
    local px, py = snapshot.plane_e_m, snapshot.plane_n_m
    local psi_i = snapshot.plane_hdg_rad

    local est = snapshot.target_est
    if est == nil then
        return nil, "adaptive_db_circle requires the state estimator: it " ..
                    "centres the ring on the predicted target and has no " ..
                    "present-position fallback"
    end

    local n_replan = cfg.replan_every
    local policy = cfg.hold_policy
    local st = snapshot.algorithm_state or {}

    -- Replan clock. ticks_since_replan is 0 on the tick the plan was committed.
    -- No plan yet (first tick) is a replan; otherwise the counter reaching
    -- n_replan is.
    local prev_ticks = st.ticks_since_replan
    local replanned = (prev_ticks == nil) or ((prev_ticks + 1) >= n_replan)

    local cx, cy, plan, ticks
    if replanned then
        cx, cy = est.e_m, est.n_m
        plan = nil          -- commit the live pose, unconditionally (D4)
        ticks = 0
    else
        cx, cy = st.centre_e_m, st.centre_n_m
        -- Carry the committed plan forward only while it is still a valid
        -- approach commit. A replan instant falling in the ORBIT phase commits
        -- no approach curve, so its stored pose can be INSIDE the ring;
        -- re-entering the approach phase with it would ask for a tangent from
        -- inside the ring, which has no solution. Passing nil re-commits from
        -- the live pose, which the phase test guarantees is outside. The
        -- aircraft crosses the boundary repeatedly while settling, so this is
        -- the ordinary case, not a rare one -- it is the regression that
        -- stopped a 180 s run at 35.8 s during TASK-033.
        if st.plan_valid then
            plan = { px = st.plan_e_m, py = st.plan_n_m, psi = st.plan_psi_rad }
        else
            plan = nil
        end
        ticks = prev_ticks + 1
    end

    local g, reason = M.guidance(px, py, psi_i, cx, cy, plan,
                                 cfg.orbit_radius_m, cfg.turn_radius_m,
                                 cfg.look_ahead_m, cfg.delta_psi_rad,
                                 cfg.delta_d_m, policy, cfg.orbit_precompensate)
    if g == nil then
        return nil, reason
    end

    -- The pose the returned approach was actually built from. Present in the
    -- approach phase only.
    local committed = g.plan
    local plan_valid = (committed ~= nil)
    if not plan_valid then
        committed = { px = px, py = py, psi = psi_i }
    end

    local state = {
        phase = g.phase,
        direction = g.direction,
        curvature = g.curvature,
        replanned = replanned,
        ticks_since_replan = ticks,
        centre_n_m = cy,
        centre_e_m = cx,
        -- How far ahead of the TRUE kangaroo the orbited centre sits. A known
        -- designed offset, not an error -- reported so it is never read as one.
        prediction_lead_m = math.sqrt(
            (cx - snapshot.target_e_m) * (cx - snapshot.target_e_m) +
            (cy - snapshot.target_n_m) * (cy - snapshot.target_n_m)),
        plan_n_m = committed.py,
        plan_e_m = committed.px,
        plan_psi_rad = committed.psi,
        plan_valid = plan_valid,
        guidance_n_m = g.gy,
        guidance_e_m = g.gx,
    }
    if g.ring_angle_rad ~= nil then
        state.ring_angle_rad = g.ring_angle_rad
    end

    -- The FR-011 / PR-008 quantity: the guidance-point jump caused by THE PLAN
    -- CHANGING, isolated from the aircraft's own motion by evaluating the OLD
    -- plan at the SAME pose. Omitted -- rather than faked as 0.0 -- when there
    -- is no old plan or it no longer solves from here.
    if replanned and prev_ticks ~= nil then
        local old_plan = nil
        if st.plan_valid then
            old_plan = { px = st.plan_e_m, py = st.plan_n_m,
                         psi = st.plan_psi_rad }
        end
        local old = M.guidance(px, py, psi_i, st.centre_e_m, st.centre_n_m,
                               old_plan, cfg.orbit_radius_m, cfg.turn_radius_m,
                               cfg.look_ahead_m, cfg.delta_psi_rad,
                               cfg.delta_d_m, policy, cfg.orbit_precompensate)
        if old ~= nil then
            local dx, dy = g.gx - old.gx, g.gy - old.gy
            state.replan_step_m = math.sqrt(dx * dx + dy * dy)
        end
    elseif not replanned then
        state.replan_step_m = 0.0
    end

    return {
        guidance_n_m = g.gy,
        guidance_e_m = g.gx,
        algorithm_state = state,
    }
end

return M
