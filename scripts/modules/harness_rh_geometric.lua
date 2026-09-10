-- =========================================================
--  harness_rh_geometric -- receding-horizon geometric planner
--  created 2026-09-03
--  TASK-006 Tranche 6c (forward port).  Algorithm: rh_geometric,
--  TASK-039 arm C.
--
--  Ported from py_harness/geometry/rh_geometric.py and its adapter.
--
--  NOT AN MPC, and it must not be described as one: no solver, no gradient, no
--  convergence criterion. An exhaustive search over a small, feasible, finite
--  candidate set -- which is exactly what makes it transliterable at all
--  (VR-015) and what makes its planning cost a FIXED, MEASURABLE number rather
--  than something discovered after the fact.
--
--  Each replan:
--    1. predict the target over N ticks on constant velocity, from the
--       UN-PROJECTED estimate;
--    2. roll the aircraft forward over the same N ticks for every candidate in a
--       two-segment curvature grid -- kappa1 held for segment_steps ticks, then
--       kappa2 -- both drawn from [-1/rho, +1/rho], so EVERY CANDIDATE
--       SATISFIES THE CURVATURE BOUND BY CONSTRUCTION. Nothing is generated and
--       then rejected, which is the cheap way to enforce FR-005 and SR-002 in an
--       optimiser;
--    3. score each rollout against the predicted standoff tube;
--    4. emit the winner's own pose command_steps ticks ahead. Only the first
--       command is ever flown.
--
--  The instruction budget is THE issue for this arm on the vehicle
--  ---------------------------------------------------------------
--  Cost is exactly n1*n2*N aircraft steps per replan -- 6075 at the harness
--  defaults, measured at 2.2 ms in CPython. That figure says NOTHING about
--  ArduPilot's interpreted Lua with a per-cycle instruction budget, which is
--  precisely why PR-002 and A-SW-002 must be measured here and not extrapolated.
--  See docs/TASK-039-ISSUES.md ISSUE-C3.
--
--  The emission rule diverges from the flight interface
--  ----------------------------------------------------
--  This arm emits its own next planned pose, roughly 2.5 m ahead, rather than a
--  look-ahead point. Against the harness's steering law that is exactly right --
--  a carrot a look-ahead along a plan curving at 1/rho is a chord the turn-rate
--  limiter cannot follow, and the aircraft then flies its MINIMUM radius
--  regardless of the plan (measured: a 70 m ring held at 45.0 m). Against
--  ArduPlane's L1, which expects a look-ahead point, it probably is not.
--  ISSUE-C4, unresolved, and it needs SITL rather than this harness.
--
--  Frame x = East, y = North, psi from North clockwise (IR-008). Stateless.
-- =========================================================

local dubins = require("harness_dubins")

local M = {}

-- ---------------------------------------------------------
-- The candidate grid
-- ---------------------------------------------------------

--- `count` curvatures spanning [-1/rho, +1/rho] inclusive, 1/m.
--
--  An ODD count puts straight flight (kappa = 0) exactly on the grid, which
--  matters: straight flight must always be a candidate, or the planner cannot
--  choose to stop turning. An even count is accepted and produces a grid without
--  it -- the caller is told, not corrected, because silently changing a
--  configured candidate count would make the reported planning cost wrong.
--
--  Returns a 1-indexed array, or nil plus a reason.
function M.curvature_grid(turn_radius_m, count)
    if turn_radius_m <= 0.0 then
        return nil, "turn_radius_m must be positive"
    end
    if count < 2 then
        return nil, "count must be >= 2; one candidate is not a search"
    end
    local k_max = 1.0 / turn_radius_m
    local span = 2.0 * k_max
    local out = {}
    for i = 0, count - 1 do
        out[i + 1] = -k_max + span * i / (count - 1)
    end
    return out
end

-- ---------------------------------------------------------
-- The rollout
-- ---------------------------------------------------------

--- Integrate the aircraft forward under a two-segment curvature command.
--
--  Constant speed, curvature-commanded heading: dpsi = kappa*V*dt (curvature is
--  dpsi/ds and ds = V*dt), then a straight step on the new heading. THE SAME
--  forward-Euler integration the harness's own state module uses, so the
--  planner's model of the aircraft and the aircraft agree -- an optimiser scored
--  against a different model than the one it flies would be measuring the
--  mismatch.
--
--  Returns a 1-indexed array of horizon_steps + 1 points {x, y}, starting at the
--  present position and spaced V*dt metres apart. nil plus a reason for
--  horizon_steps < 1.
function M.rollout(px, py, psi_i, kappa1, kappa2, hold_steps, horizon_steps,
                   airspeed_ms, dt_s)
    if horizon_steps < 1 then
        return nil, "horizon_steps must be >= 1"
    end
    local hold = hold_steps
    if hold < 0 then
        hold = 0
    elseif hold > horizon_steps then
        hold = horizon_steps
    end
    local d = airspeed_ms * dt_s
    local x, y, psi = px, py, psi_i
    local pts = { { x = x, y = y } }
    for i = 0, horizon_steps - 1 do
        local kappa = kappa2
        if i < hold then
            kappa = kappa1
        end
        psi = psi + kappa * d
        x = x + d * math.sin(psi)
        y = y + d * math.cos(psi)
        pts[#pts + 1] = { x = x, y = y }
    end
    return pts
end

-- ---------------------------------------------------------
-- The cost
-- ---------------------------------------------------------

--- Cost of one rollout against the predicted standoff tube.
--
--      J = w_standoff * mean_i (r_i - R)^2
--        + w_terminal * (r_N - R)^2
--        + w_effort   * mean_i kappa_i^2
--        + w_smooth   * (kappa1 - kappa_prev)^2
--
--  r_i is the range from the aircraft's rolled-out position at step i to the
--  target PREDICTED AT THAT SAME STEP, so the cost is target-relative
--  throughout and never compares a future aircraft with a present target --
--  which is the error the fixed-horizon arms are structurally exposed to.
--
--  The step-0 point is excluded from the running mean: it is the present pose,
--  identical for every candidate, so scoring it adds a constant that shifts
--  every cost equally and buys nothing.
function M.score(pts, est_x, est_y, vx, vy, dt_s, orbit_radius_m, kappa1,
                 kappa2, hold_steps, kappa_prev, weights)
    local n = #pts - 1
    local total = 0.0
    for i = 1, n do
        local t_i = i * dt_s
        local tx = est_x + vx * t_i
        local ty = est_y + vy * t_i
        local ddx = pts[i + 1].x - tx
        local ddy = pts[i + 1].y - ty
        local dev = math.sqrt(ddx * ddx + ddy * ddy) - orbit_radius_m
        total = total + dev * dev
    end
    local j_standoff = total / n

    local t_n = n * dt_s
    local ex = pts[n + 1].x - (est_x + vx * t_n)
    local ey = pts[n + 1].y - (est_y + vy * t_n)
    local dev_n = math.sqrt(ex * ex + ey * ey) - orbit_radius_m
    local j_terminal = dev_n * dev_n

    local hold = hold_steps
    if hold < 0 then hold = 0 elseif hold > n then hold = n end
    local j_effort = (hold * kappa1 * kappa1
                      + (n - hold) * kappa2 * kappa2) / n

    local j_smooth = 0.0
    if kappa_prev ~= nil then
        local dk = kappa1 - kappa_prev
        j_smooth = dk * dk
    end

    local cost = weights.standoff * j_standoff
                 + weights.terminal * j_terminal
                 + weights.effort * j_effort
                 + weights.smooth * j_smooth
    return {
        cost = cost,
        mean_error_m = math.sqrt(j_standoff),
        terminal_error_m = dev_n,
        j_standoff = j_standoff,
        j_terminal = j_terminal,
        j_effort = j_effort,
        j_smooth = j_smooth,
    }
end

-- ---------------------------------------------------------
-- The search
-- ---------------------------------------------------------

--- Search the curvature grid and return the least-cost candidate.
--
--  Ties break toward the FIRST candidate in grid order, which runs from the most
--  negative curvature upward; the grid is symmetric, so this is a stable rule
--  rather than a preference for one turn sense in any meaningful sense.
function M.plan(px, py, psi_i, est_x, est_y, vx, vy, dt_s, airspeed_ms,
                orbit_radius_m, turn_radius_m, horizon_steps, segment_steps,
                n_candidates, n_candidates_2, weights, kappa_prev)
    local grid1, reason = M.curvature_grid(turn_radius_m, n_candidates)
    if grid1 == nil then
        return nil, reason
    end
    local grid2
    grid2, reason = M.curvature_grid(turn_radius_m, n_candidates_2)
    if grid2 == nil then
        return nil, reason
    end
    local best = nil
    for a = 1, #grid1 do
        for b = 1, #grid2 do
            local kappa1, kappa2 = grid1[a], grid2[b]
            local pts = M.rollout(px, py, psi_i, kappa1, kappa2, segment_steps,
                                  horizon_steps, airspeed_ms, dt_s)
            local s = M.score(pts, est_x, est_y, vx, vy, dt_s, orbit_radius_m,
                              kappa1, kappa2, segment_steps, kappa_prev,
                              weights)
            if best == nil or s.cost < best.cost then
                s.kappa1 = kappa1
                s.kappa2 = kappa2
                s.pts = pts
                best = s
            end
        end
    end
    best.evaluated = #grid1 * #grid2
    best.rollout_steps = best.evaluated * horizon_steps
    return best
end

--- One guidance point, from a fresh plan or from a HELD command sequence.
--
--  `held` is nil to re-optimise now, else {kappa1, kappa2, remaining_hold_steps}
--  -- the command committed at the last replan, RE-INTEGRATED FROM THE LIVE
--  POSE. Holding the COMMAND rather than the PATH is what keeps this
--  drift-free: there is no stored trajectory for the aircraft to fall behind, so
--  a commitment interval longer than one tick costs fidelity only through the
--  staleness of the target prediction, which is the thing being studied.
--
--  `phase` is reported for comparability with the ring arms, which switch
--  construction at the ring boundary. THIS ARM DOES NOT SWITCH CONSTRUCTION:
--  one optimiser runs throughout, which is the point of it.
function M.guidance(px, py, psi_i, est_x, est_y, vx, vy, dt_s, airspeed_ms,
                    orbit_radius_m, turn_radius_m, command_steps,
                    horizon_steps, segment_steps, n_candidates, n_candidates_2,
                    weights, kappa_prev, held)
    local best, replanned, reason
    if held == nil then
        best, reason = M.plan(px, py, psi_i, est_x, est_y, vx, vy, dt_s,
                              airspeed_ms, orbit_radius_m, turn_radius_m,
                              horizon_steps, segment_steps, n_candidates,
                              n_candidates_2, weights, kappa_prev)
        if best == nil then
            return nil, reason
        end
        replanned = true
    else
        local pts = M.rollout(px, py, psi_i, held.kappa1, held.kappa2,
                              held.remaining_hold_steps, horizon_steps,
                              airspeed_ms, dt_s)
        best = M.score(pts, est_x, est_y, vx, vy, dt_s, orbit_radius_m,
                       held.kappa1, held.kappa2, held.remaining_hold_steps,
                       kappa_prev, weights)
        best.kappa1 = held.kappa1
        best.kappa2 = held.kappa2
        best.pts = pts
        best.evaluated = 0
        best.rollout_steps = horizon_steps
        replanned = false
    end

    local step = command_steps
    if step < 1 then
        step = 1
    elseif step > #best.pts - 1 then
        step = #best.pts - 1
    end
    local gx, gy = best.pts[step + 1].x, best.pts[step + 1].y
    local dx, dy = px - est_x, py - est_y
    local d = math.sqrt(dx * dx + dy * dy)
    local phase = "approach"
    if d <= orbit_radius_m then
        phase = "orbit"
    end
    return {
        gx = gx,
        gy = gy,
        phase = phase,
        -- The curvature actually commanded now is the first segment's. Reported
        -- as the FR-005 / SR-002 quantity; <= 1/rho by grid construction.
        curvature = math.abs(best.kappa1),
        kappa1 = best.kappa1,
        kappa2 = best.kappa2,
        replanned = replanned,
        cost = best.cost,
        mean_error_m = best.mean_error_m,
        terminal_error_m = best.terminal_error_m,
        evaluated = best.evaluated,
        rollout_steps = best.rollout_steps,
        command_steps = step,
    }
end

-- ---------------------------------------------------------
-- The adapter
-- ---------------------------------------------------------

--- One tick against the harness's snapshot contract.
function M.guidance_point(snapshot, cfg)
    local px, py = snapshot.plane_e_m, snapshot.plane_n_m
    local psi_i = snapshot.plane_hdg_rad

    local est = snapshot.target_est_raw or snapshot.target_est
    if est == nil then
        return nil, "rh_geometric requires the state estimator: it selects " ..
                    "its own prediction horizon from the estimated target " ..
                    "velocity and has no present-position fallback"
    end

    local n_replan = cfg.replan_every
    local st = snapshot.algorithm_state or {}
    local prev_ticks = st.ticks_since_replan
    local replanned = (prev_ticks == nil) or ((prev_ticks + 1) >= n_replan)

    local segment_steps = cfg.rh_segment_steps
    local kappa_prev = st.kappa1_1pm

    local held, ticks
    if replanned then
        held = nil
        ticks = 0
    else
        ticks = prev_ticks + 1
        local remaining = segment_steps - ticks
        if remaining < 0 then
            remaining = 0
        end
        held = { kappa1 = st.kappa1_1pm, kappa2 = st.kappa2_1pm,
                 remaining_hold_steps = remaining }
    end

    local weights = {
        standoff = cfg.rh_w_standoff,
        terminal = cfg.rh_w_terminal,
        effort = cfg.rh_w_effort,
        smooth = cfg.rh_w_smooth,
    }

    local g, reason = M.guidance(px, py, psi_i, est.e_m, est.n_m, est.ve_ms,
                                 est.vn_ms, cfg.dt_s, cfg.airspeed_ms,
                                 cfg.orbit_radius_m, cfg.turn_radius_m,
                                 cfg.rh_command_steps, cfg.rh_horizon_steps,
                                 segment_steps, cfg.rh_candidates,
                                 cfg.rh_candidates_2, weights, kappa_prev, held)
    if g == nil then
        return nil, reason
    end

    local sense = "ccw"
    if g.kappa1 >= 0.0 then
        sense = "cw"
    end
    return {
        guidance_n_m = g.gy,
        guidance_e_m = g.gx,
        algorithm_state = {
            phase = g.phase,
            -- No ring is constructed, so there is no CW/CCW sense to report.
            -- Named from the sign of the commanded curvature instead, which is
            -- the comparable quantity.
            direction = sense,
            curvature = g.curvature,
            kappa1_1pm = g.kappa1,
            kappa2_1pm = g.kappa2,
            replanned = replanned,
            ticks_since_replan = ticks,
            plan_cost = g.cost,
            rollout_rms_error_m = g.mean_error_m,
            terminal_error_m = g.terminal_error_m,
            candidates_scored = g.evaluated,
            rollout_steps = g.rollout_steps,
            command_steps = g.command_steps,
            -- The centre this arm holds its standoff about is the target it
            -- predicted from, so a ring-centred metric reports something
            -- meaningful for it rather than falling through to the truth.
            centre_n_m = est.n_m,
            centre_e_m = est.e_m,
            prediction_lead_m = math.sqrt(
                (est.e_m - snapshot.target_e_m) * (est.e_m - snapshot.target_e_m)
                + (est.n_m - snapshot.target_n_m) * (est.n_m - snapshot.target_n_m)),
        },
    }
end

return M
