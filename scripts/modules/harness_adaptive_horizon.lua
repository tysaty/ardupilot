-- =========================================================
--  harness_adaptive_horizon -- the prediction horizon SELECTED per replan
--  created 2026-09-03
--  TASK-006 Tranche 6b (forward port).  Algorithm: adaptive_horizon_cs,
--  TASK-039 arm B.
--
--  Ported from py_harness/geometry/adaptive_horizon.py and its adapter.
--
--  What this module is, and is not
--  -------------------------------
--  It is THE SELECTOR ONLY. It scores candidate horizons and returns the
--  winner; it builds no flown path. The adapter hands the winning centre to
--  harness_adaptive_db -- arm A's construction, unmodified -- so the two arms
--  differ in exactly one thing: whether k_horizon is CONFIGURED or CHOSEN.
--  Collapse the candidate set to a single horizon and this reproduces arm A at
--  that horizon, field for field. Any other difference between the arms would
--  confound the comparison TASK-039 exists to make.
--
--  The objective
--  -------------
--      J(k) = w_T*J_T + w_R*J_R + w_kappa*J_kappa + w_S*J_S
--
--  all four non-negative and in different units, so the weights are what make
--  them commensurable and are CONFIGURATION, not constants (SR-004).
--
--    J_T  tangent registration -- the TASK-038 signal evaluated predictively:
--         the plan ends at a tangent point A on the ring about c_k; the aircraft
--         reaches it n_a = reach/V/dt ticks later; the target is predicted to be
--         at p_est + v_est*n_a*dt then. e_tan = ||A - p_K(n_a)|| - R, SIGNED,
--         and J_T = e_tan^2. This term carries the timing: it is zero exactly
--         when the horizon equals the aircraft's own transit time.
--    J_R  standoff tube -- mean squared deviation from R sampled along the
--         FINAL tube_len_m of the candidate curve, each sample compared with the
--         target predicted at that sample's own arrival time. Restricted to the
--         final stretch on purpose: farther out every candidate departs from the
--         same pose, so the deviation there measures the transit rather than the
--         plan and would bias the selector toward whichever candidate is
--         shortest.
--    J_k  curvature excess -- max(0, kappa_req - 1/rho)^2, where kappa_req is
--         the curvature needed to hold R about a centre translating at the
--         estimated target speed, at the bearing this candidate arrives on.
--         Depends on k because the horizon changes WHERE ON THE RING the
--         aircraft arrives, and arriving abeam of the target's motion is the
--         expensive place.
--    J_S  switching -- (k - k_prev)^2 in ticks^2, damping oscillation of the
--         selected horizon between adjacent candidates.
--
--  Relationship to ADR-001
--  -----------------------
--  This module SCORES CANDIDATES AND SELECTS AMONG THEM, and therefore reopens
--  FR-004 and FR-008..FR-011 in substance. That is deliberate and is why
--  TASK-039 requires a decision record before any arm is adopted. Porting it is
--  not adopting it, and nothing here changes any requirement's status.
--
--  Frame x = East, y = North (IR-008). Stateless at module level.
-- =========================================================

local dtc = require("harness_cs_orbit")
local dubins = require("harness_dubins")
local adb = require("harness_adaptive_db")

local M = {}

local PI = math.pi

-- ---------------------------------------------------------
-- The objective's parts
-- ---------------------------------------------------------

--- Ring centre for candidate horizon k_steps, in (x = East, y = North).
--
--  The same constant-velocity projection harness_estimator.predict applies
--  state-side, written out here because arm B must project the UN-PROJECTED
--  estimate itself over each candidate: the state-side horizon is what this
--  algorithm replaces, so consuming it as well would lead the ring twice.
function M.predicted_centre(est_x, est_y, vx, vy, k_steps, dt_s)
    local horizon_s = k_steps * dt_s
    return est_x + vx * horizon_s, est_y + vy * horizon_s
end

--- Curvature needed to hold R about a centre translating at v_K, at ring
--  bearing theta measured from the centre's direction of travel.
--
--      R * dtheta/dt = v_K*sin(theta) + sqrt(V^2 - v_K^2*cos^2(theta))
--
--  and kappa = (dtheta/dt) / V. At v_K = 0 this reduces to 1/R as it must.
--
--  The sign of the v_K*sin(theta) term depends on the orbit sense; this returns
--  the CONSERVATIVE branch, +|v_K*sin(theta)|, because the term exists to report
--  a bound and the cheaper sense is the one that must not be assumed.
--
--  Returns nil when the geometry is UNREACHABLE -- V^2 < v_K^2*cos^2(theta),
--  i.e. the target out-runs the aircraft's along-track component. nil is the
--  honest answer: no finite curvature holds the ring, so returning a large
--  number would misreport an impossibility as an expensive option.
function M.ring_track_curvature(orbit_radius_m, target_speed_ms, airspeed_ms,
                                theta_rad)
    if orbit_radius_m <= 0.0 then
        return nil, "orbit_radius_m must be positive"
    end
    if airspeed_ms <= 0.0 then
        return nil, "airspeed_ms must be positive"
    end
    local c = math.cos(theta_rad)
    local disc = airspeed_ms * airspeed_ms
                 - target_speed_ms * target_speed_ms * c * c
    if disc <= 0.0 then
        return nil
    end
    local theta_dot = (math.abs(target_speed_ms * math.sin(theta_rad))
                       + math.sqrt(disc)) / orbit_radius_m
    return theta_dot / airspeed_ms
end

--- Bearing of (px, py) about (cx, cy), measured from (vx, vy). Radians in
--  [-pi, pi). A stationary centre gives no direction, so 0.0 is returned and the
--  caller's curvature term degenerates to the stationary case 1/R, which is
--  correct.
local function bearing_from_velocity(px, py, cx, cy, vx, vy)
    if math.abs(vx) < 1e-9 and math.abs(vy) < 1e-9 then
        return 0.0
    end
    local a = math.atan(py - cy, px - cx) - math.atan(vy, vx)
    return (a + PI) % (2.0 * PI) - PI
end

M.bearing_from_velocity = bearing_from_velocity

--- Score one candidate horizon. Returns a table, or nil when it has no plan.
--
--  nil means the CS construction does not solve for this candidate -- most often
--  because the predicted centre has been led so far that the aircraft is inside
--  the ring about it. That is a legitimate exclusion, not an error, and the
--  caller counts it.
--  preferred_direction / sense_margin_m (optional; TASK-048): the held orbit
--  sense applied to every candidate's CS solve. Both nil: unchanged.
function M.score_candidate(px, py, psi_i, est_x, est_y, vx, vy, k_steps,
                           k_prev_steps, dt_s, airspeed_ms, orbit_radius_m,
                           turn_radius_m, delta_psi, delta_d, weights,
                           path_samples, tube_len_m,
                           preferred_direction, sense_margin_m)
    local cx, cy = M.predicted_centre(est_x, est_y, vx, vy, k_steps, dt_s)
    local path = dtc.shortest_path(px, py, psi_i, cx, cy, orbit_radius_m,
                                   turn_radius_m, delta_psi, delta_d,
                                   preferred_direction, sense_margin_m)
    if path == nil then
        return nil
    end

    -- Transit: how long the aircraft takes to fly the planned curve, in ticks.
    local n_a = path.reach / airspeed_ms / dt_s

    -- J_T -- the TASK-038 registration error, at ARRIVAL and signed.
    local ax, ay = path.arrival_x, path.arrival_y
    local tx_at = est_x + vx * n_a * dt_s
    local ty_at = est_y + vy * n_a * dt_s
    local e_tan = math.sqrt((ax - tx_at) * (ax - tx_at)
                            + (ay - ty_at) * (ay - ty_at)) - orbit_radius_m

    -- J_R -- standoff tube over the final tube_len_m of the curve. Each sample
    -- is compared with the target predicted at THAT sample's own arrival time,
    -- which is what makes it a rendezvous measure rather than a snapshot.
    local s_end = path.reach
    local s_start = path.reach - tube_len_m
    if s_start < 0.0 then
        s_start = 0.0
    end
    local n_samples = path_samples
    local total = 0.0
    for i = 0, n_samples - 1 do
        local s
        if n_samples == 1 then
            s = s_end
        else
            s = s_start + (s_end - s_start) * i / (n_samples - 1)
        end
        local qx, qy = dubins.point_at_arc_length(path.points, s)
        local t_i = s / airspeed_ms
        local rx = est_x + vx * t_i
        local ry = est_y + vy * t_i
        local dev = math.sqrt((qx - rx) * (qx - rx) + (qy - ry) * (qy - ry))
                    - orbit_radius_m
        total = total + dev * dev
    end
    local j_radial = total / n_samples

    -- J_kappa -- curvature needed at the bearing this candidate arrives on.
    local v_k = math.sqrt(vx * vx + vy * vy)
    local theta = bearing_from_velocity(ax, ay, cx, cy, vx, vy)
    local kappa_req = M.ring_track_curvature(orbit_radius_m, v_k, airspeed_ms,
                                             theta)
    local feasible, excess
    if kappa_req == nil then
        feasible = false
        -- Unreachable at this bearing. Charge the excess a curvature equal to
        -- the bound would still leave, so the candidate ranks last among those
        -- that solve rather than being silently dropped: an unreachable ring is
        -- a RESULT, and dropping it would hide the boundary.
        excess = 1.0 / turn_radius_m
    else
        feasible = true
        excess = kappa_req - 1.0 / turn_radius_m
        if excess < 0.0 then
            excess = 0.0
        end
    end
    local j_curvature = excess * excess

    -- J_S -- switching. Zero when there is no previous selection to move from.
    local j_switch = 0.0
    if k_prev_steps ~= nil then
        local dk = k_steps - k_prev_steps
        j_switch = dk * dk
    end

    local cost = weights.tangent * e_tan * e_tan
                 + weights.radial * j_radial
                 + weights.curvature * j_curvature
                 + weights.switch * j_switch

    return {
        k_steps = k_steps,
        cost = cost,
        e_tan_m = e_tan,
        n_a_steps = n_a,
        reach_m = path.reach,
        centre_x = cx,
        centre_y = cy,
        arrival_x = ax,
        arrival_y = ay,
        curvature_req = kappa_req,
        feasible = feasible,
        j_tangent = e_tan * e_tan,
        j_radial = j_radial,
        j_curvature = j_curvature,
        j_switch = j_switch,
    }
end

--- Score every candidate in `horizons` and return the least-cost one.
--
--  `horizons` is a 1-indexed array of whole tick counts. Ties break toward the
--  FIRST candidate, so a shorter horizon wins a tie and the selector never
--  prefers prediction it cannot justify.
--
--  Returns the winner with `evaluated` and `solved` added, or nil when no
--  candidate solved.
function M.select_horizon(px, py, psi_i, est_x, est_y, vx, vy, k_prev_steps,
                          dt_s, airspeed_ms, orbit_radius_m, turn_radius_m,
                          delta_psi, delta_d, horizons, weights, path_samples,
                          tube_len_m, preferred_direction, sense_margin_m)
    local best = nil
    local solved = 0
    for i = 1, #horizons do
        local cand = M.score_candidate(px, py, psi_i, est_x, est_y, vx, vy,
                                       horizons[i], k_prev_steps, dt_s,
                                       airspeed_ms, orbit_radius_m,
                                       turn_radius_m, delta_psi, delta_d,
                                       weights, path_samples, tube_len_m,
                                       preferred_direction, sense_margin_m)
        if cand ~= nil then
            solved = solved + 1
            if best == nil or cand.cost < best.cost then
                best = cand
            end
        end
    end
    if best == nil then
        return nil
    end
    best.evaluated = #horizons
    best.solved = solved
    return best
end

-- ---------------------------------------------------------
-- The adapter
-- ---------------------------------------------------------

--- One tick. `cfg` adds ah_candidate_horizons (a 1-indexed array), ah_w_*,
--  ah_path_samples, airspeed_ms and dt_s to what harness_adaptive_db needs.
--
--  `snapshot.target_est_raw` -- the UN-PROJECTED estimate -- is what this reads,
--  because the state-side horizon is the thing it replaces.
function M.guidance_point(snapshot, cfg)
    local px, py = snapshot.plane_e_m, snapshot.plane_n_m
    local psi_i = snapshot.plane_hdg_rad

    local est = snapshot.target_est_raw or snapshot.target_est
    if est == nil then
        return nil, "adaptive_horizon_cs requires the state estimator: it " ..
                    "selects its own prediction horizon from the estimated " ..
                    "target velocity and has no present-position fallback"
    end

    local n_replan = cfg.replan_every
    local policy = cfg.hold_policy
    local st = snapshot.algorithm_state or {}

    local prev_ticks = st.ticks_since_replan
    local replanned = (prev_ticks == nil) or ((prev_ticks + 1) >= n_replan)
    local k_prev = st.k_horizon_steps

    local weights = {
        tangent = cfg.ah_w_tangent,
        radial = cfg.ah_w_radial,
        curvature = cfg.ah_w_curvature,
        switch = cfg.ah_w_switch,
    }
    -- The standoff term scores the final two ring radii of the approach.
    local tube_len_m = 2.0 * cfg.orbit_radius_m

    -- Phase gate (TASK-038's structural finding, enforced rather than hoped
    -- for). The horizon is re-selected only while the aircraft is OUTSIDE the
    -- ring about the centre it is currently holding -- the approach phase,
    -- where a tangent point exists to score. Without this gate the selector
    -- runs on the ring too, where only the SHORT candidates still produce a CS
    -- solution; it then picks a near-zero horizon, the ring stops leading, the
    -- aircraft is pushed off it, the transit grows and the horizon jumps back.
    -- Measured as a limit cycle spanning the whole candidate set.
    local k_hold = k_prev
    if k_hold == nil then
        k_hold = cfg.ah_candidate_horizons[1]
    end
    local hold_x, hold_y = M.predicted_centre(est.e_m, est.n_m, est.ve_ms,
                                              est.vn_ms, k_hold, cfg.dt_s)
    local dxh, dyh = px - hold_x, py - hold_y
    local in_orbit_phase = math.sqrt(dxh * dxh + dyh * dyh) <= cfg.orbit_radius_m

    local chosen = nil
    if replanned and not in_orbit_phase then
        chosen = M.select_horizon(px, py, psi_i, est.e_m, est.n_m, est.ve_ms,
                                  est.vn_ms, k_prev, cfg.dt_s, cfg.airspeed_ms,
                                  cfg.orbit_radius_m, cfg.turn_radius_m,
                                  cfg.delta_psi_rad, cfg.delta_d_m,
                                  cfg.ah_candidate_horizons, weights,
                                  cfg.ah_path_samples, tube_len_m)
    end

    local k_sel, cx, cy, plan, ticks
    if chosen ~= nil then
        k_sel = chosen.k_steps
        cx, cy = chosen.centre_x, chosen.centre_y
        plan = nil
        ticks = 0
    elseif replanned then
        -- Either the phase gate suppressed the selection, or no candidate
        -- solved. Both are the orbit phase: hold the last selection and
        -- re-centre on the target predicted at it, exactly as arm A re-centres
        -- on its fixed horizon. A phase, not a failure -- it must not raise,
        -- and the horizon must not be silently reset to zero.
        k_sel = k_hold
        cx, cy = hold_x, hold_y
        plan = nil
        ticks = 0
    else
        k_sel = k_prev
        cx, cy = st.centre_e_m, st.centre_n_m
        if st.plan_valid then
            plan = { px = st.plan_e_m, py = st.plan_n_m, psi = st.plan_psi_rad }
        else
            plan = nil
        end
        ticks = prev_ticks + 1
    end

    local g, reason = adb.guidance(px, py, psi_i, cx, cy, plan,
                                   cfg.orbit_radius_m, cfg.turn_radius_m,
                                   cfg.look_ahead_m, cfg.delta_psi_rad,
                                   cfg.delta_d_m, policy,
                                   cfg.orbit_precompensate)
    if g == nil then
        return nil, reason
    end

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
        prediction_lead_m = math.sqrt(
            (cx - snapshot.target_e_m) * (cx - snapshot.target_e_m) +
            (cy - snapshot.target_n_m) * (cy - snapshot.target_n_m)),
        plan_n_m = committed.py,
        plan_e_m = committed.px,
        plan_psi_rad = committed.psi,
        plan_valid = plan_valid,
        guidance_n_m = g.gy,
        guidance_e_m = g.gx,
        -- The selection itself, carried EVERY tick and not only on replans, so
        -- a plot of the horizon against time has no holes in it.
        k_horizon_steps = k_sel,
        -- False on the ticks the phase gate suppressed re-selection, so a plot
        -- shows where the horizon was CHOSEN and where it was merely HELD.
        horizon_selected = (chosen ~= nil),
    }
    if g.ring_angle_rad ~= nil then
        state.ring_angle_rad = g.ring_angle_rad
    end
    if chosen ~= nil then
        state.selection_cost = chosen.cost
        -- Signed, and PREDICTED: what the winner expects the TASK-038
        -- registration error to be. The ACHIEVED value is measured offline
        -- against the target's true position at arrival.
        state.e_tan_pred_m = chosen.e_tan_m
        state.n_a_steps = chosen.n_a_steps
        state.candidates_scored = chosen.evaluated
        state.candidates_solved = chosen.solved
        state.ring_feasible = chosen.feasible
    end

    return {
        guidance_n_m = g.gy,
        guidance_e_m = g.gx,
        algorithm_state = state,
    }
end

return M
