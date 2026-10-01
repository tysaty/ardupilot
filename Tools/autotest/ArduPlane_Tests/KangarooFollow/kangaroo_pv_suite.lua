-- =========================================================
--  kangaroo_pv_suite -- fly the physical-validation trajectory suite
--  (PV-1 .. PV-5) as consecutive cases in ONE flight   (TASK-059)
--  created 2026-10-01 as a SCAFFOLD.
--
--  STATUS: NOT IMPLEMENTED. Every function below is a commented TODO for the
--  author to fill in. As written, update() sends one "scaffold" statustext and
--  then idles: it never commands the vehicle, never touches the mission or
--  the fence, and never registers a parameter that could change behaviour.
--
--  What this script is for
--  -----------------------
--  kangaroo_demo.lua (TASK-058) flies ONE kangaroo mode at a time, changed by
--  hand from MAVProxy. TASK-059 needs the SAME fixed set of trajectories at
--  every stage (Python, SITL, physical), so this script runs them as a
--  scripted SEQUENCE of cases in one flight:
--
--      point hold -> PV-1 -> gap -> PV-2a -> gap -> PV-2b -> gap -> PV-3
--                 -> gap -> PV-4 -> gap -> PV-5 -> closing point hold
--
--  Each case is a leg list (harness_segments), every case is checked to FIT
--  the site box BEFORE it starts, and the kangaroo is always virtual and
--  on board (TASK-059 D1 option (a); D1 is still open).
--
--  How multiple modes run in one flight
--  ------------------------------------
--  harness_segments.make_segments(legs, start_n, start_e, opts, t0) already
--  chains legs of DIFFERENT modes into one continuous schedule: each segment
--  starts exactly where the previous one ended, so position is continuous
--  and only velocity steps at a switch (TASK-029). TASK-050's composite used
--  that to put every mode in one run. Two ways to use it here (decision S1):
--
--    (a) ONE schedule: concatenate every case's legs, with point "gap" legs
--        between them, and build it once at the anchor. Simplest; timing is
--        fixed; identical to how the Python composite runs, so the
--        Python-vs-SITL comparison (P5) is direct.
--    (b) PER-CASE rebuild: build case i's legs from wherever the kangaroo is
--        when case i starts (kangaroo_demo.lua's rebuild() does exactly this
--        on a parameter change). Lets a case be skipped, repeated or held by
--        the operator, at the cost of timing that depends on the flight.
--
--  The scaffold is laid out for (b) because it also covers (a): with
--  automatic advance and no skips, (b) flies the same legs as (a).
--
--  Waypoint selection and the boundary
--  -----------------------------------
--  ArduPilot's Lua bindings can READ mission items (mission:num_commands,
--  mission:get_item) but CANNOT read the fence polygon, only its breaches
--  (fence:get_breaches). So:
--
--    * the SITE CENTRE is a mission waypoint chosen by index (KPV_WP_CTR),
--      so the site is set by uploading a mission, not by editing the script;
--    * the SITE BOX is a square of half-side KPV_HALF_M about that centre,
--      set by parameter, and the operator must make it lie INSIDE the
--      ArduPlane fence (flight-card check);
--    * the ArduPlane FENCE stays the independent safety boundary (SR-011).
--      This script only watches fence:get_breaches() and, on a breach, stops
--      commanding; it never sets or relies on the fence to contain anything;
--    * the PV-5 ROAD PATH comes either from a file on the SD card (io.open,
--      as applets/MissionSelector.lua does) or from a run of mission
--      waypoints KPV_PATH_WP1..KPV_PATH_WPN converted to straight legs
--      (decision S3; TASK-059 D2 option "convert to legs").
--
--  Containment is BY DESIGN, not by turning (TASK-050): a case whose
--  schedule does not fit the box less the orbit radius and a margin is
--  refused before it starts. kangaroo_demo.lua's contain() turn is kept only
--  as a safety net and its firing is logged as a finding.
--
--  Reused unchanged (read these first):
--    modules/harness_segments.lua   make_segments, state_at (the schedule)
--    modules/harness_kangaroo.lua   the mode geometry the segments evaluate
--    modules/harness_estimator.lua  the Kalman filter (new, init, update, predict)
--    modules/sitl_arms.lua          arm registry: resolve(name) -> entry
--    modules/sitl_adsb.lua          ADSB_VEHICLE so the GCS map shows the target
--    kangaroo_demo.lua              anchor(), command(), log_tick(), contain(),
--                                   heading_from(): copy, do not re-derive
--    py_harness/kangaroo.py         composite_legs, schedule_fits, fit_to_box:
--                                   the Python reference for case legs and the fit
--
--  OPEN DECISIONS (TASK-059 D1 to D5, plus these scaffold choices):
--    S1  one schedule (a) or per-case rebuild (b)            -- see above
--    S2  advance between cases: on time (default), on the scripting switch
--        (RC option 300), or on KPV_NEXT from the GCS
--    S3  PV-5 path source: SD-card file or mission waypoint run
--    S4  when the pilot leaves GUIDED: kangaroo keeps moving (as the demo)
--        or the case pauses and resumes on re-entry
--    S5  case specs: the TASK-059 proposed table (D4, unconfirmed) rescaled
--        to KPV_HALF_M, or fixed values that are refused if they do not fit
--
--  Units: metres, m/s, seconds, degrees clockwise from North. None of the
--  KPV_ parameters is a flight limit (SR-004); bank, airspeed, fence and
--  failsafes come from the airframe's parameter file.
--
--  Log (planned): HREC / HALG per tick in kangaroo_demo.lua's formats, so
--  extract_bundle.py reads a suite flight like a cell; HPVC once per case
--  start and end (case code, t, fits, worst excursion, why ended).
-- =========================================================

local MAV_SEVERITY = { ERROR = 3, WARNING = 4, INFO = 6 }
local MODE_GUIDED = 15
local SCAFFOLD = true        -- remove when implemented; while true nothing runs

-- ---------------------------------------------------------------------
-- 1. Load the generated configuration and the ported modules
-- ---------------------------------------------------------------------
-- TODO(TASK-059): copy kangaroo_demo.lua lines "Generated configuration and
--  the ported modules": pcall(require, ...) each of
--    "kangaroo_pv_cfg"   -- NEW generated table (see section 9), like
--                           kangaroo_demo_cfg.lua: algorithm, cfg, dt_s,
--                           estimate, lookahead_steps, estimator, cases
--    "harness_segments", "sitl_arms", "sitl_adsb", "harness_estimator"
--  and resolve the arm once:  local entry, src = arms.resolve(pv.algorithm)
--  Refuse to load (statustext + return) on any failure, as fail_load() does.

-- ---------------------------------------------------------------------
-- 2. Parameters (KPV_ = kangaroo physical-validation suite)
-- ---------------------------------------------------------------------
-- TODO(TASK-059): register a table with param:add_table and bind each with
--  param:add_param + Parameter():init, as kangaroo_demo.lua's bind() does.
--  Proposed set (names <= 16 chars with the prefix; defaults to confirm):
--
--    KPV_ENABLE    0/1   master enable; 0 = never command (default 0)
--    KPV_CASES     bitmask of cases to fly: bit0 PV-1 .. bit5 PV-5 (63 = all)
--    KPV_START     index of the first case to fly (resume after a landing)
--    KPV_WP_CTR    mission index of the SITE CENTRE waypoint (0 = home)
--    KPV_HALF_M    site box half-side, m (must sit inside the fence)
--    KPV_MARGIN_M  clearance kept inside the box beyond the orbit radius, m
--                  (TASK-050 COMPOSITE_MARGIN_M = 10; > one tick of travel)
--    KPV_RANGE_M   initial kangaroo distance ahead of the aircraft, m,
--                  clipped so the start lies inside the usable box
--    KPV_GAP_S     point hold between cases, s (lets the orbit settle so
--                  each case starts from the same state)
--    KPV_ADVANCE   S2: 0 on time, 1 scripting switch, 2 KPV_NEXT
--    KPV_NEXT      set 1 from the GCS to end the current case (S2 = 2)
--    KPV_PATH_SRC  S3: 0 SD file, 1 mission waypoints
--    KPV_PATH_WP1  first mission index of the PV-5 road path
--    KPV_PATH_WPN  last mission index of the PV-5 road path
--    KPV_PATH_SPD  PV-5 speed along the path, m/s (TASK-059 cap 12.5)
--    KPV_ALT_M     height above home, m (as KDEM_ALT_M)

-- ---------------------------------------------------------------------
-- 3. The cases (PV-1 .. PV-5) as leg lists
-- ---------------------------------------------------------------------
--- The suite, in flying order. Each entry names a case and how to build its
--  legs. Values are TASK-059's PROPOSED table (D4, not confirmed); a case
--  whose geometry does not fit the box is refused at setup, not shrunk
--  silently (S5).
--
--  A leg is the harness_segments named form:
--    { duration_s =, mode =, heading_deg =, speed_ms = [, elastic_base =] }
local CASES = {
    -- TODO(TASK-059): fill each `legs` from the confirmed specs. Sketch:
    --
    -- { code = 1, id = "PV-1",  what = "straight, speed up and slow down",
    --   legs = { { duration_s = 120, mode = "elastic", heading_deg = 90,
    --              speed_ms = 12.5, elastic_base = "straight" } } },
    --   -- elastic slow/fast come from harness_kangaroo's ELASTIC_* (slow 3)
    --
    -- { code = 2, id = "PV-2a", what = "square, 6.25 m/s",
    --   geometry = { length_m = 300, width_m = 300 },
    --   legs = { { duration_s = 120, mode = "rectangle", heading_deg = 0,
    --              speed_ms = 6.25 } } },
    -- { code = 3, id = "PV-2b", ... same at 12.5 m/s (D3: separate cases) },
    --
    -- { code = 4, id = "PV-3",  what = "circle", geometry = { radius_m = 150 },
    --   legs = { { duration_s = 120, mode = "circle", heading_deg = 0,
    --              speed_ms = 6.25 } } },
    --
    -- { code = 5, id = "PV-4",  what = "random, seeded",
    --   legs_from = "rand" },     -- the RECORDED leg list from the cfg, never a
    --                             -- Lua seed (harness_segments header: seeds do
    --                             -- not transfer between Python and Lua)
    --
    -- { code = 6, id = "PV-5",  what = "road path",
    --   legs_from = "path" },     -- built by path_legs() below (S3)
}

--- The point hold between cases and at the ends.
local function gap_leg(duration_s, heading_deg)
    -- TODO(TASK-059):
    --    return { duration_s = duration_s, mode = "point",
    --             heading_deg = heading_deg, speed_ms = 0.0 }
    return nil
end

-- ---------------------------------------------------------------------
-- 4. Waypoint selection: site centre and the PV-5 path
-- ---------------------------------------------------------------------

--- The site centre as a Location: mission item KPV_WP_CTR, or home for 0.
--  mission:get_item(i) returns a mavlink_mission_item_int_t whose x() / y()
--  are latitude / longitude * 1e7 and command() the MAV_CMD.
local function site_centre(index)
    -- TODO(TASK-059):
    --  * index == 0 -> return ahrs:get_home()
    --  * refuse if index < 1 or index >= mission:num_commands()
    --  * local item = mission:get_item(index)
    --  * refuse unless mission:cmd_has_location(item:command())
    --  * local loc = Location(); loc:lat(item:x()); loc:lng(item:y())
    --  * return loc
    --  Report the chosen index and its offset from home in a statustext, so
    --  the flight record shows which waypoint defined the site.
    return nil, "site_centre: not implemented (TASK-059 scaffold)"
end

--- The PV-5 road path as straight legs from the site centre (S3).
--
--  Mission source (KPV_PATH_SRC 1): items WP1..WPN, each converted to
--  (n, e) about the centre with centre:get_distance_NE(loc); consecutive
--  points become one straight leg each:
--      heading_deg = deg(atan(de, dn)) wrapped to [0, 360)
--      duration_s  = hypot(dn, de) / KPV_PATH_SPD
--  File source (KPV_PATH_SRC 0): the same, read from a CSV of time, lat, lon
--  with io.open (as applets/MissionSelector.lua reads its file), speed from
--  the time stamps capped at KPV_PATH_SPD.
--
--  Validate at load (TASK-059 path mode): at least two points, finite
--  values, monotonic time (file), no leg faster than the cap, every point
--  inside the usable box. Refuse with a reason otherwise.
--  The first point is where PV-5 starts: the previous gap must end there, so
--  add a straight "transit" leg from wherever the kangaroo is (decision:
--  transit at the path speed, or start PV-5 from the kangaroo's position).
--
--  PRIVACY: a real drive recorded for PV-5 stays off the repository
--  (TASK-059 risk); only synthetic or public road geometry is committed.
local function path_legs(centre, src, wp1, wpn, speed_ms)
    -- TODO(TASK-059)
    return nil, "path_legs: not implemented (TASK-059 scaffold)"
end

-- ---------------------------------------------------------------------
-- 5. The boundary: does a case fit before it starts?
-- ---------------------------------------------------------------------

--- Usable half-width of the box for the kangaroo: the ring must stay inside
--  too, so subtract the orbit radius and the margin (TASK-050
--  contained_half_m: side/2 - containment - margin).
local function usable_half_m(half_m, orbit_radius_m, margin_m)
    -- TODO(TASK-059): return half_m - orbit_radius_m - margin_m
    --  and refuse (nil, reason) if it is <= 0: the box is too small to fly
    --  any case with this ring.
    return nil, "usable_half_m: not implemented (TASK-059 scaffold)"
end

--- Port of py_harness/kangaroo.schedule_fits, square box about the centre.
--  Build the segments from the case's start point, sample state_at every
--  dt_s over the case, and track the worst excursion
--      max(|n|, |e|) - usable_half
--  (negative = clearance). Returns fits, worst_excursion_m, t_s, n_m, e_m.
--  Running it on board at case setup costs one pass over the case
--  (1200 samples for 120 s at 10 Hz); if that exceeds the instruction
--  budget, split it across ticks or precompute it on the ground in the cfg.
local function case_fits(legs, start_n, start_e, geometry, usable_half, dt_s)
    -- TODO(TASK-059):
    --    local built, why = segs.make_segments(legs, start_n, start_e, geometry, 0.0)
    --    if built == nil then return nil, why end
    --    local t_end = built[#built].t_end
    --    for i = 0, math.ceil(t_end / dt_s) do
    --        local n, e = segs.state_at(built, math.min(i * dt_s, t_end))
    --        ... track the worst max(|n|, |e|) - usable_half ...
    --    end
    --    return worst <= 1e-6, worst, ...
    return nil, "case_fits: not implemented (TASK-059 scaffold)"
end

-- ---------------------------------------------------------------------
-- 6. Case sequencing (the state machine)
-- ---------------------------------------------------------------------
--
--   IDLE ──(armed, GUIDED, KPV_ENABLE 1)──▶ ANCHOR
--   ANCHOR: fix origin = site_centre(KPV_WP_CTR); place the kangaroo
--           KPV_RANGE_M ahead of the aircraft, clipped into the usable box;
--           init the estimator there (as kangaroo_demo.lua anchor())
--   ──▶ GAP (opening point hold, KPV_GAP_S)
--   ──▶ SETUP(i): next enabled case at or after KPV_START (KPV_CASES bit);
--           build its legs from the kangaroo's CURRENT position; run
--           case_fits; if it does not fit, log HPVC "refused", skip to i+1
--   ──▶ RUN(i): until the case's legs end (S2 = 0), the switch moves
--           (S2 = 1) or KPV_NEXT is set (S2 = 2)
--   ──▶ GAP ──▶ SETUP(i+1) ... ──▶ DONE: closing point hold, kangaroo
--           stationary, aircraft orbits it until the pilot takes over
--
--   Any state, every tick, checked FIRST:
--     * mode not GUIDED or not armed -> stop commanding at once (SR-009);
--       S4 decides whether the kangaroo pauses
--     * fence:get_breaches() ~= 0     -> stop commanding, HPVC "breach",
--       statustext; the fence's own action (RTL etc.) does the rest (SR-011)
--     * KPV_ENABLE 0                  -> stop commanding
--
local STATE = { IDLE = 0, ANCHOR = 1, GAP = 2, SETUP = 3, RUN = 4, DONE = 5 }

--- The next enabled case index at or after i, or nil when none remain.
local function next_case(i, mask)
    -- TODO(TASK-059): walk CASES from i; return the first whose bit
    --  (CASES[k].code - 1) is set in mask.
    return nil
end

--- Build case i from the kangaroo's current (n, e) at time t, check it fits,
--  and replace the schedule. Returns true, or false plus a reason (refused).
local function setup_case(i, t, n, e)
    -- TODO(TASK-059):
    --  1. legs: CASES[i].legs, or the recorded rand legs (PV-4), or
    --     path_legs(...) (PV-5)
    --  2. fits = case_fits(legs, n, e, geometry, usable_half, dt_s)
    --  3. refuse if not fits: HPVC (code, t, 0, worst, why=refused)
    --  4. segments = segs.make_segments(legs, n, e, geometry, t)   -- t0 = t
    --  5. HPVC (code, t, 1, worst, why=start); statustext "KPV: PV-x started"
    return false, "setup_case: not implemented (TASK-059 scaffold)"
end

--- Has the running case finished? (S2)
local function case_done(t)
    -- TODO(TASK-059):
    --  KPV_ADVANCE 0: t >= segments[#segments].t_end
    --  KPV_ADVANCE 1: scripting switch (rc:find_channel_for_option(300),
    --                 :get_aux_switch_pos()) moved to high since case start
    --  KPV_ADVANCE 2: KPV_NEXT ~= 0 (then set it back to 0)
    return false
end

-- ---------------------------------------------------------------------
-- 7. One control tick (same order as kangaroo_demo.lua step())
-- ---------------------------------------------------------------------
local function step(now_ms)
    -- TODO(TASK-059):
    --  a. safety gates (section 6, "Any state")
    --  b. advance the state machine (IDLE / ANCHOR / GAP / SETUP / RUN / DONE)
    --  c. kangaroo truth: kn, ke, kvn, kve = segs.state_at(segments, t)
    --  d. estimator:update(kn, ke, dt_s); est_raw; est_proj = predict(...)
    --  e. result = entry(snapshot, cfg)   -- the chosen arm (sitl_arms)
    --  f. command(pn, pe, result.guidance_n_m, result.guidance_e_m)
    --  g. log_tick(...) and, on a case change, HPVC
    --  h. sitl_adsb.send(...) every 200 ms so the GCS shows the kangaroo
end

-- ---------------------------------------------------------------------
-- 8. Entry point
-- ---------------------------------------------------------------------
local announced = false

local function update()
    if SCAFFOLD then
        if not announced then
            gcs:send_text(MAV_SEVERITY.WARNING,
                          "KPV: scaffold only (TASK-059), not commanding")
            announced = true
        end
        return update, 1000
    end
    local ok, err = pcall(step, millis():toint())
    if not ok then
        gcs:send_text(MAV_SEVERITY.ERROR, "KPV: " .. tostring(err))
        return update, 1000
    end
    return update, 100      -- TODO(TASK-059): period from cfg dt_s
end

-- ---------------------------------------------------------------------
-- 9. After it is implemented (checklist; nothing here runs)
-- ---------------------------------------------------------------------
--
--  Generated config: a kangaroo_follow/pv_suite.py beside demo.py, writing
--  modules/kangaroo_pv_cfg.lua from a Python cell spec (arm, flattened
--  config, estimator noise) plus the case legs and the recorded PV-4 legs,
--  through schedule.cell_table and kangaroo.composite_legs. Nothing typed by
--  hand (TASK-052 P4). Staging via stage_scripts' state file, as demo.py.
--
--  Python first: the same case sequence as one schedule in the harness
--  (S1 (a)) gives the reference every SITL and physical run is read against
--  (TASK-059 Stage 1, P5).
--
--  Tests: case_fits against kangaroo.schedule_fits on the same legs; path_legs
--  against a synthetic path; next_case over masks; the safety gates in SITL
--  (leave GUIDED mid-case: commands stop; fence breach: commands stop).
--
--  SITL before any flight: the TASK-058 criteria C1 to C5 and TASK-059 P1 to
--  P5, then docs/PHYSICAL_FLIGHT_PROCESS.md. Nothing flies physically before
--  the SR-010 entry checklist is signed.

return update()
