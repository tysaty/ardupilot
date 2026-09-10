-- =========================================================
--  harness_segments -- scripted leg chaining, and the kangaroo_rand replay
--  created 2026-09-03
--  TASK-006 Tranche 7 (forward port).
--
--  Chains legs into continuous segments (TASK-029) and evaluates the chain at a
--  time t, so the scripted and interactive kangaroo schedules -- and
--  kangaroo_rand's -- are all one mechanism.
--
--  THE PRNG PROBLEM, and how it is resolved here
--  ---------------------------------------------
--  kangaroo_rand has a problem that is NOT a porting problem. It draws from
--  Python's random.Random(seed) -- a Mersenne Twister. Lua's math.random is a
--  different generator (in 5.3, the C library's rand or an xoshiro in 5.4), so
--  THE SAME SEED CANNOT PRODUCE THE SAME TRAJECTORY, and PR-004's "identical
--  seed gives identical output" cannot hold across the two languages by
--  transliteration alone. A-SW-003 is already recorded as Challenged; this is
--  the concrete instance.
--
--  TASK-006 gives three options and requires one to be chosen and its limitation
--  recorded before the tranche opens.
--
--    (a) Port a small deterministic PRNG so both sides share a generator.
--        REJECTED for now: kangaroo.py would have to stop using random.Random,
--        which is a change to a harness algorithm during a port -- exactly what
--        TASK-006's no-behaviour-change rule forbids, and it would silently
--        invalidate every previously recorded seeded run.
--    (b) Drive the Lua from a RECORDED sequence emitted by the Python, making it
--        a REPLAY rather than a re-generation.  <-- CHOSEN.
--    (c) Accept that seeds do not transfer and compare statistical properties.
--        REJECTED: a materially weaker claim, and it would not have caught the
--        chaining and continuity defects a replay does.
--
--  What (b) proves and what it does not
--  ------------------------------------
--  PROVES: given the same schedule, the two implementations chain it
--  identically -- the continuity offsets, the segment boundaries, the clamping
--  past the end, and every sub-mode's evaluation agree. That is where the
--  porting risk actually is.
--  DOES NOT PROVE: that a seed transfers. It does not, and no test here claims
--  it does. Anything on the vehicle that needs a reproducible random target must
--  carry the schedule, not the seed.
--
--  Frame: (north, east) metres, as harness_kangaroo. Stateless.
-- =========================================================

local kang = require("harness_kangaroo")

local M = {}

--- Build continuous segments from a leg list.
--
--  `legs` is a 1-indexed array of {duration_s, mode, heading_deg, speed_ms}.
--  Each segment records its own start time, end time, mode parameters and the
--  POSITION OFFSET that makes it begin exactly where the previous one ended --
--  which is what keeps position continuous while allowing velocity to step at a
--  switch (a deliberate mode change, not a discontinuity to be smoothed).
--
--  `opts` supplies radius_m, length_m, width_m for the modes that need them.
--
--  Returns a 1-indexed array of segments, or nil plus a reason.
function M.make_segments(legs, start_n, start_e, opts, t0)
    opts = opts or {}
    local radius_m = opts.radius_m or 150.0
    local length_m = opts.length_m or 300.0
    local width_m = opts.width_m or 150.0
    if #legs == 0 then
        return nil, "need at least one leg"
    end
    local segments = {}
    local t_cursor = t0 or 0.0
    local pos_n, pos_e = start_n, start_e

    for i = 1, #legs do
        local leg = legs[i]
        local duration_s = leg.duration_s or leg[1]
        local mode = leg.mode or leg[2]
        local heading_deg = leg.heading_deg or leg[3]
        local speed_ms = leg.speed_ms or leg[4]
        if duration_s == nil or duration_s <= 0.0 then
            return nil, "leg duration must be positive"
        end
        if mode ~= "point" and mode ~= "straight" and mode ~= "circle"
                and mode ~= "rectangle" and mode ~= "elastic" then
            return nil, "unknown leg mode"
        end
        if speed_ms < 0.0 then
            return nil, "leg speed must be >= 0"
        end

        local seg = {
            t_start = t_cursor,
            t_end = t_cursor + duration_s,
            mode = mode,
            heading_deg = heading_deg,
            speed_ms = speed_ms,
            radius_m = radius_m,
            length_m = length_m,
            width_m = width_m,
            off_n = 0.0,
            off_e = 0.0,
        }

        -- The offset is measured against the sub-mode's OWN position at local
        -- t = 0, which is not necessarily the origin: circle_state(0) sits one
        -- radius from its centre, and rectangle_state(0) at a corner. Offsetting
        -- from the accumulated END position instead would displace every
        -- non-straight leg by that amount and the error would grow leg by leg.
        local s0n, s0e = M.local_state(seg, 0.0)
        if s0n == nil then
            return nil, s0e
        end
        seg.off_n = pos_n - s0n
        seg.off_e = pos_e - s0e
        segments[#segments + 1] = seg

        local en, ee = M.local_state(seg, duration_s)
        if en == nil then
            return nil, ee
        end
        pos_n = en + seg.off_n
        pos_e = ee + seg.off_e
        t_cursor = seg.t_end
    end
    return segments
end

--- One segment evaluated in its OWN local frame at local time `t_local`,
--  i.e. placed at the origin with no offset. Returns n, e, vn, ve.
function M.local_state(seg, t_local)
    return kang.state(seg.mode, t_local, {
        heading_deg = seg.heading_deg,
        fwd_m = 0.0,
        disp_m = 0.0,
        speed_ms = seg.speed_ms,
        radius_m = seg.radius_m,
        length_m = seg.length_m,
        width_m = seg.width_m,
        base_mode = "straight",
        slow_ms = seg.speed_ms * kang.ELASTIC_SLOW_FACTOR,
        fast_ms = seg.speed_ms,
        hold_s = kang.ELASTIC_HOLD_S,
        ramp_s = kang.ELASTIC_RAMP_S,
    })
end

--- Evaluate a chained schedule at absolute time t. Returns n, e, vn, ve.
--
--  Past the end of the schedule the LAST segment is clamped and continues to be
--  evaluated at its final local time -- so a run that outlives its schedule
--  freezes the target rather than jumping to the origin or erroring. That is the
--  harness's behaviour and it is transliterated, not improved.
function M.state_at(segments, t)
    if #segments == 0 then
        return nil, "no segments"
    end
    local seg = nil
    for i = 1, #segments do
        local s = segments[i]
        if s.t_start <= t and t < s.t_end then
            seg = s
            break
        end
    end
    if seg == nil then
        -- Outside the scheduled span the NEAREST segment is clamped, so a run
        -- that outlasts its schedule holds the last leg rather than failing or
        -- teleporting to the origin.
        if t >= segments[#segments].t_end then
            seg = segments[#segments]
        else
            seg = segments[1]
        end
        if t < seg.t_start then
            t = seg.t_start
        elseif t > seg.t_end then
            t = seg.t_end
        end
    end
    local n, e, vn, ve = M.local_state(seg, t - seg.t_start)
    if n == nil then
        return nil, e
    end
    return n + seg.off_n, e + seg.off_e, vn, ve
end

return M
