"""Kangaroo (target) motion modes, ported from ``kangaroo_MAV.lua`` (``TASK-009``).

The shipping target simulator drives the virtual kangaroo in one of five modes;
this ports the four **deterministic** ones — point, straight, circle and
rectangle — as pure kinematic functions of time. ``FR-014``'s stationary,
straight, circular and rectangular scenarios map onto these directly.

``KANG_RANDOM`` (the Lua's hop-burst random **walk**) is **deliberately not
ported** — recorded as outstanding in
``docs/decisions/ADR-003-kangaroo-random-walk-deferred.md`` and
``PROJECT_UPDATES.md``.

``kangaroo_rand`` (`RAND_MODE`, `TASK-021`) is a **different** mechanism and does
not reinstate that walk: it switches among the four deterministic modes at random
time intervals and is **reproducible from a seed** (the Lua seeded from
``millis()`` and recorded no seed — `A-SW-003`). Position stays continuous across
switches; velocity may step at a switch.

Each mode is a stateless ``state(t, ...) -> (n, e, vn, ve)`` in North/East metres
and m/s. :func:`build` binds a mode's parameters into a single
``kangaroo(t) -> (n, e, vn, ve)`` callable that the harness steps. Plain ``math``,
no ``numpy``; the geometry mirrors the Lua so it transliterates back.

Frame and conventions match the Lua and the harness (``IR-001`` to ``IR-003``):
heading degrees clockwise from North; ``heading_frame_offset`` places a point a
forward distance along the heading and a lateral displacement to its right.
"""

import math
import random

#: The four deterministic modes ported from the Lua (`TASK-009`).
MODES = ("point", "straight", "circle", "rectangle")

#: Random-interval mode switcher (`TASK-021`) — a meta-mode built from `MODES`.
RAND_MODE = "kangaroo_rand"

#: Everything selectable via ``--kang-mode``.
ALL_MODES = MODES + (RAND_MODE, "elastic")

#: Modes a scripted leg or a GUI control may select. `kangaroo_rand` is excluded
#: because it is itself a schedule, and nesting one inside a leg would be
#: ambiguous about which schedule owns the target.
LEG_MODES = MODES + ("elastic",)


def heading_frame_offset(heading_deg, fwd_m, disp_m):
    """Point ``fwd_m`` along ``heading`` and ``disp_m`` to its right, as ``(n, e)``.

    Ported from ``kangaroo_MAV.lua`` (the straight/point initial placement):
    ``n = cos·fwd − sin·disp``, ``e = sin·fwd + cos·disp``.
    """
    h = math.radians(heading_deg)
    n = math.cos(h) * fwd_m - math.sin(h) * disp_m
    e = math.sin(h) * fwd_m + math.cos(h) * disp_m
    return n, e


def point_state(t, heading_deg, fwd_m, disp_m):
    """Stationary point (``KANG_POINT``). ``t`` is ignored; velocity is zero."""
    n, e = heading_frame_offset(heading_deg, fwd_m, disp_m)
    return n, e, 0.0, 0.0


def straight_state(t, heading_deg, fwd_m, disp_m, speed_ms):
    """Straight line from the placed start on ``heading`` at ``speed_ms``."""
    n0, e0 = heading_frame_offset(heading_deg, fwd_m, disp_m)
    h = math.radians(heading_deg)
    vn = math.cos(h) * speed_ms
    ve = math.sin(h) * speed_ms
    return n0 + vn * t, e0 + ve * t, vn, ve


def circle_state(t, heading_deg, fwd_m, disp_m, radius_m, speed_ms):
    """Circle of ``radius_m`` about the placed centre, at ``speed_ms``.

    ``omega = speed/radius``; the point starts at angle 0 (``centre + (r, 0)``),
    matching ``integrate_circle``: ``n = cn + r·cos(a)``, ``e = ce + r·sin(a)``.

    Raises:
        ValueError: For a non-positive radius (division by zero).
    """
    if radius_m <= 0.0:
        raise ValueError("circle radius must be positive, got %r" % radius_m)
    cn, ce = heading_frame_offset(heading_deg, fwd_m, disp_m)
    a = (speed_ms / radius_m) * t
    n = cn + radius_m * math.cos(a)
    e = ce + radius_m * math.sin(a)
    vn = -speed_ms * math.sin(a)
    ve = speed_ms * math.cos(a)
    return n, e, vn, ve


def rectangle_state(t, heading_deg, fwd_m, disp_m, length_m, width_m, speed_ms):
    """Constant-speed traversal of a ``length x width`` rectangle perimeter.

    Corners are the rotated rectangle of ``integrate_rectangle``, traversed
    corner 0 -> 1 -> 2 -> 3 -> 0. The along-perimeter distance is ``speed*t``,
    wrapped by the perimeter, so the target loops the rectangle.

    Raises:
        ValueError: For a non-positive length or width.
    """
    if length_m <= 0.0 or width_m <= 0.0:
        raise ValueError(
            "rectangle needs positive length and width, got L=%r W=%r"
            % (length_m, width_m)
        )
    on, oe = heading_frame_offset(heading_deg, fwd_m, disp_m)
    h = math.radians(heading_deg)
    ch, sh = math.cos(h), math.sin(h)
    corners = [
        (on, oe),
        (on + length_m * ch, oe + length_m * sh),
        (on + length_m * ch - width_m * sh, oe + length_m * sh + width_m * ch),
        (on - width_m * sh, oe + width_m * ch),
    ]
    perimeter = 2.0 * (length_m + width_m)
    d = (speed_ms * t) % perimeter if speed_ms > 0.0 else 0.0
    for i in range(4):
        s0 = corners[i]
        s1 = corners[(i + 1) % 4]
        dn, de = s1[0] - s0[0], s1[1] - s0[1]
        side_len = math.hypot(dn, de)
        if d <= side_len or i == 3:
            frac = 0.0 if side_len <= 0.0 else min(d, side_len) / side_len
            ux, uy = (dn / side_len, de / side_len) if side_len > 0.0 else (0.0, 0.0)
            return s0[0] + dn * frac, s0[1] + de * frac, ux * speed_ms, uy * speed_ms
        d -= side_len
    return corners[0][0], corners[0][1], 0.0, 0.0


def _sub_state_fn(mode, heading_deg, radius_m, length_m, width_m, speed_ms,
                  elastic_base=None):
    """A local-time ``state(t) -> (n, e, vn, ve)`` for one segment at the origin.

    ``fwd``/``disp`` are 0 — the segment's absolute placement comes from the
    continuity offset in :func:`_rand_segments`, not from the sub-mode's own start.

    ``elastic_base`` (`TASK-045` D2) is the base mode an ``elastic`` leg is
    travelled over; ``None`` keeps the pre-`TASK-045` straight base, so every
    existing four-element leg is unchanged.
    """
    if mode == "point":
        return lambda t: point_state(t, heading_deg, 0.0, 0.0)
    if mode == "straight":
        return lambda t: straight_state(t, heading_deg, 0.0, 0.0, speed_ms)
    if mode == "circle":
        return lambda t: circle_state(t, heading_deg, 0.0, 0.0, radius_m, speed_ms)
    if mode == ELASTIC_MODE:
        # A scripted/interactive elastic leg: the leg's speed is the fast phase,
        # over the leg's base mode (straight unless the leg says otherwise),
        # using the module defaults for the profile shape (TASK-030). A
        # different profile shape is still a `build()` call.
        base = DEFAULT_ELASTIC_BASE if elastic_base is None else elastic_base
        return lambda t: elastic_state(
            t, base, heading_deg, 0.0, 0.0, radius_m, length_m, width_m,
            speed_ms * ELASTIC_SLOW_FACTOR, speed_ms)
    return lambda t: rectangle_state(t, heading_deg, 0.0, 0.0, length_m, width_m,
                                     speed_ms)


def _rand_segments(seed, start_n, start_e, radius_m, length_m, width_m, speed_ms,
                   min_seg_s, max_seg_s, horizon_s):
    """Seeded, deterministic schedule of continuous segments up to ``horizon_s``.

    Each segment is ``(t_start, t_end, sub_state, off_n, off_e)``. The offset makes
    a segment begin exactly where the previous ended, so **position is
    continuous**; velocity may step at a switch (a deliberate mode change). The
    same ``seed`` always yields the same schedule (`A-SW-003` — reproducible,
    unlike the Lua which seeded from ``millis()``).
    """
    legs = rand_legs(seed, speed_ms, min_seg_s, max_seg_s, horizon_s)
    # Same chaining engine as the scripted form (`TASK-029`); random legs in,
    # continuous segments out.
    return make_segments(legs, start_n, start_e, radius_m, length_m, width_m)


def rand_legs(seed, speed_ms, min_seg_s, max_seg_s, horizon_s):
    """The seeded leg list behind ``kangaroo_rand``, as ordinary scripted legs.

    ``[(duration_s, mode, heading_deg, speed_ms), ...]`` covering at least
    ``horizon_s``. Lifted out of :func:`_rand_segments` (`TASK-045`) so an
    experiment spec — whose legs must be explicit — can carry a
    ``kangaroo_rand`` schedule **and** the seed that produced it: the legs are
    the reproducible expansion of ``seed + config``, and the seed is recorded
    beside them rather than replacing them.
    """
    rng = random.Random(seed)
    legs, t0 = [], 0.0
    while t0 < horizon_s:
        # Draw order is load-bearing: mode, then heading, then duration. Changing
        # it changes every seeded trajectory (`A-SW-003`).
        mode = rng.choice(MODES)
        heading = rng.uniform(0.0, 360.0)
        dur = rng.uniform(min_seg_s, max_seg_s)
        legs.append((dur, mode, heading, speed_ms))
        t0 += dur
    return legs


def _build_rand(seed, heading_deg, fwd_m, disp_m, radius_m, length_m, width_m,
                speed_ms, min_seg_s, max_seg_s, horizon_s):
    """Build the ``kangaroo_rand`` closure (`TASK-021`)."""
    if seed is None:
        seed = 0
    if not (0.0 < min_seg_s <= max_seg_s):
        raise ValueError("need 0 < rand_min_s <= rand_max_s, got %r, %r"
                         % (min_seg_s, max_seg_s))
    start_n, start_e = heading_frame_offset(heading_deg, fwd_m, disp_m)
    segments = _rand_segments(seed, start_n, start_e, radius_m, length_m, width_m,
                              speed_ms, min_seg_s, max_seg_s, horizon_s)

    return segments_callable(segments)


# --------------------------------------------------------------------------
# Elastic speed profile (``TASK-030``)
# --------------------------------------------------------------------------
# A kangaroo does not travel at a constant pace: it surges, holds, eases, holds.
# The four base modes above are all parameterised by **arc length** — position
# depends on ``speed * t`` and nothing else — so a varying pace can be applied to
# any of them by replacing that product with an integrated distance. Elastic is
# therefore a **modifier over the existing modes**, not a fourth straight-line
# variant: `elastic` + `circle` is a kangaroo bounding round a circle.
#
# The profile holds the slow speed, ramps up, holds the fast speed, ramps down,
# and repeats. Ramps use the same ``smoothstep`` the weave uses (``TASK-004``),
# so acceleration is finite rather than a step.

#: Default fraction of the commanded speed the slow phase runs at.
ELASTIC_SLOW_FACTOR = 0.3

#: Default seconds held at each speed before ramping to the other.
ELASTIC_HOLD_S = 8.0

#: Default seconds spent ramping between them.
ELASTIC_RAMP_S = 4.0

#: Meta-mode name, alongside ``kangaroo_rand``.
ELASTIC_MODE = "elastic"

#: Base mode an ``elastic`` leg is travelled over when the leg names none — the
#: only base a scripted leg could reach before `TASK-045`.
DEFAULT_ELASTIC_BASE = "straight"

#: Base modes an ``elastic`` leg may name (`TASK-045` D2). ``point`` is excluded:
#: a stationary target has no pace to vary.
ELASTIC_BASES = tuple(m for m in MODES if m != "point")


def smoothstep(q):
    """``3q^2 - 2q^3`` on ``[0, 1]``. Same curve the weave uses."""
    if q <= 0.0:
        return 0.0
    if q >= 1.0:
        return 1.0
    return q * q * (3.0 - 2.0 * q)


def _smoothstep_integral(q):
    """``integral of smoothstep from 0 to q`` = ``q^3 - q^4/2``, for ``q`` in [0,1].

    Needed because position is the integral of speed: a closed form keeps the
    mode a pure function of ``t``, with no accumulator and no per-tick state
    (``VR-015``, ``A-VAL-003``).
    """
    if q <= 0.0:
        return 0.0
    if q >= 1.0:
        return 0.5
    return q ** 3 - 0.5 * q ** 4


def elastic_period_s(hold_s, ramp_s):
    """One full slow-ramp-fast-ramp cycle, seconds."""
    return 2.0 * (hold_s + ramp_s)


def elastic_speed(t, slow_ms, fast_ms, hold_s=ELASTIC_HOLD_S,
                  ramp_s=ELASTIC_RAMP_S):
    """Speed at time ``t``: hold slow, ramp up, hold fast, ramp down, repeat.

    Raises:
        ValueError: For negative speeds, a negative hold, or a non-positive ramp.
    """
    if slow_ms < 0.0 or fast_ms < 0.0:
        raise ValueError("elastic speeds must be >= 0, got %r and %r"
                         % (slow_ms, fast_ms))
    if hold_s < 0.0:
        raise ValueError("elastic hold must be >= 0, got %r" % hold_s)
    if ramp_s <= 0.0:
        raise ValueError("elastic ramp must be > 0, got %r" % ramp_s)
    span = fast_ms - slow_ms
    u = t % elastic_period_s(hold_s, ramp_s)
    if u < hold_s:
        return slow_ms
    u -= hold_s
    if u < ramp_s:
        return slow_ms + span * smoothstep(u / ramp_s)
    u -= ramp_s
    if u < hold_s:
        return fast_ms
    u -= hold_s
    return fast_ms - span * smoothstep(u / ramp_s)


def elastic_distance(t, slow_ms, fast_ms, hold_s=ELASTIC_HOLD_S,
                     ramp_s=ELASTIC_RAMP_S):
    """Distance travelled by time ``t`` under :func:`elastic_speed`, metres.

    The closed-form integral of the profile. Over a whole cycle the mean speed is
    exactly ``(slow + fast)/2``, because a smoothstep ramp contributes the same
    as half its span held — which makes the mode's average pace predictable
    rather than something to be measured.
    """
    if t <= 0.0:
        return 0.0
    span = fast_ms - slow_ms
    period = elastic_period_s(hold_s, ramp_s)
    per_cycle = 0.5 * (slow_ms + fast_ms) * period
    whole, u = divmod(t, period)
    dist = whole * per_cycle

    take = min(u, hold_s)                          # slow hold
    dist += slow_ms * take
    u -= take
    if u <= 0.0:
        return dist

    take = min(u, ramp_s)                          # ramp up
    dist += slow_ms * take + span * ramp_s * _smoothstep_integral(take / ramp_s)
    u -= take
    if u <= 0.0:
        return dist

    take = min(u, hold_s)                          # fast hold
    dist += fast_ms * take
    u -= take
    if u <= 0.0:
        return dist

    take = min(u, ramp_s)                          # ramp down
    dist += fast_ms * take - span * ramp_s * _smoothstep_integral(take / ramp_s)
    return dist


def elastic_state(t, base_mode, heading_deg, fwd_m, disp_m, radius_m, length_m,
                  width_m, slow_ms, fast_ms, hold_s=ELASTIC_HOLD_S,
                  ramp_s=ELASTIC_RAMP_S):
    """``base_mode`` travelled at an elastic pace (``TASK-030``).

    The base mode is evaluated at **unit speed**, so its argument is arc length
    directly and its velocity is the unit tangent; the elastic distance and speed
    are then substituted in. That is why this works for ``straight``, ``circle``
    and ``rectangle`` alike without any of them changing.

    Raises:
        ValueError: For ``point`` (a stationary target has no pace to vary), an
            unknown base mode, or an invalid profile.
    """
    base_mode = str(base_mode).lower()
    if base_mode == "point":
        raise ValueError("elastic needs a moving base mode; 'point' is stationary")
    if base_mode not in MODES:
        raise ValueError("unknown elastic base mode %r; use one of %s"
                         % (base_mode, ", ".join(m for m in MODES if m != "point")))
    dist = elastic_distance(t, slow_ms, fast_ms, hold_s, ramp_s)
    speed = elastic_speed(t, slow_ms, fast_ms, hold_s, ramp_s)
    # Unit-speed base: its time argument is arc length, its velocity a unit vector.
    if base_mode == "straight":
        n, e, vn, ve = straight_state(dist, heading_deg, fwd_m, disp_m, 1.0)
    elif base_mode == "circle":
        n, e, vn, ve = circle_state(dist, heading_deg, fwd_m, disp_m, radius_m, 1.0)
    else:
        n, e, vn, ve = rectangle_state(dist, heading_deg, fwd_m, disp_m,
                                       length_m, width_m, 1.0)
    return n, e, vn * speed, ve * speed


#: One scripted leg. ``speed_ms`` is **per segment** — the whole point of the
#: scripted form, and the thing `kangaroo_rand` cannot express (it carries one
#: speed for the entire run).
SEGMENT_FIELDS = ("duration_s", "mode", "heading_deg", "speed_ms")

#: The optional fifth leg field (`TASK-045` D2): the base mode an ``elastic``
#: leg is travelled over. Absent means :data:`DEFAULT_ELASTIC_BASE`.
ELASTIC_BASE_FIELD = "elastic_base"


def leg_elastic_base(leg):
    """The ``elastic_base`` a leg names, or ``None`` for a four-element leg."""
    if len(leg) > 4 and leg[4] is not None:
        return str(leg[4]).lower()
    return None


def make_segments(legs, start_n, start_e, radius_m=150.0, length_m=300.0,
                  width_m=150.0, t0=0.0):
    """Chain ``legs`` into continuous segments starting at ``(start_n, start_e)``.

    ``legs`` is a sequence of ``(duration_s, mode, heading_deg, speed_ms)``, or
    ``(duration_s, mode, heading_deg, speed_ms, elastic_base)`` for an
    ``elastic`` leg over a non-straight base (`TASK-045` D2; see
    :data:`ELASTIC_BASES`). Each segment carries a positional offset so it
    **begins exactly where the previous ended**: position is continuous across a
    switch and only velocity steps, which is what a manoeuvre is (``TASK-029``).

    This is the engine `kangaroo_rand` has always used (`_rand_segments`), lifted
    out so the scripted and random forms share one implementation rather than
    growing two. The difference is only where the legs come from.

    Returns ``[(t_start, t_end, sub_state, off_n, off_e), ...]``.

    Raises:
        ValueError: For an empty leg list, a non-positive duration, a negative
            speed, or an unknown mode.
    """
    if not legs:
        raise ValueError("need at least one leg")
    segments = []
    t, pos_n, pos_e = float(t0), float(start_n), float(start_e)
    for i, leg in enumerate(legs):
        dur, mode, heading_deg, speed_ms = leg[:4]
        elastic_base = leg_elastic_base(leg)
        if dur <= 0.0:
            raise ValueError("leg %d: duration must be > 0, got %r" % (i, dur))
        if speed_ms < 0.0:
            raise ValueError("leg %d: speed must be >= 0, got %r" % (i, speed_ms))
        mode = str(mode).lower()
        if mode not in LEG_MODES:
            raise ValueError("leg %d: unknown mode %r; use one of %s"
                             % (i, mode, ", ".join(LEG_MODES)))
        if elastic_base is not None and elastic_base not in ELASTIC_BASES:
            raise ValueError("leg %d: unknown elastic_base %r; use one of %s"
                             % (i, elastic_base, ", ".join(ELASTIC_BASES)))
        sub = _sub_state_fn(mode, heading_deg, radius_m, length_m, width_m,
                            speed_ms, elastic_base)
        s0n, s0e, _, _ = sub(0.0)
        off_n, off_e = pos_n - s0n, pos_e - s0e
        segments.append((t, t + dur, sub, off_n, off_e))
        en, ee, _, _ = sub(dur)
        pos_n, pos_e = en + off_n, ee + off_e
        t += dur
    return segments


def segments_callable(segments):
    """Wrap segments into a ``kangaroo(t) -> (n, e, vn, ve)`` callable.

    Outside the scheduled span the nearest segment is clamped, so a run that
    outlasts its schedule holds the last leg rather than failing or teleporting.
    """
    def kangaroo(t):
        seg = None
        for s in segments:
            if s[0] <= t < s[1]:
                seg = s
                break
        if seg is None:
            seg = segments[-1] if t >= segments[-1][1] else segments[0]
            t = min(max(t, seg[0]), seg[1])
        t_start, _t_end, sub, off_n, off_e = seg
        n, e, vn, ve = sub(t - t_start)
        return n + off_n, e + off_e, vn, ve
    return kangaroo


def build_scripted(legs, heading_deg=0.0, fwd_m=300.0, disp_m=0.0,
                   radius_m=150.0, length_m=300.0, width_m=150.0, t0=0.0,
                   start_n=None, start_e=None):
    """A kangaroo following an explicit schedule of legs (``TASK-029``).

    The authored counterpart to ``kangaroo_rand``: same continuity guarantees and
    the same callable shape the harness already accepts, but the legs are declared
    rather than drawn from a seeded random source.

    ``start_n``/``start_e`` override the ``heading_deg``/``fwd_m``/``disp_m``
    placement, which is what an interactive relocation needs (``TASK-008``).
    """
    if start_n is None or start_e is None:
        start_n, start_e = heading_frame_offset(heading_deg, fwd_m, disp_m)
    return segments_callable(
        make_segments(legs, start_n, start_e, radius_m, length_m, width_m, t0))


def build(mode, heading_deg=0.0, fwd_m=300.0, disp_m=0.0, radius_m=150.0,
          length_m=300.0, width_m=150.0, speed_ms=5.0, seed=None,
          rand_min_s=5.0, rand_max_s=20.0, rand_horizon_s=3600.0,
          elastic_base="straight", elastic_slow_factor=ELASTIC_SLOW_FACTOR,
          elastic_slow_ms=None, elastic_hold_s=ELASTIC_HOLD_S,
          elastic_ramp_s=ELASTIC_RAMP_S):
    """Bind a mode's parameters into a ``kangaroo(t) -> (n, e, vn, ve)`` callable.

    The bound parameters are immutable constants (not module state), so the
    deterministic closures transliterate to Lua without breaking ``VR-015``.
    ``kangaroo_rand`` (`TASK-021`) switches among the deterministic modes at random
    intervals; it is **reproducible from ``seed``** and continuous in position.
    ``elastic`` (`TASK-030`) applies a surge-and-ease speed profile over a moving
    base mode: ``speed_ms`` is the fast phase and the slow phase is
    ``elastic_slow_factor`` of it, so one commanded speed still describes it.

    Raises:
        ValueError: For an unknown mode (or an invalid mode parameter, from the
            underlying ``*_state`` function).
    """
    mode = mode.lower()
    if mode == "point":
        return lambda t: point_state(t, heading_deg, fwd_m, disp_m)
    if mode == "straight":
        return lambda t: straight_state(t, heading_deg, fwd_m, disp_m, speed_ms)
    if mode == "circle":
        return lambda t: circle_state(t, heading_deg, fwd_m, disp_m, radius_m, speed_ms)
    if mode == "rectangle":
        return lambda t: rectangle_state(
            t, heading_deg, fwd_m, disp_m, length_m, width_m, speed_ms
        )
    if mode == RAND_MODE:
        return _build_rand(seed, heading_deg, fwd_m, disp_m, radius_m, length_m,
                           width_m, speed_ms, rand_min_s, rand_max_s, rand_horizon_s)
    if mode == ELASTIC_MODE:
        # `speed_ms` is the FAST speed; the slow phase is a fraction of it, so a
        # single commanded speed still parameterises the mode (TASK-030).
        slow = (elastic_slow_ms if elastic_slow_ms is not None
                else speed_ms * elastic_slow_factor)
        return lambda t: elastic_state(
            t, elastic_base, heading_deg, fwd_m, disp_m, radius_m, length_m,
            width_m, slow, speed_ms, elastic_hold_s, elastic_ramp_s)
    raise ValueError(
        "unknown kangaroo mode %r; use one of %s" % (mode, ", ".join(ALL_MODES))
    )


# --------------------------------------------------------------------------
# Composite schedule fitted to a flight boundary (``TASK-050``)
# --------------------------------------------------------------------------
# One scripted leg list that visits every mode the harness offers, once, in a
# fixed order, and that is CHECKED to stay inside a square flight area without
# the inclusion zone ever having to turn the kangaroo back (TASK-032's
# containment turn is the safety net, not the design). Composition only: no new
# motion model, every leg is an ordinary `make_segments` leg.
#
# Layout (TASK-050 D1, D5). `h` is the contained half-width, `side/2 - R -
# margin`: the zone turns the kangaroo back `R` inside the wall, and the
# schedule keeps a further `margin` clear of that. The initial point sits
# `start_range_m` due North of the aircraft; the straight leg runs South
# through the centre; the elastic-straight leg bounces back North by the
# distance one slow hold plus one ramp covers, and is timed so it lands on the
# centre. Every closed shape therefore starts and ends AT THE CENTRE, which is
# what makes the fit independent of where the straight legs happened to end.

#: Fixed-order phase names. The rand block is one phase expanded to several
#: legs; the closing point is its own phase so a run ends as it began.
COMPOSITE_PHASES = ("point", "straight", "elastic-straight", "circle",
                    "elastic-circle", "rectangle", "elastic-rectangle",
                    "rand", "point-end")

#: Hold at the opening and closing points, seconds (D1).
COMPOSITE_POINT_S = 10.0

#: The elastic-straight leg: one slow hold plus one ramp, so it ends at the
#: fast speed the circle then continues at (D1).
COMPOSITE_ELASTIC_STRAIGHT_S = ELASTIC_HOLD_S + ELASTIC_RAMP_S

#: The seeded random block, seconds (D1, D4).
COMPOSITE_RAND_S = 30.0

#: Default seed for the rand block (D4).
COMPOSITE_RAND_SEED = 1

#: Clearance the schedule keeps inside the contained region, metres. Must
#: exceed one tick's travel (`speed * dt`, 3.75 m at 37.5 m/s and 0.1 s) or
#: `ScenarioSession._contain_target`'s one-step look-ahead could still fire.
COMPOSITE_MARGIN_M = 10.0

#: The grid's geometry, which a box larger than it needs does not enlarge:
#: start range, circle radius, rectangle length and width (metres) and the
#: `kangaroo_rand` leg bounds (seconds). A 2 km zone therefore reproduces the
#: `TASK-045` geometry exactly and only a small box shrinks it.
COMPOSITE_CAPS = {"start_range_m": 300.0, "radius_m": 150.0, "length_m": 300.0,
                  "width_m": 150.0, "rand_min_s": 5.0, "rand_max_s": 20.0}

#: A rand-block straight leg may cover at most this fraction of `h`, so one
#: leg from the centre cannot reach the wall on its own.
COMPOSITE_RAND_REACH = 0.5

#: How many consecutive seeds `fit_to_box` tries for the rand block before
#: refusing (D4: seed 1 first; the seed actually used is recorded).
COMPOSITE_RAND_SEED_TRIES = 25

#: Floating-point slack on the fit test, metres. The fitted circle's far side
#: lies exactly on the usable boundary (``r = h/2``), so an excursion of a
#: few 1e-14 m is the geometry, not a breach.
FIT_TOLERANCE_M = 1e-6

#: Phases whose failure to fit is a refusal outright; a failure in the rand
#: block, or at the closing point the block leaves the kangaroo on, is tried
#: again with the next seed.
_DETERMINISTIC_PHASES = COMPOSITE_PHASES[:7]


def elastic_time_for_distance(dist_m, slow_ms, fast_ms, hold_s=ELASTIC_HOLD_S,
                              ramp_s=ELASTIC_RAMP_S, tol_m=1e-9):
    """Seconds until :func:`elastic_distance` first reaches ``dist_m``.

    Bisection on the closed-form integral, which is monotone. Used to time an
    elastic lap so it closes exactly where it opened.

    Raises:
        ValueError: For a negative distance, or a profile that never moves.
    """
    if dist_m < 0.0:
        raise ValueError("distance must be >= 0, got %r" % dist_m)
    if dist_m == 0.0:
        return 0.0
    mean = 0.5 * (slow_ms + fast_ms)
    if mean <= 0.0:
        raise ValueError("an elastic profile with zero mean speed never covers "
                         "%.1f m" % dist_m)
    period = elastic_period_s(hold_s, ramp_s)
    hi = (dist_m / mean + period) * 2.0
    lo = 0.0
    while elastic_distance(hi, slow_ms, fast_ms, hold_s, ramp_s) < dist_m:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if elastic_distance(mid, slow_ms, fast_ms, hold_s, ramp_s) < dist_m:
            lo = mid
        else:
            hi = mid
        if hi - lo <= 1e-12:
            break
    return hi


def contained_half_m(side_m, containment_m, margin_m=COMPOSITE_MARGIN_M):
    """Half-width of the region a composite may use: ``side/2 - containment -
    margin``. ``containment_m`` is the zone's containment inset (the orbit
    radius by default, `TASK-032`)."""
    return 0.5 * float(side_m) - float(containment_m) - float(margin_m)


def _toward_centre_heading(n_m):
    """North when the kangaroo is at or South of the centre line, else South:
    the bounce (D1) written as a rule rather than a number."""
    return 0.0 if n_m <= 0.0 else 180.0


def composite_legs(speed_ms, start_range_m, radius_m, length_m, width_m,
                   rand_min_s, rand_max_s, rand_seed=COMPOSITE_RAND_SEED,
                   point_s=COMPOSITE_POINT_S,
                   elastic_straight_s=COMPOSITE_ELASTIC_STRAIGHT_S,
                   rand_s=COMPOSITE_RAND_S, end_point_s=None):
    """The composite schedule as an ordinary leg list (`TASK-050`).

    Every harness mode once, in :data:`COMPOSITE_PHASES` order, starting and
    ending on ``point``. The kangaroo's start is ``start_range_m`` North of
    the origin; the straight leg is timed so the elastic-straight leg that
    follows returns it to the origin, and every closed shape then starts and
    ends there.

    Returns ``(legs, phases)``: the legs, and one phase name per leg (the rand
    block's legs all carry ``"rand"``).

    Raises:
        ValueError: For a non-positive speed or geometry, or rand bounds that
            are not ``0 < min <= max``.
    """
    if speed_ms <= 0.0:
        raise ValueError("composite needs a positive speed, got %r" % speed_ms)
    if radius_m <= 0.0 or length_m <= 0.0 or width_m <= 0.0:
        raise ValueError("composite geometry must be positive, got r=%r l=%r w=%r"
                         % (radius_m, length_m, width_m))
    if not (0.0 < rand_min_s <= rand_max_s):
        raise ValueError("need 0 < rand_min_s <= rand_max_s, got %r, %r"
                         % (rand_min_s, rand_max_s))
    end_point_s = point_s if end_point_s is None else end_point_s
    slow = speed_ms * ELASTIC_SLOW_FACTOR

    # Straight South through the centre, then elastic-straight back North for
    # exactly what one slow hold plus one ramp covers, landing on the centre.
    d_elastic = elastic_distance(elastic_straight_s, slow, speed_ms)
    straight_s = (float(start_range_m) + d_elastic) / speed_ms
    n_after_straight = float(start_range_m) - speed_ms * straight_s
    heading_back = _toward_centre_heading(n_after_straight)
    n_after_elastic = n_after_straight + d_elastic * math.cos(
        math.radians(heading_back))

    # One lap and one perimeter, closed exactly, at constant and elastic pace.
    circle_m = 2.0 * math.pi * radius_m
    perimeter_m = 2.0 * (length_m + width_m)
    circle_s = circle_m / speed_ms
    elastic_circle_s = elastic_time_for_distance(circle_m, slow, speed_ms)
    rectangle_s = perimeter_m / speed_ms
    elastic_rectangle_s = elastic_time_for_distance(perimeter_m, slow, speed_ms)
    # The rectangle extends along its heading and to its right; run it
    # towards the centre line like the straight legs do.
    rect_heading = _toward_centre_heading(n_after_elastic)

    legs = [
        (point_s, "point", 0.0, 0.0),
        (straight_s, "straight", 180.0, speed_ms),
        (elastic_straight_s, ELASTIC_MODE, heading_back, speed_ms, "straight"),
        (circle_s, "circle", 0.0, speed_ms),
        (elastic_circle_s, ELASTIC_MODE, 0.0, speed_ms, "circle"),
        (rectangle_s, "rectangle", rect_heading, speed_ms),
        (elastic_rectangle_s, ELASTIC_MODE, rect_heading, speed_ms, "rectangle"),
    ]
    phases = list(COMPOSITE_PHASES[:7])
    rand = rand_legs(rand_seed, speed_ms, rand_min_s, rand_max_s, rand_s)
    # `rand_legs` covers AT LEAST `rand_s`; trim the last leg so the block is
    # exactly the stated length and the closing point starts on time.
    covered = sum(leg[0] for leg in rand[:-1])
    last = rand[-1]
    rand[-1] = (rand_s - covered, last[1], last[2], last[3])
    legs.extend(rand)
    phases.extend(["rand"] * len(rand))
    legs.append((end_point_s, "point", 0.0, 0.0))
    phases.append("point-end")
    return legs, phases


def schedule_fits(legs, start, side_m, containment_m, radius_m=150.0,
                  length_m=300.0, width_m=150.0, margin_m=COMPOSITE_MARGIN_M,
                  dt_s=0.1, centre=(0.0, 0.0)):
    """Sample a schedule at the tick and check it stays inside the contained
    region of a square zone, with ``margin_m`` to spare (`TASK-050`).

    ``start`` is the kangaroo's ``(n, e)`` at ``t = 0``; ``side_m`` the zone
    side; ``containment_m`` the inset at which the zone would turn the
    kangaroo back (the orbit radius by default, `TASK-032`); ``centre`` the
    zone centre. A schedule that passes here is one the zone never touches,
    because the one-step look-ahead in ``_contain_target`` cannot reach a wall
    ``margin_m`` away when ``margin_m`` exceeds one tick's travel.

    Returns a dict: ``fits``, ``half_m`` (the usable half-width),
    ``worst_excursion_m`` (how far the worst sample lies beyond the usable
    region; negative is clearance), ``leg_index``, ``leg_mode``, ``t_s``,
    ``n_m``, ``e_m`` of that sample, and ``samples``.
    """
    half = contained_half_m(side_m, containment_m, margin_m)
    segments = make_segments(legs, start[0], start[1], radius_m, length_m,
                             width_m)
    kangaroo = segments_callable(segments)
    total = segments[-1][1]
    steps = int(math.ceil(total / dt_s - 1e-9))
    worst = None
    for i in range(steps + 1):
        t = min(i * dt_s, total)
        n, e, _vn, _ve = kangaroo(t)
        excursion = max(abs(n - centre[0]), abs(e - centre[1])) - half
        if worst is None or excursion > worst[0]:
            worst = (excursion, t, n, e)
    # Attribute a sample on a leg boundary to the leg that ENDED there: it is
    # that leg's travel that put the kangaroo at the worst point.
    leg_index = len(segments) - 1
    for k, seg in enumerate(segments):
        if worst[1] <= seg[1]:
            leg_index = k
            break
    leg = legs[leg_index]
    mode = str(leg[1])
    base = leg_elastic_base(leg)
    if base is not None:
        mode = "%s/%s" % (mode, base)
    return {
        "fits": worst[0] <= FIT_TOLERANCE_M,
        "half_m": half,
        "margin_m": float(margin_m),
        "worst_excursion_m": worst[0],
        "leg_index": leg_index,
        "leg_mode": mode,
        "t_s": worst[1],
        "n_m": worst[2],
        "e_m": worst[3],
        "samples": steps + 1,
    }


def fit_to_box(side_m, orbit_radius_m, speed_ms, margin_m=COMPOSITE_MARGIN_M,
               rand_seed=COMPOSITE_RAND_SEED, dt_s=0.1,
               rand_seed_tries=COMPOSITE_RAND_SEED_TRIES, caps=None):
    """Fit the composite to a square flight area of ``side_m`` (`TASK-050`).

    Derives the start range, circle radius, rectangle length and width, the
    straight-leg durations and the rand-block leg bounds from the contained
    half-width ``h = side/2 - R - margin`` (each capped at the grid's value,
    :data:`COMPOSITE_CAPS`, so a large zone reproduces the `TASK-045`
    geometry), builds the schedule and **checks it** with
    :func:`schedule_fits`. Nothing is shortened to force a fit (D2).

    The rand block is seeded with ``rand_seed``; if that block leaves the
    region the next seeds are tried, up to ``rand_seed_tries``, and the seed
    used is returned (D4). A deterministic leg that leaves the region is a
    refusal outright.

    Returns a dict with the geometry, the legs, their phases, ``duration_s``
    and the ``check`` block, or raises.

    Raises:
        ValueError: When the region is too small for the ring, or no fit
            exists at ``speed_ms``; the message names the leg and the
            excursion.
    """
    caps = dict(COMPOSITE_CAPS, **(caps or {}))
    half = contained_half_m(side_m, orbit_radius_m, margin_m)
    if half <= 0.0:
        raise ValueError(
            "a %.0f m box leaves no room: %.1f m half-side minus the %.1f m "
            "ring and %.1f m margin is %.1f m" % (side_m, 0.5 * side_m,
                                                  orbit_radius_m, margin_m, half))
    if speed_ms <= 0.0:
        raise ValueError("fit_to_box needs a positive speed, got %r" % speed_ms)
    geometry = {
        "start_range_m": min(caps["start_range_m"], half),
        "radius_m": min(caps["radius_m"], 0.5 * half),
        "length_m": min(caps["length_m"], half),
        "width_m": min(caps["width_m"], 0.5 * half),
    }
    rand_max_s = min(caps["rand_max_s"], COMPOSITE_RAND_REACH * half / speed_ms)
    rand_min_s = min(caps["rand_min_s"], 0.5 * rand_max_s)

    last_check = None
    for attempt in range(max(1, int(rand_seed_tries))):
        seed = int(rand_seed) + attempt
        legs, phases = composite_legs(
            speed_ms, geometry["start_range_m"], geometry["radius_m"],
            geometry["length_m"], geometry["width_m"], rand_min_s, rand_max_s,
            rand_seed=seed)
        check = schedule_fits(legs, (geometry["start_range_m"], 0.0), side_m,
                              orbit_radius_m, geometry["radius_m"],
                              geometry["length_m"], geometry["width_m"],
                              margin_m, dt_s)
        last_check = (seed, check)
        if check["fits"]:
            out = dict(geometry)
            out.update({
                "side_m": float(side_m),
                "orbit_radius_m": float(orbit_radius_m),
                "margin_m": float(margin_m),
                "half_m": half,
                "speed_ms": float(speed_ms),
                "straight_s": legs[1][0],
                "elastic_straight_s": legs[2][0],
                "circle_s": legs[3][0],
                "elastic_circle_s": legs[4][0],
                "rectangle_s": legs[5][0],
                "elastic_rectangle_s": legs[6][0],
                "rand_s": COMPOSITE_RAND_S,
                "rand_min_s": rand_min_s,
                "rand_max_s": rand_max_s,
                "rand_seed": seed,
                "rand_seed_tries": attempt + 1,
                "point_s": COMPOSITE_POINT_S,
                "duration_s": sum(leg[0] for leg in legs),
                "legs": legs,
                "phases": phases,
                "check": check,
            })
            return out
        if phases[check["leg_index"]] in _DETERMINISTIC_PHASES:
            break                      # a deterministic leg: refuse outright
    seed, check = last_check
    raise ValueError(
        "the composite does not fit a %.0f m box at %.2f m/s: leg %d (%s, %s) "
        "reaches %.1f m beyond the usable %.1f m half-width at t = %.1f s "
        "(n = %.1f, e = %.1f; rand seed %d)"
        % (side_m, speed_ms, check["leg_index"], check["leg_mode"],
           phases[check["leg_index"]], check["worst_excursion_m"],
           check["half_m"], check["t_s"], check["n_m"], check["e_m"], seed))
