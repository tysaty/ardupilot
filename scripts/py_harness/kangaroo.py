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
ALL_MODES = MODES + (RAND_MODE,)


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


def _sub_state_fn(mode, heading_deg, radius_m, length_m, width_m, speed_ms):
    """A local-time ``state(t) -> (n, e, vn, ve)`` for one segment at the origin.

    ``fwd``/``disp`` are 0 — the segment's absolute placement comes from the
    continuity offset in :func:`_rand_segments`, not from the sub-mode's own start.
    """
    if mode == "point":
        return lambda t: point_state(t, heading_deg, 0.0, 0.0)
    if mode == "straight":
        return lambda t: straight_state(t, heading_deg, 0.0, 0.0, speed_ms)
    if mode == "circle":
        return lambda t: circle_state(t, heading_deg, 0.0, 0.0, radius_m, speed_ms)
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
    # Same chaining engine as the scripted form (`TASK-029`); random legs in,
    # continuous segments out.
    return make_segments(legs, start_n, start_e, radius_m, length_m, width_m)


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


#: One scripted leg. ``speed_ms`` is **per segment** — the whole point of the
#: scripted form, and the thing `kangaroo_rand` cannot express (it carries one
#: speed for the entire run).
SEGMENT_FIELDS = ("duration_s", "mode", "heading_deg", "speed_ms")


def make_segments(legs, start_n, start_e, radius_m=150.0, length_m=300.0,
                  width_m=150.0, t0=0.0):
    """Chain ``legs`` into continuous segments starting at ``(start_n, start_e)``.

    ``legs`` is a sequence of ``(duration_s, mode, heading_deg, speed_ms)``. Each
    segment carries a positional offset so it **begins exactly where the previous
    ended**: position is continuous across a switch and only velocity steps, which
    is what a manoeuvre is (``TASK-029``).

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
        dur, mode, heading_deg, speed_ms = leg
        if dur <= 0.0:
            raise ValueError("leg %d: duration must be > 0, got %r" % (i, dur))
        if speed_ms < 0.0:
            raise ValueError("leg %d: speed must be >= 0, got %r" % (i, speed_ms))
        mode = str(mode).lower()
        if mode not in MODES:
            raise ValueError("leg %d: unknown mode %r; use one of %s"
                             % (i, mode, ", ".join(MODES)))
        sub = _sub_state_fn(mode, heading_deg, radius_m, length_m, width_m,
                            speed_ms)
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
          rand_min_s=5.0, rand_max_s=20.0, rand_horizon_s=3600.0):
    """Bind a mode's parameters into a ``kangaroo(t) -> (n, e, vn, ve)`` callable.

    The bound parameters are immutable constants (not module state), so the
    deterministic closures transliterate to Lua without breaking ``VR-015``.
    ``kangaroo_rand`` (`TASK-021`) switches among the deterministic modes at random
    intervals; it is **reproducible from ``seed``** and continuous in position.

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
    raise ValueError(
        "unknown kangaroo mode %r; use one of %s" % (mode, ", ".join(ALL_MODES))
    )
