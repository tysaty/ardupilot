"""Kangaroo (target) motion modes, ported from ``kangaroo_MAV.lua`` (``TASK-009``).

The shipping target simulator drives the virtual kangaroo in one of five modes;
this ports the four **deterministic** ones — point, straight, circle and
rectangle — as pure kinematic functions of time. ``FR-014``'s stationary,
straight, circular and rectangular scenarios map onto these directly.

``KANG_RANDOM`` (the hop-burst random walk) is **deliberately not ported** —
recorded as outstanding in ``docs/decisions/ADR-003-kangaroo-random-walk-deferred.md``
and ``PROJECT_UPDATES.md``.

Each mode is a stateless ``state(t, ...) -> (n, e, vn, ve)`` in North/East metres
and m/s. :func:`build` binds a mode's parameters into a single
``kangaroo(t) -> (n, e, vn, ve)`` callable that the harness steps. Plain ``math``,
no ``numpy``; the geometry mirrors the Lua so it transliterates back.

Frame and conventions match the Lua and the harness (``IR-001`` to ``IR-003``):
heading degrees clockwise from North; ``heading_frame_offset`` places a point a
forward distance along the heading and a lateral displacement to its right.
"""

import math

MODES = ("point", "straight", "circle", "rectangle")


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


def build(mode, heading_deg=0.0, fwd_m=300.0, disp_m=0.0, radius_m=150.0,
          length_m=300.0, width_m=150.0, speed_ms=5.0):
    """Bind a mode's parameters into a ``kangaroo(t) -> (n, e, vn, ve)`` callable.

    The bound parameters are immutable constants (not module state), so the
    closure transliterates to Lua without breaking ``VR-015``.

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
    raise ValueError(
        "unknown kangaroo mode %r; use one of %s" % (mode, ", ".join(MODES))
    )
