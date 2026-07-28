"""Dubins path geometry, ported from ``py_plots/dubins_path.py``.

Faithful port of the pure-geometry half of that file (its lines 19-310). The
plotting half is deliberately left behind. The maths is unchanged, including:

* ``x = East``, ``y = North``;
* heading ``psi`` measured from North, positive clockwise;
* ``arc_point``:    ``x = xc + rho*sin(psi)``, ``y = yc + rho*cos(psi)``;
* ``straight_step``: ``x = x + d*sin(theta)``, ``y = y + d*cos(theta)``.

``py_plots/dubins_path.py`` is itself the Python port of the Lua
``modules/dubins_weave_full.lua`` (``SRC-GEOM``), and carries the re-derived
LSR/RSL geometry recorded nowhere else. Both remain unmodified; this file is a
third copy that exists so the geometry can run without opening a window.

``tests/unit/test_py_harness_geometry.py`` asserts this module reproduces the
original exactly, which is the working guard on ``A-VAL-002`` and ``A-VAL-004``.

No ``numpy``: the original geometry was already plain ``math``, so
``DEC-2026-06-25-04`` needed nothing relaxed here.
"""

import math

PI = math.pi


# ---- math_helpers (ported) ----
def dist2d(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_2pi(a):
    return a % (2.0 * PI)


def min_turn_radius(Vt, phi_max, g):
    """``rho = Vt^2 / (g*tan(phi_max))``. ``phi_max`` in radians."""
    return (Vt * Vt) / (g * math.tan(phi_max))


# ---- circle centres ----
def circle_center_right(x, y, psi, rho):
    return x + rho * math.cos(psi), y - rho * math.sin(psi)


def circle_center_left(x, y, psi, rho):
    return x - rho * math.cos(psi), y + rho * math.sin(psi)


# ---------------------------------------------------------
# Segment geometry (theta + straight length)
# ---------------------------------------------------------
def lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho):
    l = dist2d(xLi, yLi, xRf, yRf)
    if l < 2.0 * rho:
        return None, None
    straight_length = math.sqrt(max(0.0, l * l - 4.0 * rho * rho))
    phi = math.atan2(yRf - yLi, xRf - xLi)
    alpha = math.asin(clamp(2.0 * rho / l, -1.0, 1.0))
    theta = (PI / 2.0) - (phi + alpha)
    return theta, straight_length


def lsl_theta_and_distance(xLi, yLi, xLf, yLf, rho):
    dx, dy = xLf - xLi, yLf - yLi
    straight_length = math.hypot(dx, dy)
    theta = (PI / 2.0) - math.atan2(dy, dx)
    return theta, straight_length


def rsl_theta_and_distance(xRi, yRi, xLf, yLf, rho):
    l = dist2d(xRi, yRi, xLf, yLf)
    if l < 2.0 * rho:
        return None, None
    straight_length = math.sqrt(max(0.0, l * l - 4.0 * rho * rho))
    phi = math.atan2(yLf - yRi, xLf - xRi)
    alpha = math.asin(clamp(2.0 * rho / l, -1.0, 1.0))
    theta = (PI / 2.0) - (phi - alpha)
    return theta, straight_length


def rsr_theta_and_distance(xRi, yRi, xRf, yRf, rho):
    dx, dy = xRf - xRi, yRf - yRi
    straight_length = math.hypot(dx, dy)
    theta = (PI / 2.0) - math.atan2(dy, dx)
    return theta, straight_length


# ---- Shkel/Lumelsky RLR & LRL ----
def lua_heading_to_paper_alpha(psi):
    """Lua heading (0 North, +CW) -> paper alpha (0 East, +CCW)."""
    return wrap_2pi((PI / 2.0) - psi)


def rlr_segments(xi, yi, psi_i, xf, yf, psi_f, rho):
    dx, dy = xf - xi, yf - yi
    D = math.hypot(dx, dy)
    d = D / rho
    theta_goal = math.atan2(dy, dx) if D > 1e-9 else 0.0
    alpha = wrap_2pi(lua_heading_to_paper_alpha(psi_i) - theta_goal)
    beta = wrap_2pi(lua_heading_to_paper_alpha(psi_f) - theta_goal)
    value = (
        6.0
        - d * d
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * d * (math.sin(alpha) - math.sin(beta))
    ) / 8.0
    if value < -1.0 or value > 1.0:
        return None, None, None
    p = wrap_2pi(2.0 * PI - math.acos(clamp(value, -1.0, 1.0)))
    atan_term = math.atan2(
        math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta)
    )
    t = wrap_2pi(alpha - atan_term + p / 2.0)
    q = wrap_2pi(alpha - beta - t + p)
    return t, p, q


def lrl_segments(xi, yi, psi_i, xf, yf, psi_f, rho):
    dx, dy = xf - xi, yf - yi
    D = math.hypot(dx, dy)
    d = D / rho
    theta_goal = math.atan2(dy, dx) if D > 1e-9 else 0.0
    alpha = wrap_2pi(lua_heading_to_paper_alpha(psi_i) - theta_goal)
    beta = wrap_2pi(lua_heading_to_paper_alpha(psi_f) - theta_goal)
    # sin(beta) - sin(alpha), opposite sign to RLR
    value = (
        6.0
        - d * d
        + 2.0 * math.cos(alpha - beta)
        + 2.0 * d * (math.sin(beta) - math.sin(alpha))
    ) / 8.0
    if value < -1.0 or value > 1.0:
        return None, None, None
    p = wrap_2pi(2.0 * PI - math.acos(clamp(value, -1.0, 1.0)))
    atan_term = math.atan2(
        -math.cos(alpha) + math.cos(beta), d + math.sin(alpha) - math.sin(beta)
    )
    t = wrap_2pi(-alpha + atan_term + p / 2.0)
    q = wrap_2pi(beta - alpha - t + p)
    return t, p, q


# ---------------------------------------------------------
# Point generation
# ---------------------------------------------------------
def arc_sweep_rad(psi_start, psi_end, increasing):
    if increasing:
        if psi_end < psi_start:
            psi_end += 2 * PI
        return psi_end - psi_start
    else:
        if psi_end > psi_start:
            psi_end -= 2 * PI
        return psi_start - psi_end


def arc_point(xc, yc, rho, psi_n):
    return xc + rho * math.sin(psi_n), yc + rho * math.cos(psi_n)


def straight_step(x_prev, y_prev, theta, delta_d):
    return x_prev + delta_d * math.sin(theta), y_prev + delta_d * math.cos(theta)


def generate_arc_points(points, xc, yc, rho, psi_start, psi_end, delta_psi, increasing):
    if abs(psi_end - psi_start) < 1e-9:
        return
    if increasing:
        if psi_end < psi_start:
            psi_end += 2 * PI
        sweep = psi_end - psi_start
        sign = 1
    else:
        if psi_end > psi_start:
            psi_end -= 2 * PI
        sweep = psi_start - psi_end
        sign = -1
    if sweep > 2 * PI:
        sweep = 2 * PI
    n_steps = int(sweep // delta_psi)
    for i in range(1, n_steps + 1):
        psi = psi_start + sign * i * delta_psi
        x, y = arc_point(xc, yc, rho, psi)
        points.append((x, y, psi))
    # snap to exact endpoint if the last full step didn't reach it
    if sweep - n_steps * delta_psi > 1e-6:
        psi = psi_start + sign * sweep
        x, y = arc_point(xc, yc, rho, psi)
        points.append((x, y, psi))


def generate_straight_points(points, x_start, y_start, theta, total_d, delta_d):
    x, y = x_start, y_start
    dsum = 0.0
    while dsum <= total_d:
        x, y = straight_step(x, y, theta, delta_d)
        points.append((x, y, theta))
        dsum += delta_d
    return x, y


# ---------------------------------------------------------
# Six Dubins families
# Each returns (points, total_length); (None, inf) if no solution.
# ---------------------------------------------------------
def generate_LSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    theta, straight_len = lsr_theta_and_distance(xLi, yLi, xRf, yRf, rho)
    if theta is None:
        return None, math.inf
    generate_arc_points(
        pts, xLi, yLi, rho, psi_i + PI / 2, theta + PI / 2, delta_psi, False
    )
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(
        pts, xRf, yRf, rho, theta - PI / 2, psi_f - PI / 2, delta_psi, True
    )
    sweep1 = arc_sweep_rad(psi_i + PI / 2, theta + PI / 2, False)
    sweep2 = arc_sweep_rad(theta - PI / 2, psi_f - PI / 2, True)
    return pts, rho * (sweep1 + sweep2) + straight_len


def generate_LSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    xLf, yLf = circle_center_left(xf, yf, psi_f, rho)
    theta, straight_len = lsl_theta_and_distance(xLi, yLi, xLf, yLf, rho)
    generate_arc_points(
        pts, xLi, yLi, rho, psi_i + PI / 2, theta + PI / 2, delta_psi, False
    )
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(
        pts, xLf, yLf, rho, theta + PI / 2, psi_f + PI / 2, delta_psi, False
    )
    sweep1 = arc_sweep_rad(psi_i + PI / 2, theta + PI / 2, False)
    sweep2 = arc_sweep_rad(theta + PI / 2, psi_f + PI / 2, False)
    return pts, rho * (sweep1 + sweep2) + straight_len


def generate_RSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    xLf, yLf = circle_center_left(xf, yf, psi_f, rho)
    theta, straight_len = rsl_theta_and_distance(xRi, yRi, xLf, yLf, rho)
    if theta is None:
        return None, math.inf
    generate_arc_points(
        pts, xRi, yRi, rho, psi_i - PI / 2, theta - PI / 2, delta_psi, True
    )
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(
        pts, xLf, yLf, rho, theta + PI / 2, psi_f + PI / 2, delta_psi, False
    )
    sweep1 = arc_sweep_rad(psi_i - PI / 2, theta - PI / 2, True)
    sweep2 = arc_sweep_rad(theta + PI / 2, psi_f + PI / 2, False)
    return pts, rho * (sweep1 + sweep2) + straight_len


def generate_RSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    theta, straight_len = rsr_theta_and_distance(xRi, yRi, xRf, yRf, rho)
    generate_arc_points(
        pts, xRi, yRi, rho, psi_i - PI / 2, theta - PI / 2, delta_psi, True
    )
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(
        pts, xRf, yRf, rho, theta - PI / 2, psi_f - PI / 2, delta_psi, True
    )
    sweep1 = arc_sweep_rad(psi_i - PI / 2, theta - PI / 2, True)
    sweep2 = arc_sweep_rad(theta - PI / 2, psi_f - PI / 2, True)
    return pts, rho * (sweep1 + sweep2) + straight_len


def generate_RLR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    t, p, q = rlr_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    if t is None:
        return None, math.inf
    xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    psi_1_end = psi_i + t
    generate_arc_points(
        pts, xRi, yRi, rho, psi_i - PI / 2, psi_1_end - PI / 2, delta_psi, True
    )
    x1, y1 = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    xLm, yLm = circle_center_left(x1, y1, psi_1_end, rho)
    psi_2_end = psi_1_end - p
    generate_arc_points(
        pts, xLm, yLm, rho, psi_1_end + PI / 2, psi_2_end + PI / 2, delta_psi, False
    )
    x2, y2 = (pts[-1][0], pts[-1][1]) if pts else (x1, y1)
    xRf, yRf = circle_center_right(x2, y2, psi_2_end, rho)
    psi_3_end = psi_2_end + q
    generate_arc_points(
        pts, xRf, yRf, rho, psi_2_end - PI / 2, psi_3_end - PI / 2, delta_psi, True
    )
    return pts, rho * (t + p + q)


def generate_LRL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    t, p, q = lrl_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    if t is None:
        return None, math.inf
    xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    psi_1_end = psi_i - t
    generate_arc_points(
        pts, xLi, yLi, rho, psi_i + PI / 2, psi_1_end + PI / 2, delta_psi, False
    )
    x1, y1 = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    xRm, yRm = circle_center_right(x1, y1, psi_1_end, rho)
    psi_2_end = psi_1_end + p
    generate_arc_points(
        pts, xRm, yRm, rho, psi_1_end - PI / 2, psi_2_end - PI / 2, delta_psi, True
    )
    x2, y2 = (pts[-1][0], pts[-1][1]) if pts else (x1, y1)
    xLf, yLf = circle_center_left(x2, y2, psi_2_end, rho)
    psi_3_end = psi_2_end - q
    generate_arc_points(
        pts, xLf, yLf, rho, psi_2_end + PI / 2, psi_3_end + PI / 2, delta_psi, False
    )
    return pts, rho * (t + p + q)


#: Family order, matching ``py_plots/dubins_path.py``. The colour column of the
#: original is dropped; it was a plotting concern.
FAMILIES = [
    ("LSL", generate_LSL),
    ("LSR", generate_LSR),
    ("RSL", generate_RSL),
    ("RSR", generate_RSR),
    ("RLR", generate_RLR),
    ("LRL", generate_LRL),
]


def generate_all(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    """Return ``{family_name: (points, length)}`` for all six families."""
    out = {}
    for name, fn in FAMILIES:
        out[name] = fn(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    return out


def shortest(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    """Return ``(name, points, length)`` of the shortest solvable family.

    Returns ``None`` when no family solves. Ported from
    ``py_plots/combined.py:67``, generalised to take the start pose rather than
    assuming the origin.
    """
    paths = generate_all(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    best = None
    for name, _ in FAMILIES:
        pts, length = paths[name]
        if pts and (best is None or length < best[2]):
            best = (name, pts, length)
    return best
