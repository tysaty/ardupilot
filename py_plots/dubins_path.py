import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons

# ---------------------------------------------------------
# Python port of the Lua Dubins path generator
# (dubins_weave_full.lua).  Maths kept identical:
#   - x = East, y = North
#   - heading psi measured from North, positive clockwise
#   - arc_point:    x = xc + rho*sin(psi),  y = yc + rho*cos(psi)
#   - straight_step:x = x  + d*sin(theta),  y = y  + d*cos(theta)
# The original .lua remains in the repo / git history.
# ---------------------------------------------------------

PI = math.pi

# ---- math_helpers (ported) ----
def dist2d(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap_2pi(a):
    return a % (2.0 * PI)

# ---- minimum turn radius:  rho = Vt^2 / (g*tan(phi_max)) ----
def min_turn_radius(Vt, phi_max, g):
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
    # Lua heading (0 North, +CW) -> paper alpha (0 East, +CCW)
    return wrap_2pi((PI / 2.0) - psi)

def rlr_segments(xi, yi, psi_i, xf, yf, psi_f, rho):
    dx, dy = xf - xi, yf - yi
    D = math.hypot(dx, dy)
    d = D / rho
    theta_goal = math.atan2(dy, dx) if D > 1e-9 else 0.0
    alpha = wrap_2pi(lua_heading_to_paper_alpha(psi_i) - theta_goal)
    beta = wrap_2pi(lua_heading_to_paper_alpha(psi_f) - theta_goal)
    value = (6.0 - d * d + 2.0 * math.cos(alpha - beta)
             + 2.0 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if value < -1.0 or value > 1.0:
        return None, None, None
    p = wrap_2pi(2.0 * PI - math.acos(clamp(value, -1.0, 1.0)))
    atan_term = math.atan2(math.cos(alpha) - math.cos(beta),
                           d - math.sin(alpha) + math.sin(beta))
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
    value = (6.0 - d * d + 2.0 * math.cos(alpha - beta)
             + 2.0 * d * (math.sin(beta) - math.sin(alpha))) / 8.0
    if value < -1.0 or value > 1.0:
        return None, None, None
    p = wrap_2pi(2.0 * PI - math.acos(clamp(value, -1.0, 1.0)))
    atan_term = math.atan2(-math.cos(alpha) + math.cos(beta),
                           d + math.sin(alpha) - math.sin(beta))
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
    generate_arc_points(pts, xLi, yLi, rho, psi_i + PI / 2, theta + PI / 2, delta_psi, False)
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(pts, xRf, yRf, rho, theta - PI / 2, psi_f - PI / 2, delta_psi, True)
    sweep1 = arc_sweep_rad(psi_i + PI / 2, theta + PI / 2, False)
    sweep2 = arc_sweep_rad(theta - PI / 2, psi_f - PI / 2, True)
    return pts, rho * (sweep1 + sweep2) + straight_len

def generate_LSL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    xLf, yLf = circle_center_left(xf, yf, psi_f, rho)
    theta, straight_len = lsl_theta_and_distance(xLi, yLi, xLf, yLf, rho)
    generate_arc_points(pts, xLi, yLi, rho, psi_i + PI / 2, theta + PI / 2, delta_psi, False)
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(pts, xLf, yLf, rho, theta + PI / 2, psi_f + PI / 2, delta_psi, False)
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
    generate_arc_points(pts, xRi, yRi, rho, psi_i - PI / 2, theta - PI / 2, delta_psi, True)
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(pts, xLf, yLf, rho, theta + PI / 2, psi_f + PI / 2, delta_psi, False)
    sweep1 = arc_sweep_rad(psi_i - PI / 2, theta - PI / 2, True)
    sweep2 = arc_sweep_rad(theta + PI / 2, psi_f + PI / 2, False)
    return pts, rho * (sweep1 + sweep2) + straight_len

def generate_RSR(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    xRi, yRi = circle_center_right(xi, yi, psi_i, rho)
    xRf, yRf = circle_center_right(xf, yf, psi_f, rho)
    theta, straight_len = rsr_theta_and_distance(xRi, yRi, xRf, yRf, rho)
    generate_arc_points(pts, xRi, yRi, rho, psi_i - PI / 2, theta - PI / 2, delta_psi, True)
    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    generate_straight_points(pts, sx, sy, theta, straight_len, delta_d)
    generate_arc_points(pts, xRf, yRf, rho, theta - PI / 2, psi_f - PI / 2, delta_psi, True)
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
    generate_arc_points(pts, xRi, yRi, rho, psi_i - PI / 2, psi_1_end - PI / 2, delta_psi, True)
    x1, y1 = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    xLm, yLm = circle_center_left(x1, y1, psi_1_end, rho)
    psi_2_end = psi_1_end - p
    generate_arc_points(pts, xLm, yLm, rho, psi_1_end + PI / 2, psi_2_end + PI / 2, delta_psi, False)
    x2, y2 = (pts[-1][0], pts[-1][1]) if pts else (x1, y1)
    xRf, yRf = circle_center_right(x2, y2, psi_2_end, rho)
    psi_3_end = psi_2_end + q
    generate_arc_points(pts, xRf, yRf, rho, psi_2_end - PI / 2, psi_3_end - PI / 2, delta_psi, True)
    return pts, rho * (t + p + q)

def generate_LRL(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    pts = []
    t, p, q = lrl_segments(xi, yi, psi_i, xf, yf, psi_f, rho)
    if t is None:
        return None, math.inf
    xLi, yLi = circle_center_left(xi, yi, psi_i, rho)
    psi_1_end = psi_i - t
    generate_arc_points(pts, xLi, yLi, rho, psi_i + PI / 2, psi_1_end + PI / 2, delta_psi, False)
    x1, y1 = (pts[-1][0], pts[-1][1]) if pts else (xi, yi)
    xRm, yRm = circle_center_right(x1, y1, psi_1_end, rho)
    psi_2_end = psi_1_end + p
    generate_arc_points(pts, xRm, yRm, rho, psi_1_end - PI / 2, psi_2_end - PI / 2, delta_psi, True)
    x2, y2 = (pts[-1][0], pts[-1][1]) if pts else (x1, y1)
    xLf, yLf = circle_center_left(x2, y2, psi_2_end, rho)
    psi_3_end = psi_2_end - q
    generate_arc_points(pts, xLf, yLf, rho, psi_2_end + PI / 2, psi_3_end + PI / 2, delta_psi, False)
    return pts, rho * (t + p + q)

# Order/colour for the six families
FAMILIES = [
    ("LSL", generate_LSL, "tab:blue"),
    ("LSR", generate_LSR, "tab:orange"),
    ("RSL", generate_RSL, "tab:green"),
    ("RSR", generate_RSR, "tab:red"),
    ("RLR", generate_RLR, "tab:purple"),
    ("LRL", generate_LRL, "tab:brown"),
]

def generate_all(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d):
    out = {}
    for name, fn, _ in FAMILIES:
        out[name] = fn(xi, yi, psi_i, xf, yf, psi_f, rho, delta_psi, delta_d)
    return out

# ---------------------------------------------------------
# Tangent-ring approach geometry
# Instead of flying to the target T directly, approach tangent to a ring of
# radius R around T.  From the start S (outside the ring) there are two tangent
# lines; each touches the ring at a point P, and the arrival heading is the
# direction of that tangent line (SP is perpendicular to TP, so travelling along
# SP is automatically tangent to the ring).  The two tangent points are the
# CW / CCW orbit-entry options.
#   d = |T - S|,  L = sqrt(d^2 - R^2),  off = asin(R / d)
#   tangent dir from S: gamma +/- off   (gamma = bearing S -> T)
#   P = S + L * (cos, sin)
# ---------------------------------------------------------
def tangent_points(sx, sy, tx, ty, R):
    dx, dy = tx - sx, ty - sy
    d = math.hypot(dx, dy)
    if d <= R:
        return []                      # start is inside the ring: no tangent
    L = math.sqrt(d * d - R * R)       # tangent length
    gamma = math.atan2(dy, dx)         # standard math angle of S -> T
    off = math.asin(R / d)
    out = []
    for sign in (+1, -1):
        ang = gamma + sign * off
        px = sx + L * math.cos(ang)
        py = sy + L * math.sin(ang)
        # Lua heading convention: psi = atan2(East, North)
        psi_f = math.atan2(px - sx, py - sy)
        out.append((px, py, psi_f, sign))
    return out

# ---------------------------------------------------------
# Interactive plot
# ---------------------------------------------------------
rho = 5.0                       # min turn radius
delta_psi = math.radians(5.0)   # arc point spacing
delta_d = 0.5                   # straight point spacing
psi_i = 0.0                     # start heading (0 = North)

ring_R = 3.0                    # ring radius around the target
dist_init = 15.0
head_init_deg = 90.0
dist_max = 30.0
reach = dist_max + 4.0 * rho + 5.0

# unit circle for drawing the ring
_circle_t = np.linspace(0, 2 * np.pi, 100)
circle_c = np.cos(_circle_t)
circle_s = np.sin(_circle_t)

def main():
    fig, ax = plt.subplots(figsize=(9, 8))
    plt.subplots_adjust(left=0.22, bottom=0.30)
    ax.set_title("Dubins approach tangent to a ring around the target")
    ax.set_xlabel("East (x)")
    ax.set_ylabel("North (y)")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)

    # start marker, ring, target, and tangent construction
    ax.plot([0.0], [0.0], "ks", markersize=8, label="start")
    ring_line = ax.plot([], [], color="grey", linestyle="--", linewidth=1, label="ring")[0]
    target_marker = ax.plot([], [], "k*", markersize=14)[0]
    tan_lines = [ax.plot([], [], color="grey", linestyle=":", linewidth=1)[0] for _ in range(2)]
    tan_marks = [ax.plot([], [], "o", color="grey", markersize=5)[0] for _ in range(2)]
    entry_marker = ax.plot([], [], "o", color="black", markersize=8)[0]

    # one Line2D per Dubins family, drawn to the chosen tangent point
    lines = {}
    for name, _, color in FAMILIES:
        lines[name] = ax.plot([], [], color=color, linewidth=2, label=name)[0]

    # ---- sliders ----
    ax_dist = plt.axes([0.25, 0.18, 0.5, 0.03])
    dist_slider = Slider(ax=ax_dist, label="Distance", valmin=1.0, valmax=dist_max, valinit=dist_init)

    ax_head = plt.axes([0.25, 0.13, 0.5, 0.03])
    head_slider = Slider(ax=ax_head, label="Heading (deg)", valmin=0.0, valmax=360.0, valinit=head_init_deg)

    ax_rho = plt.axes([0.25, 0.08, 0.5, 0.03])
    rho_slider = Slider(ax=ax_rho, label="Turn radius (rho)", valmin=0.5, valmax=15.0, valinit=rho)

    ax_ring = plt.axes([0.25, 0.03, 0.5, 0.03])
    ring_slider = Slider(ax=ax_ring, label="Orbit radius", valmin=0.5, valmax=10.0, valinit=ring_R)

    # ---- visibility toggles ----
    # each label maps to the artist(s) it shows/hides
    toggle_map = {name: [lines[name]] for name, _, _ in FAMILIES}
    toggle_map["ring"] = [ring_line]
    toggle_map["construction"] = tan_lines + tan_marks + [entry_marker]
    toggle_labels = [name for name, _, _ in FAMILIES] + ["ring", "construction"]
    ax_check = plt.axes([0.01, 0.35, 0.16, 0.45])
    ax_check.set_title("show", fontsize=9)
    check = CheckButtons(ax_check, toggle_labels, [True] * len(toggle_labels))

    def on_check(label):
        for art in toggle_map[label]:
            art.set_visible(not art.get_visible())
        fig.canvas.draw_idle()

    check.on_clicked(on_check)

    def update(val=None):
        d = dist_slider.val
        heading = math.radians(head_slider.val)
        rho_val = rho_slider.val
        R = ring_slider.val

        # target in polar (distance, bearing), and its ring
        tx = d * math.sin(heading)   # East
        ty = d * math.cos(heading)   # North
        target_marker.set_data([tx], [ty])
        ring_line.set_data(tx + R * circle_c, ty + R * circle_s)

        entries = tangent_points(0.0, 0.0, tx, ty, R)
        # draw both tangent constructions (S -> P)
        for i in range(2):
            if i < len(entries):
                px, py, _, _ = entries[i]
                tan_lines[i].set_data([0.0, px], [0.0, py])
                tan_marks[i].set_data([px], [py])
            else:
                tan_lines[i].set_data([], [])
                tan_marks[i].set_data([], [])

        # route the six Dubins families to the first tangent entry
        if entries:
            px, py, psi_f, _ = entries[0]
            entry_marker.set_data([px], [py])
            paths = generate_all(0.0, 0.0, psi_i, px, py, psi_f, rho_val, delta_psi, delta_d)
            for name, _, _ in FAMILIES:
                pts, length = paths[name]
                line = lines[name]
                if pts:
                    line.set_data([p[0] for p in pts], [p[1] for p in pts])
                    line.set_label(f"{name} ({length:.1f} m)")
                else:
                    line.set_data([], [])
                    line.set_label(f"{name} (n/a)")
        else:
            entry_marker.set_data([], [])
            for name, _, _ in FAMILIES:
                lines[name].set_data([], [])
                lines[name].set_label(f"{name} (inside ring)")

        ax.legend(loc="upper right", fontsize=8)
        fig.canvas.draw_idle()

    dist_slider.on_changed(update)
    head_slider.on_changed(update)
    rho_slider.on_changed(update)
    ring_slider.on_changed(update)

    update()          # populate the initial paths
    plt.show()


if __name__ == "__main__":
    main()
