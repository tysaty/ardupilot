import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ---------------------------------------------------------
# Combined demo: Dubins approach  ->  cylinder/helix orbit.
#
# The plane is tracked out from the origin to a target using the shortest Dubins
# path (logic from dubins_path.py), arriving *tangent* to an orbit ring of radius
# R around the target.  At that hand-off point the cylinder/helix behaviour from
# constrained_curve.py kicks in and the plane orbits the target.
#
# Unifying idea: the z-axis IS time.  The Dubins approach ramps up in z as it is
# flown, then transitions seamlessly into the orbit helix (which keeps climbing
# in z because time keeps passing).  One continuous space-time curve.
#
# Constraints kept consistent across both phases:
#   - shared bank angle + gravity, one tangential_speed(R) formula for both
#   - Dubins turn radius `rho` is its OWN value (NOT the orbit radius)
#   - a single `orbit_R` drives the ring, the cylinder surface AND the helix
# ---------------------------------------------------------

# Reuse the Dubins machinery. dubins_path guards its GUI under __main__, so this
# import only pulls in the pure functions (no window pops up).
from dubins_path import FAMILIES, generate_all, tangent_points

# ---- shared flight constraints (consistent across approach + orbit) ----
BANK_ANGLE_DEG = 45.0
GRAVITY = 9.8
bank_angle_rad = math.radians(BANK_ANGLE_DEG)

def tangential_speed(radius):
    # speed for a coordinated turn of given radius at the bank-angle limit:
    #   R = v^2 / (g*tan(phi))  ->  v = sqrt(g*tan(phi)*R)
    # used for BOTH the Dubins arcs (radius rho) and the orbit (radius R).
    return math.sqrt(GRAVITY * math.tan(bank_angle_rad) * radius)

# ---- Dubins sampling resolution ----
delta_psi = math.radians(5.0)   # arc point spacing
delta_d = 0.5                   # straight point spacing
psi_i = 0.0                     # start heading (0 = North)

# ---- initial parameters / slider ranges ----
dist_init = 15.0
head_init_deg = 90.0
rho_init = 5.0          # Dubins min turn radius (independent of the orbit)
orbit_R_init = 3.0      # orbit radius == ring radius == cylinder radius
win_init = 6.0          # orbit time window (seconds of orbiting to draw)

dist_max = 30.0
rho_max = 15.0
orbit_R_max = 10.0
win_max = 20.0

reach = dist_max + orbit_R_max + 2.0 * rho_max + 5.0

# unit circle for drawing the ring
_ct = np.linspace(0, 2 * np.pi, 100)
_cc = np.cos(_ct)
_cs = np.sin(_ct)


# ---------------------------------------------------------
# Helpers
# ---------------------------------------------------------
def shortest_dubins(px, py, psi_f, rho):
    """Return (name, pts, length) of the shortest solvable family, or None."""
    paths = generate_all(0.0, 0.0, psi_i, px, py, psi_f, rho, delta_psi, delta_d)
    best = None
    for name, _, _ in FAMILIES:
        pts, length = paths[name]
        if pts and (best is None or length < best[2]):
            best = (name, pts, length)
    return best


def cumulative_time(pts, speed):
    """Time at each path point = cumulative arc length / speed (z == time)."""
    ts = [0.0]
    for i in range(1, len(pts)):
        seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        ts.append(ts[-1] + seg / speed)
    return ts


def cylinder(tx, ty, R, z0, z1, n_th=40, n_z=20):
    """Vertical cylinder (radius R about the target) spanning the orbit time band."""
    th = np.linspace(0, 2 * np.pi, n_th)
    zz = np.linspace(z0, z1, n_z)
    TH, ZZ = np.meshgrid(th, zz)
    X = tx + R * np.cos(TH)
    Y = ty + R * np.sin(TH)
    return X, Y, ZZ


def orbit_direction(psi0, psi_f):
    """Pick CW/CCW so the orbit's initial velocity continues the arrival heading.

    Orbit point: P = T + R*(sin psi, cos psi), psi = psi0 + dir*omega*t.
    d/dt -> velocity direction proportional to dir*(cos psi0, -sin psi0).
    Choose dir whose start velocity best matches the arrival direction.
    """
    arrival = (math.sin(psi_f), math.cos(psi_f))   # (East, North)
    best_dir, best_dot = 1, -math.inf
    for d in (1, -1):
        v = (d * math.cos(psi0), -d * math.sin(psi0))
        dot = v[0] * arrival[0] + v[1] * arrival[1]
        if dot > best_dot:
            best_dir, best_dot = d, dot
    return best_dir


# ---------------------------------------------------------
# Interactive plot
# ---------------------------------------------------------
def main():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(left=0.06, bottom=0.32)
    ax.set_xlabel("East (x)")
    ax.set_ylabel("North (y)")
    ax.set_zlabel("time (z)")
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_box_aspect((1, 1, 0.6))

    # 3D artists (z = time)
    ax.plot([0], [0], [0], "ks", markersize=6, label="start")
    approach_line = ax.plot([], [], [], color="tab:blue", lw=2, label="Dubins approach")[0]
    orbit_line = ax.plot([], [], [], color="crimson", lw=2, label="orbit (helix)")[0]
    ring0 = ax.plot([], [], [], color="grey", ls="--", lw=1, label="orbit ring (t=0)")[0]
    target_line = ax.plot([], [], [], color="k", ls=":", lw=1)[0]   # target through time
    surf = [None]

    # 2D top-down companion
    fig2, ax2 = plt.subplots(figsize=(7, 7))
    ax2.set_title("Top-down (X-Y)")
    ax2.set_aspect("equal")
    ax2.grid(True)
    ax2.set_xlim(-reach, reach)
    ax2.set_ylim(-reach, reach)
    ax2.plot([0], [0], "ks", label="start")
    approach2 = ax2.plot([], [], color="tab:blue", lw=2, label="approach")[0]
    orbit2 = ax2.plot([], [], color="crimson", lw=2, label="orbit")[0]
    ring2 = ax2.plot([], [], color="grey", ls="--", lw=1, label="ring")[0]
    target2 = ax2.plot([], [], "k*", markersize=14)[0]
    entry2 = ax2.plot([], [], "o", color="black", markersize=7)[0]
    ax2.legend(loc="upper right", fontsize=8)

    # ---- sliders (on the 3D figure) ----
    ax_dist = plt.axes([0.25, 0.22, 0.5, 0.03])
    dist_slider = Slider(ax=ax_dist, label="Distance", valmin=1.0, valmax=dist_max, valinit=dist_init)

    ax_head = plt.axes([0.25, 0.17, 0.5, 0.03])
    head_slider = Slider(ax=ax_head, label="Heading (deg)", valmin=0.0, valmax=360.0, valinit=head_init_deg)

    ax_rho = plt.axes([0.25, 0.12, 0.5, 0.03])
    rho_slider = Slider(ax=ax_rho, label="Dubins radius (rho)", valmin=0.5, valmax=rho_max, valinit=rho_init)

    ax_ring = plt.axes([0.25, 0.07, 0.5, 0.03])
    ring_slider = Slider(ax=ax_ring, label="Orbit radius (=cylinder)", valmin=1.0, valmax=orbit_R_max, valinit=orbit_R_init)

    ax_win = plt.axes([0.25, 0.02, 0.5, 0.03])
    win_slider = Slider(ax=ax_win, label="Orbit time window", valmin=0.5, valmax=win_max, valinit=win_init)

    def clear_orbit():
        orbit_line.set_data([], [])
        orbit_line.set_3d_properties([])
        orbit2.set_data([], [])
        if surf[0] is not None:
            surf[0].remove()
            surf[0] = None

    def update(val=None):
        d = dist_slider.val
        heading = math.radians(head_slider.val)
        rho = rho_slider.val
        R = ring_slider.val
        window = win_slider.val

        # target (polar -> East/North) and its ring
        tx = d * math.sin(heading)
        ty = d * math.cos(heading)
        ring_x, ring_y = tx + R * _cc, ty + R * _cs
        target2.set_data([tx], [ty])
        ring2.set_data(ring_x, ring_y)
        ring0.set_data(ring_x, ring_y)
        ring0.set_3d_properties(np.zeros_like(ring_x))

        entries = tangent_points(0.0, 0.0, tx, ty, R)
        if not entries:
            # start inside the ring: nothing to fly
            approach_line.set_data([], [])
            approach_line.set_3d_properties([])
            approach2.set_data([], [])
            entry2.set_data([], [])
            clear_orbit()
            ax.set_title("start inside orbit ring")
            fig.canvas.draw_idle(); fig2.canvas.draw_idle()
            return

        px, py, psi_f, _sign = entries[0]
        entry2.set_data([px], [py])

        best = shortest_dubins(px, py, psi_f, rho)
        if best is None:
            approach_line.set_data([], [])
            approach_line.set_3d_properties([])
            approach2.set_data([], [])
            clear_orbit()
            ax.set_title("no Dubins solution")
            fig.canvas.draw_idle(); fig2.canvas.draw_idle()
            return

        name, pts, length = best
        v_app = tangential_speed(rho)        # approach speed (radius rho, same bank limit)
        ts = cumulative_time(pts, v_app)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        approach_line.set_data(xs, ys)
        approach_line.set_3d_properties(ts)
        approach2.set_data(xs, ys)
        t0 = ts[-1]                          # time at orbit hand-off

        # ---- orbit helix (z = time), starting exactly at the entry point ----
        v_t = tangential_speed(R)            # orbit speed (radius R, same bank limit)
        psi0 = math.atan2(px - tx, py - ty)  # entry angle on the ring (atan2(E, N))
        dir_orbit = orbit_direction(psi0, psi_f)
        tt = np.linspace(0.0, window, 300)
        psi = psi0 + dir_orbit * (v_t / R) * tt
        ox = tx + R * np.sin(psi)
        oy = ty + R * np.cos(psi)
        oz = t0 + tt
        orbit_line.set_data(ox, oy)
        orbit_line.set_3d_properties(oz)
        orbit2.set_data(ox, oy)

        # ---- cylinder surface over the orbit time band ----
        if surf[0] is not None:
            surf[0].remove()
        cx, cy, cz = cylinder(tx, ty, R, t0, t0 + window)
        surf[0] = ax.plot_surface(cx, cy, cz, color="deepskyblue", alpha=0.25,
                                  rstride=2, cstride=2, linewidth=0)

        # target as a vertical line through time; size the time axis to the orbit end
        zmax = t0 + window
        ax.set_zlim(0, zmax * 1.05)
        target_line.set_data([tx, tx], [ty, ty])
        target_line.set_3d_properties([0.0, zmax])

        ax.set_title(f"approach {name} {length:.1f} m  |  v_app {v_app:.1f}, "
                     f"v_orbit {v_t:.1f}  |  hand-off t={t0:.1f}s")
        ax.legend(loc="upper left", fontsize=8)
        fig.canvas.draw_idle()
        fig2.canvas.draw_idle()

    for s in (dist_slider, head_slider, rho_slider, ring_slider, win_slider):
        s.on_changed(update)

    update()        # populate
    plt.show()


if __name__ == "__main__":
    main()
