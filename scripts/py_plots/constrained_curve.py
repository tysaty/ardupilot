import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Geometric properties
radius = 5.0
height = 2.0
velocity_init = 2.0
heading = np.radians(0)


#additional constraints
#min_turn_radius = 4
bank_angle = 45                  # degrees
gravity = 9.8
# broken (kept per convention): 'velocity' undefined here, 'tan' not imported, deg not rad
# min_rel_rad = velocity * velocity / (gravity * tan(bank_angle))
bank_angle_rad = np.radians(bank_angle)
# tangential (around-circle) speed implied by flying at the bank-angle limit for this radius:
#   R = v_t^2 / (g * tan(phi))  ->  v_t = sqrt(g * tan(phi) * R)
# now a function of radius so the orbit-radius slider can drive it:
# v_t = np.sqrt(gravity * np.tan(bank_angle_rad) * radius)
def tangential_speed(radius):
    return np.sqrt(gravity * np.tan(bank_angle_rad) * radius)
# climb_rate dropped: z IS time (dz/dt = 1), so it was never a real DOF.
# old: climb_rate_init = 0.5

# angle grid for the cylinder surface; the z (time) grid is rebuilt per-call from
# the time-window slider, so only theta is fixed here.
theta = np.linspace(0, 2 * np.pi, 100)
# old (z fixed once; now a function of the time-window slider):
# z = np.linspace(0, height, 100)
# Theta, Z = np.meshgrid(theta, z)

# plot
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')

plt.subplots_adjust(bottom=0.25)  # make room for the slider

# surface defintiion
def surface(velocity, radius, height):
    z = np.linspace(0, height, 100)          # z == time; rebuilt from the window
    Theta, Z = np.meshgrid(theta, z)
    dx = velocity * np.cos(heading) * Z
    dy = velocity * np.sin(heading) * Z
    X = radius * np.cos(Theta) + dx
    Y = radius * np.sin(Theta) + dy
    return X, Y, Z

# create surface
surf = [ax.plot_surface(*surface(velocity_init, radius, height), color='deepskyblue', alpha=0.7, rstride=4, cstride=4)]


# old (t_line fixed once; now rebuilt from the time-window slider):
# t_line = np.linspace(0, height, 200)
def helix(velocity, radius, height, direction=1):
    # direction=+1 forward, -1 reverse. cos is even / sin is odd, so both
    # directions share the same start point (radius, 0) at t=0 and wrap oppositely.
    #
    # z (t_line) IS time, so dz/dt = 1 by construction. the orbit is in the X-Y
    # plane at fixed altitude; nothing climbs. the angle is just the orbit's
    # angular rate omega = v_t / R integrated over time:
    #   ang(t) = dir * (v_t / R) * t
    # turns over the window fall out as N = (v_t/R) * height / (2*pi),
    # so the time-window slider (height) sets how many turns are shown.
    t_line = np.linspace(0, height, 200)
    v_t = tangential_speed(radius)
    #
    # broken (kept per convention):
    #ang = direction * turns * 2 * np.pi * (t_line / height)
    #ang = /..
    # old (turn count fitted to the height; 'height' cancels out):
    #turns = v_t * height / (2 * np.pi * radius * climb_rate)
    #ang = direction * 2 * np.pi * turns * (t_line / height)
    # also wrong: dividing by climb_rate slowed ang against a clock that ticks at 1
    #ang = direction * (v_t / (radius * climb_rate)) * t_line
    ang = direction * (v_t / radius) * t_line
    X = radius * np.cos(ang) + velocity * np.cos(heading) * t_line
    Y = radius * np.sin(ang) + velocity * np.sin(heading) * t_line
    Z = t_line
    return X, Y, Z

# ax.plot returns a list; grab the single Line3D with [0]
helix_line = ax.plot(*helix(velocity_init, radius, height, 1), color='crimson', linewidth=2, label='Helix (fwd)')[0]
helix_line_rev = ax.plot(*helix(velocity_init, radius, height, -1), color='darkorange', linewidth=2, label='Helix (rev)')[0]


# slider reframed
# Equal aspect so cross-sections render as round circles. Needs BOTH equal data ranges and an equal box aspect.
# Size to the slider's max velocity / radius so the cylinder stays in frame at any tilt.
vel_max = 20.0
radius_max = 15.0
height_max = 10.0
reach = radius_max + vel_max * height_max
ax.set_xlim(-reach, reach)
ax.set_ylim(-reach, reach)
ax.set_zlim(0, height_max)
ax.set_box_aspect((1, 1, 1))

# Chart metadata
ax.set_title("Cylinder")
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")

# Slider axes: [left, bottom, width, height]
ax_vel = plt.axes([0.25, 0.18, 0.5, 0.03])
vel_slider = Slider(ax=ax_vel, label='Velocity', valmin=0.0, valmax=vel_max, valinit=velocity_init)

ax_head = plt.axes([0.25, 0.12, 0.5, 0.03])
head_slider = Slider(ax=ax_head, label='Heading (deg)', valmin=0.0, valmax=360.0, valinit=np.degrees(heading))

# time-window slider: z is time, so a longer window draws more of the orbit (more turns)
ax_win = plt.axes([0.25, 0.06, 0.5, 0.03])
win_slider = Slider(ax=ax_win, label='Time window (height)', valmin=0.5, valmax=height_max, valinit=height)

# orbit-radius slider: bigger radius -> faster v_t (bank limit) but fewer/looser turns
ax_rad = plt.axes([0.25, 0.00, 0.5, 0.03])
rad_slider = Slider(ax=ax_rad, label='Orbit radius', valmin=1.0, valmax=radius_max, valinit=radius)

# Second figure: top-down X-Y projection of the helix only.
# Created AFTER the slider's plt.axes so the slider stays on the 3D figure.
fig2, ax2 = plt.subplots(figsize=(6, 6))
hx0, hy0, _ = helix(velocity_init, radius, height, 1)
helix_xy = ax2.plot(hx0, hy0, color='crimson', linewidth=2, label='fwd')[0]
hxr, hyr, _ = helix(velocity_init, radius, height, -1)
helix_xy_rev = ax2.plot(hxr, hyr, color='darkorange', linewidth=2, label='rev')[0]
ax2.set_title("Helix X-Y projection")
ax2.set_xlabel("X Axis")
ax2.set_ylabel("Y Axis")
ax2.set_aspect('equal')        # keep the projection undistorted
ax2.grid(True)
# Fixed limits so the projection doesn't "breathe" as the slider moves.
ax2.set_xlim(-reach, reach)
ax2.set_ylim(-reach, reach)

# update loop
def update(val):
    global heading
    # read both sliders; surface()/helix() use velocity + the global heading
    vel = vel_slider.val
    heading = np.radians(head_slider.val)
    win = win_slider.val
    rad = rad_slider.val

    #update surface
    surf[0].remove()
    surf[0] = ax.plot_surface(*surface(vel, rad, win), color='deepskyblue', alpha=0.7, rstride=4, cstride=4)

    #update lines (forward + reverse)
    hx, hy, hz = helix(vel, rad, win, 1)
    helix_line.set_data(hx, hy)
    helix_line.set_3d_properties(hz)

    hxr, hyr, hzr = helix(vel, rad, win, -1)
    helix_line_rev.set_data(hxr, hyr)
    helix_line_rev.set_3d_properties(hzr)

    #update X-Y projection (reuse data from above)
    helix_xy.set_data(hx, hy)
    helix_xy_rev.set_data(hxr, hyr)

    #update idle
    fig.canvas.draw_idle()
    fig2.canvas.draw_idle()

vel_slider.on_changed(update)
head_slider.on_changed(update)
win_slider.on_changed(update)
rad_slider.on_changed(update)
plt.show()

