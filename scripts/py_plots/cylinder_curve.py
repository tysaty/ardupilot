import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Geometric properties
radius = 5.0
height = 2.0
velocity_init = 2.0
heading = np.radians(0)

# Grid of angles (theta) and heights (z)
theta = np.linspace(0, 2 * np.pi, 100)
z = np.linspace(0, height, 100)
Theta, Z = np.meshgrid(theta, z)

# plot
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')

plt.subplots_adjust(bottom=0.25)  # make room for the slider

# surface defintiion
def surface(velocity):
    dx = velocity * np.cos(heading) * Z
    dy = velocity * np.sin(heading) * Z
    X = radius * np.cos(Theta) + dx
    Y = radius * np.sin(Theta) + dy
    return X, Y, Z

# create surface
surf = [ax.plot_surface(*surface(velocity_init), color='deepskyblue', alpha=0.7, rstride=4, cstride=4)]

# path helix on the (tilted) cylinder surface
# old (returned the full meshgrid -> plotted a tangle of lines, not one helix):
# def line(velocity):
#     dx = velocity * np.cos(heading) * Z
#     dy = velocity * np.sin(heading) * Z
#     X = radius * np.cos(Theta) + dx
#     Y = radius * np.sin(Theta) + dy
#     return X, Y, Z
# helix_line = [ax.plot(*line(velocity_init), color='crimson', linewidth=2, label='Helix line')]

turns = 3                                # how many times the helix wraps the cylinder
t_line = np.linspace(0, height, 200)     # z == time, parameter along the helix
def helix(velocity, direction=1):
    # direction=+1 forward, -1 reverse. cos is even / sin is odd, so both
    # directions share the same start point (radius, 0) at t=0 and wrap oppositely.
    ang = direction * turns * 2 * np.pi * (t_line / height)
    X = radius * np.cos(ang) + velocity * np.cos(heading) * t_line
    Y = radius * np.sin(ang) + velocity * np.sin(heading) * t_line
    Z = t_line
    return X, Y, Z

# ax.plot returns a list; grab the single Line3D with [0]
helix_line = ax.plot(*helix(velocity_init, 1), color='crimson', linewidth=2, label='Helix (fwd)')[0]
helix_line_rev = ax.plot(*helix(velocity_init, -1), color='darkorange', linewidth=2, label='Helix (rev)')[0]

# slider reframed
# Equal aspect so cross-sections render as round circles. Needs BOTH equal data ranges and an equal box aspect.
# Size to the slider's max velocity so the cylinder stays in frame at any tilt.
vel_max = 20.0
reach = radius + vel_max * height
ax.set_xlim(-reach, reach)
ax.set_ylim(-reach, reach)
ax.set_zlim(0, height)
ax.set_box_aspect((1, 1, 1))

# Chart metadata
ax.set_title("Cylinder")
ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")

# Slider axes: [left, bottom, width, height]
ax_vel = plt.axes([0.25, 0.12, 0.5, 0.03])
vel_slider = Slider(ax=ax_vel, label='Tilt induced by velocity', valmin=0.0, valmax=vel_max, valinit=velocity_init)

ax_head = plt.axes([0.25, 0.06, 0.5, 0.03])
head_slider = Slider(ax=ax_head, label='Heading (deg)', valmin=0.0, valmax=360.0, valinit=np.degrees(heading))

# Second figure: top-down X-Y projection of the helix only.
# Created AFTER the slider's plt.axes so the slider stays on the 3D figure.
fig2, ax2 = plt.subplots(figsize=(6, 6))
hx0, hy0, _ = helix(velocity_init, 1)
helix_xy = ax2.plot(hx0, hy0, color='crimson', linewidth=2, label='fwd')[0]
hxr, hyr, _ = helix(velocity_init, -1)
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

    #update surface
    surf[0].remove()
    surf[0] = ax.plot_surface(*surface(vel), color='deepskyblue', alpha=0.7, rstride=4, cstride=4)

    #update lines (forward + reverse)
    hx, hy, hz = helix(vel, 1)
    helix_line.set_data(hx, hy)
    helix_line.set_3d_properties(hz)

    hxr, hyr, hzr = helix(vel, -1)
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
plt.show()

