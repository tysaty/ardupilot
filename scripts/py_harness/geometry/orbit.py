"""Orbit-ring geometry, ported from the ``py_plots/`` orbit scripts.

Sources, all unmodified and still the reference (``DEC-2026-06-25-06``):

* ``tangent_points`` — ``py_plots/dubins_path.py:294``, the ring-entry geometry.
* ``orbit_direction`` — ``py_plots/combined.py:97``.
* ``tangential_speed`` — ``py_plots/constrained_curve.py:23``.
* the orbit parameterisation — ``py_plots/constrained_curve.py:57-82``, with
  ``z`` as time and the aircraft circling at ``omega = v_t / R``.

``cylinder_curve.py`` and ``constrained_curve.py`` could not be imported: neither
has an ``if __name__ == "__main__"`` guard, so importing them executes
module-level code and opens matplotlib windows. Their pure functions are copied
here instead. ``numpy`` is replaced with ``math`` and plain loops
(``DEC-2026-06-25-04``).

Frame: ``x = East``, ``y = North``, ``psi`` from North increasing clockwise, as
in the originals. Orbit points are ``P = T + R*(sin psi, cos psi)`` in
``(East, North)``.
"""

import math


def tangential_speed(radius_m, bank_deg, gravity_ms2):
    """Speed for a coordinated turn of ``radius_m`` flown at ``bank_deg``.

    ``R = v^2 / (g*tan(phi))  ->  v = sqrt(g*tan(phi)*R)``.

    Ported unchanged from ``constrained_curve.py:23``, with bank and gravity
    lifted from module globals into arguments so the function holds no state.

    **The harness does not use this to set speed.** It fixes airspeed and
    derives bank instead (``config.required_bank_deg``). This function is
    retained because it is the originals' relation and the equivalence tests
    compare against it. See ``A-VAL-004``.
    """
    return math.sqrt(gravity_ms2 * math.tan(math.radians(bank_deg)) * radius_m)


def tangent_points(sx, sy, tx, ty, R):
    """Ring-entry points and arrival headings from ``S`` to a ring about ``T``.

    Instead of flying to the target directly, approach tangent to a ring of
    radius ``R`` around it. From a start outside the ring there are two tangent
    lines; each touches the ring at a point ``P``, and the arrival heading is
    the direction of that line, since ``SP`` is perpendicular to ``TP``. The
    two are the CW and CCW orbit-entry options.

    ``d = |T - S|``, ``L = sqrt(d^2 - R^2)``, ``off = asin(R / d)``.

    Returns a list of ``(px, py, psi_f, sign)``; empty when the start is inside
    the ring, which is the no-solution case.

    Ported unchanged from ``py_plots/dubins_path.py:294``.
    """
    dx, dy = tx - sx, ty - sy
    d = math.hypot(dx, dy)
    if d <= R:
        return []  # start is inside the ring: no tangent
    L = math.sqrt(d * d - R * R)  # tangent length
    gamma = math.atan2(dy, dx)  # standard math angle of S -> T
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


def orbit_direction(psi0, psi_f):
    """Pick CW/CCW so the orbit's initial velocity continues the arrival heading.

    Orbit point ``P = T + R*(sin psi, cos psi)`` with ``psi = psi0 +
    dir*omega*t``. Differentiating gives a velocity direction proportional to
    ``dir*(cos psi0, -sin psi0)``; choose the ``dir`` whose start velocity best
    matches the arrival direction.

    Ported unchanged from ``py_plots/combined.py:97``.
    """
    arrival = (math.sin(psi_f), math.cos(psi_f))  # (East, North)
    best_dir, best_dot = 1, -math.inf
    for d in (1, -1):
        v = (d * math.cos(psi0), -d * math.sin(psi0))
        dot = v[0] * arrival[0] + v[1] * arrival[1]
        if dot > best_dot:
            best_dir, best_dot = d, dot
    return best_dir


def entry_angle(px, py, tx, ty):
    """Angle of the ring point ``P`` about the target ``T``, ``atan2(E, N)``.

    From ``py_plots/combined.py``'s ``psi0`` computation.
    """
    return math.atan2(px - tx, py - ty)


def orbit_point(tx, ty, R, psi):
    """One point on the ring at angle ``psi``, returned as ``(East, North)``."""
    return tx + R * math.sin(psi), ty + R * math.cos(psi)


def orbit_points(tx, ty, R, psi0, direction, speed_ms, duration_s, n=300):
    """Sample the orbit, returning ``[(x, y, psi), ...]`` and the times.

    Angular rate is ``omega = speed / R``, so the aircraft covers the ring at
    its flown speed. ``direction`` is +1 or -1 from :func:`orbit_direction`.

    Ported from ``constrained_curve.py``'s helix with ``numpy.linspace``
    replaced by an explicit loop. The originals drew this against ``z`` as time;
    here time is returned separately and the geometry stays planar.
    """
    if n < 2:
        raise ValueError("orbit_points needs at least 2 samples, got %d" % n)
    omega = speed_ms / R
    pts, times = [], []
    for i in range(n):
        t = duration_s * i / (n - 1)
        psi = psi0 + direction * omega * t
        x, y = orbit_point(tx, ty, R, psi)
        pts.append((x, y, psi))
        times.append(t)
    return pts, times


def orbit_point_at_arc_length(tx, ty, R, psi0, direction, arc_length_m):
    """The ring point ``arc_length_m`` around the ring from ``psi0``.

    This is the orbit's answer to "where is the guidance point?": advancing a
    fixed arc length along a circle is a fixed angle, ``arc/R``. Closed form,
    so no sampling resolution enters the guidance output.
    """
    psi = psi0 + direction * (arc_length_m / R)
    return orbit_point(tx, ty, R, psi) + (psi,)


def cumulative_arc_length(pts):
    """Cumulative planar arc length along ``[(x, y, ...), ...]``.

    Ported from ``py_plots/combined.py:78``'s ``cumulative_time``, with the
    division by speed removed so the result is length rather than time. The
    caller divides by speed if it wants time.
    """
    out = [0.0]
    for i in range(1, len(pts)):
        seg = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        out.append(out[-1] + seg)
    return out


def point_at_arc_length(pts, arc_length_m):
    """The point ``arc_length_m`` along a sampled path, clamped to its end.

    Linear interpolation between the bracketing samples, so the guidance point
    does not jump by the sampling interval. Returns ``(x, y)``.

    Raises:
        ValueError: If ``pts`` is empty.
    """
    if not pts:
        raise ValueError("point_at_arc_length needs a non-empty path")
    if len(pts) == 1:
        return pts[0][0], pts[0][1]
    lengths = cumulative_arc_length(pts)
    if arc_length_m <= 0.0:
        return pts[0][0], pts[0][1]
    if arc_length_m >= lengths[-1]:
        return pts[-1][0], pts[-1][1]
    for i in range(1, len(lengths)):
        if lengths[i] >= arc_length_m:
            span = lengths[i] - lengths[i - 1]
            frac = 0.0 if span <= 0.0 else (arc_length_m - lengths[i - 1]) / span
            x0, y0 = pts[i - 1][0], pts[i - 1][1]
            x1, y1 = pts[i][0], pts[i][1]
            return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)
    return pts[-1][0], pts[-1][1]
