"""Dubins path whose final turn circle is centred on the target (``TASK-024``).

Standard Dubins builds the **final** turn circle offset to the left or right of a
terminal *pose*, of radius ``rho = R_min`` (see :mod:`py_harness.geometry.dubins`).
This module replaces that final circle with a circle **centred on the target** — the
standoff ring of radius ``R = orbit_radius_m`` — as the last tangent arc. The
aircraft therefore arrives **tangent to the ring** rather than flying onto the
target point.

Construction, per tick (stateless, no ``numpy`` — ``VR-015``):

1. Choose the aircraft's initial turn circle ``C1`` (left or right of its pose,
   radius ``rho``), sense ``s1 = +1`` (right/clockwise) or ``-1`` (left).
2. Choose the target circle ``C2`` sense ``s2`` (which tangent of the ring the
   path meets → clockwise or counter-clockwise around the target).
3. Solve the common tangent between ``C1`` (radius ``rho``) and ``C2`` (radius
   ``R``). With ``O1`` the centre of ``C1``, ``D = |T - O1|`` and ``phi`` the
   heading from ``O1`` to ``T``, the straight heading ``theta`` satisfies

       sin(theta - phi) = (rho*s1 - R*s2) / D,   theta = phi + asin(k),

   which is the **external** tangent when the senses match (``s1 == s2``) and the
   **internal** tangent when they oppose. The straight length is
   ``L = D*cos(theta - phi) >= 0``. This is the unequal-radius generalisation of
   the equal-radius ``*_theta_and_distance`` helpers in :mod:`dubins`.
4. Emit the ``C1`` arc then the straight, ending **at the tangency point on the
   ring** — the aircraft **flies into the tangent**. There is **no terminal arc**
   and no ``psi_f``: it is a **CS** path (turn + straight), not CSC, and the orbit
   (``TASK-025``) takes over from the tangency point.

The four ``(s1, s2)`` combinations are all tried; the one with the least **turn-in**
cost (``rho * sweep_C1 + L``) is returned — the open-ended orbit is not scored, so
it cannot bias the choice — which fixes the clockwise / counter-clockwise sense by
geometry. The **final orbit circle radius is the kangaroo ring ``R`` (decoupled from
the turn radius ``rho``)** and must satisfy ``R >= rho`` so its curvature
``1/R <= 1/R_min`` (``FR-005``, ``SR-002``).

Reaching the ring is this task's job; **continuing** around it is ``TASK-025``.
The frame is ``x = East, y = North``, ``psi`` from North clockwise (``IR-008``).
"""

import math

from . import dubins as dub
from . import orbit as orbit_geom

PI = math.pi


def _reach_path(px, py, psi_i, tx, ty, R, rho, s1, s2, delta_psi, delta_d):
    """One ``(s1, s2)`` candidate, or ``None`` if its tangent does not exist.

    A **CS** path — initial turn (radius ``rho``) then straight — that **flies into
    the tangent**, ending exactly at the tangency point on the ring. There is **no
    terminal arc** and no ``psi_f``: the aircraft arrives tangent to the ring and
    the orbit (``TASK-025``) takes over from there.

    Returns ``(points, reach_length, direction, arrival)`` where ``points`` is the
    sampled path (``C1`` arc then straight), ``reach_length`` is ``rho*sweep_C1 + L``
    (the turn-in cost, used for ranking — the open-ended orbit is excluded),
    ``direction`` is ``"cw"``/``"ccw"`` and ``arrival`` is the tangency point
    ``(ex, en)`` on the ring.
    """
    if s1 > 0:
        o1x, o1y = dub.circle_center_right(px, py, psi_i, rho)
    else:
        o1x, o1y = dub.circle_center_left(px, py, psi_i, rho)

    dx, dy = tx - o1x, ty - o1y
    D = math.hypot(dx, dy)
    if D < 1e-9:
        return None
    phi = math.atan2(dx, dy)  # heading O1 -> T (from North, clockwise)
    k = (rho * s1 - R * s2) / D
    if abs(k) > 1.0:
        return None  # no such tangent for this sense pair
    off = math.asin(k)
    theta = phi + off
    L = D * math.cos(off)  # = D*sqrt(1 - k^2) >= 0
    if L < -1e-9:
        return None
    L = max(0.0, L)

    # Starboard(theta): the fixed perpendicular the radii are measured along.
    nx, ny = math.cos(theta), -math.sin(theta)
    arrival = (tx - R * s2 * nx, ty - R * s2 * ny)  # tangent point on the ring

    pts = []
    if s1 > 0:  # right / clockwise initial arc
        start_ph, end_ph, inc = psi_i - PI / 2, theta - PI / 2, True
    else:       # left / counter-clockwise initial arc
        start_ph, end_ph, inc = psi_i + PI / 2, theta + PI / 2, False
    dub.generate_arc_points(pts, o1x, o1y, rho, start_ph, end_ph, delta_psi, inc)
    sweep1 = dub.arc_sweep_rad(start_ph, end_ph, inc)

    sx, sy = (pts[-1][0], pts[-1][1]) if pts else (px, py)
    if L > 1e-6:
        dub.generate_straight_points(pts, sx, sy, theta, L, delta_d)
    # No terminal arc: the path flies into the tangent and ends on the ring; the
    # orbit (TASK-025) continues from here.

    reach = rho * sweep1 + L
    direction = "cw" if s2 > 0 else "ccw"
    return pts, reach, direction, arrival


def shortest_path(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                  delta_psi, delta_d, preferred_direction=None,
                  sense_margin_m=0.0):
    """The least-cost CS target-circle path that flies into the ring tangent.

    Forms the four candidates ({L, R} initial turn x {CW, CCW} orbit sense) and
    returns the one with the least **turn-in** cost (``rho*sweep + L``); the
    open-ended orbit is not scored. Returns ``(points, reach_length, direction,
    arrival)``.

    ``preferred_direction`` / ``sense_margin_m`` (`TASK-047`, `ISSUE-G11`):
    when a direction (``"cw"``/``"ccw"``) is preferred, its best candidate is
    kept unless the other sense's best candidate is cheaper by **more than**
    ``sense_margin_m`` metres of turn-in cost. With no preference — the
    default — this is the plain argmin the baseline has always used, so the
    baseline's output is unchanged. :func:`sense_costs` exposes both costs.

    Raises:
        ValueError: If ``orbit_radius_m < turn_radius_m`` (the orbit would exceed
            the curvature bound), or if no tangent solves the geometry (e.g. the
            aircraft is inside the ring).
    """
    best_by_sense = sense_costs(px, py, psi_i, tx, ty, orbit_radius_m,
                                turn_radius_m, delta_psi, delta_d)
    return choose_sense(best_by_sense, preferred_direction, sense_margin_m)


def sense_costs(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
                delta_psi, delta_d):
    """The best candidate per orbit sense: ``{"cw": cand | None, "ccw": ...}``.

    Each candidate is the ``(points, reach_length, direction, arrival)`` tuple
    of :func:`_reach_path`, the cheaper of the two initial-turn senses for that
    orbit sense. The guards of :func:`shortest_path` apply.
    """
    if orbit_radius_m < turn_radius_m - 1e-9:
        raise ValueError(
            "target circle radius %.3f m < minimum turn radius %.3f m: the orbit "
            "would exceed the curvature bound" % (orbit_radius_m, turn_radius_m))
    if math.hypot(px - tx, py - ty) < orbit_radius_m - 1e-6:
        # Inside the ring: an outward tangent may exist geometrically, but the
        # approach-from-outside construction is out of this task's envelope
        # (continuing on the ring is TASK-025). Fail deterministically.
        raise ValueError("aircraft is inside the target ring; no approach tangent")

    best_by_sense = {"cw": None, "ccw": None, "argmin": None}
    for s1 in (1, -1):
        for s2 in (1, -1):
            cand = _reach_path(px, py, psi_i, tx, ty, orbit_radius_m,
                               turn_radius_m, s1, s2, delta_psi, delta_d)
            if cand is None:
                continue
            key = cand[2]
            if best_by_sense[key] is None or cand[1] < best_by_sense[key][1]:
                best_by_sense[key] = cand
            # The baseline's own choice: first visited wins a tie (strict <).
            if (best_by_sense["argmin"] is None
                    or cand[1] < best_by_sense["argmin"][1]):
                best_by_sense["argmin"] = cand
    return best_by_sense


def choose_sense(best_by_sense, preferred_direction=None, sense_margin_m=0.0):
    """Pick the candidate from :func:`sense_costs`, with optional hysteresis.

    No preference: the baseline's argmin over all four candidates, in its
    visit order with its strict-less tie rule (``best_by_sense["argmin"]``).
    With a preference: the preferred sense's candidate unless it does not
    exist or the other is cheaper by more than ``sense_margin_m``.

    Raises:
        ValueError: If neither sense has a candidate.
    """
    cw, ccw = best_by_sense.get("cw"), best_by_sense.get("ccw")
    if cw is None and ccw is None:
        raise ValueError("no target-circle tangent solves this configuration")
    if preferred_direction in ("cw", "ccw"):
        held = best_by_sense.get(preferred_direction)
        other = ccw if preferred_direction == "cw" else cw
        if held is None:
            return other
        if other is None or other[1] >= held[1] - float(sense_margin_m):
            return held
        return other
    return best_by_sense["argmin"]


def guidance(px, py, psi_i, tx, ty, orbit_radius_m, turn_radius_m,
             look_ahead_m, delta_psi, delta_d, preferred_direction=None,
             sense_margin_m=0.0):
    """One guidance point a look-ahead along the shortest target-circle path.

    Returns a dict ``{"gx", "gy", "direction", "reach_length_m", "curvature",
    "arrival_e", "arrival_n", "cost_cw_m", "cost_ccw_m"}``. ``curvature`` is
    ``1/orbit_radius_m`` (the final arc's), which is ``<= 1/turn_radius_m`` by
    the guard in :func:`shortest_path`. ``cost_*_m`` are the two senses' best
    turn-in costs (``None`` where no tangent exists), so a caller can see how
    close the choice was. ``preferred_direction`` / ``sense_margin_m`` as in
    :func:`shortest_path`; the defaults reproduce the baseline.

    Raises:
        ValueError: Propagated from :func:`shortest_path`.
    """
    best_by_sense = sense_costs(px, py, psi_i, tx, ty, orbit_radius_m,
                                turn_radius_m, delta_psi, delta_d)
    pts, reach, direction, arrival = choose_sense(
        best_by_sense, preferred_direction, sense_margin_m)
    gx, gy = orbit_geom.point_at_arc_length(pts, look_ahead_m)
    return {
        "gx": gx,
        "gy": gy,
        "direction": direction,
        "reach_length_m": reach,
        "curvature": 1.0 / orbit_radius_m,
        "arrival_e": arrival[0],
        "arrival_n": arrival[1],
        "cost_cw_m": None if best_by_sense["cw"] is None else best_by_sense["cw"][1],
        "cost_ccw_m": None if best_by_sense["ccw"] is None else best_by_sense["ccw"][1],
    }
