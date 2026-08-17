"""Run metrics computed from recorded history (``TASK-018``).

The **minimum on-orbit distance**: the closest the plane gets to the kangaroo's
orbit *circle* (the ring of radius ``orbit_radius_m`` centred on the moving
target) over a run. At each recorded instant the distance to the ring is
``| ‖plane − target‖ − orbit_radius_m |``, and the metric is the minimum of that
over the run.

It measures **convergence** onto the ring: **a minimum of 0 means the plane
reached the orbit** (`A-VAL-007`). It is **time-agnostic** — it is the closest
approach, not a check that the plane is on the ring *in phase / in sync*. A
phase/synchronisation measure is deliberately left as possible future work.

Pure: computed from the recorded history, no re-run, no ``numpy``, and no change
to the estimator, geometry or algorithm. The runner reports it and stores it in
the saved-run metadata so a sweep (`TASK-019`) can catalogue it against the
look-ahead horizon (`lookahead_steps`).
"""

import math


def orbit_distance(plane_n_m, plane_e_m, target_n_m, target_e_m, orbit_radius_m):
    """Distance from one plane position to the ring: ``| range − R |`` (metres)."""
    range_m = math.hypot(plane_n_m - target_n_m, plane_e_m - target_e_m)
    return abs(range_m - orbit_radius_m)


def orbit_distances(history, orbit_radius_m):
    """Per-sample ring distance for the whole run (list of metres)."""
    return [
        orbit_distance(s["plane_n_m"], s["plane_e_m"],
                       s["target_n_m"], s["target_e_m"], orbit_radius_m)
        for s in history
    ]


def min_orbit_distance(history, orbit_radius_m):
    """Minimum on-orbit distance over the run, metres, or ``None`` if empty.

    ``0`` means the plane reached the ring at some instant (`A-VAL-007`).
    """
    distances = orbit_distances(history, orbit_radius_m)
    return min(distances) if distances else None


def min_orbit_distance_at(history, orbit_radius_m):
    """The minimum and where it occurred: ``(min_m, index, t_s)`` or ``None``.

    The instant is kept so a caller can tell a converged steady state from a
    single transient dip (the time-agnostic minimum does not distinguish them).
    """
    if not history:
        return None
    best_i, best_d = 0, orbit_distance(
        history[0]["plane_n_m"], history[0]["plane_e_m"],
        history[0]["target_n_m"], history[0]["target_e_m"], orbit_radius_m)
    for i, s in enumerate(history):
        d = orbit_distance(s["plane_n_m"], s["plane_e_m"],
                           s["target_n_m"], s["target_e_m"], orbit_radius_m)
        if d < best_d:
            best_i, best_d = i, d
    return best_d, best_i, history[best_i]["t_s"]
