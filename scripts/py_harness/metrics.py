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



#: Ring-centre selectors for the metrics below (``TASK-033``).
#:
#: ``"target"``
#:     The **true** kangaroo position. The default, and what every metric meant
#:     before ``TASK-033``; existing results are unchanged.
#: ``"ring"``
#:     The centre the algorithm actually held its ring about. For
#:     ``adaptive_db_circle`` that is the **held predicted** centre from
#:     ``algorithm_state`` — which between replan instants is neither the true
#:     target nor the live estimate. Falls back to the projected estimate
#:     (``target_est_*``) and then to the true target, so it is meaningful for
#:     every algorithm.
#:
#: Both must be reported for a prediction-planning algorithm and labelled
#: distinctly: the ring is *designed* to lead the target by
#: ``k * dt_s * |v_target|`` (`TASK-033` D2), so a large error against
#: ``"target"`` may be a correctly-held ring rather than a defect, and a small
#: error against ``"ring"`` says nothing about where the kangaroo is (`A-VAL-007`).
CENTRES = ("target", "ring")


def centre_of(sample, centre="target"):
    """Ring centre ``(n_m, e_m)`` for one history sample under ``centre``.

    Raises:
        ValueError: For an unknown selector.
    """
    if centre == "target":
        return sample["target_n_m"], sample["target_e_m"]
    if centre != "ring":
        raise ValueError("centre must be one of %s, got %r"
                         % (", ".join(CENTRES), centre))
    st = sample.get("algorithm_state") or {}
    if "centre_n_m" in st and "centre_e_m" in st:
        return st["centre_n_m"], st["centre_e_m"]
    if sample.get("target_est_n_m") is not None:
        return sample["target_est_n_m"], sample["target_est_e_m"]
    return sample["target_n_m"], sample["target_e_m"]


def orbit_distance(plane_n_m, plane_e_m, target_n_m, target_e_m, orbit_radius_m):
    """Distance from one plane position to the ring: ``| range − R |`` (metres)."""
    range_m = math.hypot(plane_n_m - target_n_m, plane_e_m - target_e_m)
    return abs(range_m - orbit_radius_m)


def orbit_distances(history, orbit_radius_m, centre="target"):
    """Per-sample ring distance for the whole run (list of metres).

    ``centre`` selects which centre the ring is measured about; see
    :data:`CENTRES`. Default ``"target"`` is the pre-``TASK-033`` behaviour.
    """
    out = []
    for s in history:
        cn, ce = centre_of(s, centre)
        out.append(orbit_distance(s["plane_n_m"], s["plane_e_m"], cn, ce,
                                  orbit_radius_m))
    return out


def min_orbit_distance(history, orbit_radius_m, centre="target"):
    """Minimum on-orbit distance over the run, metres, or ``None`` if empty.

    ``0`` means the plane reached the ring at some instant (`A-VAL-007`).
    ``centre`` selects the ring centre; see :data:`CENTRES`.
    """
    distances = orbit_distances(history, orbit_radius_m, centre)
    return min(distances) if distances else None


def steady_state_stats(history, orbit_radius_m, fraction=0.25,
                       settled_tol_m=1.0, centre="target"):
    """Steady-state ring statistics over the converged tail (``TASK-027``).

    The complement to :func:`min_orbit_distance`. That metric is the *closest
    approach*, so a run that grazes the ring once and then settles well inside it
    scores zero; these statistics describe where the aircraft actually **ends up**
    (`A-VAL-005`, `A-VAL-007`).

    Args:
        history: Recorded run history.
        orbit_radius_m: Commanded ring radius, m.
        fraction: Tail fraction treated as steady state. Default 0.25, matching
            ``state.Harness.achieved_orbit_radius_m``.
        settled_tol_m: Largest drift across the tail, m, for the run to be
            called settled.
        centre: Which centre the ring is measured about; see :data:`CENTRES`.
            Default ``"target"`` is the pre-``TASK-033`` behaviour.

    Returns:
        ``None`` for an empty history, else a dict:

        ``mean_radius_m``
            Mean aircraft-to-target range over the tail — the achieved radius.
        ``rms_ring_error_m``
            RMS of ``|range - R|`` over the tail.
        ``max_ring_error_m``
            Worst ``|range - R|`` over the tail.
        ``drift_m``
            Mean range over the tail's second half minus its first half.
        ``settled``
            ``|drift_m| <= settled_tol_m``. **False means the other figures do
            not describe a steady state** and must not be read as an achieved
            radius — the common case is a run that terminated on arrival, whose
            tail is still the final approach rather than a hold.
    """
    if not history:
        return None
    n = max(1, int(len(history) * fraction))
    tail = history[-n:]
    ranges = []
    for s in tail:
        cn, ce = centre_of(s, centre)
        ranges.append(math.hypot(s["plane_n_m"] - cn, s["plane_e_m"] - ce))
    errors = [abs(r - orbit_radius_m) for r in ranges]
    half = max(1, len(ranges) // 2)
    first = sum(ranges[:half]) / half
    second = sum(ranges[-half:]) / half
    drift = second - first
    return {
        "mean_radius_m": sum(ranges) / len(ranges),
        "rms_ring_error_m": math.sqrt(sum(e * e for e in errors) / len(errors)),
        "max_ring_error_m": max(errors),
        "drift_m": drift,
        "settled": abs(drift) <= settled_tol_m,
    }


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


def dual_centre_stats(history, orbit_radius_m, fraction=0.25,
                      settled_tol_m=1.0):
    """Ring statistics about **both** centres, labelled (``TASK-033`` D2).

    Returns ``{"target": {...}, "ring": {...}}``, each the
    :func:`steady_state_stats` dict for that centre plus its
    ``min_orbit_distance_m``, and a top-level ``mean_prediction_lead_m`` — the
    mean separation between the two centres over the tail.

    Exists so the two cannot be reported singly by accident. Against a
    prediction-planning algorithm the ``"target"`` figures include the designed
    lead and the ``"ring"`` figures exclude it; quoting either alone is
    misleading in opposite directions.

    Returns ``None`` for an empty history.
    """
    if not history:
        return None
    n = max(1, int(len(history) * fraction))
    tail = history[-n:]
    leads = []
    for s in tail:
        tn, te = centre_of(s, "target")
        rn, re_ = centre_of(s, "ring")
        leads.append(math.hypot(rn - tn, re_ - te))
    out = {"mean_prediction_lead_m": sum(leads) / len(leads)}
    for centre in CENTRES:
        stats = steady_state_stats(history, orbit_radius_m, fraction,
                                   settled_tol_m, centre)
        stats["min_orbit_distance_m"] = min_orbit_distance(
            history, orbit_radius_m, centre)
        out[centre] = stats
    return out
