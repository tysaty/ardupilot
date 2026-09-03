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


# --------------------------------------------------------------------------
# Orbit deformation (TASK-039) — the ring is not round about a moving target
# --------------------------------------------------------------------------

#: Speed at or below which a target has no usable direction of travel, m/s.
#: Below it the bearing-relative binning has no reference axis, so the sample is
#: excluded and counted rather than binned against an arbitrary direction.
STATIONARY_SPEED_MS = 0.05

#: Default speed bands for :func:`deformation_by_speed`, m/s, as
#: ``(label, lower_inclusive, upper_exclusive)``. Chosen to straddle the elastic
#: kangaroo's surge-and-ease profile, whose whole-run mean is a fiction: measured
#: 5.3 m of ring spread while slow and 41.0 m while fast, reading as 14.7 m when
#: the two are averaged together.
DEFAULT_SPEED_BANDS = (
    ("slow", 0.0, 4.0),
    ("mid", 4.0, 9.0),
    ("fast", 9.0, float("inf")),
)


def bearing_from_target_velocity(sample, centre="target"):
    """Aircraft bearing about the ring centre, from the target's heading, radians.

    Measured in ``[-pi, pi)`` from the target's direction of travel: ``0`` is
    directly **ahead** of the target, ``+/-pi`` directly **behind** it, and
    ``+pi/2`` abeam. This is the frame the deformation lives in — the ring is
    short in front and long behind — and a bearing measured from North instead
    would average the deformation away as the target turns.

    Returns:
        Radians, or ``None`` when the target is slower than
        :data:`STATIONARY_SPEED_MS` and so has no direction of travel.
    """
    vn = sample.get("target_vn_ms")
    ve = sample.get("target_ve_ms")
    if vn is None or ve is None:
        return None
    if math.hypot(vn, ve) <= STATIONARY_SPEED_MS:
        return None
    cn, ce = centre_of(sample, centre)
    a = (math.atan2(sample["plane_e_m"] - ce, sample["plane_n_m"] - cn)
         - math.atan2(ve, vn))
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def orbit_deformation(history, orbit_radius_m, n_bins=4, fraction=0.25,
                      centre="target", speed_min_ms=None, speed_max_ms=None):
    """Achieved standoff radius by bearing relative to the target's heading.

    The metric ``TASK-039`` needs and no existing one supplies. `min_orbit_distance`
    is the closest approach and `steady_state_stats` is a whole-tail mean; both are
    **direction-blind**, so a ring flown at 49 m in front of the target and 113 m
    behind it reads as a 70 m ring with a large RMS. This says *where* the error is.

    Measured on 2026-09-02 against a 12.5 m/s straight kangaroo at ``R = 70 m``:
    front 55.8 m, abeam 75.2 m, behind 112.6 m — a 63.7 m spread — and reproduced
    to within 0.5 m by a standalone kinematic model with no estimator, no
    prediction and no algorithm, which is why the cause is recorded as
    guidance-point placement against a *translating* ring rather than as a
    prediction defect (``TASK-027``, reopened; ``A-VAL-005``).

    Args:
        history: Recorded run history.
        orbit_radius_m: Commanded ring radius ``R``, metres. Reported alongside
            the bins so the deviation can be read directly.
        n_bins: Bearing bins over the full circle. 4 gives front/right/back/left
            about the target's heading; 8 resolves the asymmetry more finely at
            the cost of samples per bin.
        fraction: Tail fraction treated as steady state, matching
            :func:`steady_state_stats`. Pass ``1.0`` for the whole run.
        centre: Ring centre selector; see :data:`CENTRES`.
        speed_min_ms: Include only samples at or above this target speed, m/s.
        speed_max_ms: Include only samples **below** this target speed, m/s.

    Returns:
        ``None`` for an empty history, else a dict:

        ``bins``
            One entry per bearing bin, ``{centre_deg, lower_deg, upper_deg,
            mean_radius_m, mean_error_m, samples}``, ordered from directly ahead
            and going clockwise. ``mean_radius_m`` is ``None`` for an empty bin.
        ``spread_m``
            Largest bin mean minus smallest, over the **populated** bins. The
            headline number: ``0`` is a round ring, and it is ``0`` by
            construction for a stationary target.
        ``mean_radius_m``
            Mean achieved radius over the included samples.
        ``samples``
            Samples binned.
        ``excluded_stationary``
            Samples dropped for having no direction of travel. **Reported, not
            silently absorbed**: a run that is mostly stationary produces a small
            spread from very few samples, and the count is how a reader tells
            that apart from a genuinely round ring.
        ``excluded_speed``
            Samples dropped by the speed window.
    """
    if not history:
        return None
    n = max(1, int(len(history) * fraction))
    tail = history[-n:]
    width = 2.0 * math.pi / int(n_bins)
    sums = [0.0] * int(n_bins)
    counts = [0] * int(n_bins)
    excluded_stationary = 0
    excluded_speed = 0
    total = 0.0
    binned = 0
    for sample in tail:
        speed = math.hypot(sample.get("target_vn_ms") or 0.0,
                           sample.get("target_ve_ms") or 0.0)
        if speed_min_ms is not None and speed < speed_min_ms:
            excluded_speed += 1
            continue
        if speed_max_ms is not None and speed >= speed_max_ms:
            excluded_speed += 1
            continue
        theta = bearing_from_target_velocity(sample, centre)
        if theta is None:
            excluded_stationary += 1
            continue
        cn, ce = centre_of(sample, centre)
        radius = math.hypot(sample["plane_n_m"] - cn, sample["plane_e_m"] - ce)
        # Bin 0 is centred on "directly ahead", so the boundaries sit at
        # +/- half a bin about it and the front/back asymmetry lands in the
        # middle of a bin rather than on its edge.
        index = int(((theta + width / 2.0) % (2.0 * math.pi)) / width)
        if index >= int(n_bins):
            index = int(n_bins) - 1
        sums[index] += radius
        counts[index] += 1
        total += radius
        binned += 1

    bins = []
    means = []
    for i in range(int(n_bins)):
        centre_deg = math.degrees(i * width)
        if centre_deg > 180.0:
            centre_deg -= 360.0
        mean = sums[i] / counts[i] if counts[i] else None
        if mean is not None:
            means.append(mean)
        bins.append({
            "centre_deg": centre_deg,
            "lower_deg": centre_deg - math.degrees(width) / 2.0,
            "upper_deg": centre_deg + math.degrees(width) / 2.0,
            "mean_radius_m": mean,
            "mean_error_m": None if mean is None else mean - orbit_radius_m,
            "samples": counts[i],
        })
    return {
        "bins": bins,
        "spread_m": (max(means) - min(means)) if len(means) >= 2 else 0.0,
        "mean_radius_m": (total / binned) if binned else None,
        "orbit_radius_m": float(orbit_radius_m),
        "samples": binned,
        "excluded_stationary": excluded_stationary,
        "excluded_speed": excluded_speed,
    }


def deformation_by_speed(history, orbit_radius_m, bands=None, n_bins=4,
                         fraction=1.0, centre="target"):
    """:func:`orbit_deformation` split by **instantaneous** target speed.

    Required by ``TASK-039``'s acceptance criteria and not optional: against a
    target whose speed varies — the ``elastic`` kangaroo, or any interactively
    driven run — the ring *breathes*, and a whole-run mean averages a deforming
    ring into a fiction. Measured on the elastic mode: 5.3 m of spread while
    slow, 18.4 m mid, 41.0 m fast; aggregated over the run it reads 14.7 m, which
    describes no instant of the run.

    Args:
        bands: ``((label, lower_ms, upper_ms), ...)``; lower inclusive, upper
            exclusive. Defaults to :data:`DEFAULT_SPEED_BANDS`.
        (others): as :func:`orbit_deformation`. ``fraction`` defaults to the
            **whole run** here, because a speed band may not be present in the
            tail at all.

    Returns:
        ``None`` for an empty history, else ``{label: orbit_deformation(...)}``
        in the order the bands were given. A band with no samples is present with
        ``samples = 0`` rather than absent — an empty band is a fact about the
        run, and dropping it would make two runs look comparable when they are not.
    """
    if not history:
        return None
    out = {}
    for label, low, high in (bands or DEFAULT_SPEED_BANDS):
        out[label] = orbit_deformation(
            history, orbit_radius_m, n_bins=n_bins, fraction=fraction,
            centre=centre, speed_min_ms=low,
            speed_max_ms=None if high == float("inf") else high)
    return out
