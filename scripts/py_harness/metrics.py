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


# --------------------------------------------------------------------------
# Post-contact radial error (TASK-045) — how well the ring is held once reached
# --------------------------------------------------------------------------

def signed_radial_errors(history, orbit_radius_m, centre="target"):
    """Signed radial error ``e_r(i) = ‖p − c‖ − R`` per tick, metres.

    Negative is **inside** the ring. ``None`` where the centre is undefined for
    that tick (see :func:`centre_of`). Plain loop, no array library (`VR-015`):
    the same expression will be needed against ``.bin`` logs in `TASK-046`.
    """
    out = []
    for s in history:
        try:
            cn, ce = centre_of(s, centre)
        except (KeyError, ValueError):
            out.append(None)
            continue
        out.append(math.hypot(s["plane_n_m"] - cn, s["plane_e_m"] - ce)
                   - orbit_radius_m)
    return out


def first_contact_index(radial_errors):
    """Index of the **first contact** tick, ``min { i : e_r(i) <= 0 }``, or ``None``.

    Contact is the first instant the aircraft reaches or crosses inside the
    ring. ``None`` — never ``0`` — when ``e_r > 0`` for the whole run, which is
    the expected outcome for every unreachable cell of the `TASK-045` grid.
    """
    for i, e in enumerate(radial_errors):
        if e is not None and e <= 0.0:
            return i
    return None


def post_contact_radial(history, orbit_radius_m, dt_s, centre="target"):
    """Post-contact cumulative radial error (`TASK-045`, decision `D4`).

    Let ``t_c`` be the first contact instant (:func:`first_contact_index`).
    Then::

        post_contact_cum_radial_m_s = sum_{i >= i_c} |e_r(i)| * dt_s

    Nothing before ``t_c`` contributes, so the approach transient — which
    :func:`min_orbit_distance` measures — is excluded by construction. This is
    the only scalar that isolates **how well the ring is held once it has been
    reached**: the steady-state RMS uses a fixed tail fraction, not the contact
    instant.

    Also reported, per `D4`: the **time-normalised** form
    ``post_contact_mean_radial_m = cum / window_s`` where ``window_s`` is the
    post-contact window ``n_post * dt_s`` (contact tick to the end of the run,
    inclusive). It is the mean absolute radial error over that window, in
    metres, and is what is comparable across cells with different ``t_c`` — a
    run that makes contact at 55 s of a 60 s run accumulates for 5 s and its
    cumulative is not comparable with one that makes contact at 10 s.

    Args:
        history: Recorded run history.
        orbit_radius_m: Commanded ring radius ``R``, metres.
        dt_s: Control interval, seconds — the integration step.
        centre: Ring centre selector; see :data:`CENTRES`. Both must be
            reported (`TASK-033` D2).

    Returns:
        ``None`` for an empty history, else a dict with ``contact_tick``,
        ``t_contact_s``, ``post_contact_ticks``, ``post_contact_window_s``,
        ``post_contact_cum_radial_m_s``, ``post_contact_mean_radial_m``,
        ``post_contact_max_abs_radial_m`` and ``post_contact_coverage`` (the
        post-contact fraction of the run). **Every value is ``None`` when
        contact never occurs** — never ``0.0``, which would read as a perfect
        hold.
    """
    if not history:
        return None
    errors = signed_radial_errors(history, orbit_radius_m, centre)
    i_c = first_contact_index(errors)
    empty = {
        "contact_tick": None, "t_contact_s": None, "post_contact_ticks": None,
        "post_contact_window_s": None, "post_contact_cum_radial_m_s": None,
        "post_contact_mean_radial_m": None, "post_contact_max_abs_radial_m": None,
        "post_contact_coverage": None,
    }
    if i_c is None:
        return empty
    total, worst, count = 0.0, 0.0, 0
    for e in errors[i_c:]:
        if e is None:
            continue
        a = abs(e)
        total += a * dt_s
        worst = max(worst, a)
        count += 1
    window_s = count * dt_s
    return {
        "contact_tick": i_c,
        "t_contact_s": history[i_c]["t_s"],
        "post_contact_ticks": count,
        "post_contact_window_s": window_s,
        "post_contact_cum_radial_m_s": total,
        "post_contact_mean_radial_m": (total / window_s) if window_s > 0.0
        else None,
        "post_contact_max_abs_radial_m": worst,
        "post_contact_coverage": count / float(len(history)),
    }


# --------------------------------------------------------------------------
# Velocity errors (TASK-045) — two forms, one of them zero here by construction
# --------------------------------------------------------------------------

#: Below this magnitude a plane velocity error is recorded as exactly ``0.0``,
#: m/s. In the kinematic harness the aircraft's ground velocity is its commanded
#: velocity **by construction** (`A-VAL-001`: no wind, no airframe response), and
#: the finite difference of the recorded position reproduces it only to
#: floating-point rounding (~1e-14 m/s). Snapping that residual to zero records
#: the value the harness actually models rather than its rounding noise; a SITL
#: wind error is metres per second, never this small.
VELOCITY_ZERO_TOL_MS = 1e-9


def plane_velocity_errors(history, airspeed_ms, dt_s, plane_start_ne=None):
    """``‖v_ground(plane) − v_commanded(plane)‖`` per tick, m/s (`TASK-045` (a)).

    The commanded velocity is ``airspeed_ms`` along the recorded ``plane_hdg_rad``;
    the ground velocity is the **backward finite difference** of the recorded
    position over ``dt_s``. Tick 0 needs the pre-run position, ``plane_start_ne``
    as ``(n_m, e_m)``; when it is not given tick 0 is ``None``.

    **Identically zero in the Python harness** (`A-VAL-001`), and recorded
    anyway so the column exists, with this name and unit, when the SITL half
    runs and wind makes it non-zero (`TASK-046`). A zero here means "no wind
    model", not "no velocity error". See :data:`VELOCITY_ZERO_TOL_MS`.
    """
    out = []
    prev = plane_start_ne
    for s in history:
        if prev is None:
            out.append(None)
        else:
            vn = (s["plane_n_m"] - prev[0]) / dt_s
            ve = (s["plane_e_m"] - prev[1]) / dt_s
            cn = airspeed_ms * math.cos(s["plane_hdg_rad"])
            ce = airspeed_ms * math.sin(s["plane_hdg_rad"])
            err = math.hypot(vn - cn, ve - ce)
            out.append(0.0 if err < VELOCITY_ZERO_TOL_MS else err)
        prev = (s["plane_n_m"], s["plane_e_m"])
    return out


def target_velocity_estimate_errors(history):
    """``‖v̂_K − v_K‖`` per tick, m/s (`TASK-045` (b)) — the Kalman velocity
    estimate against the true target velocity.

    The prediction input every predicting arm consumes; it spikes at every
    elastic ramp and every ``kangaroo_rand`` leg change. ``None`` on every tick
    for a run without an estimator (arm 0), and on any tick before the
    estimator produced its first output.

    Compared **on the same recorded tick**, as ``prediction_lead`` is: the
    estimate on row ``i`` is the one the guidance law consumed during the step
    that ended at ``t_i``, formed from the measurement one tick earlier, so the
    figure includes that one-tick staleness along with the filter's own lag.
    Requires the ``target_est_raw_vn_ms`` / ``_ve_ms`` history fields.
    """
    out = []
    for s in history:
        vn = s.get("target_est_raw_vn_ms")
        ve = s.get("target_est_raw_ve_ms")
        if vn is None or ve is None:
            out.append(None)
            continue
        out.append(math.hypot(vn - s["target_vn_ms"], ve - s["target_ve_ms"]))
    return out


def _rms_max(values):
    present = [v for v in values if v is not None]
    if not present:
        return {"rms_ms": None, "max_ms": None, "samples": 0}
    return {"rms_ms": math.sqrt(sum(v * v for v in present) / len(present)),
            "max_ms": max(present), "samples": len(present)}


def velocity_errors(history, config, plane_start_ne=None, contact_tick=None):
    """Both velocity-error forms summarised over the run and post-contact.

    Args:
        history: Recorded run history.
        config: A :class:`~py_harness.config.HarnessConfig` (``airspeed_ms``,
            ``dt_s``).
        plane_start_ne: The aircraft's pre-run ``(n_m, e_m)``, for tick 0 of
            the plane form.
        contact_tick: First-contact index from :func:`post_contact_radial`
            (about the true target); ``None`` when contact never occurred, in
            which case the post-contact blocks are ``None``.

    Returns:
        ``None`` for an empty history, else ``{"plane": {...},
        "target_estimate": {...} | None, "not_a_wind_measurement": str}``.
        Each block is ``{rms_ms, max_ms, samples, coverage, post_contact}``;
        ``target_estimate`` is ``None`` when no estimator ran.
    """
    if not history:
        return None
    plane = plane_velocity_errors(history, config.airspeed_ms, config.dt_s,
                                  plane_start_ne)
    est = target_velocity_estimate_errors(history)

    def block(values):
        out = _rms_max(values)
        out["coverage"] = out["samples"] / float(len(values))
        out["post_contact"] = (None if contact_tick is None
                               else _rms_max(values[contact_tick:]))
        return out

    est_block = block(est) if any(v is not None for v in est) else None
    return {
        "plane": block(plane),
        "target_estimate": est_block,
        "not_a_wind_measurement": (
            "plane_velocity_error_ms is 0.0 by construction in the kinematic "
            "harness (A-VAL-001: no wind, no airframe response). The column "
            "exists so the SITL repeat (TASK-046) has a like-named baseline; "
            "a zero here means no wind model, not no velocity error."),
    }
