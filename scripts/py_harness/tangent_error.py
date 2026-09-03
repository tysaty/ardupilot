"""Tangent-point registration error ``e_tan`` (``TASK-038``).

The signed, geometric error that says whether the **planned handover point lands
on the kangaroo's true standoff ring**. At tick ``i`` the CS path ends at a
tangent point ``T(i)`` on the ring about the *predicted* centre; the aircraft
reaches it ``n_a(i)`` ticks later; the error is

    e_tan(i) = || T(i) - p_K(i + n_a(i)) || - R          [metres, signed]

Zero when the planned tangent point lies exactly on the true standoff ring at the
moment the aircraft arrives there. **Positive** means the handover point is
outside the true ring, **negative** inside it.

Why signed, and why this rather than the Euclidean prediction error
-------------------------------------------------------------------
``TASK-035``'s primary metric, ``|| p_K(i+k) - p_hat(i+k|i) ||``, was measured on
2026-09-02 and is **degenerate as a selection basis**: monotonically increasing in
``k_horizon`` on every motion mode at measurement noise 0, 1 and 3 m, so
``argmin RMSE_k = 0`` always. Selecting on it returns "do not predict". The cause
is that a norm discards the sign, so it measures the **cost** of a horizon and
never its **benefit** — and the benefit is entirely in the timing. That metric is
retained in ``TASK-035`` as a diagnostic; this one replaces it as the selection
basis.

**Evaluate at arrival, not at the horizon.** The metric was specified as
comparing the planned tangent with the true ring "at the same time interval,
offset by ``k``". Both were prototyped. Offsetting by ``k`` reproduces the
degeneracy exactly (RMS 1.29 -> 9.11 m, monotone), and it has to: ``T(i)`` is *by
construction* at distance ``R`` from the predicted centre, so
``|| T(i) - p_K(i+k) || - R`` is just the prediction error at its own horizon
projected radially. It measures the estimator, not the rendezvous. Evaluated at
the aircraft's own arrival time it has an **interior minimum** near
``k_horizon ~ n_a``, which is the property the metric exists to have. The two
offsets coincide at the optimum, which is presumably what the specification
intended; they are not the same measurement anywhere else.

Approach-phase only — undefined, not zero, on the ring
------------------------------------------------------
There is a tangent point only while there is a CS component. Once the aircraft is
on the ring, ``shortest_path`` raises *"aircraft is inside the target ring; no
approach tangent"* and the orbit continuation (``TASK-025``) takes over with no
handover point to score. Those ticks are **excluded and counted**, never scored as
perfect: a run that spends most of its time orbiting would otherwise report a
flatteringly small RMS built from a handful of approach samples.

Decomposition
-------------
``e_tan`` is a **scalar radial** error and cannot itself be split into
components. What is decomposed is the vector it comes from: the **registration
offset**

    delta(i) = c(i) - p_K(i + n_a(i))

between the ring centre the plan was built about and the target's true position
when the aircraft arrives. ``e_tan`` is the radial part of exactly that offset,
and the split says which way the registration went wrong:

``along_track_m``
    Component of ``delta`` along the target's velocity at arrival. A **mistimed
    horizon** — the ring is registered in front of or behind where the target
    will be. Positive is ahead of it.
``cross_track_m``
    Component across it. **Constant velocity failing** (``A-TGT-002``,
    ``A-TGT-006``) — the target turned and the straight-line projection did not.
    **Zero by construction against a straight target** (measured: 0.00 m), and
    tens of metres on a tight turn, which is how the assumption's failure is
    localised to the turn rather than to the horizon.

Decomposing the tangent-point offset ``T - p_K`` instead would be a mistake and
was made once: that vector has magnitude about ``R`` wherever the plan is right,
so its cross-track component runs to the full standoff radius on a perfectly
registered straight run and reports 63.7 m where the answer is 0.

Purity and frame
----------------
Computed **from the recorded history**, with no re-run and no change to any
algorithm, estimator or geometry module: the sample carries the aircraft pose and
the held ring centre, and :mod:`py_harness.geometry.dubins_target_circle` is
deterministic, so re-solving it on those inputs returns the path the algorithm
planned. Plain ``math``, floats and dicts; no ``numpy`` (``VR-015``).

The geometry works in ``(x = East, y = North)`` and the history in
``(n_m, e_m)`` (``IR-008``). Every prototype error in this area was an axis swap,
so the conversion is written out at each boundary rather than hidden, and a test
pins it.
"""

import math

from .geometry import dubins_target_circle as dtc


#: Reason codes for a sample that could not be scored. Reported per sample and
#: counted per run, so an excluded tick is visible rather than absent (``VR-012``).
#:
#: ``"on_ring"``
#:     The aircraft was inside the ring about the held centre, so the plan had no
#:     CS component and there was no tangent point. The ordinary orbit phase.
#: ``"no_estimate"``
#:     No estimator was running, so there is no predicted centre to plan about
#:     and no target velocity to evaluate the arrival against.
#: ``"no_solution"``
#:     The aircraft was outside the ring but no tangent solved the geometry.
#: ``"past_end"``
#:     The aircraft's predicted arrival falls beyond the end of the recorded run,
#:     so the true target position at arrival is not known. Excluding these is
#:     what stops the metric quietly shortening its own horizon at the end of a
#:     run and reporting an optimistic tail.
#: ``"transit_beyond_range"``
#:     The transit ``n_a`` exceeds ``n_a_max_steps`` — the longest horizon any
#:     candidate could register against. See :func:`run_errors`.
EXCLUSION_REASONS = ("on_ring", "no_estimate", "no_solution", "past_end",
                    "transit_beyond_range")


def _centre_of(sample):
    """The ring centre the algorithm actually planned about, ``(n_m, e_m)``.

    Prefers the algorithm's own reported centre (``algorithm_state``), then the
    projected estimate, then the true target — the same precedence
    :func:`py_harness.metrics.centre_of` uses for ``centre="ring"``, repeated
    here rather than imported so this module depends on no other metric.

    Returns ``(n_m, e_m, source)``; ``source`` is one of ``"algorithm_state"``,
    ``"target_est"`` or ``"target"``.
    """
    st = sample.get("algorithm_state") or {}
    if "centre_n_m" in st and "centre_e_m" in st:
        return st["centre_n_m"], st["centre_e_m"], "algorithm_state"
    if sample.get("target_est_n_m") is not None:
        return sample["target_est_n_m"], sample["target_est_e_m"], "target_est"
    return sample["target_n_m"], sample["target_e_m"], "target"


def sample_error(history, index, orbit_radius_m, turn_radius_m, airspeed_ms,
                 dt_s, delta_psi_rad, delta_d_m, n_a_max_steps=None):
    """``e_tan`` for one recorded tick, or an exclusion.

    Args:
        history: The recorded run.
        index: Index of the tick to score.
        orbit_radius_m: Commanded standoff radius ``R``, metres.
        turn_radius_m: Minimum turn radius ``rho``, metres.
        airspeed_ms: Aircraft speed ``V``, m/s. Used only for the transit time.
        dt_s: Control interval, seconds.
        delta_psi_rad, delta_d_m: Dubins sampling, as configured for the run.
        n_a_max_steps: Longest transit that is still in scope, whole ticks, or
            ``None`` for no limit. See :func:`run_errors` for why this is not
            optional in practice.

    Returns:
        A dict. When scored:

        ``{"index", "t_s", "e_tan_m", "along_track_m", "cross_track_m",
        "n_a_steps", "arrival_index", "reach_m", "tangent_n_m", "tangent_e_m",
        "centre_source", "excluded": None}``

        When not scored, the same keys with the measurements set to ``None`` and
        ``excluded`` one of :data:`EXCLUSION_REASONS`.

    Note:
        ``n_a`` is rounded to the nearest whole tick to index the history, and the
        rounding is *not* hidden: the unrounded value is returned as
        ``n_a_steps``. At ``dt_s = 0.1 s`` and ``V = 25 m/s`` half a tick is
        1.25 m of flight, which is below the errors this metric reports but not
        negligibly so.
    """
    sample = history[index]
    out = {
        "index": index,
        "t_s": sample["t_s"],
        "e_tan_m": None,
        "along_track_m": None,
        "cross_track_m": None,
        "n_a_steps": None,
        "arrival_index": None,
        "reach_m": None,
        "tangent_n_m": None,
        "tangent_e_m": None,
        "centre_source": None,
        "excluded": None,
    }
    cn, ce, source = _centre_of(sample)
    out["centre_source"] = source
    if source == "target":
        out["excluded"] = "no_estimate"
        return out

    # History -> geometry frame: x = East, y = North (IR-008).
    px, py = sample["plane_e_m"], sample["plane_n_m"]
    psi_i = sample["plane_hdg_rad"]
    cx, cy = ce, cn

    if math.hypot(px - cx, py - cy) < orbit_radius_m:
        out["excluded"] = "on_ring"
        return out

    try:
        _pts, reach, _direction, arrival = dtc.shortest_path(
            px, py, psi_i, cx, cy, orbit_radius_m, turn_radius_m,
            delta_psi_rad, delta_d_m)
    except ValueError:
        out["excluded"] = "no_solution"
        return out

    # Geometry -> history frame.
    tangent_e, tangent_n = arrival[0], arrival[1]
    out["reach_m"] = reach
    out["tangent_n_m"] = tangent_n
    out["tangent_e_m"] = tangent_e

    n_a = reach / airspeed_ms / dt_s
    out["n_a_steps"] = n_a
    if n_a_max_steps is not None and n_a > n_a_max_steps:
        out["excluded"] = "transit_beyond_range"
        return out
    arrival_index = index + int(round(n_a))
    if arrival_index >= len(history):
        out["excluded"] = "past_end"
        return out
    out["arrival_index"] = arrival_index

    at = history[arrival_index]
    dn = tangent_n - at["target_n_m"]
    de = tangent_e - at["target_e_m"]
    out["e_tan_m"] = math.hypot(dn, de) - orbit_radius_m

    # Decomposition of the REGISTRATION OFFSET -- the held centre against the
    # target's true position at arrival -- not of the tangent-point offset. See
    # the module docstring: the tangent point sits a standoff radius off the
    # centre by construction, so splitting its offset reports the radius, not
    # the error.
    #
    # Taken against the target's direction of travel AT ARRIVAL. A stationary
    # target has no direction, so the split is withheld rather than taken against
    # an arbitrary axis; the scalar e_tan is still reported.
    vn, ve = at.get("target_vn_ms"), at.get("target_ve_ms")
    if vn is not None and ve is not None:
        speed = math.hypot(vn, ve)
        if speed > 1e-6:
            ux, uy = ve / speed, vn / speed          # unit velocity, (East, North)
            offset_e = ce - at["target_e_m"]
            offset_n = cn - at["target_n_m"]
            out["along_track_m"] = offset_e * ux + offset_n * uy
            # Right-hand normal to the velocity in the (East, North) plane.
            out["cross_track_m"] = offset_e * uy - offset_n * ux
    return out


def run_errors(history, orbit_radius_m, turn_radius_m, airspeed_ms, dt_s,
               delta_psi_rad, delta_d_m, n_a_max_steps=None):
    """:func:`sample_error` for every tick of a run, in order.

    Returns the full per-sample list, excluded ticks included, so a caller can
    plot the coverage as readily as the error.

    ``n_a_max_steps`` restricts the metric to the regime it can speak about.
    A run that starts 300 to 400 m out spends its opening in a transit of 120 to
    160 ticks — 12 to 16 s — which **no candidate horizon in scope can register
    against**, and constant velocity is not credible that far ahead either. Left
    in, those samples dominate the RMS and drag it monotonically downward with
    the horizon, reproducing the very degeneracy this metric exists to escape:
    measured on 2026-09-03 over ``adaptive_db_circle`` against a 12.5 m/s
    straight target, unrestricted RMS falls 37.1 -> 20.0 m from ``k = 0`` to
    ``k = 50`` with no interior minimum at all.

    Set it to the largest horizon under consideration (``ah_k_max_steps``, or the
    top of whatever sweep is being run). The restriction is a **scope statement,
    not a filter for a nicer answer**, and the count of samples it removes is
    reported by :func:`summarise` so the reader can see how much of the run it
    took.
    """
    return [sample_error(history, i, orbit_radius_m, turn_radius_m, airspeed_ms,
                         dt_s, delta_psi_rad, delta_d_m, n_a_max_steps)
            for i in range(len(history))]


def summarise(samples):
    """Run-level statistics over the output of :func:`run_errors`.

    Returns:
        ``None`` when ``samples`` is empty, else a dict:

        ``rms_e_tan_m``
            RMS of the signed error over the **scored** samples, metres. ``None``
            when nothing was scored — which is the honest answer for a run that
            never left the ring, and is why this is not defaulted to ``0``.
        ``mean_e_tan_m``
            Mean of the signed error. Its **sign** is the informative part: a
            systematically positive value is a plan that consistently hands over
            **outside** the true ring, and negative one that hands over inside
            it. An RMS alone cannot show that, which is the whole reason the
            metric is signed. Note that the sign is about the *handover point*,
            not directly about the direction of the lead — a mis-registration
            along the approach axis lands outside the ring whichever way it
            points, while a lateral one can land inside; use ``along_track_m``
            for the direction.
        ``max_abs_e_tan_m``, ``rms_along_track_m``, ``rms_cross_track_m``
            As named. The cross-track RMS is the direct measurement of
            ``A-TGT-002`` failing.
        ``median_n_a_steps``
            Median transit in ticks over the scored samples — the quantity a
            selected ``k_horizon`` should be compared against.
        ``scored``, ``total``, ``coverage``
            Samples scored, samples offered, and their ratio. **Coverage must be
            reported with the RMS.** A run that orbits for most of its length
            scores few ticks, and an RMS built from them is not comparable with
            one built from a full approach.
        ``excluded``
            Counts per reason; see :data:`EXCLUSION_REASONS`.
    """
    if not samples:
        return None
    excluded = dict((reason, 0) for reason in EXCLUSION_REASONS)
    errs, along, cross, n_as = [], [], [], []
    for s in samples:
        if s["excluded"] is not None:
            excluded[s["excluded"]] = excluded.get(s["excluded"], 0) + 1
            continue
        errs.append(s["e_tan_m"])
        n_as.append(s["n_a_steps"])
        if s["along_track_m"] is not None:
            along.append(s["along_track_m"])
            cross.append(s["cross_track_m"])

    def _rms(values):
        if not values:
            return None
        return math.sqrt(sum(v * v for v in values) / len(values))

    def _median(values):
        if not values:
            return None
        ordered = sorted(values)
        mid = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[mid]
        return 0.5 * (ordered[mid - 1] + ordered[mid])

    return {
        "rms_e_tan_m": _rms(errs),
        "mean_e_tan_m": (sum(errs) / len(errs)) if errs else None,
        "max_abs_e_tan_m": max((abs(e) for e in errs), default=None),
        "rms_along_track_m": _rms(along),
        "rms_cross_track_m": _rms(cross),
        "median_n_a_steps": _median(n_as),
        "scored": len(errs),
        "total": len(samples),
        "coverage": len(errs) / float(len(samples)),
        "excluded": excluded,
    }


def run_summary(history, config, n_a_max_steps=None):
    """Convenience: :func:`run_errors` then :func:`summarise`, from a config object.

    Args:
        history: Recorded run history.
        config: A :class:`~py_harness.config.HarnessConfig`; the radii, airspeed,
            step and Dubins sampling are read from it so a caller cannot pass a
            sampling resolution the run was not planned at.
        n_a_max_steps: Transit limit; see :func:`run_errors`. ``None`` defaults to
            the configuration's own largest candidate horizon
            (``ah_k_max_steps``), which is the range any selection is being made
            over. Pass an explicit value to widen or narrow it; pass
            ``float("inf")`` to lift it and reproduce the unrestricted figures.
    """
    if n_a_max_steps is None:
        n_a_max_steps = config.ah_k_max_steps
    samples = run_errors(history, config.orbit_radius_m, config.turn_radius_m,
                         config.airspeed_ms, config.dt_s, config.delta_psi_rad,
                         config.delta_d_m, n_a_max_steps)
    return summarise(samples)
