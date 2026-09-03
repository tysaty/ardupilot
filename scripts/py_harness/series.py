"""Error-versus-time extraction from a recorded run (``TASK-040``).

Created 2026-09-03.

Every metric in the harness before this was a **run-level aggregate** or a live
readout: an RMS, a mean radius, a spread. None of them answers *when* it went
wrong, so once the window closed that question could not be asked at all.

This module turns a recorded history into time series. It is the **single owner**
of that extraction (`TASK-040` D3): the post-run graphs and the scenario driver's
live graph both consume these functions, so a live plot and a saved plot of the
same run cannot disagree — which they would, sooner or later, if each computed
its own.

Undefined is undefined
----------------------
The hard rule here, and the reason the shape is what it is. A value that is not
defined at a tick is recorded as ``None``, never as ``0.0``:

* ``e_tan`` has no value while the aircraft is on the ring — there is no
  handover point to score (`TASK-038`);
* the selected horizon has no value for an algorithm that does not choose one;
* the held-centre ring error has no value before the estimator has produced an
  estimate.

A zero in any of those places is a lie that reads as perfect performance, and it
would draw as a line through the origin rather than as the gap it is. Consumers
must break the line at ``None``; :func:`gaps` says where.

Pure: computed from recorded history, no re-run, no ``numpy``, and no change to
any estimator, geometry module or algorithm. Plain ``math``, floats and lists,
so it transliterates (`VR-015`) — though nothing on the vehicle needs it.
"""

import math

from . import metrics
from . import tangent_error


class Series:
    """One named quantity against time, with ``None`` where it is undefined.

    Attributes:
        name: Stable machine name, used as a key and a filename stem.
        label: Human label for an axis or legend.
        unit: Unit string, or ``""`` for a dimensionless count.
        t_s: Simulated time of each sample, seconds.
        values: One value per ``t_s`` entry, ``None`` where undefined.
        excluded: ``{reason: count}`` for the ``None`` entries, so a mostly-empty
            series is distinguishable from a mostly-zero one. Empty when the
            quantity is simply not produced by this run.
        signed: True when the sign is meaningful and a zero line should be drawn.
    """

    __slots__ = ("name", "label", "unit", "t_s", "values", "excluded", "signed")

    def __init__(self, name, label, unit, t_s, values, excluded=None,
                 signed=False):
        if len(t_s) != len(values):
            raise ValueError(
                "series %r has %d times and %d values" % (name, len(t_s),
                                                          len(values)))
        self.name = name
        self.label = label
        self.unit = unit
        self.t_s = list(t_s)
        self.values = list(values)
        self.excluded = dict(excluded or {})
        self.signed = bool(signed)

    def __len__(self):
        return len(self.t_s)

    @property
    def defined(self):
        """How many samples carry a value."""
        return sum(1 for v in self.values if v is not None)

    @property
    def coverage(self):
        """Defined fraction in ``[0, 1]``, or ``None`` for an empty series.

        **Report this beside any statistic taken from the series.** An RMS over
        20% of a run is not comparable with one over all of it, and the number
        alone does not say which it is.
        """
        if not self.values:
            return None
        return self.defined / float(len(self.values))

    def stats(self):
        """``{min, max, mean, rms, last}`` over the defined samples, or ``None``.

        ``None`` — rather than zeros — when nothing is defined, which is the
        honest answer for a run that never entered the phase the quantity
        belongs to.
        """
        present = [v for v in self.values if v is not None]
        if not present:
            return None
        total = sum(present)
        return {
            "min": min(present),
            "max": max(present),
            "mean": total / len(present),
            "rms": math.sqrt(sum(v * v for v in present) / len(present)),
            "last": present[-1],
        }

    def as_dict(self):
        """Plain-JSON form, for the experiment bundle."""
        return {
            "name": self.name,
            "label": self.label,
            "unit": self.unit,
            "signed": self.signed,
            "t_s": self.t_s,
            "values": self.values,
            "defined": self.defined,
            "coverage": self.coverage,
            "excluded": self.excluded,
            "stats": self.stats(),
        }


def from_dict(data):
    """Rebuild a :class:`Series` written by :meth:`Series.as_dict`."""
    return Series(data["name"], data["label"], data["unit"], data["t_s"],
                  data["values"], data.get("excluded"), data.get("signed", False))


def gaps(series):
    """Contiguous runs of defined samples, as ``[(start, end), ...]`` index pairs.

    ``end`` is exclusive. A consumer draws one line per run and leaves the space
    between them empty — which is what makes an undefined stretch *look*
    undefined rather than looking like a value of zero.
    """
    spans = []
    start = None
    for index, value in enumerate(series.values):
        if value is None:
            if start is not None:
                spans.append((start, index))
                start = None
        elif start is None:
            start = index
    if start is not None:
        spans.append((start, len(series.values)))
    return spans


# --------------------------------------------------------------------------
# The individual series
# --------------------------------------------------------------------------

def ring_error(history, orbit_radius_m, centre="target"):
    """Signed ring error over time: ``range - R``, metres.

    **Signed**, unlike `metrics.orbit_distance`, which takes the absolute value.
    Over a whole run the sign is the informative part — an aircraft settling
    *inside* the commanded ring and one settling outside it are different
    failures with the same magnitude, and `A-VAL-005`'s chord-cutting produces
    the first specifically.

    ``centre`` selects the ring centre; see :data:`metrics.CENTRES`. About
    ``"ring"`` the value is the error against the centre the algorithm actually
    held, which for a prediction-planning algorithm is not the kangaroo.
    """
    t_s, values, excluded = [], [], {}
    for sample in history:
        t_s.append(sample["t_s"])
        try:
            centre_n, centre_e = metrics.centre_of(sample, centre)
        except (KeyError, ValueError):
            values.append(None)
            excluded["no_centre"] = excluded.get("no_centre", 0) + 1
            continue
        range_m = math.hypot(sample["plane_n_m"] - centre_n,
                             sample["plane_e_m"] - centre_e)
        values.append(range_m - orbit_radius_m)
    label = ("ring error\nvs true target" if centre == "target"
             else "ring error\nvs held centre")
    return Series("ring_error_%s_m" % centre, label, "m", t_s, values,
                  excluded, signed=True)


def prediction_lead(history):
    """Separation between the held ring centre and the true target, metres.

    A **designed** offset for a prediction-planning algorithm, not an error
    (`TASK-033` D2) — which is why it is plotted alongside the ring errors
    rather than among them. ``None`` on ticks with no estimate.
    """
    t_s, values, excluded = [], [], {}
    for sample in history:
        t_s.append(sample["t_s"])
        state = sample.get("algorithm_state") or {}
        if "prediction_lead_m" in state:
            values.append(state["prediction_lead_m"])
            continue
        if sample.get("target_est_n_m") is not None:
            values.append(math.hypot(
                sample["target_est_n_m"] - sample["target_n_m"],
                sample["target_est_e_m"] - sample["target_e_m"]))
            continue
        values.append(None)
        excluded["no_estimate"] = excluded.get("no_estimate", 0) + 1
    return Series("prediction_lead_m", "prediction\nlead", "m", t_s, values,
                  excluded)


def tangent_registration(history, config, n_a_max_steps=None):
    """The `TASK-038` signed registration error over time, metres.

    ``None`` — and counted by reason — wherever the metric is undefined: on the
    ring, without an estimator, past the end of the run, or beyond the transit
    range any candidate horizon could register against. Those are the majority
    of ticks in a settled run, and the gaps in the graph are the point: they show
    which parts of a run this quantity can speak about.
    """
    if n_a_max_steps is None:
        n_a_max_steps = config.ah_k_max_steps
    samples = tangent_error.run_errors(
        history, config.orbit_radius_m, config.turn_radius_m,
        config.airspeed_ms, config.dt_s, config.delta_psi_rad,
        config.delta_d_m, n_a_max_steps)
    t_s, values, excluded = [], [], {}
    for entry in samples:
        t_s.append(entry["t_s"])
        if entry["excluded"] is None:
            values.append(entry["e_tan_m"])
        else:
            values.append(None)
            reason = entry["excluded"]
            excluded[reason] = excluded.get(reason, 0) + 1
    return Series("e_tan_m", "tangent\nregistration", "m", t_s, values,
                  excluded, signed=True)


def selected_horizon(history, dt_s):
    """The prediction horizon in **seconds**, for an algorithm that selects one.

    Seconds rather than ticks, so the axis is comparable with the time axis it is
    plotted against. ``None`` throughout for an algorithm that does not report a
    horizon, which :func:`all_series` then drops rather than plotting an empty
    panel.
    """
    t_s, values, excluded = [], [], {}
    for sample in history:
        t_s.append(sample["t_s"])
        state = sample.get("algorithm_state") or {}
        if "k_horizon_steps" in state:
            values.append(state["k_horizon_steps"] * dt_s)
        else:
            values.append(None)
            excluded["not_reported"] = excluded.get("not_reported", 0) + 1
    return Series("selected_horizon_s", "selected\nhorizon", "s", t_s, values,
                  excluded)


def target_speed(history):
    """Kangaroo ground speed over time, m/s.

    Not an error, and included deliberately: every error in this set varies with
    it, and a step in the ring error is unreadable without knowing whether the
    target had just changed pace.
    """
    t_s, values = [], []
    for sample in history:
        t_s.append(sample["t_s"])
        vn = sample.get("target_vn_ms")
        ve = sample.get("target_ve_ms")
        values.append(None if vn is None else math.hypot(vn, ve))
    return Series("target_speed_ms", "target\nspeed", "m/s", t_s, values)


def commanded_curvature(history):
    """Curvature the algorithm commanded, 1/m.

    The `FR-005`/`SR-002` quantity. Plotted against the ``1/turn_radius_m``
    bound by the consumer, so a breach is visible rather than needing to be
    computed.
    """
    t_s, values, excluded = [], [], {}
    for sample in history:
        t_s.append(sample["t_s"])
        state = sample.get("algorithm_state") or {}
        if "curvature" in state:
            values.append(state["curvature"])
        else:
            values.append(None)
            excluded["not_reported"] = excluded.get("not_reported", 0) + 1
    return Series("commanded_curvature_1pm", "commanded\ncurvature", "1/m",
                  t_s, values, excluded)


#: The series extracted for every run, in plotting order. Ring errors first
#: because they are what the mission cares about; the target's speed last
#: because it is context for the rest rather than a result.
SERIES_ORDER = ("ring_error_target_m", "ring_error_ring_m", "e_tan_m",
                "prediction_lead_m", "selected_horizon_s",
                "commanded_curvature_1pm", "target_speed_ms")


def all_series(history, config, n_a_max_steps=None, drop_empty=True):
    """Every series for a run, as ``{name: Series}`` in :data:`SERIES_ORDER`.

    Args:
        history: Recorded run history.
        config: A :class:`~py_harness.config.HarnessConfig`.
        n_a_max_steps: Transit limit for the registration error; see
            :func:`tangent_registration`.
        drop_empty: Omit a series with no defined sample at all. A series that is
            empty because the algorithm does not produce it (a horizon for a
            fixed-horizon algorithm) is noise; a series that is empty because the
            run never left the ring is a **finding**, and the two are
            indistinguishable from the series alone — so `False` keeps both and
            the caller reads ``excluded`` to tell them apart.
    """
    produced = {
        "ring_error_target_m": ring_error(history, config.orbit_radius_m,
                                          "target"),
        "ring_error_ring_m": ring_error(history, config.orbit_radius_m, "ring"),
        "e_tan_m": tangent_registration(history, config, n_a_max_steps),
        "prediction_lead_m": prediction_lead(history),
        "selected_horizon_s": selected_horizon(history, config.dt_s),
        "commanded_curvature_1pm": commanded_curvature(history),
        "target_speed_ms": target_speed(history),
    }
    out = {}
    for name in SERIES_ORDER:
        series = produced[name]
        if drop_empty and series.defined == 0:
            continue
        out[name] = series
    return out


def to_plot_data(series_map, marker_times=None, curvature_bound_1pm=None):
    """Flatten series into the plain lists :mod:`py_harness.plotter` accepts.

    The read-only plotter imports no harness module (`DEC-2026-07-22-01`), so it
    cannot be handed a :class:`Series`. This is the boundary: plain dicts of
    plain lists cross it, and the plotter stays unable to reach back into the
    harness.
    """
    panels = []
    for name, series in series_map.items():
        panels.append({
            "name": name,
            "label": series.label,
            "unit": series.unit,
            "signed": series.signed,
            "t_s": list(series.t_s),
            "values": list(series.values),
            "spans": gaps(series),
            "coverage": series.coverage,
        })
    return {
        "panels": panels,
        "markers": list(marker_times or []),
        "curvature_bound_1pm": curvature_bound_1pm,
    }
