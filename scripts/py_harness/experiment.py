"""Run a standoff experiment and archive it as one auditable bundle (``TASK-040``).

Created 2026-09-03.

Two modes, one code path, one output format:

* **spec mode** — a JSON specification describes the kangaroo's schedule, the
  algorithm and the run; the experiment executes it, headlessly or watched;
* **interactive mode** — the scenario driver opens, the kangaroo is flown by
  hand, and the specification is *derived* from what was actually done.

Both write the **same bundle**, distinguished only by a ``source`` field
(`TASK-040` D1). That is not tidiness: `TASK-029` established that an
interactive session's export, replayed, reproduces its history, *"which is what
makes an interactive result evidence rather than a demonstration"*. A hand-driven
run that wrote less would be second-class, and every interesting exploration
would have to be re-run through a file before it could be cited.

What a bundle contains
----------------------
``experiments/YYYY-MM-DD/<experiment-id>/`` (`TASK-040` D2)::

    spec.json             the specification that reproduces the run
    record.json           metrics, provenance and the replay verification
    history.json          the recorded run, in the plotter's run format
    series.json           the error-versus-time series behind the graphs
    error-timeline.png    those series, stacked, with manoeuvre markers
    view.png              the final scene: track, target, ring, zone

Why the replay verification is on by default
--------------------------------------------
Writing a specification is a claim that it reproduces the run. `TASK-040` D4
makes the runner test its own claim: it re-runs the written spec headlessly,
compares the history and the metrics, and records the answer **inside the
bundle**. A failure is recorded rather than raised, because a specification that
does not reproduce its run is the most valuable thing this module can find — it
means the format is missing a field, and every earlier bundle is suspect.

Not a configuration of record
-----------------------------
A spec file is full of radii, speeds and headings and will read like an approved
configuration. It is not one (`SR-004`). Every bundle carries that statement in
its own fields so it travels with the file rather than living in prose the reader
never sees.
"""

import argparse
import datetime
import json
import math
import os
import platform
import subprocess
import sys

from . import algorithms
from . import export as export_mod
from . import metrics
from . import plotter
from . import scenario
from . import series as series_mod
from . import tangent_error
from .config import HarnessConfig, InfeasibleConfiguration
from .kangaroo import LEG_MODES


#: Bundle schema version. Bumped whenever a field's **meaning** changes or a
#: field the replay depends on is added. The loader refuses a version it does not
#: know rather than guessing (`TASK-040` D5): a bundle is meant to be read months
#: later, and a silently mis-parsed one is worse than an unreadable one.
SCHEMA_VERSION = 1

#: Where bundles live (`TASK-040` D2). Separate from ``plots/``, which is scratch
#: render output and already carries over 100 MB.
DEFAULT_ROOT = "experiments"

#: Agreement bound for the replay check, metres. The same run executed twice from
#: the same spec is deterministic (`PR-004`), so this is a floating-point
#: tolerance and not a modelling one — a real divergence means the spec is
#: incomplete, and shows up far above this.
REPLAY_TOLERANCE_M = 1e-9


class SpecError(ValueError):
    """A specification is malformed. The message names the field and its units."""


# --------------------------------------------------------------------------
# The specification
# --------------------------------------------------------------------------

#: Aircraft and harness parameters. Every one is an existing `HarnessConfig`
#: field under its existing name, units and default — the format introduces no
#: new tunable, which is what keeps a spec from becoming a second source of truth.
AIRCRAFT_FIELDS = ("airspeed_ms", "turn_radius_m", "orbit_radius_m",
                   "look_ahead_m", "dt_s")

#: Algorithm settings that are `HarnessConfig` fields rather than session ones.
ALGORITHM_CONFIG_FIELDS = ("replan_every", "hold_policy", "orbit_precompensate")


def default_spec():
    """A complete, valid specification with the harness defaults.

    Every field present and explicit: a defaulted field is a field that changes
    silently between runs, and the whole point of the format is that it does not.
    """
    cfg = HarnessConfig()
    return {
        "schema_version": SCHEMA_VERSION,
        "objective": "",
        "experiment_id": None,
        "aircraft": dict((name, getattr(cfg, name)) for name in AIRCRAFT_FIELDS),
        "algorithm": {
            "name": scenario.DEFAULT_ALGORITHM,
            "estimate": False,
            "lookahead_steps": 0,
            "replan_every": cfg.replan_every,
            "hold_policy": cfg.hold_policy,
            "orbit_precompensate": cfg.orbit_precompensate,
            "overrides": {},
        },
        "initial_conditions": {
            "start_range_m": 300.0,
            "plane_heading_deg": 0.0,
            "target_n_m": None,
            "target_e_m": None,
        },
        "kangaroo": {
            "radius_m": 150.0,
            "length_m": 300.0,
            "width_m": 150.0,
            "seed": None,
            "legs": [
                {"duration_s": 60.0, "mode": "straight", "heading_deg": 0.0,
                 "speed_ms": 5.0},
            ],
        },
        "zone": {"side_m": 2000.0, "contain_target": True,
                 "containment_margin_m": None},
        "run": {"duration_s": 60.0, "visualise": False},
    }


def _require(container, key, where):
    if key not in container:
        raise SpecError("%s is missing the required field %r" % (where, key))
    return container[key]


def validate_spec(spec):
    """Check a specification and return it, or raise :class:`SpecError`.

    Every failure names the offending field. Validation happens **before any run
    starts**, because discovering an unknown mode 90 simulated seconds in wastes
    the run and produces a bundle that documents a typo.
    """
    if not isinstance(spec, dict):
        raise SpecError("a specification must be a JSON object, got %s"
                        % type(spec).__name__)

    version = spec.get("schema_version")
    if version != SCHEMA_VERSION:
        raise SpecError(
            "schema_version is %r; this build writes and reads version %d. "
            "Refusing rather than guessing at the meaning of an unknown "
            "version." % (version, SCHEMA_VERSION))

    aircraft = _require(spec, "aircraft", "the specification")
    for name in AIRCRAFT_FIELDS:
        value = _require(aircraft, name, "aircraft")
        if not isinstance(value, (int, float)) or value <= 0.0:
            raise SpecError("aircraft.%s must be a positive number, got %r"
                            % (name, value))
    if aircraft["orbit_radius_m"] < aircraft["turn_radius_m"]:
        raise SpecError(
            "aircraft.orbit_radius_m (%.1f m) is below aircraft.turn_radius_m "
            "(%.1f m): the orbit would exceed the curvature bound (FR-005, "
            "SR-002)" % (aircraft["orbit_radius_m"], aircraft["turn_radius_m"]))

    algorithm = _require(spec, "algorithm", "the specification")
    name = _require(algorithm, "name", "algorithm")
    if name not in algorithms.REGISTRY:
        raise SpecError("algorithm.name %r is not registered; available: %s"
                        % (name, ", ".join(sorted(algorithms.REGISTRY))))
    lookahead = algorithm.get("lookahead_steps", 0)
    if not isinstance(lookahead, int) or lookahead < 0:
        raise SpecError("algorithm.lookahead_steps must be a whole tick count "
                        ">= 0, got %r" % (lookahead,))
    cls = algorithms.REGISTRY[name]
    if getattr(cls, "owns_horizon", False) and lookahead:
        raise SpecError(
            "algorithm %r selects its own prediction horizon, so "
            "algorithm.lookahead_steps must be 0 (it is the state-side horizon "
            "this algorithm replaces; both together lead the ring twice and the "
            "second lead appears in no recorded figure). Got %d. Set the "
            "candidate range through algorithm.overrides instead."
            % (name, lookahead))
    if not isinstance(algorithm.get("overrides", {}), dict):
        raise SpecError("algorithm.overrides must be an object of "
                        "HarnessConfig field names to values")

    kangaroo = _require(spec, "kangaroo", "the specification")
    legs = _require(kangaroo, "legs", "kangaroo")
    if not isinstance(legs, list) or not legs:
        raise SpecError("kangaroo.legs must be a non-empty list of legs")
    for index, leg in enumerate(legs):
        where = "kangaroo.legs[%d]" % index
        if not isinstance(leg, dict):
            raise SpecError("%s must be an object with duration_s, mode, "
                            "heading_deg and speed_ms" % where)
        duration = _require(leg, "duration_s", where)
        if not isinstance(duration, (int, float)) or duration <= 0.0:
            raise SpecError("%s.duration_s must be > 0 seconds, got %r"
                            % (where, duration))
        mode = _require(leg, "mode", where)
        if mode not in LEG_MODES:
            raise SpecError("%s.mode %r is unknown; use one of %s"
                            % (where, mode, ", ".join(LEG_MODES)))
        speed = _require(leg, "speed_ms", where)
        if not isinstance(speed, (int, float)) or speed < 0.0:
            raise SpecError("%s.speed_ms must be >= 0 m/s, got %r"
                            % (where, speed))
        _require(leg, "heading_deg", where)

    run = _require(spec, "run", "the specification")
    duration = _require(run, "duration_s", "run")
    if not isinstance(duration, (int, float)) or duration <= 0.0:
        raise SpecError("run.duration_s must be > 0 seconds, got %r" % duration)
    return spec


def spec_warnings(spec):
    """Non-fatal oddities worth telling the operator about, as a list of strings.

    Separate from validation on purpose. These are configurations that are legal
    and probably not what was meant; refusing them would be presumptuous, and
    saying nothing would let a run be wasted on a typo.
    """
    out = []
    kangaroo = spec["kangaroo"]
    for index, leg in enumerate(kangaroo["legs"]):
        if leg["mode"] == "point" and leg["speed_ms"] != 0.0:
            out.append("kangaroo.legs[%d] is a 'point' leg with speed_ms=%.1f; "
                       "a stationary target does not move and the speed is "
                       "ignored" % (index, leg["speed_ms"]))
        if leg["mode"] == "circle":
            radius = kangaroo["radius_m"]
            if radius > 0.0 and leg["speed_ms"] > 0.0:
                rate_deg_s = math.degrees(leg["speed_ms"] / radius)
                out.append(
                    "kangaroo.legs[%d] circles at %.1f m/s on a %.0f m radius: "
                    "that is %.1f deg/s, a %.0f s lap. speed_ms is a GROUND "
                    "speed, not an angular rate"
                    % (index, leg["speed_ms"], radius, rate_deg_s,
                       2.0 * math.pi * radius / leg["speed_ms"]))
    scheduled = sum(leg["duration_s"] for leg in kangaroo["legs"])
    if spec["run"]["duration_s"] > scheduled + 1e-9:
        out.append(
            "run.duration_s is %.1f s but the schedule covers %.1f s; the last "
            "leg is held for the remainder (the schedule clamps rather than "
            "stopping)" % (spec["run"]["duration_s"], scheduled))
    algorithm = spec["algorithm"]
    cls = algorithms.REGISTRY[algorithm["name"]]
    if getattr(cls, "requires_estimate", False) and not algorithm.get("estimate"):
        out.append("algorithm %r requires the state estimator; it will be "
                   "enabled automatically from the declared capability flag"
                   % algorithm["name"])
    return out


def load_spec(path):
    """Read and validate a specification file."""
    with open(path) as handle:
        return validate_spec(json.load(handle))


def save_spec(spec, path):
    """Write a specification, validated first."""
    validate_spec(spec)
    with open(path, "w") as handle:
        json.dump(spec, handle, indent=2)
        handle.write("\n")
    return path


# --------------------------------------------------------------------------
# Building and running
# --------------------------------------------------------------------------

def config_from_spec(spec):
    """The `HarnessConfig` a specification describes.

    Raises:
        SpecError: For an unknown override field, or one the config refuses.
    """
    settings = dict(spec["aircraft"])
    algorithm = spec["algorithm"]
    for name in ALGORITHM_CONFIG_FIELDS:
        if name in algorithm:
            settings[name] = algorithm[name]
    settings["lookahead_steps"] = algorithm.get("lookahead_steps", 0)
    for name, value in (algorithm.get("overrides") or {}).items():
        if name not in HarnessConfig.__slots__:
            raise SpecError(
                "algorithm.overrides.%s is not a HarnessConfig field; see "
                "docs/GLOSSARY.md's variable reference for the full list"
                % name)
        settings[name] = value
    try:
        return HarnessConfig(**settings)
    except (ValueError, TypeError) as exc:
        raise SpecError("the configuration is invalid: %s" % exc)


def session_from_spec(spec):
    """A :class:`~scenario.ScenarioSession` built from a specification."""
    validate_spec(spec)
    config = config_from_spec(spec)
    algorithm = spec["algorithm"]
    initial = spec["initial_conditions"]
    kangaroo = spec["kangaroo"]
    zone_spec = spec.get("zone") or {}

    legs = [(leg["duration_s"], leg["mode"], leg["heading_deg"], leg["speed_ms"])
            for leg in kangaroo["legs"]]

    zone = None
    if zone_spec.get("side_m") is None:
        zone = False                      # explicitly unbounded
    else:
        from . import zone as zone_mod
        zone = zone_mod.InclusionZone(side_m=zone_spec["side_m"])

    return scenario.ScenarioSession(
        legs=legs,
        algorithm=algorithm["name"],
        config=config,
        start_range_m=initial.get("start_range_m", 300.0),
        plane_heading_deg=initial.get("plane_heading_deg", 0.0),
        target_n_m=initial.get("target_n_m"),
        target_e_m=initial.get("target_e_m"),
        radius_m=kangaroo.get("radius_m", 150.0),
        length_m=kangaroo.get("length_m", 300.0),
        width_m=kangaroo.get("width_m", 150.0),
        zone=zone,
        contain_target=zone_spec.get("contain_target", True),
        containment_margin_m=zone_spec.get("containment_margin_m"),
        estimate=algorithm.get("estimate", False),
        lookahead_steps=algorithm.get("lookahead_steps", 0),
    )


def run_spec(spec):
    """Run a specification headlessly. Returns the finished session."""
    session = session_from_spec(spec)
    session.run(spec["run"]["duration_s"])
    return session


def spec_from_session(session, base_spec, source="interactive"):
    """Derive the specification that reproduces a finished session.

    The interactive half of `TASK-040` D1. `ScenarioSession.export_legs()` turns
    the applied changes back into an ordinary leg list — including the
    inclusion zone's own containment turns, which are applied through the same
    mechanism and so replay identically.
    """
    spec = json.loads(json.dumps(base_spec))       # deep copy, JSON-safe
    spec["kangaroo"]["legs"] = [
        {"duration_s": leg[0], "mode": leg[1], "heading_deg": leg[2],
         "speed_ms": leg[3]}
        for leg in session.export_legs()
    ]
    spec["run"]["duration_s"] = session.t_s
    spec["source"] = source
    return validate_spec(spec)


# --------------------------------------------------------------------------
# Provenance — VR-010
# --------------------------------------------------------------------------

def _git(*args):
    try:
        out = subprocess.run(("git",) + args, capture_output=True, text=True,
                             timeout=10,
                             cwd=os.path.dirname(os.path.abspath(__file__)))
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:                                   # pragma: no cover
        return None


def provenance():
    """Everything `VR-010` asks for about *how* a run was produced.

    The Git commit and the **dirty flag** together: a commit alone is misleading
    when the tree had uncommitted changes, which during active work it usually
    does. A figure that cannot be tied to the code that produced it is not
    evidence, and "the commit at the time" is not enough on its own.
    """
    status = _git("status", "--porcelain")
    return {
        "git_commit": _git("rev-parse", "HEAD"),
        "git_dirty": None if status is None else bool(status),
        "git_branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "harness_schema_version": SCHEMA_VERSION,
        "recorded_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(timespec="seconds"),
    }


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------

def metrics_record(session, n_a_max_steps=None):
    """Every defined error quantity for a finished session, as plain JSON.

    Deliberately reports **all** of them and nominates no headline (`TASK-040`
    D6). `TASK-033` D2 already established that quoting one ring error alone
    misleads in opposite directions — the true-target figure includes a designed
    prediction lead, and the held-centre figure excludes the thing the operator
    cares about.
    """
    history = session.history
    config = session.config
    record = {
        "steps": len(history),
        "duration_s": session.t_s,
        "stopped_reason": session.stopped_reason,
        "partial": bool(session.stopped_reason),
    }
    if not history:
        # A run that produced nothing is a row of nulls, not an absent record
        # (`VR-012`): a bundle of failures must not look like a shorter bundle
        # of successes.
        record.update({"ring": None, "deformation": None,
                       "deformation_by_speed": None, "tangent": None,
                       "reconvergence": None, "zone": None,
                       "max_curvature_1pm": None, "curvature_ok": None})
        return record

    radius = config.orbit_radius_m
    record["ring"] = metrics.dual_centre_stats(history, radius)
    record["min_orbit_distance_m"] = metrics.min_orbit_distance(history, radius)
    record["deformation"] = metrics.orbit_deformation(history, radius, n_bins=4)
    record["deformation_by_speed"] = metrics.deformation_by_speed(
        history, radius, n_bins=4, fraction=1.0)
    record["tangent"] = tangent_error.run_summary(history, config,
                                                  n_a_max_steps)
    record["reconvergence"] = session.reconvergence()
    record["zone"] = session.zone_report()

    curvatures = [(s.get("algorithm_state") or {}).get("curvature")
                  for s in history]
    curvatures = [c for c in curvatures if c is not None]
    bound = 1.0 / config.turn_radius_m
    record["max_curvature_1pm"] = max(curvatures) if curvatures else None
    record["curvature_bound_1pm"] = bound
    record["curvature_ok"] = (record["max_curvature_1pm"] is None
                              or record["max_curvature_1pm"] <= bound + 1e-9)
    return record


def _history_matches(a, b, tolerance_m=REPLAY_TOLERANCE_M):
    """Compare two histories, returning ``(ok, detail)``.

    Compares the **trajectory**, not the metrics. A spec missing a field that
    changes the outcome shows up here as a diverging path; two runs can agree on
    an RMS and be different flights, so metric agreement alone would not catch
    the failure this check exists for.
    """
    if len(a) != len(b):
        return False, "history length %d vs %d" % (len(a), len(b))
    worst_field, worst = None, 0.0
    for index, (x, y) in enumerate(zip(a, b)):
        for field in ("plane_n_m", "plane_e_m", "plane_hdg_rad",
                      "target_n_m", "target_e_m", "guidance_n_m",
                      "guidance_e_m"):
            delta = abs(x[field] - y[field])
            if delta > worst:
                worst, worst_field = delta, "%s at step %d" % (field, index)
    return worst <= tolerance_m, {
        "worst_delta": worst,
        "worst_field": worst_field,
        "tolerance_m": tolerance_m,
    }


def verify_replay(spec, session, n_a_max_steps=None):
    """Re-run the written spec and compare. Returns the verification block.

    `TASK-040` D4, on by default. A failure is **recorded, not raised**: a
    specification that does not reproduce its run means the format is missing a
    field, which is the single most valuable thing this module can discover, and
    it must be visible in the bundle rather than aborting the write that would
    have preserved the evidence.
    """
    block = {"attempted": True, "history_matches": None, "metrics_match": None,
             "detail": None, "error": None}
    try:
        replayed = run_spec(spec)
    except Exception as exc:                            # pragma: no cover
        block["error"] = "%s: %s" % (type(exc).__name__, exc)
        return block
    ok, detail = _history_matches(session.history, replayed.history)
    block["history_matches"] = ok
    block["detail"] = detail

    original = metrics_record(session, n_a_max_steps)
    again = metrics_record(replayed, n_a_max_steps)
    block["metrics_match"] = (
        json.dumps(original, sort_keys=True, default=str)
        == json.dumps(again, sort_keys=True, default=str))
    return block


# --------------------------------------------------------------------------
# The bundle
# --------------------------------------------------------------------------

def bundle_dir(experiment_id, root=DEFAULT_ROOT, when=None):
    """``<root>/YYYY-MM-DD/<experiment-id>/`` (`TASK-040` D2)."""
    when = when or datetime.date.today()
    return os.path.join(root, when.isoformat(), experiment_id)


def make_experiment_id(spec, when=None):
    """A stable, readable id: date, algorithm and the leading kangaroo mode."""
    when = when or datetime.date.today()
    mode = spec["kangaroo"]["legs"][0]["mode"]
    return "%s-%s-%s" % (when.isoformat(), spec["algorithm"]["name"], mode)


def marker_times(spec, session):
    """Simulated times at which the kangaroo was re-commanded.

    An interactive session records these in ``session.markers`` as it goes. A
    spec-driven one records **none** — its legs come from the schedule and never
    pass through ``apply_change`` — so the leg boundaries are derived here
    instead. Without them the graphs show a step in the error with nothing to
    attribute it to, which is most of what these graphs are for.

    Excludes ``t = 0`` and anything past the run's end.
    """
    if session.markers:
        return list(session.markers)
    times, elapsed = [], 0.0
    for leg in spec["kangaroo"]["legs"][:-1]:
        elapsed += leg["duration_s"]
        if 0.0 < elapsed < session.t_s:
            times.append(elapsed)
    return times


def write_bundle(spec, session, root=DEFAULT_ROOT, verify=True,
                 source="spec", n_a_max_steps=None, render=True):
    """Write the whole bundle and return ``{name: path}``.

    Args:
        spec: The specification that produced ``session``. For an interactive
            run this is the **derived** one.
        session: The finished session.
        root: Bundle root; see :data:`DEFAULT_ROOT`.
        verify: Run the replay check (`TASK-040` D4, default on).
        source: ``"spec"`` or ``"interactive"`` — the only difference between
            the two modes' bundles.
        n_a_max_steps: Transit limit for the registration error.
        render: Draw the PNGs. False for a headless test that only wants the JSON.
    """
    spec = json.loads(json.dumps(spec))
    spec["source"] = source
    experiment_id = spec.get("experiment_id") or make_experiment_id(spec)
    spec["experiment_id"] = experiment_id

    directory = bundle_dir(experiment_id, root)
    os.makedirs(directory, exist_ok=True)
    written = {}

    written["spec"] = save_spec(spec, os.path.join(directory, "spec.json"))

    config = session.config
    all_series = series_mod.all_series(session.history, config, n_a_max_steps)
    record = {
        "schema_version": SCHEMA_VERSION,
        "experiment_id": experiment_id,
        "objective": spec.get("objective", ""),
        "source": source,
        # SR-004, carried in the bundle's own fields so it travels with the file.
        "not_a_flight_configuration": (
            "Harness configuration for offline geometric validation. No value "
            "here is an approved flight-safety limit (SR-004), and no figure "
            "here is tracking evidence (A-VAL-001, PR-006)."),
        "provenance": provenance(),
        "algorithm": dict(spec["algorithm"]),
        "config": dict((name, getattr(config, name))
                       for name in HarnessConfig.__slots__
                       if not name.startswith("_")),
        "kangaroo": {
            "legs_flown": [
                {"duration_s": leg[0], "mode": leg[1], "heading_deg": leg[2],
                 "speed_ms": leg[3]} for leg in session.export_legs()],
            "changes": session.change_log,
            "containment_turns": len(session.containment_events),
            "seed": spec["kangaroo"].get("seed"),
        },
        "metrics": metrics_record(session, n_a_max_steps),
        "series_summary": dict(
            (name, {"coverage": s.coverage, "stats": s.stats(),
                    "excluded": s.excluded})
            for name, s in all_series.items()),
    }
    if verify:
        record["replay_verification"] = verify_replay(spec, session,
                                                      n_a_max_steps)
    else:
        record["replay_verification"] = {
            "attempted": False,
            "why": "disabled with --no-verify; the spec's claim to reproduce "
                   "this run is therefore untested",
        }

    with open(os.path.join(directory, "record.json"), "w") as handle:
        json.dump(record, handle, indent=2, default=str)
        handle.write("\n")
    written["record"] = os.path.join(directory, "record.json")

    written["history"] = plotter.save_run(
        os.path.join(directory, "history.json"), experiment_id,
        session.history,
        meta={"algorithm": session.algorithm_name,
              "orbit_radius_m": config.orbit_radius_m,
              "experiment_id": experiment_id})

    with open(os.path.join(directory, "series.json"), "w") as handle:
        json.dump({"schema_version": SCHEMA_VERSION,
                   "series": dict((n, s.as_dict())
                                  for n, s in all_series.items())},
                  handle, indent=2)
        handle.write("\n")
    written["series"] = os.path.join(directory, "series.json")

    if render and session.history:
        plot_data = series_mod.to_plot_data(
            all_series, marker_times(spec, session), 1.0 / config.turn_radius_m)
        path = os.path.join(directory, "error-timeline.png")
        if plotter.plot_error_timeline(
                plot_data, save_path=path,
                title="%s — %s" % (experiment_id, session.algorithm_name)):
            written["error_timeline"] = path

        from . import scenario_driver
        out = scenario_driver.render_session(
            session, save_path=os.path.join(directory, "view.png"))
        if "png" in out:
            written["view"] = out["png"]

    return written


def load_bundle(directory):
    """Read a bundle back. Refuses an unknown ``schema_version``.

    Returns ``{"spec", "record", "series"}``; ``history.json`` is left on disk
    and loaded by the caller through :func:`plotter.load_run` when wanted, since
    it is the large part and most analysis does not need it.
    """
    with open(os.path.join(directory, "record.json")) as handle:
        record = json.load(handle)
    version = record.get("schema_version")
    if version != SCHEMA_VERSION:
        raise SpecError(
            "bundle at %s has schema_version %r; this build reads version %d. "
            "Refusing rather than guessing." % (directory, version,
                                                SCHEMA_VERSION))
    with open(os.path.join(directory, "spec.json")) as handle:
        spec = json.load(handle)
    series_path = os.path.join(directory, "series.json")
    loaded = {}
    if os.path.isfile(series_path):
        with open(series_path) as handle:
            raw = json.load(handle)
        loaded = dict((n, series_mod.from_dict(d))
                      for n, d in raw.get("series", {}).items())
    return {"spec": spec, "record": record, "series": loaded}


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(
        description="Run a standoff experiment from a specification or by hand, "
                    "and archive it as one auditable bundle (TASK-040).")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--spec", dest="spec_path",
                      help="Run this JSON specification.")
    mode.add_argument("--interactive", action="store_true",
                      help="Open the scenario driver, fly the kangaroo by hand, "
                           "and derive the specification from what was done.")
    mode.add_argument("--write-template", dest="template_path",
                      help="Write a complete default specification here and "
                           "exit, as a starting point to edit.")
    parser.add_argument("--visualise", action="store_true",
                        help="Watch a --spec run in the scenario driver. The "
                             "result is identical to the headless run; the view "
                             "does not touch the session.")
    parser.add_argument("--out-dir", default=DEFAULT_ROOT,
                        help="Bundle root (default: %(default)s).")
    parser.add_argument("--experiment-id", default=None,
                        help="Override the generated experiment id.")
    parser.add_argument("--objective", default=None,
                        help="One line on what this run is meant to answer. "
                             "Recorded in the bundle (VR-010).")
    parser.add_argument("--no-verify", dest="verify", action="store_false",
                        help="Skip the replay verification. It is ON by default "
                             "(TASK-040 D4); skipping is recorded in the bundle, "
                             "because an unverified spec is an untested claim.")
    parser.add_argument("--no-render", dest="render", action="store_false",
                        help="Write the JSON only, no PNGs.")
    # Interactive-mode starting point; everything else comes from the template.
    parser.add_argument("--algorithm", default=None,
                        choices=sorted(algorithms.REGISTRY),
                        help="Interactive mode: algorithm to fly.")
    parser.add_argument("--mode", default=None, choices=list(LEG_MODES),
                        help="Interactive mode: initial kangaroo mode.")
    parser.add_argument("--speed-ms", type=float, default=None,
                        help="Interactive mode: initial kangaroo speed.")
    parser.add_argument("--heading-deg", type=float, default=None,
                        help="Interactive mode: initial kangaroo heading.")
    parser.add_argument("--estimate", action="store_true",
                        help="Interactive mode: run the state estimator.")
    parser.add_argument("--lookahead-steps", type=int, default=None,
                        help="Interactive mode: prediction horizon, ticks.")
    return parser


def _interactive_spec(args):
    spec = default_spec()
    if args.algorithm:
        spec["algorithm"]["name"] = args.algorithm
    if args.estimate:
        spec["algorithm"]["estimate"] = True
    if args.lookahead_steps is not None:
        spec["algorithm"]["lookahead_steps"] = args.lookahead_steps
    leg = spec["kangaroo"]["legs"][0]
    if args.mode:
        leg["mode"] = args.mode
    if args.speed_ms is not None:
        leg["speed_ms"] = args.speed_ms
    if args.heading_deg is not None:
        leg["heading_deg"] = args.heading_deg
    # An interactive run has no scheduled end; the operator closes the window.
    leg["duration_s"] = 3600.0
    spec["run"]["duration_s"] = 3600.0
    return validate_spec(spec)


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.template_path:
        save_spec(default_spec(), args.template_path)
        print("wrote a default specification -> %s" % args.template_path)
        return 0

    if args.interactive:
        spec = _interactive_spec(args)
        source = "interactive"
    else:
        try:
            spec = load_spec(args.spec_path)
        except SpecError as exc:
            print("INVALID SPECIFICATION: %s" % exc, file=sys.stderr)
            return 2
        source = "spec"

    if args.objective is not None:
        spec["objective"] = args.objective
    if args.experiment_id is not None:
        spec["experiment_id"] = args.experiment_id

    for warning in spec_warnings(spec):
        print("note: %s" % warning, file=sys.stderr)

    try:
        session = session_from_spec(spec)
    except (SpecError, InfeasibleConfiguration) as exc:
        print("REFUSED: %s" % exc, file=sys.stderr)
        return 2

    if args.interactive or (args.visualise and not args.interactive):
        from . import scenario_driver
        view = scenario_driver.ScenarioView(session)
        view.show()
        if source == "interactive":
            spec = spec_from_session(session, spec, source="interactive")
    else:
        session.run(spec["run"]["duration_s"])

    if session.stopped_reason:
        print("run stopped early: %s" % session.stopped_reason, file=sys.stderr)

    written = write_bundle(spec, session, root=args.out_dir,
                           verify=args.verify, source=source,
                           render=args.render)
    print("experiment bundle -> %s" % os.path.dirname(written["spec"]))
    for name, path in sorted(written.items()):
        print("  %-15s %s" % (name, path))

    with open(written["record"]) as handle:
        record = json.load(handle)
    verification = record.get("replay_verification") or {}
    if verification.get("attempted"):
        if verification.get("history_matches"):
            print("replay verified: the spec reproduces this run")
        else:
            print("REPLAY DIVERGED — the specification does not reproduce this "
                  "run, so the format is probably missing a field. Detail: %s"
                  % verification.get("detail"), file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
