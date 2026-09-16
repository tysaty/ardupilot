"""Spec-to-schedule carriage: a Python ``spec.json`` becomes the Lua cell table
the vehicle replays (``TASK-052`` D2, ``TASK-046`` D4).

The spec is the input (`TASK-052` P4): nothing here is typed by hand. The
generated ``sitl_cell.lua`` carries the cell's leg list, geometry, start
position, algorithm name, the **flattened HarnessConfig** the ported modules
read (``algorithms.config_dict``), and the estimator settings the Python
session would use. ``harness_segments.lua`` replays the legs on the vehicle
exactly as ``kangaroo.make_segments`` does in the harness, which
:func:`check_against_lua` proves tick for tick through the same sandbox the
`TASK-006` differential gates use.

The generated file is a Lua **module** (``return { ... }``) placed in the SITL
``scripts/modules/`` directory by :mod:`kangaroo_follow.stage_scripts`; the runner
``require``s it. It is regenerated per cell and never edited.
"""

import json
import math
import os

from . import paths  # noqa: F401  (sys.path side effect)

from py_harness import algorithms, experiment
from py_harness import kangaroo as kang
from py_harness.estimator import KalmanFilter

#: Every cell's control interval is the spec's ``aircraft.dt_s``; the runner
#: schedules itself at that period in milliseconds.
DEFAULT_HEADING_SOURCE = "course"
HEADING_SOURCES = ("course", "yaw")

#: Tick-for-tick agreement bound for the Python versus Lua schedule replay,
#: metres. The two are the same arithmetic in two languages; the bound is the
#: differential harness's position tolerance, not a modelling one.
REPLAY_TOLERANCE_M = 1e-6


class ScheduleError(ValueError):
    """The spec cannot be carried to the vehicle. The message names why."""


# --------------------------------------------------------------------------
# Lua serialisation
# --------------------------------------------------------------------------

def _lua_string(text):
    return '"%s"' % (str(text).replace("\\", "\\\\").replace('"', '\\"'))


def to_lua(value, indent=0):
    """A JSON-shaped Python value as a Lua literal.

    ``None`` becomes ``nil`` (a table field set to nil is absent, which is the
    Lua reading of "not given"); booleans, ints and floats are literal; a list
    is a 1-indexed array; a dict is keyed by identifier where possible.
    ``inf``/``nan`` are refused: a spec carrying either is not a cell.
    """
    pad = "  " * indent
    if value is None:
        return "nil"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            raise ScheduleError("cannot carry %r to Lua" % value)
        return repr(value)
    if isinstance(value, str):
        return _lua_string(value)
    if isinstance(value, (list, tuple)):
        if not value:
            return "{}"
        items = [to_lua(v, indent + 1) for v in value]
        return "{\n" + "".join("%s  %s,\n" % (pad, i) for i in items) + pad + "}"
    if isinstance(value, dict):
        if not value:
            return "{}"
        lines = []
        for key in sorted(value):
            body = to_lua(value[key], indent + 1)
            if str(key).isidentifier():
                lines.append("%s  %s = %s,\n" % (pad, key, body))
            else:
                lines.append("%s  [%s] = %s,\n" % (pad, _lua_string(key), body))
        return "{\n" + "".join(lines) + pad + "}"
    raise ScheduleError("cannot carry %s to Lua" % type(value).__name__)


# --------------------------------------------------------------------------
# Spec -> cell table
# --------------------------------------------------------------------------

def target_start(spec):
    """The kangaroo's tick-0 ``(n_m, e_m)``, as ``ScenarioSession`` places it."""
    initial = spec["initial_conditions"]
    tn = initial.get("target_n_m")
    te = initial.get("target_e_m")
    tn = float(initial.get("start_range_m", 300.0)) if tn is None else float(tn)
    te = 0.0 if te is None else float(te)
    return tn, te


def legs_to_named(legs):
    """A leg list (spec dicts or harness tuples) as the Lua's named form,
    ``elastic_base`` only when set."""
    out = []
    for leg in legs:
        if not isinstance(leg, dict):
            leg = experiment.leg_dict(leg)
        row = {
            "duration_s": float(leg["duration_s"]),
            "mode": str(leg["mode"]),
            "heading_deg": float(leg.get("heading_deg", 0.0)),
            "speed_ms": float(leg.get("speed_ms", 0.0)),
        }
        base = leg.get(kang.ELASTIC_BASE_FIELD)
        if base is not None:
            row["elastic_base"] = str(base)
        out.append(row)
    return out


def legs_from_spec(spec):
    """The spec's own legs, named."""
    return legs_to_named(spec["kangaroo"]["legs"])


def geometry_from_spec(spec):
    k = spec["kangaroo"]
    return {"radius_m": float(k.get("radius_m", 150.0)),
            "length_m": float(k.get("length_m", 300.0)),
            "width_m": float(k.get("width_m", 150.0))}


def cell_table(spec, cell_id, heading_source=DEFAULT_HEADING_SOURCE,
               legs=None):
    """The plain dict ``sitl_cell.lua`` returns, from a validated spec.

    Args:
        legs: The legs to carry instead of the spec's own: the Python
            bundle's ``record.kangaroo.legs_flown``, which includes the zone's
            containment turns the harness applied during the run
            (`TASK-046` D4: the schedule must match the Python cell, and the
            spec's legs are only the schedule *before* containment). When
            absent the spec's legs are carried and ``legs_source`` says so.

    Raises:
        ScheduleError: When the spec is invalid, the heading source unknown,
            or the algorithm needs something the vehicle cannot supply.
    """
    if heading_source not in HEADING_SOURCES:
        raise ScheduleError("heading_source must be one of %s, got %r"
                            % (", ".join(HEADING_SOURCES), heading_source))
    try:
        experiment.validate_spec(spec)
        config = experiment.config_from_spec(spec)
    except experiment.SpecError as exc:
        raise ScheduleError(str(exc))
    algorithm = spec["algorithm"]
    name = algorithm["name"]
    built = algorithms.build(name, algorithms.config_dict(config))
    estimate = bool(algorithm.get("estimate", False)) or bool(
        getattr(built, "requires_estimate", False))
    lookahead_steps = int(algorithm.get("lookahead_steps", 0))
    if getattr(built, "owns_horizon", False) and lookahead_steps:
        raise ScheduleError(
            "%s owns its horizon; lookahead_steps must be 0 (it is %d)"
            % (name, lookahead_steps))
    tn, te = target_start(spec)
    kf = KalmanFilter()
    return {
        "cell_id": str(cell_id),
        "experiment_id": spec.get("experiment_id"),
        "algorithm": name,
        "cfg": algorithms.config_dict(config),
        "legs": legs_to_named(legs) if legs is not None else legs_from_spec(spec),
        "legs_source": "legs_flown" if legs is not None else "spec",
        "geometry": geometry_from_spec(spec),
        "target_n_m": tn,
        "target_e_m": te,
        "plane_heading_deg": float(
            spec["initial_conditions"].get("plane_heading_deg", 0.0)),
        "duration_s": float(spec["run"]["duration_s"]),
        "dt_s": float(config.dt_s),
        "estimate": estimate,
        "lookahead_steps": lookahead_steps,
        "estimator": {"process_noise": kf.process_noise,
                      "measurement_noise": kf.measurement_noise},
        "heading_source": heading_source,
        "cs_sense_margin_m": float(config.cs_sense_margin_m),
    }


def render_cell_module(table):
    """``sitl_cell.lua`` text for a :func:`cell_table` dict."""
    cfg = dict(table["cfg"])
    # The hysteresis margin is a HarnessConfig field the flattened dict does
    # not carry; the arm 0H adapter reads it from cfg like every other value.
    cfg["cs_sense_margin_m"] = table["cs_sense_margin_m"]
    body = dict(table)
    body["cfg"] = cfg
    return ("-- Generated by kangaroo_follow/schedule.py from the Python spec; do not edit.\n"
            "-- Cell %s, algorithm %s. Regenerated for every run.\n"
            "return %s\n" % (table["cell_id"], table["algorithm"], to_lua(body)))


def write_cell_module(spec, cell_id, path, heading_source=DEFAULT_HEADING_SOURCE,
                      legs=None):
    """Generate and write the module. Returns the table it was built from."""
    table = cell_table(spec, cell_id, heading_source, legs=legs)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as handle:
        handle.write(render_cell_module(table))
    return table


# --------------------------------------------------------------------------
# The Python side of the same schedule, and the tick-for-tick check
# --------------------------------------------------------------------------

def python_schedule(spec, legs=None):
    """``kangaroo(t) -> (n, e, vn, ve)`` exactly as the harness session builds it."""
    tn, te = target_start(spec)
    geometry = geometry_from_spec(spec)
    legs = [experiment.leg_tuple(leg) for leg in
            (legs if legs is not None else spec["kangaroo"]["legs"])]
    segments = kang.make_segments(legs, tn, te, **geometry)
    return kang.segments_callable(segments)


def check_against_lua(spec, sandbox, every_s=None, tolerance_m=REPLAY_TOLERANCE_M,
                      legs=None, history=None):
    """Replay the cell's schedule through ``harness_segments.lua`` and compare.

    Args:
        spec: The cell's spec.
        sandbox: A :class:`py_harness.luadiff.LuaSandbox`.
        every_s: Sample interval; default the spec's ``dt_s``.
        tolerance_m: Position agreement bound.
        legs: Legs to carry (see :func:`cell_table`).
        history: When given, the Python bundle's recorded history: the Lua
            replay is then compared against the **recorded** target positions
            at the recorded times, which is the like-for-like the SITL cell
            claims, rather than against the Python schedule function.

    Returns:
        ``{"ticks", "max_divergence_m", "ok"}``.

    Raises:
        ScheduleError: On the first tick past tolerance, naming the time.
    """
    table = cell_table(spec, spec.get("experiment_id") or "check", legs=legs)
    lua_segments = sandbox.call("harness_segments", "make_segments",
                                table["legs"], table["target_n_m"],
                                table["target_e_m"], table["geometry"], 0.0)
    if lua_segments is None:
        raise ScheduleError("harness_segments.make_segments refused: %s"
                            % sandbox.last_reason)
    if history is not None:
        samples = [(h["t_s"], (h["target_n_m"], h["target_e_m"]))
                   for h in history]
    else:
        py = python_schedule(spec, legs=legs)
        dt = float(every_s or table["dt_s"])
        n = int(round(table["duration_s"] / dt)) + 1
        samples = [(i * dt, py(i * dt)[:2]) for i in range(n)]
    n = len(samples)
    worst = 0.0
    for t, expected in samples:
        got = sandbox.call("harness_segments", "state_at", lua_segments, t)
        if got is None:
            raise ScheduleError("state_at refused at t=%.1f: %s"
                                % (t, sandbox.last_reason))
        d = math.hypot(got[0] - expected[0], got[1] - expected[1])
        worst = max(worst, d)
        if d > tolerance_m:
            raise ScheduleError(
                "schedule diverges at t=%.1f s: python (%.6f, %.6f) lua "
                "(%.6f, %.6f), %.3g m" % (t, expected[0], expected[1],
                                           got[0], got[1], d))
    return {"ticks": n, "max_divergence_m": worst, "ok": True}


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(
        description="Generate sitl_cell.lua from a Python spec.json.")
    parser.add_argument("spec")
    parser.add_argument("--cell-id", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--heading-source", default=DEFAULT_HEADING_SOURCE,
                        choices=HEADING_SOURCES)
    parser.add_argument("--check", action="store_true",
                        help="replay through harness_segments.lua and compare")
    args = parser.parse_args(argv)
    with open(args.spec) as handle:
        spec = json.load(handle)
    table = write_cell_module(spec, args.cell_id, args.out, args.heading_source)
    print("wrote %s (%d legs, %.0f s, %s)" % (
        args.out, len(table["legs"]), table["duration_s"], table["algorithm"]))
    if args.check:
        from py_harness import luadiff
        result = check_against_lua(spec, luadiff.LuaSandbox())
        print("replay agrees over %d ticks, worst %.3g m"
              % (result["ticks"], result["max_divergence_m"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
