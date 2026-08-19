"""Configuration front-end for the validation harness (``TASK-013``).

Captures a run's configuration and initial state **before** the harness runs and
plots, so a user assembles a run in one place instead of memorising a long
``run_harness`` command line. Two front doors: a **JSON config file** and an
**interactive prompt**. Both build a plain *spec* dict, which is translated to the
canonical ``run_harness`` argv and delegated to :func:`run_harness.main`.

Design (provisional UI mechanism, ``DEC-2026-07-28-01``; recorded, retunable):
translating a spec to the CLI and delegating makes the front-end **CLI-equivalent
by construction** — it cannot drift from the command line, ``config.py`` stays
immutable per run, and the plotter stays read-only (``DEC-2026-07-22-01``): the
front-end only assembles settings, it never drives an algorithm or the plotter.
A graphical (e.g. Tk) front door is deferred.

Mode and start-position **selection** (``TASK-015``) live here as spec fields: the
``kangaroo.mode`` choice and a ``target`` start position given either explicitly
(``n_m``/``e_m``) or as a ``range_m`` + ``bearing_deg`` preset.

The spec is a nested dict with these sections (all optional)::

    {
      "algorithms": ["dubins_orbit", ...],        # repeated --algorithm
      "config":   {airspeed_ms, turn_radius_m, orbit_radius_m, look_ahead_m,
                   weave_lambda_m, weave_a_cap_m, weave_d_start_m,
                   weave_d_full_m, weave_eta, weave_vaw_lead_s},
      "plane":    {heading_deg},
      "target":   {n_m, e_m | range_m[, bearing_deg], speed_ms,
                   heading_deg, vn_ms, ve_ms},
      "kangaroo": {mode, heading_deg, fwd_m, disp_m, radius_m,
                   length_m, width_m, speed_ms},
      "run":      {duration_s, estimate, no_plot, no_3d, compare_eta,
                   allow_infeasible, save_run, save_plot, animate},
    }
"""

import argparse
import json
import math

from . import kangaroo
from . import run_harness

# --------------------------------------------------------------------------
# Spec -> CLI flag mappings. Only real run_harness flags appear here, so a spec
# can never emit a flag the CLI does not have.
# --------------------------------------------------------------------------

_CONFIG_FLAGS = {
    "airspeed_ms": "--airspeed-ms",
    "turn_radius_m": "--turn-radius-m",
    "orbit_radius_m": "--orbit-radius-m",
    "weave_lambda_m": "--weave-lambda",
    "weave_a_cap_m": "--weave-a-cap",
    "weave_d_start_m": "--weave-d-start",
    "weave_d_full_m": "--weave-d-full",
    "weave_eta": "--eta",
    "weave_vaw_lead_s": "--weave-vaw-lead-s",
    "lookahead_steps": "--lookahead-steps",
    "look_ahead_m": "--look-ahead-m",
}

# Config keys whose flag takes an int, not a float (emitted without decimals).
_CONFIG_INT_FLAGS = ("lookahead_steps",)

_PLANE_FLAGS = {"heading_deg": "--plane-heading-deg"}

_TARGET_FLAGS = {
    "n_m": "--target-n-m",
    "e_m": "--target-e-m",
    "speed_ms": "--target-speed-ms",
    "heading_deg": "--target-heading-deg",
    "vn_ms": "--target-vn-ms",
    "ve_ms": "--target-ve-ms",
}

_KANG_FLAGS = {
    "mode": "--kang-mode",
    "heading_deg": "--kang-heading-deg",
    "fwd_m": "--kang-fwd-m",
    "disp_m": "--kang-disp-m",
    "radius_m": "--kang-radius-m",
    "length_m": "--kang-length-m",
    "width_m": "--kang-width-m",
    "speed_ms": "--kang-speed-ms",
    "seed": "--kang-seed",
}

# Kangaroo keys whose flag takes an int, not a float.
_KANG_INT_FLAGS = ("seed",)

_RUN_BOOL_FLAGS = {
    "estimate": "--estimate",
    "no_plot": "--no-plot",
    "no_3d": "--no-3d",
    "compare_eta": "--compare-eta",
    "allow_infeasible": "--allow-infeasible",
    "no_orbit_precomp": "--no-orbit-precomp",
}

_RUN_VALUE_FLAGS = {
    "duration_s": "--duration-s",
    "save_run": "--save-run",
    "save_plot": "--save-plot",
    "animate": "--animate",
}

_SECTIONS = ("algorithms", "config", "plane", "target", "kangaroo", "run")


def _reject_unknown(where, keys, allowed):
    extra = set(keys) - set(allowed)
    if extra:
        raise ValueError(
            "unknown %s key(s) %s; allowed: %s"
            % (where, ", ".join(sorted(extra)), ", ".join(sorted(allowed)))
        )


def _start_position_argv(target):
    """Argv for the target start position from a spec ``target`` section.

    Accepts either an explicit ``n_m``/``e_m``, or a ``range_m`` (+ optional
    ``bearing_deg``, from North clockwise) preset that is resolved to
    ``--target-n-m``/``--target-e-m``. A bare ``range_m`` with no bearing keeps
    the CLI's due-North placement via ``--start-range-m``.

    Raises:
        ValueError: If both an explicit position and a range are given.
    """
    has_ne = "n_m" in target or "e_m" in target
    has_range = "range_m" in target
    if has_ne and has_range:
        raise ValueError(
            "target start position over-specified: give either n_m/e_m or "
            "range_m (+ bearing_deg), not both"
        )
    if has_range and "bearing_deg" in target:
        rng = float(target["range_m"])
        bearing = math.radians(float(target["bearing_deg"]))
        n = rng * math.cos(bearing)
        e = rng * math.sin(bearing)
        return ["--target-n-m", repr(n), "--target-e-m", repr(e)]
    if has_range:
        return ["--start-range-m", repr(float(target["range_m"]))]
    return []


def spec_to_argv(spec):
    """Translate a spec dict to a ``run_harness`` argv list.

    Pure and deterministic. Rejects unknown sections or keys so a malformed spec
    fails before any run starts (the front-end owns configuration). Value
    validation (bounds, choices) is left to ``run_harness``/``HarnessConfig``, so
    there is exactly one validator.
    """
    if not isinstance(spec, dict):
        raise ValueError("spec must be a dict, got %r" % type(spec).__name__)
    _reject_unknown("spec", spec, _SECTIONS)

    argv = []

    for name in spec.get("algorithms", []):
        argv += ["--algorithm", str(name)]

    config = spec.get("config", {})
    _reject_unknown("config", config, _CONFIG_FLAGS)
    for key, flag in _CONFIG_FLAGS.items():
        if key in config:
            if key in _CONFIG_INT_FLAGS:
                argv += [flag, str(int(config[key]))]
            else:
                argv += [flag, repr(float(config[key]))]

    plane = spec.get("plane", {})
    _reject_unknown("plane", plane, _PLANE_FLAGS)
    for key, flag in _PLANE_FLAGS.items():
        if key in plane:
            argv += [flag, repr(float(plane[key]))]

    target = spec.get("target", {})
    _reject_unknown("target", target,
                    set(_TARGET_FLAGS) | {"range_m", "bearing_deg"})
    argv += _start_position_argv(target)
    for key, flag in _TARGET_FLAGS.items():
        if key in target:
            argv += [flag, repr(float(target[key]))]

    kang = spec.get("kangaroo", {})
    _reject_unknown("kangaroo", kang, _KANG_FLAGS)
    if "mode" in kang:
        argv += ["--kang-mode", str(kang["mode"])]
    for key, flag in _KANG_FLAGS.items():
        if key == "mode" or key not in kang:
            continue
        if key in _KANG_INT_FLAGS:
            argv += [flag, str(int(kang[key]))]
        else:
            argv += [flag, repr(float(kang[key]))]

    run = spec.get("run", {})
    _reject_unknown("run", run, set(_RUN_BOOL_FLAGS) | set(_RUN_VALUE_FLAGS))
    for key, flag in _RUN_BOOL_FLAGS.items():
        if run.get(key):
            argv.append(flag)
    for key, flag in _RUN_VALUE_FLAGS.items():
        if key in run:
            value = run[key]
            argv += [flag, str(value) if key != "duration_s" else repr(float(value))]

    return argv


def load_spec(path):
    """Load a spec from a JSON file."""
    with open(path) as handle:
        spec = json.load(handle)
    if not isinstance(spec, dict):
        raise ValueError("config file must contain a JSON object, got %r"
                         % type(spec).__name__)
    return spec


def run_spec(spec, extra_argv=None):
    """Launch a run from a spec by delegating to :func:`run_harness.main`.

    Returns the ``run_harness.main`` exit code. ``extra_argv`` is appended after
    the translated flags (e.g. a one-off ``--no-plot``).
    """
    argv = spec_to_argv(spec)
    if extra_argv:
        argv += list(extra_argv)
    return run_harness.main(argv)


def prompt_spec(input_fn=input, print_fn=print):
    """Build a spec interactively. ``input_fn`` is injectable for testing.

    Only the common choices are prompted; blank answers keep the harness
    defaults. This is a thin convenience over :func:`spec_to_argv`; it adds no
    configuration of its own.
    """
    def ask(label, default=None):
        prompt = "%s%s: " % (label, "" if default is None else " [%s]" % default)
        answer = input_fn(prompt).strip()
        return answer or default

    print_fn("Configure a harness run (blank = default).")
    spec = {"algorithms": [], "target": {}, "kangaroo": {}, "run": {}}

    algo = ask("algorithm", "dubins_orbit")
    if algo:
        spec["algorithms"].append(algo)

    mode = ask("kangaroo mode %s (blank = simple target)" % (kangaroo.MODES,))
    if mode:
        spec["kangaroo"]["mode"] = mode
        speed = ask("kangaroo speed m/s", "5")
        if speed:
            spec["kangaroo"]["speed_ms"] = float(speed)
    else:
        rng = ask("start range m", "800")
        if rng:
            spec["target"]["range_m"] = float(rng)
        bearing = ask("start bearing deg (from North)", "")
        if bearing:
            spec["target"]["bearing_deg"] = float(bearing)
        speed = ask("target speed m/s", "")
        if speed:
            spec["target"]["speed_ms"] = float(speed)

    duration = ask("duration s", "60")
    if duration:
        spec["run"]["duration_s"] = float(duration)

    return spec


def main(argv=None):
    """Entry point: ``python3 -m py_harness.frontend [--config F | --interactive]``."""
    parser = argparse.ArgumentParser(
        description="Configuration front-end for the validation harness "
        "(TASK-013). Build a run from a JSON config file or interactively, then "
        "launch it through run_harness.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--config", help="Path to a JSON spec file.")
    source.add_argument("--interactive", action="store_true",
                        help="Prompt for the common settings.")
    args, passthrough = parser.parse_known_args(argv)

    if args.config:
        spec = load_spec(args.config)
    else:
        spec = prompt_spec()

    try:
        return run_spec(spec, extra_argv=passthrough)
    except ValueError as exc:
        print("INVALID SPEC: %s" % exc)
        return 2


if __name__ == "__main__":
    import sys
    sys.exit(main())
