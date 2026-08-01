"""Batch / loop runner (``TASK-016``).

Runs the harness **repeatedly under one parameter set**, varying only the
selected mode/position, and collates the saved runs for comparison — e.g. a
straight-line walk then a rectangular walk with otherwise identical parameters.

A batch is a **base** front-end spec (:mod:`py_harness.frontend`) plus a list of
**variants**, each a partial spec with a unique ``label`` that overrides only what
should differ (typically the ``kangaroo``/``target`` sections). Every run shares
the base's ``config``, so a batch cannot silently change a flight parameter
between runs.

Each variant is run through :func:`frontend.run_spec` with ``--save-run`` and
``--no-plot`` forced, so the run is written as the existing run JSON
(``plotter.save_run``). The runner then collates the saved runs with the
**read-only** plotter (``DEC-2026-07-22-01``): a combined overlay and/or one plot
per run. Nothing here drives an algorithm — it launches runs and hands *files*
(then data) to the plotter.

Out of scope (``A-VAL-001``): this is offline geometry batching, **not** the
outstanding parameter sweep of the shipping Lua controller.
"""

import argparse
import copy
import glob
import json
import os

from . import frontend
from . import plotter


def merge_specs(base, override):
    """Deep-merge ``override`` onto a copy of ``base`` (override wins).

    Nested dict sections merge key-by-key; scalars and lists replace. Neither
    argument is mutated.
    """
    result = copy.deepcopy(base) if base else {}
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_specs(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _run_variant(base_spec, label, override, out_dir):
    """Run one variant to disk; return the loaded ``(name, history, meta, path)``."""
    spec = merge_specs(base_spec, override)
    stem = os.path.join(out_dir, label)
    spec = merge_specs(spec, {"run": {"save_run": stem + ".json", "no_plot": True}})
    frontend.run_spec(spec)
    loaded = []
    for path in sorted(glob.glob(stem + "-*.json")):
        name, history, meta = plotter.load_run(path)
        loaded.append((name, history, meta, path))
    return loaded


def run_batch(base_spec, variants, out_dir, orbit_radius_m=None,
              combined_save=None, separate=False, show=False):
    """Run every variant under the shared ``base_spec`` and collate the results.

    Args:
        base_spec: The shared front-end spec (its ``config`` is common to all runs).
        variants: List of dicts, each with a unique ``label`` plus the spec fields
            it overrides (typically ``kangaroo``/``target``).
        out_dir: Directory for the saved run JSON (created if absent).
        orbit_radius_m: Ring radius for the plots; taken from the first run's
            metadata when omitted.
        combined_save: PNG path for the combined overlay, or ``None``.
        separate: Also write one PNG per run, beside its JSON.
        show: Call ``plt.show()`` on the plots (off by default; headless-safe).

    Returns:
        List of ``(display_name, history, path)`` for the collated runs.

    Raises:
        ValueError: If ``variants`` is empty or a label is missing/duplicated.
    """
    if not variants:
        raise ValueError("run_batch needs at least one variant")
    labels = [v.get("label") for v in variants]
    if None in labels:
        raise ValueError("each variant needs a 'label'")
    if len(set(labels)) != len(labels):
        raise ValueError("variant labels must be unique, got %s" % labels)

    os.makedirs(out_dir, exist_ok=True)

    collected = []  # (display_name, history, meta, path)
    for variant in variants:
        label = variant["label"]
        override = {k: v for k, v in variant.items() if k != "label"}
        loaded = _run_variant(base_spec, label, override, out_dir)
        multi = len(loaded) > 1
        for name, history, meta, path in loaded:
            display = "%s / %s" % (label, name) if multi else label
            collected.append((display, history, meta, path))

    if not collected:
        return []

    orbit = orbit_radius_m
    if orbit is None:
        orbit = collected[0][2].get("orbit_radius_m")

    if separate:
        for name, history, _meta, path in collected:
            plotter.plot_runs([(name, history)], orbit_radius_m=orbit,
                              save_path=path.rsplit(".", 1)[0] + ".png",
                              show=False)

    if combined_save or show:
        runs = [(name, history) for name, history, _meta, _path in collected]
        plotter.plot_runs(runs, orbit_radius_m=orbit,
                          save_path=combined_save, show=show)

    return [(name, history, path) for name, history, _meta, path in collected]


def main(argv=None):
    """Entry point: ``python3 -m py_harness.batch --config B.json --out-dir D``."""
    parser = argparse.ArgumentParser(
        description="Batch/loop runner (TASK-016): run several modes/positions "
        "under one parameter set and collate the saved runs.")
    parser.add_argument("--config", required=True,
                        help="Batch JSON: {\"base\": <spec>, \"variants\": [...]}.")
    parser.add_argument("--out-dir", required=True,
                        help="Directory for the saved run JSON (and PNGs).")
    parser.add_argument("--combined", default=None,
                        help="PNG path for the combined overlay.")
    parser.add_argument("--separate", action="store_true",
                        help="Also write one PNG per run.")
    parser.add_argument("--show", action="store_true", help="Show the plots.")
    parser.add_argument("--orbit-radius-m", type=float, default=None,
                        help="Ring radius for the plots (default from run metadata).")
    args = parser.parse_args(argv)

    with open(args.config) as handle:
        data = json.load(handle)
    if "variants" not in data:
        print("INVALID BATCH: needs a 'variants' list")
        return 2

    try:
        collected = run_batch(
            data.get("base", {}), data["variants"], args.out_dir,
            orbit_radius_m=args.orbit_radius_m or data.get("orbit_radius_m"),
            combined_save=args.combined, separate=args.separate, show=args.show,
        )
    except ValueError as exc:
        print("INVALID BATCH: %s" % exc)
        return 2

    print("batch: %d run(s) collated" % len(collected))
    for name, _history, path in collected:
        print("  %-24s %s" % (name, path))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
