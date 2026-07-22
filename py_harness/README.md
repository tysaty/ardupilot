# Offline geometric validation harness

**Status: running, and nothing here is verified.** Three geometries work;
`continuous_weave` is still a stub, so the shipping guidance law has no
comparison yet (`ACT-2026-06-25-04`).

Tracked by `tasks/active/TASK-001-python-geometric-validation-harness.md`.

## Configuration and the infeasible turn radius

Parameters live in `config.py` — one immutable module, units and provenance on
every value. Directed 2026-07-22: airspeed **25 m/s**, turn radius **45 m**,
orbit radius **70 m**. The **45-degree** bank limit is held as governing.

At 25 m/s that limit gives a minimum turn radius of **63.73 m**, so the
configured 45 m is **infeasible** — it would need 54.78 degrees and 1.73 g
against 1.41 g at the limit. The harness refuses to run rather than flying a
path the aircraft cannot:

```
$ python3 -m py_harness.run_harness --algorithm dubins
REFUSED: configuration violates the 45.0 deg bank limit: ...
```

Pass `--allow-infeasible` to proceed; every history sample and the plot title
are then stamped infeasible.

## Commanded is not achieved

A guidance point placed a look-ahead ahead on the ring is chased from **inside**
it, so the flown circle settles near `max(R·cos(L/R), turn radius)`, not `R`. At
the configured 70 m ring and 50 m look-ahead the aircraft flies about **64 m**
when the turn radius is feasible, and about **54 m** at the directed 45 m — 77%
of what was commanded. The harness measures and prints this on every orbit run.

This is the same commanded-versus-achieved gap that `A-DEC-009` records for the
weave's amplitude, from a different cause. It is reported, **not** corrected;
reducing the look-ahead to about 10 m recovers roughly 99%.

## Why this exists

`ADR-001` removed candidate generation and cost-based selection on 2026-06-09,
which left the project with a single un-compared guidance law and eight
requirements describing behaviour that no longer exists. Deciding whether to
retire, reinstate or rewrite those requirements needs a cheap way to put
alternative geometries side by side. Python is where that comparison is
affordable; Lua is not.

The four scripts in `../py_plots/` cannot support that comparison: each mixes
geometry, flight constraints, figure construction and a slider GUI loop in one
module, there is no shared notion of aircraft state, and adding an algorithm
means writing a fifth self-contained script with its own plotting code.

## Structure

| File | Responsibility |
|---|---|
| `config.py` | Flight parameters, derived bank angles, the feasibility check. Immutable. |
| `interface.py` | The algorithm contract: snapshot fields, return value, units, frame, no-solution convention. No geometry. |
| `state.py` | Plane and target state, fixed-step time advance, recorded history, and the read-only plotter. No geometry. |
| `algorithms.py` | Thin adapters and the name registry. Converts frames, calls geometry, picks one guidance point. **No maths.** |
| `geometry/dubins.py` | Six Dubins families, ported from `../py_plots/dubins_path.py`. Stateless. |
| `geometry/orbit.py` | Ring tangents, orbit direction and sampling, ported from `dubins_path.py`, `combined.py` and `constrained_curve.py`. Stateless. |
| `run_harness.py` | Entry point. Selecting a geometry is a change to `--algorithm` and nothing else. |

`geometry/` holds no state at all — no globals, no matplotlib, no time. That is
what lets the same functions serve the harness and a later Lua port.

## Geometries

| `--algorithm` | Behaviour |
|---|---|
| `dubins` | Shortest of six families to the target. Terminates on arrival. |
| `orbit` | Circles the target on the 70 m ring. |
| `dubins_orbit` | Approach tangent to the ring, then circle. The `combined.py` behaviour. |
| `continuous_weave` | **Stub.** `ACT-2026-06-25-04`. |

Two modules with separated responsibilities, per `DEC-2026-06-25-01`. The
plotter sits with state by request (`DEC-2026-06-25-03`); the mitigation for
the resulting coupling is that it reads a recorded history and never mutates
state or calls the algorithm.

## Rules for an algorithm

From `DEC-2026-06-25-04`. Python is the validation language, Lua is the target
language, and an algorithm validated using constructs that do not carry to Lua
has not been validated for the thing that will actually fly.

- Plain floats in plain dicts in and out — these map onto Lua tables.
- No `numpy` inside an algorithm.
- No closures over mutable module state. Accumulated state, such as the
  weave's arc-length accumulator, travels through the snapshot's
  `algorithm_state` field.
- Returns a **single guidance point**, matching what
  `control_cont.lua:370` actually commands through
  `vehicle:set_target_location()`. Not a path.

## Limitations

This is a kinematic model. It does not model L1, TECS, EKF or airframe
response. It can validate **geometry**; it cannot validate **tracking**, and it
must not be cited as evidence for `SR-001` to `SR-003` (`A-VAL-001`).

Whether the harness is *normative* — the Lua must reproduce its output within a
stated tolerance — or *exploratory only* is unresolved. It is carried as
`DEC-2026-06-25-05` and blocks knowing what evidence this harness is expected
to produce.

## A note on version control

This directory sits inside the vendored ArduPilot checkout, which carries its
own nested `.git`. Files here **do not appear in the ResearchProject
repository's history**; the parent repo sees `src/ardupilot` as a single
gitlink. Commit changes here in the nested repository. See
`docs/ARCHITECTURE.md`, "Intended repository mapping".

## Running

```bash
cd src/ardupilot/scripts
python3 -m py_harness.run_harness --algorithm dubins_orbit --allow-infeasible
python3 -m py_harness.run_harness --algorithm orbit --turn-radius-m 63.8
```

Plotting needs `matplotlib`, which is **not installed** in the current
environment — `plot_history` has therefore never been executed. Use `--no-plot`
until it is available.

## Tests

In the parent repository, because `tests/` belongs there:

```bash
cd /Users/samueltyrie/ResearchProject && python3 -m pytest tests/unit -q
```

| File | What it covers |
|---|---|
| `test_py_harness_geometry.py` | The port reproduces `py_plots/dubins_path.py` to 1e-12 across all six families. Guards `A-VAL-002`/`A-VAL-004`. |
| `test_py_harness_config.py` | The feasibility arithmetic, refusal behaviour, immutability. |
| `test_py_harness_contract.py` | Interface shape, substitution, turn-rate limit, no-solution handling. |

These show the port is **faithful**, not that the geometry is **correct** — the
originals carry no tests either. No Lua file is under test.
