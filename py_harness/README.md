# Offline geometric validation harness

**Status: skeleton. No geometry is implemented and nothing here is verified.**

Structure only, created under
`tasks/active/TASK-001-python-geometric-validation-harness.md` to satisfy
`ACT-2026-06-25-03` (interface contract) and `ACT-2026-06-25-06` (destination
directory). Porting algorithms in is `ACT-2026-06-25-04` and `-05`, both still
open.

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
| `interface.py` | The algorithm contract: snapshot fields, return value, units, frame, no-solution convention. No geometry. |
| `state.py` | Plane and target state, fixed-step time advance, recorded history, and the read-only plotter. No geometry. |
| `algorithms.py` | Substitutable geometric algorithms and the name registry. All geometry lives here. |
| `run_harness.py` | Entry point. Selecting an algorithm is a change to `--algorithm` and nothing else. |

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
python3 -m py_harness.run_harness --algorithm continuous_weave
```

This currently raises `NotImplementedError`, by design.

Structural contract tests live in the parent repository at
`tests/unit/test_py_harness_contract.py`. They verify the shape of the
contract, not any geometry.
