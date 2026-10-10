# Kangaroo-follow SITL campaign runbook (`TASK-052`)

The `KangarooFollowCell` and `KangarooFollowCampaign` tests of the plane
test suite (`Tools/autotest/arduplane.py`, beside the mid-term `DubinsSweep`),
the package `Tools/autotest/kangaroo_follow/` behind them and the assets under
`ArduPlane_Tests/KangarooFollow/`. Repeats the Python harness experiments (`CAMP-002`, `CAMP-003`, their
composite subs) in ArduPlane SITL through ArduPilot's AutoTest framework,
against the ported Lua guidance laws, and writes **the same bundle** the
Python campaigns write, so every figure and table can be produced for the
SITL column without new analysis code. This file takes a reader from a
fresh clone to a completed cell; nothing else needs to be read first.

Nothing under `kangaroo_follow/` or `ArduPlane_Tests/KangarooFollow/` is a flight configuration (`SR-004`); the SITL result
is tracked evidence for `A-VAL-001`, not flight evidence.

## 0. What flies

| Piece | File | Role |
|---|---|---|
| Test | `Tools/autotest/arduplane.py` `KangarooFollowCell`, `KangarooFollowCampaign` | The AutoTest sequence per cell: parameters, reboot, take-off, `GUIDED` lead-in onto the start heading, `SHR_START`, wait on `SHR_DONE`, copy the log. |
| Runner | `ArduPlane_Tests/KangarooFollow/sitl_harness_runner.lua` | Vehicle-side script: replays the cell's kangaroo schedule, runs the arm, commands `set_target_location()`, logs the harness record (`HREC`/`HEST`/`HALG`/`HAIR`). Replaces `control_cont.lua` and `kangaroo_MAV.lua` for the cell. |
| Arm registry | `ArduPlane_Tests/KangarooFollow/sitl_arms.lua` | Maps a spec's algorithm name to a ported `harness_*.lua` entry; carries the arm 0 / 0H snapshot adapter (gated against Python). Unported arms are refused with a reason. |
| Cell table | `scripts/modules/sitl_cell.lua` (generated) | The Python `spec.json` and its `legs_flown`, as a Lua table. Never hand-edited. |
| Parameters | `ArduPlane_Tests/KangarooFollow/kangaroo-follow.parm` + `SOURCES.md` | SITL test configuration; every value sourced. |
| Ported modules | `scripts/modules/harness_*.lua` | `TASK-006`; staged unchanged, hashed into every bundle. |

Arms that can fly today: `dubins_target_orbit`, `dubins_target_orbit_hyst`
(0, 0H), `adaptive_db_circle` (A), `adaptive_horizon_cs` (B),
`rh_geometric` (C). Arm D and the A/B/D hysteresis variants have no Lua
entry (`TASK-046` D3, `TASK-006` Tranche 9 note) and are planned as
`unflyable` findings.

## 1. Fresh clone to first cell

```bash
git clone <repo> ResearchProject && cd ResearchProject
git submodule update --init src/ardupilot            # pinned commit in kangaroo_follow/environment.json
python3 -m pip install -r src/ardupilot/Tools/autotest/kangaroo_follow/requirements.txt
cd src/ardupilot && ./waf configure --board sitl && ./waf plane
cd Tools/autotest
python3 -m kangaroo_follow.check_env                 # every FAIL names what to fix
```

Every `kangaroo_follow` command below is run from `src/ardupilot/Tools/autotest`,
the directory `autotest.py` runs from. `--from` takes the Python campaign
directory in the parent repository.

`check_env.py` compares the live machine with `src/ardupilot/Tools/autotest/kangaroo_follow/environment.json`
(ArduPilot commit, Python minor, package versions, SITL binary, location,
the Lua files) and exits non-zero on any `FAIL`. `--dry-run` relaxes the
build-dependent checks; `--allow-commit` accepts another ArduPilot commit and
is recorded in provenance.

## 2. Plan, run, aggregate, compare

All commands are `python3 -m kangaroo_follow.campaign` from `Tools/autotest`; `<repo>` is the parent repository.

```bash
# plan the reference subset (0H-point, 0H-straight-constant-half, one composite cell per arm), 3 repeats
python3 -m kangaroo_follow.campaign --plan --from <repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst --reference --repeats 3

# prove the machinery without SITL: environment, staging, schedule replay against the Python history, plan.json per cell
python3 -m kangaroo_follow.campaign --dry-run --from <repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst

# fly every planned cell: one `autotest.py test.Plane.KangarooFollowCell` process per cell, killed at the cell timeout; --resume skips complete cells
python3 -m kangaroo_follow.campaign --all --from <repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst [--resume] [--only 0H-point-r1]

# or one cell by hand, exactly as the campaign does it
KANGAROO_FOLLOW_PLAN=<repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst/sitl/0H-point-r1/plan.json ./autotest.py --no-clean --no-configure test.Plane.KangarooFollowCell

# or every planned cell of a manifest in one SITL process (reboot per cell), from autotest.py alone
KANGAROO_FOLLOW_CAMPAIGN=<repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst/sitl ./autotest.py --no-clean --no-configure test.Plane.KangarooFollowCampaign

# master.csv (Python columns + sitl_*), then the Python-versus-SITL table
python3 -m kangaroo_follow.campaign --aggregate --from <repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst
python3 -m kangaroo_follow.campaign --compare   --from <repo>/experiments/campaigns/CAMP-003-arm-mode-ratio-hyst
```

Output lands beside the Python campaign:

```
<campaign>/sitl/MANIFEST.json
<campaign>/sitl/<python-cell>-r<k>/   plan.json result.json spec.json log.bin driver.log
                                      record.json history.json series.json ticks.csv sitl_extras.json
<campaign>/sitl/master.csv  compare.csv
```

`--only` takes Python cell ids when planning and SITL cell ids when running.
`--plan` on an existing manifest adds cells and keeps complete ones.

## 3. The wind sub-campaign (`--sub wind`, `TASK-051`)

Every planned cell is flown under calm plus a steady wind from N, E, S and W
(`SIM_WIND_SPD` 10 m/s, `SIM_WIND_DIR` 0/90/180/270, turbulence 0):

```bash
python3 -m kangaroo_follow.campaign --plan --sub wind --from <campaign> --only 0H-point 0H-straight-constant-half --repeats 3
python3 -m kangaroo_follow.campaign --dry-run --sub wind --from <campaign>
python3 -m kangaroo_follow.campaign --all --sub wind --from <campaign>
python3 -m kangaroo_follow.campaign --aggregate --sub wind --from <campaign>     # + wind-frame.csv, wind-compare.csv
```

Cells are `<cell>-wind-<N|E|S|W|calm>-r<k>` under `<campaign>/sitl-wind/`
with `MANIFEST-wind.json`. `kangaroo_follow/wind_frame.py` closes the wind frame on
every bundle (`|v_ground - w| - airspeed`, RMS within 1.0 m/s) under both
readings of `SIM_WIND_DIR` and reports which closes, so the from/to
convention is settled by measurement; it also gives the crab angle,
groundspeed and ring radius by bearing relative to the wind, and the velocity
error along and across the wind. `wind-compare.csv` lays each direction
against the calm SITL cell and the Python cell.

## 3b. The site and the fence (`environment.json` `flight_area`; `ADR-012`, 2026-10-07)

Cells fly at **Spring Valley Farm** (`SpringValley2` in `locations.txt`). The
kangaroo is contained in the **flight-test fence**
`sitl-runs/springvalley2-test-fence.txt` (`flight_area.fence_file`), a convex
QGC WPL 110 inclusion polygon about 281 to 329 m East-West by 570 m
North-South. The fence is the one boundary for all three paths:

- **This campaign.** Each selected Python cell is re-run (`campaign.box_counterpart`,
  into `<campaign>/sitl/python/`) with the fence as its zone, converted to
  North/East metres from home with ArduPilot's `get_distance_NE` formula and
  placed in the aircraft's anchor frame:
  - grid cells: the aircraft starts 250 m South of the fence's area centroid,
    with the kangaroo 300 m North of it;
  - composite cells: anchored at the centroid and fitted to the largest square
    there (284 m).

  The vehicle replays its `legs_flown`, containment turns included, and the
  test uploads the file's own vertices as a **report-only** fence
  (`FENCE_TYPE 4`, `FENCE_ACTION 0`). A cell whose kangaroo would leave the
  fence is refused ("the kangaroo leaves the fence"), as a composite that does
  not fit is. The window is flown at **60 m** above home for **60 s**.
- **The demo** (`demo.py --stage`; `--fence <file>` or `--no-fence`) and
  **`hardware_val.lua`** read the same vertices from `spec.json` `fence` and
  contain the kangaroo with `harness_zone.lua`, the gated port of the Python
  rule.

`check_env` reports the fence. `--no-box` plans the original 2 km cells
instead; `--window-s`, `--alt-m` and `--location` override the pins.
`plan.json` names the site and the fence file, and carries no coordinates.
Without `fence_file`, the 600 x 800 m box of 2026-09-17 (`box_e_m`,
`box_n_m`, `centre_offset_ne_m`) is still accepted.

The default kangaroo geometry (circle radius 60 m, rectangle 140 x 70 m,
`kangaroo.DEFAULT_GEOMETRY`) fits the fence less the ring from the grid start.
The cells planned before 2026-10-07 carry 150 m and 300 x 150 m, and their
circles leave the fence. Values actually run: `docs/CAMPAIGN_RUN_VALUES.md`.

## 4. What a cell does (`KangarooFollowCell`)

1. Applies `kangaroo-follow.parm` and the cell's `SIM_WIND_*`, reboots so
   scripting starts, waits for `SHR: loaded cell`.
2. Takes off (`TAKEOFF`, 60 m), enters `GUIDED`, and with a site box flies
   a 1 km run-in through the anchor point on the spec's initial heading
   (140 deg); without one, toward a point 3 km along that heading until on
   heading at cruise. The `TASK-046` D6 start pose (tolerances 5 deg,
   3 m/s, 60 m of the anchor, recorded in `result.json`).
3. Sets `SHR_START = 1`. The runner anchors the local frame at the aircraft
   (`HANC`), places the kangaroo 300 m North of it (the spec's start), and
   evaluates the schedule from that instant.
4. Each 100 ms: kangaroo from `harness_segments.state_at`, estimator
   update then projection (when the arm needs it), the arm's
   `guidance_point(snapshot, cfg)`, `set_target_location()` at 100 m above
   home (60 m), and the record rows.
5. `SHR_DONE = 1` at the end of the window (2 on a refusal, which becomes
   `partial = true`); disarm; the `.bin` is copied to the cell directory and
   `extract_bundle.py` writes the bundle through the unchanged Python
   pipeline.

The scripts directory is restored after every cell (`stage_scripts.py`),
including on failure. If a run was killed hard, `python3 -m kangaroo_follow.stage_scripts --restore`.

Restore deletes only what staging created. A support module (`sitl_arms.lua`,
`sitl_spec.lua`, the demo's `sitl_adsb.lua`) already in `scripts/modules/` and
identical to its source here is left in place (`kept` in `.sitl-staged.json`);
one that differs is set aside as `<name>.sitl-off` and put back. So
`scripts/modules/sitl_arms.lua`, which `hardware_val.lua` requires, survives a
campaign or demo run. A state file written before 2026-10-07 (no `kept` key)
is restored the same way: a module it lists that is identical to its source is
not deleted. Top-level scripts in `scripts/` (including `hardware_val.lua`) are
still renamed to `.sitl-off` for the run: save them first.

## 4b. The live demonstration (`TASK-058`)

Not a cell and not evidence: the baseline flown against a kangaroo whose mode
and speed are changed while you watch. `kangaroo_demo.lua` takes its algorithm,
configuration and geometry from a Python cell's spec (default
`0H-straight-constant-half`), shows the kangaroo on the map as the ADS-B
contact `KANGAROO` (`sitl_adsb.lua`), and rebuilds the kangaroo's schedule
from where it is whenever a `KDEM_*` parameter changes.

```bash
# by hand, changing the mode from MAVProxy
cd src/ardupilot/Tools/autotest               # kangaroo_follow is a package here: -m fails elsewhere
python3 -m kangaroo_follow.demo --restore     # only if scripts/.sitl-staged.json exists (stage refuses otherwise)
python3 -m kangaroo_follow.demo --stage       # prints the sim_vehicle.py command and the KDEM_ cheat sheet
                                              #   stages the test fence into spec.json (ADR-012); --no-fence for the KDEM_BOUND_M circle
                                              #   add --look-ahead-m 5 for the short-carrot baseline
cd ../..                                      # sim_vehicle.py runs from src/ardupilot
Tools/autotest/sim_vehicle.py -v ArduPlane -L SpringValley2 --console --map -N \
    --add-param-file=Tools/autotest/ArduPlane_Tests/KangarooFollow/kangaroo-follow.parm \
    --add-param-file=Tools/autotest/ArduPlane_Tests/KangarooFollow/kangaroo-demo.parm
#   mode TAKEOFF ; arm throttle ; mode GUIDED ; param set KDEM_MODE 2 ; param set KDEM_SPD 18.75 ...
cd Tools/autotest
python3 -m kangaroo_follow.demo --restore     # put control_cont.lua and kangaroo_MAV.lua back

# automatically: every mode through 6.25 to 37.5 m/s, 30 s each (from src/ardupilot)
./Tools/autotest/autotest.py --map test.Plane.KangarooFollowDemo
# options: KANGAROO_DEMO_LOOK_AHEAD_M=25 (carrot, m), KANGAROO_DEMO_ROLL_LIMIT_DEG=60
# (the harness's bank; default the parameter file's 45), KANGAROO_DEMO_SPEEDUP=10

# the physical-validation run plan (pv_plan.json, docs/Physical_validation.md), one arm at a time
python3 -m kangaroo_follow.pv_plan                        # run table and fence check, every arm
python3 -m kangaroo_follow.demo --stage --plan --arm 0H   # suite mode (KDEM_MODE 5); then FH, AH
```

With `--plan`, the kangaroo is placed from the site reference (the mean of the
plan fence's vertices), not ahead of the aircraft, and flies the arm's fixed
schedule: the point run, the transit, then the nine shared-start runs. The
console announces each run and ends with `KDEM: suite complete; least spare
... m`. The plan is not turned at the fence; a pass of the orbit-radius limit is
warned instead.

The autotest is also the demonstration's certification: every mode at five
speeds plus elastic pace, judged against five criteria fixed in `arduplane.py`
(on the map; straight speed; stationary kangaroo on the ring; the ring reached
at or below 12.5 m/s; no script fault). It clears `src/ardupilot/logs/` when it
starts, so copy a flight log out before the next run.

`KDEM_LOOK` sets the carrot look-ahead in flight (30 m in the physical-validation plan, set by `--plan`; 50 m, the thesis default;
5 m, the short-carrot baseline). `KDEM_MODE` 0 point, 1 straight, 2 circle,
3 rectangle, 4 rand; `KDEM_PACE`
0 constant, 1 elastic, 2 stopstart (`TASK-064`: slows to `KDEM_PSLOW` x
`KDEM_SPD`, holds `KDEM_PHOLD` s, speeds up, holds `KDEM_PFAST` s, ramps
`KDEM_PRAMP` s); `KDEM_SPD` m/s; `KDEM_HDG` degrees (-1 = the aircraft's
heading at start); `KDEM_RESET` 1 re-places the kangaroo ahead; `KDEM_BOUND_M`
turns a straight kangaroo back towards the start.

**Command channel.** `KDEM_CHAN` 1 (default) steers the aircraft at the
guidance point with a course command (`GUIDED_CHANGE_HEADING`, COG), the
follow applet's channel. `KDEM_CHAN` 0 sends the point with
`set_target_location()` as the campaign runner does; GUIDED then loiters
about it at `WP_LOITER_RAD` (80 m), and because the point is about 50 m
ahead the aircraft circles hard and never closes (2026-09-24 flight: range
200 to 390 m from a stationary kangaroo for 30 s at 45 deg bank). Channel 1
needs `kangaroo-demo.parm` (`GUIDED_P` 15000, `SCR_VM_I_COUNT` 1000000;
`SOURCES.md`).

## 5. Adding a cell

Plan it from its Python cell: `--plan --only <python-cell-id>`. There is no
other way to add a cell, by design: the spec is the input.

## 6. Tests

From the parent repository, `python3 -m pytest tests/unit/test_sitl_campaign.py -q` runs without an
ArduPilot build, MAVProxy or a recorded log: the schedule replay against
`harness_segments.lua`, the arm 0/0H adapter against `algorithms.py` tick
for tick, the extractor on a synthesised DataFlash log, the wind frame with
a known wind, the runner under stubbed ArduPilot bindings, staging, the
environment check, the planner, the dry run, aggregation and comparison.

## 7. Known limits (findings, not fixes)

- Arm D, AH, BH, DH cannot fly (no Lua entry); they appear as `unflyable`.
- The cell's `plane_hdg_rad` is the ground **course** by default
  (`heading_source = course`); `yaw` is logged beside it and selectable.
  Under wind the two differ by the crab angle.
- SITL has no zone actor: the kangaroo is replayed from the Python
  `legs_flown` (which already include the Python zone's containment turns),
  and only the aircraft's breaches are measured. `containment_turns` is 0 in
  every SITL bundle.
- The harness metrics assume a uniform `dt_s`; the SITL tick spacing is
  recorded (`record.sitl.tick_spacing`) so the approximation is visible.
- `FENCE_ENABLE` is 0 until `TASK-046` D7 is answered.
- `KangarooFollowCell` has not been run against a live SITL on the capture
  machine (MAVProxy not installed); see the completion record in
  `tasks/active/TASK-052`. Both tests are listed in `disabled_tests` so an
  ordinary `test.Plane` run skips them; `autotest.py test.Plane.KangarooFollowCell`
  runs them regardless.
