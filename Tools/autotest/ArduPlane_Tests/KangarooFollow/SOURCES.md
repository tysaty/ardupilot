# Sources for `kangaroo-follow.parm`

Every value in the parameter file, where it comes from, and what it is not
(`SR-004`: none is a flight limit; this is a test configuration for SITL).

| Parameter | Value | Source | Note |
|---|---|---|---|
| `SCR_ENABLE` | 1 | ArduPilot scripting | The runner is a Lua script. |
| `SCR_HEAP_SIZE` | 1048576 | Chosen for SITL | Arm C rolls out `rh_candidates^2` trajectories per replan and arm B scores nine horizons; the default heap is sized for a flight controller. SITL has the memory; the value is recorded, not tuned. |
| `SCR_VM_I_COUNT` | 200000 | Chosen for SITL | Instruction budget per script call; the default (10000) can stop arm C's rollout mid-tick (`ISSUE-C3`). Recorded so a "script exceeded budget" statustext is a finding, not a mystery. |
| `AIRSPEED_CRUISE` | 25 | `py_harness/config.py` `AIRSPEED_MS` | The harness's `V`; every Python cell flies at it. The SITL plane model ships 22. |
| `AIRSPEED_MIN` / `AIRSPEED_MAX` | 10 / 30 | SITL plane model (`Tools/autotest/models/plane.parm`) | Unchanged. |
| `ARSPD_USE` | 1 | SITL plane model | TECS holds airspeed, which is what makes groundspeed wind-dependent (`A-ENV-002`). |
| `ROLL_LIMIT_DEG` | 45 | `ADR-002` (flight code 45 deg; harness 60 deg); `TASK-046` D7 | The flight code's bank limit, `rho` = 63.7 m at 25 m/s. `TASK-046` D7 owns the choice between 45 and 60; 45 is the recorded flight value and is what is written here until D7 says otherwise. The SITL plane model ships 65. |
| `WP_LOITER_RAD` | 80 | SITL plane model | GUIDED loiters about the commanded point at this radius; it is the mechanism of Chapter 4 "Why ArduPlane's loiter is not the standoff". Left at the model's value so the SITL result is the harness carrot seen through the shipping L1. |
| `NAVL1_PERIOD` / `NAVL1_DAMPING` | 15 / 0.75 | SITL plane model / `AP_L1_Control` default | Unchanged. |
| `TKOFF_ALT` | 100 | Chosen for SITL | Take-off target; equals the runner's default `SHR_ALT_M` so the cell starts level. |
| `SIM_WIND_SPD` / `SIM_WIND_DIR` / `SIM_WIND_TURB` | 0 / 0 / 0 | Calm baseline | `sitl_campaign.py --sub wind` sets `SIM_WIND_SPD` (10 m/s, `TASK-051` D1) and `SIM_WIND_DIR` (0, 90, 180, 270: the direction the wind blows **from**, `libraries/SITL/SITL.cpp`) per cell and records them. Turbulence 0 (`TASK-051` D3). |
| `SIM_SPEEDUP` | 1 | `TASK-052` risk "wall clock" | Real time. A speedup is a `--speedup` argument recorded in provenance; its effect is a finding. |
| `FENCE_ENABLE` | 0 | `TASK-046` D7 pending | Report-only fence recommended; not enabled until D7 is answered. Breaches are measured from the log against the Python zone regardless. |

Script parameters set per cell by the driver (they exist only after the
runner registers them): `SHR_ALT_M` (100 m above home), `SHR_REPORT` (5 s),
`SHR_START` (the window trigger). `KANG_*` and `CTRL_*` are not set: the
shipping `kangaroo_MAV.lua` and `control_cont.lua` are moved aside for a cell
(`stage_scripts.py`), so their tables are not registered.
