# Sources for `kangaroo-follow.parm`

Every value in the parameter file, where it comes from, and what it is not
(`SR-004`: none is a flight limit; this is a test configuration for SITL).

| Parameter | Value | Source | Note |
|---|---|---|---|
| `SCR_ENABLE` | 1 | ArduPilot scripting | The runner is a Lua script. |
| `SCR_HEAP_SIZE` | 2097152 | Chosen for SITL; raised from 1048576 on 2026-10-01 (`TASK-061`), the demonstration's value | Arm C rolls out `rh_candidates^2` trajectories per replan and arm B scores nine horizons; the default heap is sized for a flight controller. The approach's paths are sampled every 0.5 m, so their memory grows with range, and the demonstration script ran out at 1 MiB about 750 m from the kangaroo (`TASK-058`). Recorded, not tuned. |
| `SCR_VM_I_COUNT` | 1000000 | Chosen for SITL; raised from 200000 on 2026-10-01 (`TASK-061`), the demonstration's value and the one other plane autotests use | Instruction budget per script call; the default (10000) can stop arm C's rollout mid-tick (`ISSUE-C3`). At 200000 arm A was counted at 252k per tick from 300 m and could not fly a tick, and the baseline passes 200k at about 585 m (`TASK-058`). Coarser path sampling would change the law (`VR-014`), so the budget is raised instead. A "script exceeded budget" statustext is still a finding. |
| `AIRSPEED_CRUISE` | 25 | `py_harness/config.py` `AIRSPEED_MS` | The harness's `V`; every Python cell flies at it. The SITL plane model ships 22. |
| `AIRSPEED_MIN` / `AIRSPEED_MAX` | 10 / 30 | SITL plane model (`Tools/autotest/models/plane.parm`) | Unchanged. |
| `ARSPD_USE` | 1 | SITL plane model | TECS holds airspeed, which is what makes groundspeed wind-dependent (`A-ENV-002`). |
| `ROLL_LIMIT_DEG` | 60 | `ADR-011` (2026-10-04, the demonstration's value); `ADR-002` (harness 60 deg; flight code 45 deg) | The bank limit every SITL script and spec.json share: the cell configuration's `bank_limit_deg`, written into spec.json as `roll_limit_deg`. The campaign driver and `KangarooFollowDemo` set `ROLL_LIMIT_DEG` from spec.json per run (an override, `--roll-limit-deg` / `KANGAROO_DEMO_ROLL_LIMIT_DEG`, is written into spec.json too); this file's value makes a hand-run `sim_vehicle.py` demonstration agree with it. At 25 m/s the spec's 45 m turn radius needs 54.8 deg, which 60 allows and 45 does not (`TASK-061`: at 45 neither arm held the ring). Was 45 (the flight code's, `TASK-046` D7) until 2026-10-04. Not a flight value: the aircraft's limit is the approver's, and `hardware_val.lua` only checks it. The SITL plane model ships 65. |
| `WP_LOITER_RAD` | 80 | SITL plane model | GUIDED loiters about the commanded point at this radius; it is the mechanism of Chapter 4 "Why ArduPlane's loiter is not the standoff". Left at the model's value so the SITL result is the harness carrot seen through the shipping L1. |
| `NAVL1_PERIOD` / `NAVL1_DAMPING` | 15 / 0.75 | SITL plane model / `AP_L1_Control` default | Unchanged. |
| `GUIDED_P` | 15000 | `PlaneFollowAppletStandoff` (`arduplane.py`), `plane_follow.md`; added 2026-10-01 (`TASK-061`) | The runner's default command channel (`SHR_CHAN` 1) is a course over ground at the guidance point (`GUIDED_CHANGE_HEADING`), as the demonstration's. The GUIDED heading controller is proportional; at the default 5000 cd/rad a 30 deg course error gives 27 deg of bank, too little for the planned arc. On the location channel (`SHR_CHAN` 0) GUIDED loiters about the point instead (`TASK-058` thesis point 1), so this value is unused there. |
| `TKOFF_ALT` | 60 | The 2026-07-18 flight log (`Sam_follow-03.bin`: 60 to 70 m above home in GUIDED); `environment.json` `flight_area.alt_m` | Take-off target; the driver sets `TKOFF_ALT` and `SHR_ALT_M` from the manifest's `alt_m` (60 m) so the window is flown at the height the real flight used. |
| `SIM_WIND_SPD` / `SIM_WIND_DIR` / `SIM_WIND_TURB` | 0 / 0 / 0 | Calm baseline | `sitl_campaign.py --sub wind` sets `SIM_WIND_SPD` (10 m/s, `TASK-051` D1) and `SIM_WIND_DIR` (0, 90, 180, 270: the direction the wind blows **from**, `libraries/SITL/SITL.cpp`) per cell and records them. Turbulence 0 (`TASK-051` D3). |
| `SIM_SPEEDUP` | 1 | `TASK-052` risk "wall clock" | Real time. A speedup is a `--speedup` argument recorded in provenance; its effect is a finding. |
| `FENCE_ENABLE` | 0 in the file; 1 per cell | `TASK-046` D7 (report only, as recommended); `environment.json` `flight_area` | The test uploads the site box (600 x 800 m about the flight-area centre) as an inclusion polygon with `FENCE_TYPE 4`, `FENCE_ACTION 0` (report only), `FENCE_AUTOENABLE 0`, `FENCE_MARGIN 0` before take-off. A breach is logged, never acted on; the Python counterpart's zone is the same box in the anchor frame, so breaches are measured identically on both sides. |

Script parameters set per cell by the driver (they exist only after the
runner registers them): `SHR_ALT_M` (60 m above home), `SHR_REPORT` (5 s),
`SHR_START` (the window trigger). `KANG_*` and `CTRL_*` are not set: the
shipping `kangaroo_MAV.lua` and `control_cont.lua` are moved aside for a cell
(`stage_scripts.py`), so their tables are not registered.

## `kangaroo-demo.parm` (the live demonstration only, `TASK-058`)

Layered on top of `kangaroo-follow.parm` by `KangarooFollowDemo` and by the
`sim_vehicle.py` command `kangaroo_follow/demo.py --stage` prints. Campaign
cells never load it.

| Parameter | Value | Source | Note |
|---|---|---|---|
| `SCR_VM_I_COUNT` | 1000000 | 2026-09-24 demonstration flight; the value other plane autotests use (`arduplane.py`) | The baseline's approach samples its candidate paths every 0.5 m (`DELTA_D_M`), so one call costs roughly in proportion to the distance to the kangaroo: about 91k instructions from 300 m (`TASK-055` feasibility item 5), and over 200k at about 585 m, where the 2026-09-24 flight's script was stopped. A demonstration runs up to 37.5 m/s and lets the kangaroo lead by well over 1 km. Coarser sampling would change the law (`VR-014`), so the budget is raised instead. |
| `GUIDED_P` | 15000 | `PlaneFollowAppletStandoff` (`arduplane.py`), `plane_follow.md` | The demonstration's default command channel is a course (`GUIDED_CHANGE_HEADING`, type COG) at the guidance point. The GUIDED heading controller is proportional; at the default 5000 cd/rad a 30 deg course error gives 27 deg of bank, too little for the planned arc. Not needed on the location channel (`KDEM_CHAN 0`). |
| `SCR_HEAP_SIZE` | 2097152 | 2026-09-24 hand session (`KDEM: not enough mem, increase SCR_HEAP_SIZE` at about 750 m from the kangaroo) | The approach's candidate paths are sampled every 0.5 m, so their memory grows with range; at the campaign's 1 MiB the demonstration script stopped once the kangaroo led by about 750 m. Doubled for the demonstration only, where a 37.5 m/s kangaroo can lead by well over 1 km. |
