# Captured environment

The machine-readable pin is `src/ardupilot/Tools/autotest/kangaroo_follow/environment.json`; this is its reading.
`python3 -m kangaroo_follow.check_env` compares the live machine against it.

| Item | Pinned | Where it is checked |
|---|---|---|
| ArduPilot | commit `ef19597660f2edf820ab2982e4880275fbce0219` (`ArduPilot-4.6.0-beta1-7773-gef19597660`), checked out at `src/ardupilot` | `git rev-parse HEAD` in the checkout; `--allow-commit` to override, recorded |
| Parent gitlink | `3c9b154a1b8ed6d47a44ceea5b6493af0c0ead24` (differs; uncommitted submodule move) | reported as a warning; the checkout is what flies |
| Build | `./waf configure --board sitl && ./waf plane` -> `build/sitl/bin/arduplane` | binary present and executable (warning under `--dry-run`) |
| SITL model / frame | `plane` | `plan.json` |
| Python | 3.13 | minor version |
| pymavlink | 2.4.49 | import and version |
| lupa | 2.8 (Lua 5.3, the version ArduPilot runs) | the sandbox loads every module |
| numpy / matplotlib | 2.4.1 / 3.11.1 | import and version |
| MAVProxy | 1.8.71, **required to fly** (`vehicle_test_suite.py` imports it); not installed on the capture machine | import; warning only under `--dry-run` |
| Location | `CMAC` (ArduPilot's default site). The local frame is anchored at the aircraft, so the site does not enter a result. A real flight-area location is never committed (`AGENTS.md`). | `Tools/autotest/locations.txt` |
| Parameter file | `src/ardupilot/Tools/autotest/ArduPlane_Tests/KangarooFollow/kangaroo-follow.parm`, sources in `SOURCES.md` | present; SHA-256 in every bundle |
| Lua | runner, registry and the ten `harness_*.lua` modules | present; SHA-256 in every bundle |
| OS last captured on | macOS 26.5.2 (Darwin 25.5.0), arm64, 2026-09-16 | reported beside the live platform |

Dry runs recorded: 2026-09-16, the capture machine (macOS 26.5.2), `CAMP-003`
reference subset, `--sub main` 12 `dry_run_ok` + 9 `unflyable`, `--sub wind`
30 `dry_run_ok`. A second-machine dry run is still to be recorded.
