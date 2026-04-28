#!/usr/bin/env python3

import csv
import math
import signal
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil


SIM_CMD = [
    "./Tools/autotest/sim_vehicle.py",
    "-v", "ArduPlane",
    "--console",
    "--map",
    "--out", "udp:127.0.0.1:14550",
]

MAVLINK = "udp:127.0.0.1:14550"

# Parameter sweep matrix
# PARAM_RANGES = {
#     "NAVL1_PERIOD": [15, 20, 25],
#     "NAVL1_DAMPING": [0.6, 0.75, 0.9],
#     "TECS_SPDWEIGHT": [1.0, 1.5, 2.0],
# }

PARAM_SETS = [
    {
        "name":             "baseline",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "fast_rebuild",
        "CTRL_REBUILD_MS":  2000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "slow_rebuild",
        "CTRL_REBUILD_MS":  10000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "wide_accept",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      60,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "tight_accept",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      25,
        "CTRL_MIN_WP":      15,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "streak_2",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      2,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "dub_near",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    400,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "dub_far",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    1200,
        "CTRL_DUB_VEL":     30,
    },
    {
        "name":             "dub_slow_vel",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     15,
    },
    {
        "name":             "dub_fast_vel",
        "CTRL_REBUILD_MS":  6000,
        "CTRL_WP_RAD":      40,
        "CTRL_MIN_WP":      20,
        "CTRL_STREAK":      1,
        "CTRL_DUB_DIST":    750,
        "CTRL_DUB_VEL":     45,
    },
]

OUTPUT_CSV = "param_sweep_results.csv"


def start_sitl():
    proc = subprocess.Popen(
        SIM_CMD,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=None,
    )

    time.sleep(12)  # give SITL/MAVProxy time to start
    return proc


def stop_sitl(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        time.sleep(3)

    if proc.poll() is None:
        proc.kill()


def connect_mavlink():
    mav = mavutil.mavlink_connection(MAVLINK)
    mav.wait_heartbeat(timeout=30)
    print(f"Heartbeat from system {mav.target_system}, component {mav.target_component}")
    return mav


def set_param(mav, name, value):
    print(f"Setting {name} = {value}")

    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    start = time.time()
    while time.time() - start < 5:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.strip("\x00") == name:
            print(f"Confirmed {name} = {msg.param_value}")
            return True

    print(f"WARNING: parameter {name} not confirmed")
    return False


def arm_and_takeoff(mav, alt=80):
    mav.set_mode_auto()

    mav.arducopter_arm()
    mav.motors_armed_wait()

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        alt,
    )

    time.sleep(20)


def get_position(mav):
    msg = mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if not msg:
        return None

    return {
        "lat": msg.lat / 1e7,
        "lon": msg.lon / 1e7,
        "alt": msg.relative_alt / 1000.0,
    }


def haversine_m(pos1, pos2):
    R = 6371000.0

    lat1 = math.radians(pos1["lat"])
    lat2 = math.radians(pos2["lat"])
    dlat = math.radians(pos2["lat"] - pos1["lat"])
    dlon = math.radians(pos2["lon"] - pos1["lon"])

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )

    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def run_trial(mav, params):
    for name, value in params.items():
        if name == "name":
            continue
        set_param(mav, name, value)

    # Optional: reboot after parameter setting if needed
    # mav.reboot_autopilot()
    # time.sleep(15)

    arm_and_takeoff(mav, alt=80)

    # Example target/error definition.
    # Replace this with your kangaroo/Dubins/final-target error.
    target = {
        "lat": -35.363261,
        "lon": 149.165230,
        "alt": 80,
    }

    run_time_s = 90
    print(f"Running trial for {run_time_s} seconds...")
    time.sleep(run_time_s)

    final_pos = get_position(mav)

    if final_pos is None:
        return None

    horizontal_error = haversine_m(final_pos, target)
    altitude_error = abs(final_pos["alt"] - target["alt"])

    final_error = math.sqrt(horizontal_error**2 + altitude_error**2)

    return {
        "final_error_m": final_error,
        "horizontal_error_m": horizontal_error,
        "altitude_error_m": altitude_error,
        "final_lat": final_pos["lat"],
        "final_lon": final_pos["lon"],
        "final_alt": final_pos["alt"],
    }


# def generate_param_matrix(param_ranges):
#     names = list(param_ranges.keys())
#     values = list(param_ranges.values())
#     for combo in itertools.product(*values):
#         yield dict(zip(names, combo))


def main():
    results = []

    print(f"Running {len(PARAM_SETS)} parameter sets")

    for i, params in enumerate(PARAM_SETS, start=1):
        set_name = params.get("name", str(i))
        print("=" * 60)
        print(f"Trial {i}/{len(PARAM_SETS)}: {set_name}")
        print(params)

        proc = start_sitl()

        try:
            mav = connect_mavlink()
            trial_result = run_trial(mav, params)

            row = {
                "trial": i,
                "name": set_name,
                **{k: v for k, v in params.items() if k != "name"},
            }

            if trial_result:
                row.update(trial_result)
            else:
                row["final_error_m"] = None

            results.append(row)

        except Exception as e:
            print(f"Trial failed: {e}")
            row = {
                "trial": i,
                "name": set_name,
                **{k: v for k, v in params.items() if k != "name"},
                "final_error_m": None,
                "error": str(e),
            }
            results.append(row)

        finally:
            stop_sitl(proc)

        write_results(results)

    print("Sweep complete.")


def write_results(results):
    if not results:
        return

    fieldnames = sorted({key for row in results for key in row.keys()})

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    print(f"Saved results to {OUTPUT_CSV}")


if __name__ == "__main__":
    main() 