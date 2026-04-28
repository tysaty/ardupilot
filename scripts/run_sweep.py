#!/usr/bin/env python3
"""
Parameter sweep for ArduPlane SITL.

Launches a fresh sim_vehicle.py instance per trial, arms, cycles TAKEOFF →
GUIDED, collects STATUSTEXT for 60 s, parses the Dubins tracking error from
control.lua, then kills SITL and moves to the next parameter combination.

Usage:
    python3 scripts/run_sweep.py
    python3 scripts/run_sweep.py --sets baseline fast_rebuild
    python3 scripts/run_sweep.py --duration 90 --output results.csv

Requirements:
    pip install pymavlink
"""

import argparse
import csv
import datetime
import itertools
import os
import re
import signal
import subprocess
import sys
import threading
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not installed: pip install pymavlink")

# ── SITL command ──────────────────────────────────────────────────────────────

SITL_CMD = [
    "./Tools/autotest/sim_vehicle.py",
    "-v", "ArduPlane",
    "-f", "plane",
    "--console",
    "--map",
]

MAVLINK_ADDR = "udpin:0.0.0.0:14550"

TAKEOFF_MODE = 13
GUIDED_MODE  = 15

# ── Parameter sweep ───────────────────────────────────────────────────────────

# One-at-a-time sensitivity sets (10 named trials, used by default / --sets).
# Commented out in favour of the grid below; restore if needed.
# PARAM_SETS = [
#     {
#         "name":             "baseline",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "fast_rebuild",
#         "CTRL_REBUILD_MS":  2000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "slow_rebuild",
#         "CTRL_REBUILD_MS":  10000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "wide_accept",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      60,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "tight_accept",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      25,
#         "CTRL_MIN_WP":      15,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "streak_2",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      2,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "dub_near",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    400,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "dub_far",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    1200,
#         "CTRL_DUB_VEL":     30,
#     },
#     {
#         "name":             "dub_slow_vel",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     15,
#     },
#     {
#         "name":             "dub_fast_vel",
#         "CTRL_REBUILD_MS":  6000,
#         "CTRL_WP_RAD":      40,
#         "CTRL_MIN_WP":      20,
#         "CTRL_STREAK":      1,
#         "CTRL_DUB_DIST":    750,
#         "CTRL_DUB_VEL":     45,
#     },
# ]

# ── Reduced grid (36 combinations, ~72 min) ───────────────────────────────────
# Dropped from grid based on sensitivity sweep results:
#   CTRL_WP_RAD / CTRL_MIN_WP  — <6% effect vs baseline → fixed at baseline
#   CTRL_DUB_VEL=45            — +0.3% vs baseline → indistinguishable from noise

GRID_PARAMS = {
    "CTRL_REBUILD_MS": [2000, 6000, 10000],
    "CTRL_STREAK":     [1, 2],
    "CTRL_DUB_DIST":   [400, 750, 1200],
    "CTRL_DUB_VEL":    [15, 30],
}

GRID_FIXED = {
    "CTRL_WP_RAD": 40,
    "CTRL_MIN_WP": 20,
}


def build_grid():
    keys = list(GRID_PARAMS.keys())
    for combo in itertools.product(*GRID_PARAMS.values()):
        vals = dict(zip(keys, combo))
        name = "_".join(
            f"{k.replace('CTRL_', '').lower()}{v}" for k, v in vals.items()
        )
        yield {"name": name, **GRID_FIXED, **vals}


# ── SITL lifecycle ────────────────────────────────────────────────────────────

def start_sitl():
    # start_new_session puts sim_vehicle.py and all its children (MAVProxy,
    # SITL binary) in their own process group so we can kill them all at once.
    proc = subprocess.Popen(
        SITL_CMD,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return proc


def stop_sitl(proc):
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    # Graceful shutdown of the entire process group.
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        pass

    time.sleep(4)

    # Force-kill anything still alive.
    try:
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass

    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass

    # Let the OS reclaim ports before the next trial binds to 14550.
    time.sleep(2)


# ── MAVLink connection ────────────────────────────────────────────────────────

def connect(boot_wait=15, timeout=30):
    """Wait for SITL to boot, then connect and return (conn, stop_event)."""
    print(f"  Waiting {boot_wait}s for SITL to boot...", flush=True)
    time.sleep(boot_wait)

    conn = mavutil.mavlink_connection(MAVLINK_ADDR, source_system=255)

    stop = threading.Event()

    def _heartbeat():
        while not stop.is_set():
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
            except Exception:
                pass
            time.sleep(1.0)

    threading.Thread(target=_heartbeat, daemon=True).start()

    print(f"  Waiting for vehicle heartbeat (up to {timeout}s)...", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        sysid = msg.get_srcSystem()
        if sysid in (0, 255):
            continue
        conn.target_system    = sysid
        conn.target_component = msg.get_srcComponent()
        print(f"  Vehicle heartbeat: sysid={sysid}", flush=True)
        return conn, stop

    stop.set()
    raise RuntimeError(f"No vehicle heartbeat on {MAVLINK_ADDR} after {timeout}s")


# ── Parameters ────────────────────────────────────────────────────────────────

def set_param(conn, name, value, retries=5, ack_timeout=3.0):
    for _ in range(retries):
        conn.mav.param_set_send(
            conn.target_system, conn.target_component,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + ack_timeout
        while time.monotonic() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg and msg.param_id.rstrip("\x00") == name:
                return True
    print(f"  WARNING: no ack for {name}={value}", flush=True)
    return False


def set_params(conn, param_set):
    for k, v in param_set.items():
        if k == "name":
            continue
        ok = set_param(conn, k, v)
        print(f"  {k} = {v}  [{'ok' if ok else 'no-ack'}]", flush=True)


# ── Arm and mode ──────────────────────────────────────────────────────────────

def arm_vehicle(conn, timeout=60):
    """Retry arming until COMMAND_ACK ACCEPTED (pre-arm checks gate this)."""
    print(f"  Waiting to arm (up to {timeout}s)...", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0,
        )
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=2.0)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("  Armed.", flush=True)
                return True
    raise RuntimeError("Vehicle did not arm within timeout")


def set_mode(conn, mode_id):
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


# ── STATUSTEXT collection ─────────────────────────────────────────────────────

def collect_statustext(conn, duration):
    """Receive all STATUSTEXT messages for `duration` seconds."""
    messages = []
    deadline = time.monotonic() + duration
    t0 = time.monotonic()
    print(f"  Collecting STATUSTEXT for {duration}s ", end="", flush=True)
    last_dot = time.monotonic()

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = conn.recv_match(blocking=True, timeout=min(0.2, remaining))
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            messages.append({
                "t": round(time.monotonic() - t0, 2),
                "severity": msg.severity,
                "text": msg.text.rstrip("\x00"),
            })
        if time.monotonic() - last_dot >= 10:
            elapsed = duration - (deadline - time.monotonic())
            print(f"[{int(elapsed)}s]", end="", flush=True)
            last_dot = time.monotonic()

    print(" done.", flush=True)
    return messages


# ── Dubins error parsing ──────────────────────────────────────────────────────

_DUBINS_RE = re.compile(
    r"Dubins error L1 Norm:([\d.]+)m L2 Norm:([\d.]+)m"
)


def parse_dubins_error(messages):
    for m in reversed(messages):
        match = _DUBINS_RE.search(m["text"])
        if match:
            return float(match.group(1)), float(match.group(2))
    return None, None


# ── Trial ─────────────────────────────────────────────────────────────────────

def run_trial(param_set, duration):
    name = param_set.get("name", "?")
    print(f"\n  Starting SITL for trial: {name}", flush=True)

    proc = start_sitl()
    conn = stop_event = None

    try:
        conn, stop_event = connect()

        print("  Setting parameters...", flush=True)
        set_params(conn, param_set)

        arm_vehicle(conn)

        time.sleep(1)
        print("  Mode → TAKEOFF", flush=True)
        set_mode(conn, TAKEOFF_MODE)

        time.sleep(1)
        print("  Mode → GUIDED", flush=True)
        set_mode(conn, GUIDED_MODE)

        messages = collect_statustext(conn, duration)
        l1, l2 = parse_dubins_error(messages)

        log = " | ".join(
            f"[{m['t']}s sev={m['severity']}] {m['text']}" for m in messages
        )

        return {
            "name":       name,
            **{k: v for k, v in param_set.items() if k != "name"},
            "l1_error_m": l1,
            "l2_error_m": l2,
            "statustext_log": log,
            "error": None,
        }

    except Exception as exc:
        print(f"  Trial failed: {exc}", flush=True)
        return {
            "name":       name,
            **{k: v for k, v in param_set.items() if k != "name"},
            "l1_error_m": None,
            "l2_error_m": None,
            "statustext_log": "",
            "error": str(exc),
        }

    finally:
        if stop_event is not None:
            stop_event.set()
        # Close the UDP socket so port 14550 is free before the next trial.
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
        time.sleep(0.5)  # let heartbeat thread see the stop event
        print("  Stopping SITL...", flush=True)
        stop_sitl(proc)


# ── Output ────────────────────────────────────────────────────────────────────

_PARAM_KEYS = [
    "CTRL_REBUILD_MS", "CTRL_WP_RAD", "CTRL_MIN_WP",
    "CTRL_STREAK", "CTRL_DUB_DIST", "CTRL_DUB_VEL",
]

FIELDNAMES = ["trial", "name"] + _PARAM_KEYS + [
    "l1_error_m", "l2_error_m", "statustext_log", "error"
]


def write_csv(results, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDNAMES, extrasaction="ignore")
        w.writeheader()
        for i, r in enumerate(results, 1):
            w.writerow({"trial": i, **r})
    print(f"  Saved → {path}", flush=True)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Sweep CTRL_* params over fresh SITL instances, log Dubins tracking error"
    )
    ap.add_argument(
        "--grid", action="store_true",
        help="Run the reduced 36-combination grid (3×2×3×2) instead of named sets",
    )
    ap.add_argument(
        "--sets", nargs="*",
        help="Run only these named sets from PARAM_SETS (default: all named sets)",
    )
    ap.add_argument(
        "--duration", type=int, default=45,
        help="GUIDED collection window per trial in seconds (default: 45)",
    )
    ap.add_argument(
        "--output",
        default=f"sweep_results_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
    )
    args = ap.parse_args()

    if args.grid:
        sets_to_run = list(build_grid())
    elif args.sets:
        # PARAM_SETS is commented out; restore it above if you need --sets support
        sys.exit("--sets requires PARAM_SETS to be uncommented in the script")
    else:
        # PARAM_SETS is commented out; default falls through to grid
        sets_to_run = list(build_grid())

    results = []
    total = len(sets_to_run)

    for i, ps in enumerate(sets_to_run, 1):
        print(f"\n{'='*60}\nTrial {i}/{total}: {ps['name']}", flush=True)
        result = run_trial(ps, args.duration)
        results.append(result)
        print(
            f"  l1={result['l1_error_m']}m  l2={result['l2_error_m']}m",
            flush=True,
        )
        write_csv(results, args.output)

    print(f"\n{'='*60}\nSweep complete. {total} trial(s). Results in {args.output}")


if __name__ == "__main__":
    main()
