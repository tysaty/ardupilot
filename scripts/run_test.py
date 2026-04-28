#!/usr/bin/env python3
"""
Control/Dubins parameter sweep.

SITL must already be running (sim_vehicle.py).  This script connects to
MAVProxy's UDP output (default port 14551), sets CTRL_ parameters, collects
pursuit-distance data for DURATION seconds, then moves to the next set.

Usage:
    python3 scripts/run_test.py
    python3 scripts/run_test.py --sets baseline fast_rebuild
    python3 scripts/run_test.py --port 14550 --duration 120

Requirements:
    pip install pymavlink tabulate
"""

import argparse
import csv
import math
import sys
import threading
import time
from collections import defaultdict

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink not installed: pip install pymavlink")

# ── Kangaroo ADSB identifier ──────────────────────────────────────────────────
KANG_ICAO = 0xCAFE00

# ── Parameter sweep ───────────────────────────────────────────────────────────
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


# ── Geometry ──────────────────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    r1, r2 = math.radians(lat1), math.radians(lat2)
    dlat   = math.radians(lat2 - lat1)
    dlon   = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(r1) * math.cos(r2) * math.sin(dlon / 2) ** 2
    return R * 2 * math.asin(min(1.0, math.sqrt(a)))


# ── Connection ────────────────────────────────────────────────────────────────

def open_connection(port, timeout=30):
    """
    Bind to UDP port and wait for a vehicle heartbeat from MAVProxy.

    sim_vehicle.py by default outputs MAVLink to UDP 14550 and 14551.
    We listen on one of those ports; MAVProxy drives the traffic to us.
    """
    addr = f"udpin:0.0.0.0:{port}"
    print(f"Listening on {addr} (MAVProxy must output to this port)...", flush=True)
    conn = mavutil.mavlink_connection(addr, source_system=255)

    # Send periodic GCS heartbeats so MAVProxy keeps routing messages to us.
    stop = threading.Event()
    def _hb():
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
    threading.Thread(target=_hb, daemon=True).start()

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        sysid = msg.get_srcSystem()
        if sysid in (0, 255):   # skip broadcast / GCS
            continue
        conn.target_system    = sysid
        conn.target_component = msg.get_srcComponent()
        print(f"Vehicle heartbeat: sysid={sysid}", flush=True)
        return conn, stop

    stop.set()
    sys.exit(
        f"\nNo vehicle heartbeat on port {port} after {timeout}s.\n"
        f"Check that sim_vehicle.py was started with --out udp:127.0.0.1:{port}\n"
        f"(default outputs are 14550 and 14551)."
    )


# ── Parameters ────────────────────────────────────────────────────────────────

def set_param(conn, name, value, retries=5, ack_timeout=2.0):
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


def apply_params(conn, param_set):
    for k, v in param_set.items():
        if k == "name":
            continue
        ok = set_param(conn, k, v)
        status = "ok" if ok else "no-ack"
        print(f"    {k} = {v}  [{status}]", flush=True)


# ── Trial ─────────────────────────────────────────────────────────────────────

def run_trial(conn, duration_s):
    """Collect MAVLink messages for duration_s, return metrics dict."""
    kang_lat = kang_lon = None
    dists    = []
    deadline = time.monotonic() + duration_s
    last_dot = time.monotonic()

    print(f"  Collecting {duration_s}s ", end="", flush=True)

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        msg = conn.recv_match(blocking=True, timeout=min(0.1, remaining))
        if msg is None:
            continue

        t = msg.get_type()

        if t == "ADSB_VEHICLE" and msg.ICAO_address == KANG_ICAO:
            kang_lat = msg.lat * 1e-7
            kang_lon = msg.lon * 1e-7

        elif t == "GLOBAL_POSITION_INT" and kang_lat is not None:
            dists.append(haversine_m(
                msg.lat * 1e-7, msg.lon * 1e-7,
                kang_lat, kang_lon,
            ))

        if time.monotonic() - last_dot >= 10:
            elapsed = duration_s - (deadline - time.monotonic())
            print(f"[{int(elapsed)}s]", end="", flush=True)
            last_dot = time.monotonic()

    print(" done.", flush=True)

    n    = len(dists)
    mean = sum(dists) / n               if n       else None
    mx   = max(dists)                   if n       else None
    p95  = sorted(dists)[int(n * 0.95)] if n >= 20 else None

    return {
        "n_samples":   n,
        "mean_dist_m": round(mean, 1) if mean is not None else None,
        "p95_dist_m":  round(p95,  1) if p95  is not None else None,
        "max_dist_m":  round(mx,   1) if mx   is not None else None,
    }


# ── Output ────────────────────────────────────────────────────────────────────

METRICS_ORDER = [
    ("mean_dist_m", "Mean pursuit dist (m)  ★"),
    ("p95_dist_m",  "p95 pursuit dist (m)"),
    ("max_dist_m",  "Max pursuit dist (m)"),
    ("n_samples",   "Distance samples"),
]


def print_table(results):
    names = [r["name"] for r in results]
    sums  = [r["metrics"] for r in results]
    try:
        from tabulate import tabulate
        rows = [
            [label] + [str(s.get(key, "—")) for s in sums]
            for key, label in METRICS_ORDER
        ]
        print(tabulate(rows, headers=["Metric"] + names, tablefmt="github"))
    except ImportError:
        col = max(32, *(len(n) + 2 for n in names))
        hdr = f"{'Metric':<42}" + "".join(f"{n:>{col}}" for n in names)
        print(hdr)
        print("─" * len(hdr))
        for key, label in METRICS_ORDER:
            print(f"{label:<42}" + "".join(f"{str(s.get(key, '—')):>{col}}" for s in sums))


def save_csv(results, path):
    fields = ["name"] + [k for k, _ in METRICS_ORDER]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in results:
            w.writerow({"name": r["name"], **r["metrics"]})
    print(f"Saved → {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Sweep CTRL_ params and report pursuit-distance error"
    )
    ap.add_argument(
        "--port", type=int, default=14551,
        help="UDP port MAVProxy outputs to (default: 14551)",
    )
    ap.add_argument(
        "--duration", type=int, default=60,
        help="Collection window per trial in seconds (default: 60)",
    )
    ap.add_argument(
        "--settle", type=int, default=5,
        help="Seconds after param change before collecting (default: 5)",
    )
    ap.add_argument(
        "--output", default="sweep_results.csv",
    )
    ap.add_argument(
        "--sets", nargs="*",
        help="Run only these named sets (default: all)",
    )
    args = ap.parse_args()

    sets_to_run = PARAM_SETS
    if args.sets:
        sets_to_run = [s for s in PARAM_SETS if s["name"] in args.sets]
        if not sets_to_run:
            sys.exit(f"No matching sets. Available: {[s['name'] for s in PARAM_SETS]}")

    conn, stop_hb = open_connection(args.port)

    results = []
    total   = len(sets_to_run)

    for i, ps in enumerate(sets_to_run, 1):
        name = ps["name"]
        print(f"\n── [{i}/{total}] {name} " + "─" * max(0, 50 - len(name)))

        apply_params(conn, ps)

        print(f"  Settling {args.settle}s...", end="", flush=True)
        time.sleep(args.settle)
        print(" go.", flush=True)

        metrics = run_trial(conn, args.duration)
        results.append({"name": name, "metrics": metrics})

        print(
            f"  mean={metrics['mean_dist_m']}m  "
            f"p95={metrics['p95_dist_m']}m  "
            f"samples={metrics['n_samples']}"
        )

    stop_hb.set()

    print("\n" + "═" * 60)
    print("SWEEP RESULTS")
    print("═" * 60)
    print_table(results)
    save_csv(results, args.output)


if __name__ == "__main__":
    main()
