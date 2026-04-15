#!/usr/bin/env python3
"""
Plot plane and kangaroo traces in 3D from kangaroo_plane_logger.lua CSV output.

Example:
  python3 Tools/scripts/plot_kangaroo_plane_3d.py logs/kangaroo_plane_trace_boot12345.csv --time-ms 65000
"""

import argparse
import csv
import math
from dataclasses import dataclass
from typing import List, Optional
import matplotlib.pyplot as plt

# radius of earth
EARTH_RADIUS_M = 6378137.0

def _to_float(value: str) -> Optional[float]:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None

def _to_int(value: str) -> Optional[int]:
    fv = _to_float(value)
    if fv is None:
        return None
    return int(round(fv))

def ll_to_ne_m(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    dlat = lat - lat0
    dlon = lon - lon0
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(lat0)
    return north, east


@dataclass
class Sample:
    time_ms: int
    plane_lat_deg: Optional[float]
    plane_lon_deg: Optional[float]
    plane_alt_m: Optional[float]
    kang_lat_deg: Optional[float]
    kang_lon_deg: Optional[float]
    kang_alt_m: Optional[float]

def load_samples(path: str) -> List[Sample]:
    out: List[Sample] = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = _to_int(row.get("time_ms", ""))
            if t is None:
                continue
            out.append(
                Sample(
                    time_ms=t,
                    plane_lat_deg=_to_float(row.get("plane_lat_deg", "")),
                    plane_lon_deg=_to_float(row.get("plane_lon_deg", "")),
                    plane_alt_m=_to_float(row.get("plane_alt_m", "")),
                    kang_lat_deg=_to_float(row.get("kang_lat_deg", "")),
                    kang_lon_deg=_to_float(row.get("kang_lon_deg", "")),
                    kang_alt_m=_to_float(row.get("kang_alt_m", "")),
                )
            )
    return out


def nearest_with_both(samples: List[Sample], target_ms: int) -> Optional[Sample]:
    best = None
    best_dt = None
    for s in samples:
        has_plane = s.plane_lat_deg is not None and s.plane_lon_deg is not None and s.plane_alt_m is not None
        has_kang = s.kang_lat_deg is not None and s.kang_lon_deg is not None and s.kang_alt_m is not None
        if not (has_plane and has_kang):
            continue
        dt = abs(s.time_ms - target_ms)
        if best is None or dt < best_dt:
            best = s
            best_dt = dt
    return best

# main loop
def main() -> None:
    parser = argparse.ArgumentParser(description="Plot plane vs kangaroo in 3D.")
    parser.add_argument("csv_path", help="CSV output from scripts/kangaroo_plane_logger.lua")
    parser.add_argument("--time-ms", type=int, default=None, help="Target time (ms) for point-in-time comparison")
    parser.add_argument("--time-s", type=float, default=None, help="Target time (s); alternative to --time-ms")
    parser.add_argument("--save", default=None, help="If set, save figure to this path instead of showing interactively")
    args = parser.parse_args()

    samples = load_samples(args.csv_path)
    if not samples:
        raise SystemExit("No samples found in CSV.")

    origin = None
    for s in samples:
        if s.plane_lat_deg is not None and s.plane_lon_deg is not None:
            origin = (s.plane_lat_deg, s.plane_lon_deg)
            break
    if origin is None:
        for s in samples:
            if s.kang_lat_deg is not None and s.kang_lon_deg is not None:
                origin = (s.kang_lat_deg, s.kang_lon_deg)
                break
    if origin is None:
        raise SystemExit("No valid lat/lon samples found.")

    lat0, lon0 = origin

    plane_n: List[float] = []
    plane_e: List[float] = []
    plane_z: List[float] = []
    kang_n: List[float] = []
    kang_e: List[float] = []
    kang_z: List[float] = []

    for s in samples:
        if s.plane_lat_deg is not None and s.plane_lon_deg is not None and s.plane_alt_m is not None:
            n, e = ll_to_ne_m(s.plane_lat_deg, s.plane_lon_deg, lat0, lon0)
            plane_n.append(n)
            plane_e.append(e)
            plane_z.append(s.plane_alt_m)
        if s.kang_lat_deg is not None and s.kang_lon_deg is not None and s.kang_alt_m is not None:
            n, e = ll_to_ne_m(s.kang_lat_deg, s.kang_lon_deg, lat0, lon0)
            kang_n.append(n)
            kang_e.append(e)
            kang_z.append(s.kang_alt_m)

    if not plane_n:
        raise SystemExit("No valid plane 3D samples found.")
    if not kang_n:
        raise SystemExit("No valid kangaroo 3D samples found.")

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(plane_e, plane_n, plane_z, color="tab:blue", linewidth=1.5, label="Plane")
    ax.plot(kang_e, kang_n, kang_z, color="tab:orange", linewidth=1.5, label="Kangaroo")
    ax.scatter(
        [plane_e[0]],
        [plane_n[0]],
        [plane_z[0]],
        s=220,
        facecolors="none",
        edgecolors="tab:blue",
        linewidths=2.5,
        label="Plane Start",
    )
    ax.scatter(
        [kang_e[0]],
        [kang_n[0]],
        [kang_z[0]],
        s=220,
        facecolors="none",
        edgecolors="tab:orange",
        linewidths=2.5,
        label="Kangaroo Start",
    )

    target_ms = args.time_ms
    if target_ms is None and args.time_s is not None:
        target_ms = int(round(args.time_s * 1000.0))

    if target_ms is not None:
        point = nearest_with_both(samples, target_ms)
        if point is not None:
            pn, pe = ll_to_ne_m(point.plane_lat_deg, point.plane_lon_deg, lat0, lon0)
            kn, ke = ll_to_ne_m(point.kang_lat_deg, point.kang_lon_deg, lat0, lon0)
            pz = point.plane_alt_m
            kz = point.kang_alt_m

            ax.scatter([pe], [pn], [pz], color="tab:blue", s=45)
            ax.scatter([ke], [kn], [kz], color="tab:orange", s=45)
            ax.plot([pe, ke], [pn, kn], [pz, kz], color="black", linestyle="--", linewidth=1.2, label="Separation")

            dx = ke - pe
            dy = kn - pn
            dz = kz - pz
            d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
            print(
                f"Nearest sample to {target_ms} ms: {point.time_ms} ms "
                f"(dt={abs(point.time_ms - target_ms)} ms)\n"
                f"Plane   E={pe:.2f} N={pn:.2f} Z={pz:.2f} m\n"
                f"Kangaroo E={ke:.2f} N={kn:.2f} Z={kz:.2f} m\n"
                f"Delta   dE={dx:.2f} dN={dy:.2f} dZ={dz:.2f} m | 3D distance={d3:.2f} m"
            )
            ax.set_title(f"Plane vs Kangaroo 3D (nearest to {target_ms} ms)")
        else:
            print("No sample with both plane and kangaroo data found for point-in-time comparison.")
            ax.set_title("Plane vs Kangaroo 3D")
    else:
        ax.set_title("Plane vs Kangaroo 3D")

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Altitude MSL (m)")
    ax.legend(loc="best")
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f"Saved figure to {args.save}")
    else:
        plt.show()

# calling main
if __name__ == "__main__":
    main()
