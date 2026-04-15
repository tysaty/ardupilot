
"""
Plot the first Dubins curve logged by kangaroo_first_dubins_logger.lua.

Example:
  python3 Tools/scripts/plot_kangaroo_first_dubins_3d.py logs/kangaroo_first_dubins_boot1742.csv
"""

import argparse
import csv
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
import matplotlib.pyplot as plt

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


def ll_to_ne_m(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float) -> Tuple[float, float]:
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
class CurvePoint:
    time_ms: int
    point_index: int
    point_lat_deg: Optional[float]
    point_lon_deg: Optional[float]
    point_alt_m: Optional[float]
    point_rel_n_m: Optional[float]
    point_rel_e_m: Optional[float]
    point_heading_rad: Optional[float]
    plane_lat_deg: Optional[float]
    plane_lon_deg: Optional[float]
    plane_alt_m: Optional[float]
    kang_lat_deg: Optional[float]
    kang_lon_deg: Optional[float]
    kang_alt_m: Optional[float]
    build_rho_m: Optional[float]
    build_target_distance_m: Optional[float]
    build_point_count: Optional[int]


def load_points(path: str) -> List[CurvePoint]:
    out: List[CurvePoint] = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_ms = _to_int(row.get("time_ms", ""))
            point_index = _to_int(row.get("point_index", ""))
            if time_ms is None or point_index is None:
                continue

            out.append(
                CurvePoint(
                    time_ms=time_ms,
                    point_index=point_index,
                    point_lat_deg=_to_float(row.get("point_lat_deg", "")),
                    point_lon_deg=_to_float(row.get("point_lon_deg", "")),
                    point_alt_m=_to_float(row.get("point_alt_m", "")),
                    point_rel_n_m=_to_float(row.get("point_rel_n_m", "")),
                    point_rel_e_m=_to_float(row.get("point_rel_e_m", "")),
                    point_heading_rad=_to_float(row.get("point_heading_rad", "")),
                    plane_lat_deg=_to_float(row.get("plane_lat_deg", "")),
                    plane_lon_deg=_to_float(row.get("plane_lon_deg", "")),
                    plane_alt_m=_to_float(row.get("plane_alt_m", "")),
                    kang_lat_deg=_to_float(row.get("kang_lat_deg", "")),
                    kang_lon_deg=_to_float(row.get("kang_lon_deg", "")),
                    kang_alt_m=_to_float(row.get("kang_alt_m", "")),
                    build_rho_m=_to_float(row.get("build_rho_m", "")),
                    build_target_distance_m=_to_float(row.get("build_target_distance_m", "")),
                    build_point_count=_to_int(row.get("build_point_count", "")),
                )
            )

    out.sort(key=lambda p: p.point_index)
    return out


def resolve_origin(points: List[CurvePoint]) -> Tuple[float, float]:
    first = points[0]
    if first.plane_lat_deg is not None and first.plane_lon_deg is not None:
        return first.plane_lat_deg, first.plane_lon_deg
    if first.point_lat_deg is not None and first.point_lon_deg is not None:
        return first.point_lat_deg, first.point_lon_deg
    if first.kang_lat_deg is not None and first.kang_lon_deg is not None:
        return first.kang_lat_deg, first.kang_lon_deg
    raise SystemExit("No valid lat/lon origin found in CSV.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot first Dubins curve in 3D.")
    parser.add_argument("csv_path", help="CSV output from scripts/kangaroo_first_dubins_logger.lua")
    parser.add_argument("--save", default=None, help="Save figure to this path instead of showing interactively")
    args = parser.parse_args()

    points = load_points(args.csv_path)
    if not points:
        raise SystemExit("No points found in CSV.")

    lat0, lon0 = resolve_origin(points)

    curve_n: List[float] = []
    curve_e: List[float] = []
    curve_z: List[float] = []

    for p in points:
        if p.point_lat_deg is None or p.point_lon_deg is None or p.point_alt_m is None:
            continue
        n, e = ll_to_ne_m(p.point_lat_deg, p.point_lon_deg, lat0, lon0)
        curve_n.append(n)
        curve_e.append(e)
        curve_z.append(p.point_alt_m)

    if not curve_n:
        raise SystemExit("No valid Dubins curve points with lat/lon/alt were found.")

    first = points[0]

    plane_point = None
    if first.plane_lat_deg is not None and first.plane_lon_deg is not None and first.plane_alt_m is not None:
        pn, pe = ll_to_ne_m(first.plane_lat_deg, first.plane_lon_deg, lat0, lon0)
        plane_point = (pe, pn, first.plane_alt_m)

    kang_point = None
    if first.kang_lat_deg is not None and first.kang_lon_deg is not None and first.kang_alt_m is not None:
        kn, ke = ll_to_ne_m(first.kang_lat_deg, first.kang_lon_deg, lat0, lon0)
        kang_point = (ke, kn, first.kang_alt_m)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(curve_e, curve_n, curve_z, color="tab:orange", linewidth=2.0, label="First Dubins Curve")
    ax.scatter(
        [curve_e[0]],
        [curve_n[0]],
        [curve_z[0]],
        s=180,
        facecolors="none",
        edgecolors="tab:orange",
        linewidths=2.5,
        label="Curve Start",
    )
    ax.scatter(
        [curve_e[-1]],
        [curve_n[-1]],
        [curve_z[-1]],
        s=70,
        color="tab:red",
        label="Curve End",
    )

    if plane_point is not None:
        pe, pn, pz = plane_point
        ax.scatter([pe], [pn], [pz], s=65, color="tab:blue", label="Plane At Build")
        ax.plot([pe, curve_e[0]], [pn, curve_n[0]], [pz, curve_z[0]], color="tab:blue", linestyle="--", linewidth=1.1)

    if kang_point is not None:
        ke, kn, kz = kang_point
        ax.scatter([ke], [kn], [kz], s=85, marker="*", color="tab:green", label="Kangaroo At Build")

    rho_text = "n/a" if first.build_rho_m is None else f"{first.build_rho_m:.1f}"
    dist_text = "n/a" if first.build_target_distance_m is None else f"{first.build_target_distance_m:.1f}"
    count_text = "n/a" if first.build_point_count is None else str(first.build_point_count)
    ax.set_title(f"First Dubins Curve (rho={rho_text} m, dist={dist_text} m, points={count_text})")

    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Altitude MSL (m)")
    ax.legend(loc="best")
    plt.tight_layout()

    print(
        f"Loaded {len(points)} rows from {args.csv_path}\n"
        f"Build time: {first.time_ms} ms | curve points: {count_text} | rho: {rho_text} m | target distance: {dist_text} m"
    )

    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f"Saved figure to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
