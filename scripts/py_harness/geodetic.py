"""Local planar metres to geodetic coordinates (``TASK-029`` U1b).

The harness works entirely in local planar North/East metres and has **no
origin** — that is deliberate, and everything inside the harness stays that way.
Exporting to SITL is the one place an absolute coordinate is needed, so the
conversion lives here at the boundary rather than leaking a latitude into the
geometry.

This is the **inverse** of ``tools/log_view.py``'s ``geodetic_to_enu``, using the
same equirectangular local tangent-plane approximation about an origin, so the two
round-trip. It is reimplemented rather than imported because ``log_view`` lives in
the parent repository while the harness lives in the nested one, and reaching
across that boundary would couple them.

    north = d_lat * R                         d_lat = north / R
    east  = d_lon * R * cos(lat0)     ->      d_lon = east / (R * cos(lat0))

Accurate to well under a metre over the few-kilometre spans the harness produces,
and degrades near the poles where ``cos(lat0)`` collapses — which is guarded.

Privacy (``AGENTS.md``)
-----------------------
An export against a **real** origin is location data. The default origin here is
deliberately **fictional** (mid-Pacific, no site), so a run can never be
accidentally published against a real location: supplying a real origin has to be
a conscious act.
"""

import math

#: Mean Earth radius, metres. Matches ``tools/log_view.py`` so the two invert.
EARTH_RADIUS_M = 6378137.0

#: Deliberately fictional default origin — open ocean, no site, no vehicle.
#: Override explicitly when a real location is actually wanted.
DEFAULT_ORIGIN = (0.0, -160.0, 0.0)

#: Beyond this latitude the ``cos(lat0)`` scaling is too degenerate to trust.
MAX_ABS_LATITUDE_DEG = 85.0


def enu_to_geodetic(east_m, north_m, up_m, lat0_deg, lon0_deg, alt0_m):
    """Local ``(east, north, up)`` metres to geodetic ``(lat, lon, alt)``.

    The inverse of ``log_view.geodetic_to_enu``. Pure, so it is testable without
    a log, a vehicle or a network.

    Raises:
        ValueError: For an origin latitude beyond
            :data:`MAX_ABS_LATITUDE_DEG`, where the longitude scaling degenerates.
    """
    if abs(lat0_deg) > MAX_ABS_LATITUDE_DEG:
        raise ValueError(
            "origin latitude %.3f deg is beyond +/-%.0f deg, where the "
            "equirectangular longitude scaling degenerates"
            % (lat0_deg, MAX_ABS_LATITUDE_DEG))
    lat = lat0_deg + math.degrees(north_m / EARTH_RADIUS_M)
    cos_lat0 = math.cos(math.radians(lat0_deg))
    lon = lon0_deg + math.degrees(east_m / (EARTH_RADIUS_M * cos_lat0))
    return lat, lon, alt0_m + up_m


def ne_to_latlon(n_m, e_m, origin=None):
    """Harness ``(north, east)`` metres to ``(lat, lon)``. Convenience wrapper.

    Note the argument order: the harness says ``(north, east)`` while the ENU
    convention says ``(east, north)``. Written out rather than hidden, because a
    silent transpose here would put the export in the wrong hemisphere.
    """
    lat0, lon0, alt0 = origin or DEFAULT_ORIGIN
    lat, lon, _alt = enu_to_geodetic(e_m, n_m, 0.0, lat0, lon0, alt0)
    return lat, lon


def is_default_origin(origin):
    """True when ``origin`` is the fictional default, for warning on export."""
    if origin is None:
        return True
    return tuple(float(v) for v in origin) == DEFAULT_ORIGIN
