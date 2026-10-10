"""The flight-test fence: the one boundary the kangaroo is contained in (`ADR-012`).

``environment.json`` ``flight_area.fence_file`` names a QGC WPL 110 inclusion
polygon (``sitl-runs/springvalley2-test-fence.txt``). Everything that contains
the kangaroo reads it from there, and no copy of its vertices is typed
anywhere else:

* the SITL campaign (:mod:`campaign`): the Python counterpart of each cell is
  contained against the polygon in the cell's anchor frame, its
  ``legs_flown`` (with the containment turns) are what the vehicle replays,
  and ``arduplane.py`` uploads the file's own vertices as the report-only
  fence;
* the demonstration and the hardware validation script: :func:`spec_block`
  puts the vertices (degrees) in the vehicle ``spec.json``, and the scripts
  convert them to their anchor frame with the vehicle's own
  ``Location:get_distance_NE`` and contain the kangaroo with
  ``harness_zone.lua``, the gated port of the Python rule.
"""

import json
import os

from . import paths

from py_harness import zone as zone_mod


class FenceError(ValueError):
    """The fence cannot be used to contain the kangaroo. The message names why."""


def environment_fence_path(env=None):
    """Absolute path of the fence ``environment.json`` pins, or ``None``."""
    if env is None:
        with open(paths.ENVIRONMENT_FILE) as handle:
            env = json.load(handle)
    rel = (env.get("flight_area") or {}).get("fence_file")
    if not rel:
        return None
    return rel if os.path.isabs(rel) else os.path.join(paths.REPO_ROOT, rel)


def load(path):
    """The fence's vertices ``[(lat_deg, lng_deg), ...]``, checked to be a
    convex polygon the containment rule can use.

    Raises:
        FenceError: The file is missing, malformed or not a convex polygon.
    """
    if not os.path.isfile(path):
        raise FenceError("fence file %s does not exist" % paths.rel(path))
    try:
        vertices = zone_mod.read_fence_file(path)
        lat0, lng0 = vertices[0]
        zone_mod.PolygonZone(zone_mod.fence_ne_m(vertices, lat0, lng0))
    except ValueError as exc:
        raise FenceError("%s: %s" % (paths.rel(path), exc))
    return vertices


def polygon_ne_m(path, ref_lat_deg, ref_lng_deg):
    """The fence as ``[[n_m, e_m], ...]`` from a reference point (home)."""
    return [list(v) for v in zone_mod.fence_ne_m(load(path), ref_lat_deg, ref_lng_deg)]


def spec_block(path, containment_margin_m):
    """The vehicle ``spec.json`` ``fence`` field: the file, its vertices in
    degrees and the inset the kangaroo is turned back at (the orbit radius
    unless the spec's zone says otherwise, as ``ScenarioSession``)."""
    return {
        "file": paths.rel(path),
        "vertices_latlng": [[lat, lng] for lat, lng in load(path)],
        "containment_margin_m": float(containment_margin_m),
    }


def containment_margin_m(spec):
    """The inset a spec's kangaroo is turned back at, metres."""
    margin = (spec.get("zone") or {}).get("containment_margin_m")
    return float(spec["aircraft"]["orbit_radius_m"] if margin is None else margin)
