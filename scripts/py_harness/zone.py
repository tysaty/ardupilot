"""Terrain inclusion zone — a bounded operating area (``TASK-032``).

A rectangular (by default **square**) region in local North/East metres that both
actors are expected to stay inside. Motivated by measurement, not tidiness: in the
``TASK-028`` sweep the target ran 1195 m north at 5 m/s and close to **12 km** at
double airspeed, so unbounded runs leave any realistic operating area entirely and
the resulting plots are mostly empty space.

Two different treatments, and the difference is a requirements matter rather than
a convenience:

* **The kangaroo is contained.** It is a scripted actor the harness owns, so it can
  simply be turned back at the boundary. Containment is applied **state-side**, as
  an ordinary heading change, so it flows through the same schedule machinery as
  any other manoeuvre (``TASK-029``): position stays continuous, the turn is
  logged, and it appears in the exported schedule.
* **The aircraft is measured, never steered.** Making the aircraft respect a
  boundary is a *guidance* decision, and this module must not make it — that would
  mean editing a guidance law, which ``VR-014`` forbids and which would invalidate
  every comparison already recorded. Breaches are therefore **detected and
  reported**, not corrected. If the aircraft leaves the zone, that is a finding
  about the guidance law under test.

In practice containing the kangaroo largely contains the aircraft, since it orbits
at ``orbit_radius_m`` about a contained target — but "largely" is not "always", and
the margin is exactly what the breach report measures.

Pure geometry: plain ``math``, plain floats, no ``numpy`` (``VR-015``).
Frame is ``x = East``, ``y = North`` as everywhere else (``IR-008``).
"""

import math

#: Default zone side, metres. A 2 km square about the origin — large enough for
#: the standoff geometry (70 m ring, 45 m turn radius) to be unconstrained near
#: the centre, small enough to keep a run on one legible plot.
DEFAULT_SIDE_M = 2000.0


class InclusionZone:
    """An axis-aligned rectangle both actors are expected to remain within."""

    def __init__(self, side_m=DEFAULT_SIDE_M, centre_n_m=0.0, centre_e_m=0.0,
                 height_m=None):
        """
        Args:
            side_m: Full width (East extent), metres. Must be positive.
            centre_n_m, centre_e_m: Zone centre in local metres.
            height_m: Full North extent; defaults to ``side_m`` (a square).

        Raises:
            ValueError: For a non-positive extent.
        """
        height_m = side_m if height_m is None else height_m
        if side_m <= 0.0 or height_m <= 0.0:
            raise ValueError("zone extents must be positive, got %r x %r"
                             % (side_m, height_m))
        self.side_m = float(side_m)
        self.height_m = float(height_m)
        self.centre_n_m = float(centre_n_m)
        self.centre_e_m = float(centre_e_m)

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------

    @property
    def half_e(self):
        return self.side_m / 2.0

    @property
    def half_n(self):
        return self.height_m / 2.0

    def bounds(self):
        """``(n_min, n_max, e_min, e_max)`` in metres."""
        return (self.centre_n_m - self.half_n, self.centre_n_m + self.half_n,
                self.centre_e_m - self.half_e, self.centre_e_m + self.half_e)

    def corners(self):
        """The rectangle as a closed ``[(east, north), ...]`` ring, for drawing."""
        n0, n1, e0, e1 = self.bounds()
        return [(e0, n0), (e1, n0), (e1, n1), (e0, n1), (e0, n0)]

    def contains(self, n_m, e_m, margin_m=0.0):
        """True when ``(n, e)`` lies inside, shrunk by ``margin_m``."""
        n0, n1, e0, e1 = self.bounds()
        return (n0 + margin_m <= n_m <= n1 - margin_m
                and e0 + margin_m <= e_m <= e1 - margin_m)

    def depth_outside_m(self, n_m, e_m):
        """How far outside the zone a point lies, metres. ``0`` when inside."""
        n0, n1, e0, e1 = self.bounds()
        dn = max(n0 - n_m, 0.0, n_m - n1)
        de = max(e0 - e_m, 0.0, e_m - e1)
        return math.hypot(dn, de)

    # ------------------------------------------------------------------
    # Containment — the kangaroo only
    # ------------------------------------------------------------------

    def reflect_heading_deg(self, n_m, e_m, heading_deg, margin_m=0.0):
        """Heading turned back inside after crossing a wall, degrees from North.

        A specular reflection: the velocity component normal to the wall that was
        crossed is negated and the tangential component kept, so the target turns
        away rather than stopping or reversing. Both components flip in a corner.

        ``margin_m`` shrinks the walls inward, which is how both actors are kept
        inside: the aircraft orbits at ``orbit_radius_m`` about the target, so
        turning the target back at the bare wall still leaves the aircraft
        outside it.

        Returns the heading unchanged when the point is inside.
        """
        n0, n1, e0, e1 = self.bounds()
        n0, n1 = n0 + margin_m, n1 - margin_m
        e0, e1 = e0 + margin_m, e1 - margin_m
        vn = math.cos(math.radians(heading_deg))
        ve = math.sin(math.radians(heading_deg))
        if (n_m <= n0 and vn < 0.0) or (n_m >= n1 and vn > 0.0):
            vn = -vn
        if (e_m <= e0 and ve < 0.0) or (e_m >= e1 and ve > 0.0):
            ve = -ve
        return math.degrees(math.atan2(ve, vn)) % 360.0

    def would_exit(self, n_m, e_m, heading_deg, speed_ms, dt_s, margin_m=0.0):
        """True when a step along ``heading_deg`` would leave the zone.

        Looks one step ahead so the turn is commanded *before* the boundary is
        crossed, rather than after the target has already left.
        """
        step = speed_ms * dt_s
        nn = n_m + step * math.cos(math.radians(heading_deg))
        ne = e_m + step * math.sin(math.radians(heading_deg))
        return not self.contains(nn, ne, margin_m)

    # ------------------------------------------------------------------
    # Measurement — the aircraft, and any actor
    # ------------------------------------------------------------------

    def breaches(self, history, key_prefix="plane"):
        """Contiguous excursions outside the zone in a recorded history.

        Returns ``[{t_start, t_end, duration_s, max_depth_m}, ...]``. An empty
        list means the actor stayed inside for the whole run.
        """
        n_key, e_key = key_prefix + "_n_m", key_prefix + "_e_m"
        out, current = [], None
        for sample in history:
            depth = self.depth_outside_m(sample[n_key], sample[e_key])
            if depth > 0.0:
                if current is None:
                    current = {"t_start": sample["t_s"], "t_end": sample["t_s"],
                               "max_depth_m": depth}
                else:
                    current["t_end"] = sample["t_s"]
                    current["max_depth_m"] = max(current["max_depth_m"], depth)
            elif current is not None:
                current["duration_s"] = current["t_end"] - current["t_start"]
                out.append(current)
                current = None
        if current is not None:
            current["duration_s"] = current["t_end"] - current["t_start"]
            out.append(current)
        return out

    def breach_summary(self, history):
        """Breach counts and worst depth for both actors, as a plain dict."""
        plane = self.breaches(history, "plane")
        target = self.breaches(history, "target")
        return {
            "plane_breaches": len(plane),
            "plane_max_depth_m": max((b["max_depth_m"] for b in plane),
                                     default=0.0),
            "target_breaches": len(target),
            "target_max_depth_m": max((b["max_depth_m"] for b in target),
                                      default=0.0),
        }

    def containment_heading_deg(self, n_m, e_m, heading_deg, speed_ms, dt_s,
                                margin_m=0.0):
        """The heading a containment turn would command now, or ``None``.

        The whole rule ``ScenarioSession._contain_target`` applies, in one
        place so the vehicle's port (``harness_zone.lua`` ``contain_heading``)
        can be gated against it: a stationary target is left alone; a target
        whose next step stays inside is left alone; otherwise the heading is
        reflected, and a reflection that changes nothing is not a turn.
        """
        if speed_ms <= 0.0:
            return None
        if not self.would_exit(n_m, e_m, heading_deg, speed_ms, dt_s,
                               margin_m=margin_m):
            return None
        turned = self.reflect_heading_deg(n_m, e_m, heading_deg, margin_m=margin_m)
        if abs((turned - heading_deg + 180.0) % 360.0 - 180.0) < 1e-9:
            return None
        return turned

    def __repr__(self):
        return ("InclusionZone(side_m=%.1f, height_m=%.1f, centre=(%.1f, %.1f))"
                % (self.side_m, self.height_m, self.centre_n_m, self.centre_e_m))


class PolygonZone(InclusionZone):
    """A convex polygon both actors are expected to remain within (`ADR-012`).

    The flight-test fence (`sitl-runs/springvalley2-test-fence.txt`) is a
    skewed quadrilateral, not a rectangle, so the kangaroo is contained against
    the polygon itself. The rule is the rectangle's, generalised wall by wall:
    the walls are moved ``margin_m`` inward, a step that would leave the inset
    region triggers a turn, and the turn is a specular reflection off every
    inset wall the target is on or beyond while moving outward. With four
    axis-aligned walls this is exactly :meth:`InclusionZone.reflect_heading_deg`.

    Convex only: a reflection off one wall of a concave fence can point the
    target straight at another, and the containment rule has no answer for
    that. A concave or degenerate polygon is refused.

    The bounding box fills the rectangle attributes (``side_m``, ``height_m``,
    ``centre_*``, :meth:`bounds`) so plotting and the GUI's view extent work
    unchanged; containment and breach depth use the polygon.
    """

    def __init__(self, vertices_ne_m):
        """
        Args:
            vertices_ne_m: ``[(n_m, e_m), ...]``, at least three, in either
                winding order, not closed (the first vertex is not repeated).

        Raises:
            ValueError: Fewer than three vertices, a non-finite coordinate, a
                repeated or collinear vertex, or a concave polygon.
        """
        verts = [(float(v[0]), float(v[1])) for v in vertices_ne_m]
        if len(verts) >= 2 and verts[0] == verts[-1]:
            verts = verts[:-1]
        if len(verts) < 3:
            raise ValueError("a polygon zone needs at least 3 vertices, got %d"
                             % len(verts))
        for n, e in verts:
            if not (math.isfinite(n) and math.isfinite(e)):
                raise ValueError("polygon vertices must be finite, got %r" % ((n, e),))
        # Signed area in the (x = East, y = North) plane; positive = CCW.
        area2 = 0.0
        for i, (n0, e0) in enumerate(verts):
            n1, e1 = verts[(i + 1) % len(verts)]
            area2 += e0 * n1 - e1 * n0
        if abs(area2) < 1e-6:
            raise ValueError("polygon zone has no area")
        if area2 < 0.0:
            verts = verts[::-1]
        normals, offsets = [], []
        count = len(verts)
        for i in range(count):
            n0, e0 = verts[i]
            n1, e1 = verts[(i + 1) % count]
            n2, e2 = verts[(i + 2) % count]
            dn, de = n1 - n0, e1 - e0
            length = math.hypot(dn, de)
            if length < 1e-6:
                raise ValueError("polygon zone has a repeated vertex at %r" % ((n0, e0),))
            # cross of this edge and the next, (x, y) = (e, n): > 0 is a left turn
            cross = de * (n2 - n1) - dn * (e2 - e1)
            if cross <= 1e-9 * length * max(math.hypot(n2 - n1, e2 - e1), 1.0):
                raise ValueError(
                    "polygon zone must be convex with no collinear vertices "
                    "(vertex %r); the containment reflection is defined for a "
                    "convex fence only" % ((n1, e1),))
            # Inward unit normal: left of the edge in (x, y) = (e, n), i.e.
            # (x, y) = (-dn, de), written as (n, e) components.
            nn, ne = de / length, -dn / length
            normals.append((nn, ne))
            offsets.append(nn * n0 + ne * e0)
        self.vertices = verts
        self._normals = normals
        self._offsets = offsets
        ns = [v[0] for v in verts]
        es = [v[1] for v in verts]
        super().__init__(side_m=max(es) - min(es), height_m=max(ns) - min(ns),
                         centre_n_m=0.5 * (max(ns) + min(ns)),
                         centre_e_m=0.5 * (max(es) + min(es)))

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------

    def inward_distances(self, n_m, e_m):
        """Signed distance inside each wall, metres (negative = outside it)."""
        return [nn * n_m + ne * e_m - off
                for (nn, ne), off in zip(self._normals, self._offsets)]

    def corners(self):
        """The polygon as a closed ``[(east, north), ...]`` ring, for drawing."""
        ring = [(e, n) for n, e in self.vertices]
        return ring + [ring[0]]

    def centroid(self):
        """The polygon's area centroid ``(n_m, e_m)``."""
        a = cn = ce = 0.0
        count = len(self.vertices)
        for i in range(count):
            n0, e0 = self.vertices[i]
            n1, e1 = self.vertices[(i + 1) % count]
            w = e0 * n1 - e1 * n0
            a += w
            ce += (e0 + e1) * w
            cn += (n0 + n1) * w
        a *= 0.5
        return cn / (6.0 * a), ce / (6.0 * a)

    def contains(self, n_m, e_m, margin_m=0.0):
        """True when ``(n, e)`` lies inside, every wall moved ``margin_m`` in."""
        return all(d >= margin_m for d in self.inward_distances(n_m, e_m))

    def depth_outside_m(self, n_m, e_m):
        """How far outside the polygon a point lies, metres. ``0`` when inside."""
        if self.contains(n_m, e_m):
            return 0.0
        best = None
        count = len(self.vertices)
        for i in range(count):
            n0, e0 = self.vertices[i]
            n1, e1 = self.vertices[(i + 1) % count]
            dn, de = n1 - n0, e1 - e0
            q = ((n_m - n0) * dn + (e_m - e0) * de) / (dn * dn + de * de)
            q = min(1.0, max(0.0, q))
            d = math.hypot(n_m - (n0 + q * dn), e_m - (e0 + q * de))
            best = d if best is None else min(best, d)
        return best

    def square_half_m(self, n_m, e_m, inset_m=0.0):
        """Half-side of the largest axis-aligned square centred on ``(n, e)``
        that fits inside the walls moved ``inset_m`` in (negative: none).

        A square of half-side ``h`` reaches ``h * (|n_n| + |n_e|)`` towards a
        wall with unit normal ``(n_n, n_e)``, so ``h`` is the smallest
        clearance over that reach. Used to size a composite schedule, whose fit
        is a square about its anchor (`TASK-050`), before it is checked
        against the polygon itself.
        """
        best = None
        for (nn, ne), d in zip(self._normals, self.inward_distances(n_m, e_m)):
            h = (d - inset_m) / (abs(nn) + abs(ne))
            best = h if best is None else min(best, h)
        return best

    # ------------------------------------------------------------------
    # Containment — the kangaroo only
    # ------------------------------------------------------------------

    def reflect_heading_deg(self, n_m, e_m, heading_deg, margin_m=0.0):
        """Heading turned back inside, reflected off each inset wall the point
        is on or beyond while moving outward, in wall order. Unchanged when
        no wall applies."""
        vn = math.cos(math.radians(heading_deg))
        ve = math.sin(math.radians(heading_deg))
        for (nn, ne), d in zip(self._normals, self.inward_distances(n_m, e_m)):
            along = vn * nn + ve * ne
            if d <= margin_m and along < 0.0:
                vn -= 2.0 * along * nn
                ve -= 2.0 * along * ne
        return math.degrees(math.atan2(ve, vn)) % 360.0

    def __repr__(self):
        return "PolygonZone(%d vertices, bbox %.1f x %.1f m)" % (
            len(self.vertices), self.side_m, self.height_m)


# ----------------------------------------------------------------------
# The flight-test fence file (`ADR-012`)
# ----------------------------------------------------------------------

#: ``MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION`` and the return point, the
#: only fence items a containment fence file may carry.
FENCE_VERTEX_INCLUSION = 5001
FENCE_RETURN_POINT = 5000

#: ArduPilot's ``LATLON_TO_M`` (``AP_Math/definitions.h``): metres per 1e-7 deg.
LATLON_TO_M = 0.011131884502145034


def read_fence_file(path):
    """The inclusion polygon of a QGC WPL 110 fence file, as
    ``[(lat_deg, lng_deg), ...]``.

    One inclusion polygon only; a return point is ignored. Anything else (an
    exclusion polygon, a circle, a second inclusion polygon) is refused,
    because the kangaroo's containment would then not be the fence the
    aircraft carries.

    Raises:
        ValueError: An unreadable file, an unsupported item, or a vertex count
            that does not match the item's declared count.
    """
    with open(path) as handle:
        lines = [line.strip() for line in handle if line.strip()]
    if not lines or not lines[0].startswith("QGC WPL"):
        raise ValueError("%s is not a QGC WPL fence file" % path)
    vertices, declared = [], None
    for number, line in enumerate(lines[1:], start=2):
        fields = line.split()
        if len(fields) < 12:
            raise ValueError("%s line %d: expected 12 fields, got %d"
                             % (path, number, len(fields)))
        command = int(float(fields[3]))
        if command == FENCE_RETURN_POINT:
            continue
        if command != FENCE_VERTEX_INCLUSION:
            raise ValueError("%s line %d: fence item %d is not an inclusion "
                             "polygon vertex (%d); only one inclusion polygon is "
                             "supported" % (path, number, command,
                                            FENCE_VERTEX_INCLUSION))
        count = int(float(fields[4]))
        if declared is None:
            declared = count
        elif count != declared:
            raise ValueError("%s line %d: a second polygon (count %d after %d); "
                             "only one inclusion polygon is supported"
                             % (path, number, count, declared))
        vertices.append((float(fields[8]), float(fields[9])))
    if declared is None:
        raise ValueError("%s has no inclusion polygon" % path)
    if len(vertices) != declared:
        raise ValueError("%s declares %d vertices but lists %d"
                         % (path, declared, len(vertices)))
    return vertices


def _e7(deg):
    return int(round(float(deg) * 1e7))


def latlng_to_ne_m(ref_lat_deg, ref_lng_deg, lat_deg, lng_deg):
    """``(n_m, e_m)`` of a point from a reference, as ArduPilot's
    ``Location::get_distance_NE`` computes it on the vehicle (1e-7 degree
    integers, ``LATLON_TO_M``, longitude scaled at the mean latitude), so a
    fence converted here is the fence the vehicle's scripts convert."""
    lat1, lng1 = _e7(ref_lat_deg), _e7(ref_lng_deg)
    lat2, lng2 = _e7(lat_deg), _e7(lng_deg)
    dlng = lng2 - lng1
    if dlng > 1800000000:
        dlng -= 3600000000
    elif dlng < -1800000000:
        dlng += 3600000000
    mean_lat = int((lat1 + lat2) / 2)          # C integer division truncates
    scale = max(math.cos(mean_lat * 1.0e-7 * math.pi / 180.0), 0.01)
    return (lat2 - lat1) * LATLON_TO_M, dlng * LATLON_TO_M * scale


def fence_ne_m(vertices_latlng, ref_lat_deg, ref_lng_deg):
    """Fence vertices ``[(lat, lng), ...]`` as ``[(n_m, e_m), ...]`` from a
    reference point."""
    return [latlng_to_ne_m(ref_lat_deg, ref_lng_deg, lat, lng)
            for lat, lng in vertices_latlng]
