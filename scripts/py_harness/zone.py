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

    def __repr__(self):
        return ("InclusionZone(side_m=%.1f, height_m=%.1f, centre=(%.1f, %.1f))"
                % (self.side_m, self.height_m, self.centre_n_m, self.centre_e_m))
