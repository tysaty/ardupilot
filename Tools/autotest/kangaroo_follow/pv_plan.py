"""The physical-validation run plan as one ``spec.json`` per arm (2026-10-10).

``pv_plan.json`` holds the plan in metres from the **site reference**, the
mean of the test fence's vertices (``TASK-068`` D9): the point run, then the
shared-start cycle of straight, circle and rectangle at constant, elastic and
stop-start pace, with a rest between runs (``docs/Physical_validation.md``).
This module turns it into ordinary spec legs and a validated spec for one arm.

The spec's frame **is** the site frame: the kangaroo starts at the point run's
offset, the fence is the spec's ``zone.polygon_ne_m`` about the site
reference, and ``kangaroo.composite`` makes :func:`experiment.validate_spec`
sample the whole schedule against the fence less the orbit radius
(``ADR-012``) and refuse the spec, naming the leg, when it does not fit. So a
plan that is built has been checked against the fence before anything flies.

The vehicle places the same legs in its own frame from the same fence
(``kangaroo_demo.lua``'s suite mode; ``kangaroo_source.lua`` later), so the
run is flown at the same ground position wherever the aircraft engages.

Usage, from ``src/ardupilot/Tools/autotest``::

    python3 -m kangaroo_follow.pv_plan              # the run table and fit
    python3 -m kangaroo_follow.pv_plan --arm FH --json
"""

import argparse
import json
import math
import os

from . import fence as fence_mod
from . import paths

from py_harness import experiment
from py_harness import kangaroo as kang
from py_harness import zone as zone_mod

#: The plan this module builds by default.
PLAN_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pv_plan.json")


class PlanError(ValueError):
    """The plan is malformed, names an unknown arm, or does not fit."""


def load(path=PLAN_FILE):
    with open(path) as handle:
        plan = json.load(handle)
    for key in ("fence_file", "aircraft", "kangaroo", "point", "cycle", "arms", "order"):
        if key not in plan:
            raise PlanError("%s has no %r" % (paths.rel(path), key))
    for arm in plan["order"]:
        if arm not in plan["arms"]:
            raise PlanError("order names %r, which is not in arms" % arm)
    return plan


def fence_path(plan):
    rel = plan["fence_file"]
    return rel if os.path.isabs(rel) else os.path.join(paths.REPO_ROOT, rel)


def site_frame(plan):
    """``(vertices_latlng, polygon_ne_m)``: the fence, and the fence in metres
    from the site reference (the mean of its vertices, as ``kangaroo_source``
    [A] and the demo's suite mode compute it)."""
    verts = fence_mod.load(fence_path(plan))
    lat = sum(v[0] for v in verts) / len(verts)
    lng = sum(v[1] for v in verts) / len(verts)
    return verts, [list(p) for p in zone_mod.fence_ne_m(verts, lat, lng)]


# ---------------------------------------------------------------------------
# Legs (the Python counterpart of kangaroo_source.lua section 14 [E])
# ---------------------------------------------------------------------------

def _time_for_distance(dist_fn, d):
    """Time for a monotone distance function to reach ``d``, by bisection."""
    lo, hi = 0.0, 1.0
    while dist_fn(hi) < d:
        hi *= 2.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if dist_fn(mid) < d:
            lo = mid
        else:
            hi = mid
    return hi


def paced_leg(base, pace, heading_deg, speed_ms, dist_m):
    """One leg covering ``dist_m`` along ``base`` at ``pace``. A paced leg
    lasts longer than a constant one; it ends when the distance is covered."""
    if pace == "constant":
        return {"duration_s": dist_m / speed_ms, "mode": base,
                "heading_deg": heading_deg, "speed_ms": speed_ms}
    if pace == "elastic":
        slow = speed_ms * kang.ELASTIC_SLOW_FACTOR
        dur = _time_for_distance(
            lambda t: kang.elastic_distance(t, slow, speed_ms), dist_m)
        return {"duration_s": dur, "mode": "elastic", "heading_deg": heading_deg,
                "speed_ms": speed_ms, "elastic_base": base}
    if pace == "stopstart":
        dur = _time_for_distance(
            lambda t: kang.stopstart_distance(t, speed_ms), dist_m)
        return {"duration_s": dur, "mode": "stopstart", "heading_deg": heading_deg,
                "speed_ms": speed_ms, "elastic_base": base}
    raise PlanError("unknown pace %r" % pace)


def geometry_legs(geometry, pace, plan):
    """One geometry at one pace, ``laps`` times, ending where it started."""
    k, c = plan["kangaroo"], plan["cycle"]
    v, laps = float(k["speed_ms"]), int(k["laps"])
    if geometry == "straight":
        out = []
        for i in range(2 * laps):           # out, back, out, back, ...
            hdg = (c["straight_heading_deg"] + (180.0 if i % 2 else 0.0)) % 360.0
            out.append(paced_leg("straight", pace, hdg, v, c["straight_m"]))
        return out
    if geometry == "circle":
        per = 2.0 * math.pi * k["radius_m"]
    elif geometry == "rectangle":
        per = 2.0 * (k["length_m"] + k["width_m"])
    else:
        raise PlanError("unknown geometry %r" % geometry)
    return [paced_leg(geometry, pace, c["closed_heading_deg"], v, laps * per)]


def _rest(seconds):
    return {"duration_s": float(seconds), "mode": "point", "heading_deg": 0.0,
            "speed_ms": 0.0}


def runs(plan):
    """The plan as ``[(run_name, [legs])]`` in flight order: the point run,
    the transit to the shared start, then each geometry at each pace, each
    followed by a rest."""
    k, p, c = plan["kangaroo"], plan["point"], plan["cycle"]
    out = [("point", [_rest(p["duration_s"])])]
    dn, de = c["start_n_m"] - p["n_m"], c["start_e_m"] - p["e_m"]
    dist = math.hypot(dn, de)
    transit = []
    if dist > 0.0:
        transit.append({"duration_s": dist / float(k["speed_ms"]), "mode": "straight",
                        "heading_deg": math.degrees(math.atan2(de, dn)) % 360.0,
                        "speed_ms": float(k["speed_ms"])})
    if k["rest_s"] > 0:
        transit.append(_rest(k["rest_s"]))
    out.append(("transit", transit))
    for geometry in k["geometries"]:
        for pace in k["paces"]:
            legs = geometry_legs(geometry, pace, plan)
            if k["rest_s"] > 0:
                legs.append(_rest(k["rest_s"]))
            out.append(("%s-%s" % (geometry, pace), legs))
    return out


def run_table(plan):
    """``[{name, t_start_s, t_end_s, legs}]``: when each run starts and ends on
    the schedule clock (the rest after a run is counted in it)."""
    t, out = 0.0, []
    for name, legs in runs(plan):
        dur = sum(leg["duration_s"] for leg in legs)
        out.append({"name": name, "t_start_s": t, "t_end_s": t + dur, "legs": len(legs)})
        t += dur
    return out


# ---------------------------------------------------------------------------
# The spec
# ---------------------------------------------------------------------------

def build_spec(plan, arm_id):
    """The validated spec for one arm. Raises :class:`PlanError` when the arm is
    unknown or the schedule leaves the fence less the orbit radius."""
    if arm_id not in plan["arms"]:
        raise PlanError("unknown arm %r; the plan has %s"
                        % (arm_id, ", ".join(sorted(plan["arms"]))))
    arm = plan["arms"][arm_id]
    a, k, p = plan["aircraft"], plan["kangaroo"], plan["point"]
    _verts, polygon = site_frame(plan)
    legs = [leg for _name, run_legs in runs(plan) for leg in run_legs]

    spec = experiment.default_spec()
    spec["experiment_id"] = "%s-%s" % (plan.get("plan_id", "PV"), arm_id)
    spec["objective"] = "physical-validation plan %s, arm %s (%s)" % (
        plan.get("plan_id", ""), arm_id, arm.get("label", ""))
    for key in ("airspeed_ms", "turn_radius_m", "orbit_radius_m", "look_ahead_m", "dt_s"):
        spec["aircraft"][key] = float(a[key])
    alg = spec["algorithm"]
    alg["name"] = arm["algorithm"]
    alg["estimate"] = bool(arm.get("estimate", False))
    alg["lookahead_steps"] = int(arm.get("lookahead_steps", 0))
    for key in ("replan_every", "hold_policy"):
        if key in arm:
            alg[key] = arm[key]
    alg["overrides"] = dict(arm.get("overrides") or {},
                            bank_limit_deg=float(a["bank_limit_deg"]))
    spec["initial_conditions"]["target_n_m"] = float(p["n_m"])
    spec["initial_conditions"]["target_e_m"] = float(p["e_m"])
    spec["kangaroo"].update({
        "radius_m": float(k["radius_m"]), "length_m": float(k["length_m"]),
        "width_m": float(k["width_m"]), "composite": True,
        "composite_margin_m": float(plan.get("fit_margin_m", 0.0)), "legs": legs})
    spec["zone"] = {"polygon_ne_m": polygon, "contain_target": True,
                    "containment_margin_m": float(a["orbit_radius_m"])}
    spec["run"]["duration_s"] = sum(leg["duration_s"] for leg in legs)
    try:
        return experiment.validate_spec(spec)
    except experiment.SpecError as exc:
        raise PlanError("arm %s: %s" % (arm_id, exc))


def fit(plan):
    """The schedule sampled at the tick against the fence: ``(least spare to
    the orbit-radius limit in metres, at run name)``, and the distance the
    cycle ends from its start."""
    _verts, polygon = site_frame(plan)
    zone = zone_mod.PolygonZone([tuple(v) for v in polygon])
    R = float(plan["aircraft"]["orbit_radius_m"])
    p, c = plan["point"], plan["cycle"]
    geometry = {key: plan["kangaroo"][key] for key in ("radius_m", "length_m", "width_m")}
    worst = (float("inf"), None)
    t0 = 0.0
    n, e = float(p["n_m"]), float(p["e_m"])
    dt = float(plan["aircraft"]["dt_s"])
    end = (n, e)
    for name, legs in runs(plan):
        state = kang.segments_callable(
            kang.make_segments([experiment.leg_tuple(l) for l in legs], n, e,
                               t0=t0, **geometry))
        dur = sum(l["duration_s"] for l in legs)
        steps = int(math.ceil(dur / dt))
        for i in range(steps + 1):
            sn, se = state(t0 + min(i * dt, dur))[:2]
            spare = min(zone.inward_distances(sn, se)) - R
            if spare < worst[0]:
                worst = (spare, name)
            end = (sn, se)
        n, e = end
        t0 += dur
    closes = math.hypot(end[0] - c["start_n_m"], end[1] - c["start_e_m"])
    return {"least_spare_m": worst[0], "at": worst[1], "ends_from_start_m": closes}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--plan", default=PLAN_FILE)
    parser.add_argument("--arm", default=None, help="print this arm's spec")
    parser.add_argument("--json", action="store_true", help="with --arm: the spec as JSON")
    args = parser.parse_args(argv)
    plan = load(args.plan)
    if args.arm and args.json:
        print(json.dumps(build_spec(plan, args.arm), indent=1))
        return 0
    for arm in plan["order"]:
        build_spec(plan, arm)                 # refuses a plan that does not fit
    table = run_table(plan)
    print("%-22s %9s %9s %7s" % ("run", "start s", "end s", "dur s"))
    for row in table:
        print("%-22s %9.1f %9.1f %7.1f" % (row["name"], row["t_start_s"], row["t_end_s"],
                                           row["t_end_s"] - row["t_start_s"]))
    total = table[-1]["t_end_s"]
    f = fit(plan)
    print("\nper arm %.1f min; %d arms in order %s: %.1f min of runs"
          % (total / 60.0, len(plan["order"]), " ".join(plan["order"]),
             len(plan["order"]) * total / 60.0))
    print("least spare to the %.0f m limit: %.1f m (%s); cycle ends %.2f m from its start"
          % (plan["aircraft"]["orbit_radius_m"], f["least_spare_m"], f["at"],
             f["ends_from_start_m"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
