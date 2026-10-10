import math, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from kangaroo_follow import pv_plan
from py_harness import experiment, kangaroo as kang

plan = pv_plan.load()
_v, poly = pv_plan.site_frame(plan)
R = plan["aircraft"]["orbit_radius_m"]
g = {k: plan["kangaroo"][k] for k in ("radius_m", "length_m", "width_m")}

def inset(poly, d):
    # each wall moved d inward, intersect consecutive walls (convex, CW or CCW)
    n = len(poly); area = sum(poly[i][1]*poly[(i+1)%n][0]-poly[(i+1)%n][1]*poly[i][0] for i in range(n))
    lines = []
    for i in range(n):
        (n0, e0), (n1, e1) = poly[i], poly[(i+1)%n]
        dn, de = n1-n0, e1-e0; L = math.hypot(dn, de)
        # inward normal
        nn, ne = (de/L, -dn/L) if area > 0 else (-de/L, dn/L)
        lines.append(((n0+nn*d, e0+ne*d), (dn, de)))
    out = []
    for i in range(n):
        (p, u), (q, w) = lines[i-1], lines[i]
        den = u[0]*w[1]-u[1]*w[0]
        t = ((q[0]-p[0])*w[1]-(q[1]-p[1])*w[0])/den
        out.append((p[0]+u[0]*t, p[1]+u[1]*t))
    return out

COL = {"straight": "#2a78d6", "circle": "#eb6834", "rectangle": "#1baf7a"}
INK, MUTED, GRID = "#1f2328", "#59636e", "#d0d7de"
fig, ax = plt.subplots(figsize=(7.2, 7.0), dpi=160)
fe = [p[1] for p in poly] + [poly[0][1]]; fn = [p[0] for p in poly] + [poly[0][0]]
ax.plot(fe, fn, color=INK, lw=1.6, label="test fence (SV2_fence_line2)")
ins = inset(poly, R)
ax.plot([p[1] for p in ins]+[ins[0][1]], [p[0] for p in ins]+[ins[0][0]], color=MUTED,
        lw=1.0, ls=(0, (5, 3)), label="kangaroo limit (70 m inside)")

n, e = plan["point"]["n_m"], plan["point"]["e_m"]
t0 = 0.0
for name, legs in pv_plan.runs(plan):
    dur = sum(l["duration_s"] for l in legs)
    f = kang.segments_callable(kang.make_segments([experiment.leg_tuple(l) for l in legs], n, e, t0=t0, **g))
    pts = [f(t0 + min(i*0.2, dur))[:2] for i in range(int(dur/0.2)+2)]
    geo = name.split("-")[0]
    if geo in COL and name.endswith("constant"):
        ax.plot([p[1] for p in pts], [p[0] for p in pts], color=COL[geo], lw=2.0,
                label="%s, %d %s" % (geo, int(plan["kangaroo"]["laps"]),
                                     ("out-and-back" if geo == "straight" else "lap")
                                     + ("s" if int(plan["kangaroo"]["laps"]) != 1 else "")),
                zorder=3)
    elif name == "transit":
        ax.plot([p[1] for p in pts], [p[0] for p in pts], color=MUTED, lw=1.4, ls=":",
                label="transit (unscored)", zorder=3)
    n, e = pts[-1]; t0 += dur

LAPS = int(plan["kangaroo"]["laps"])
pn, pe = plan["point"]["n_m"], plan["point"]["e_m"]
th = [i*2*math.pi/180 for i in range(181)]
ax.plot([pe+R*math.sin(a) for a in th], [pn+R*math.cos(a) for a in th], color=INK, lw=0.9,
        ls=(0, (1, 2)), label="aircraft ring at the point (70 m)")
ax.plot([pe], [pn], marker="o", ms=9, mfc="#ffffff", mec=INK, mew=2, ls="none",
        label="point run, %.0f s (first)" % plan["point"]["duration_s"], zorder=5)
sn, se = plan["cycle"]["start_n_m"], plan["cycle"]["start_e_m"]
ax.plot([se], [sn], marker="s", ms=8, color=INK, ls="none", label="shared start and end", zorder=5)
ax.plot([0], [0], marker="+", ms=12, mew=1.6, color=MUTED, ls="none", label="site reference (fence centre)")
ax.annotate("point\n30 N, 0 E", (pe, pn), (pe+82, pn-40), color=INK, fontsize=8,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
ax.annotate("start\n90 N, 10 E", (se, sn), (se+40, sn+45), color=INK, fontsize=8,
            arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
ax.set_aspect("equal"); ax.grid(color=GRID, lw=0.6); ax.set_axisbelow(True)
for s in ax.spines.values(): s.set_color(GRID)
ax.tick_params(colors=MUTED, labelsize=8)
ax.set_xlabel("East of the site reference (m)", color=INK, fontsize=9)
ax.set_ylabel("North of the site reference (m)", color=INK, fontsize=9)
f = pv_plan.fit(plan)
ax.set_title("Run plan per arm: point run, then the shared-start cycle\n"
             "%d lap%s, %.1f m/s; least spare %.1f m; ends %.2f m from the start"
             % (LAPS, "" if LAPS == 1 else "s", plan["kangaroo"]["speed_ms"],
                f["least_spare_m"], f["ends_from_start_m"]), color=INK, fontsize=9.5, loc="left")
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.09), ncol=2, fontsize=8, frameon=False)
fig.tight_layout()
fig.savefig(sys.argv[1], facecolor="#ffffff")
