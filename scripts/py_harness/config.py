"""Harness configuration: the one place flight parameters are defined.

Every value here is **harness configuration for offline geometric validation**.
None of it is an approved flight-safety limit, and none of it changes any Lua
`CTRL_*` parameter. See `docs/REQUIREMENTS.md`, "Reported settings that are not
requirements", and `SR-004`.

Provenance
----------
The four flight values below come from user direction on 2026-07-22 under
``TASK-001``. They have not been derived from airframe data and have not been
reviewed (`SR-003`).

The **flight code** holds a 45-degree bank limit — what
`modules/dubins_weave_full.lua` implements and what `docs/ARCHITECTURE.md`,
`docs/PROJECT_CONTEXT.md` and the `SR` requirements record. The **harness** bank
limit was raised to **60 degrees** on 2026-07-27 by direction (`ADR-002`), so the
harness deliberately **diverges** from the flight code. This is a modelling
choice for the offline harness, not an approved limit (`SR-004`).

The consequence: at 25 m/s a 60-degree bank gives a minimum turn radius of
**36.80 m** (2.0 g), so the requested 45 m turn radius is now **feasible** (it
needs 54.8 degrees, 1.73 g). At the flight code's 45 degrees the floor would be
63.73 m and the 45 m radius would be infeasible; that comparison is preserved in
`ADR-002`. The harness still refuses any radius below its 36.80 m floor unless
``--allow-infeasible`` is passed.

Conventions (`IR-001` to `IR-005`)
-----------------------------------
Distances metres, speeds m/s, angles radians internally with degrees only at
this boundary, time seconds.
"""

import math

# --------------------------------------------------------------------------
# Requested flight parameters — user direction, 2026-07-22
# --------------------------------------------------------------------------

#: Constant aircraft ground speed, m/s. The harness does not model
#: acceleration, so this is both the commanded and the achieved speed.
AIRSPEED_MS = 25.0

#: Requested minimum turn radius, m. Feasible since the harness bank limit was
#: raised to 60 degrees on 2026-07-27 (needs 54.8 deg, 1.73 g) — see
#: BANK_LIMIT_DEG, ADR-002 and bank_limited_turn_radius().
TURN_RADIUS_M = 45.0

#: Radius of the orbit ring projected around the target, m. This is the ring
#: the aircraft flies tangent to and then circles.
ORBIT_RADIUS_M = 70.0

# --------------------------------------------------------------------------
# Harness bank limit — raised to 60 deg on 2026-07-27 (ADR-002)
# --------------------------------------------------------------------------

#: Maximum bank angle, degrees, used by the harness for its feasibility check.
#: **Raised from 45 to 60 degrees on 2026-07-27 by direction (ADR-002)** so the
#: directed 45 m turn radius is flyable in the harness. This **diverges from the
#: flight code**, where `modules/dubins_weave_full.lua` and the `SR` requirements
#: still hold 45 degrees; the harness value is a modelling choice, not an
#: approved limit (`SR-004`). At 60 degrees the load factor is 2.0 g.
BANK_LIMIT_DEG = 60.0

#: Gravitational acceleration, m/s^2. The `py_plots/` scripts use 9.8; this
#: uses the standard value, which changes derived radii by about 0.07%.
GRAVITY_MS2 = 9.80665

# --------------------------------------------------------------------------
# Harness-only parameters — not from the 2026-07-22 direction
# --------------------------------------------------------------------------

#: Arc length along a generated path at which the guidance point is taken, m.
#: **Carried from `CTRL_C_LOOK` (50 m); not specified by the 2026-07-22
#: direction.** The interface returns one guidance point but Dubins and orbit
#: generate paths, so a selection rule is required; this is the carrot rule
#: already defined under "Guidance point" in `docs/GLOSSARY.md`.
LOOK_AHEAD_M = 50.0

#: Dubins arc sampling, radians. From `py_plots/dubins_path.py`.
DELTA_PSI_RAD = math.radians(5.0)

#: Dubins straight sampling, m. From `py_plots/dubins_path.py`.
DELTA_D_M = 0.5

#: Fixed simulation time step, s. Matches the controller's 100 ms loop.
DT_S = 0.1

#: State-estimate look-ahead horizon, in whole steps (TASK-017). 0 = off (the
#: default; unchanged behaviour). When >= 1 and an estimator runs, the target
#: estimate is projected this many steps ahead (horizon = LOOKAHEAD_STEPS * DT_S)
#: on constant velocity, and guidance aims at the prediction. Serves FR-003.
LOOKAHEAD_STEPS = 0

#: Replan interval in whole control ticks (TASK-033). The committed plan is
#: regenerated every this many steps and held in between; the replan period is
#: REPLAN_EVERY * DT_S seconds and the rate its reciprocal. 1 = replan every tick
#: (the default; unchanged behaviour). Distinct from DT_S, which is the
#: integration step: varying DT_S coarsens the aircraft's motion as well as the
#: planning rate, which is why the two are separate parameters.
REPLAN_EVERY = 1

#: What the adaptive CS-orbit holds between replan instants (TASK-033). "plan"
#: holds the predicted ring centre and the committed CS curve; "centre_only"
#: holds just the centre and re-solves the curve from the live pose each tick.
#: Only the adaptive_db_circle algorithm reads it.
HOLD_POLICY = "plan"

#: Accepted values for HOLD_POLICY.
HOLD_POLICIES = ("plan", "centre_only")

#: Pre-compensate the orbit guidance ring for chord-cutting (TASK-027, Option A).
#: True places the orbit guidance point on a virtual ring `R / cos(L/R)` so the
#: circle actually *flown* is the commanded `orbit_radius_m`; False restores the
#: as-built law, which settles at about `R*cos(L/R)` (`A-VAL-005`). Default True:
#: the commanded radius is what the operator asked for. Set False to reproduce
#: results recorded before TASK-027.
ORBIT_PRECOMPENSATE = True

# --------------------------------------------------------------------------
# Arm B — adaptive prediction horizon (TASK-039 / TASK-022)
# --------------------------------------------------------------------------
# `adaptive_horizon_cs` evaluates a set of candidate prediction horizons at each
# replan instant and selects one, instead of consuming a fixed `LOOKAHEAD_STEPS`.
# These are the candidate set and the objective weights. **Illustrative, not
# tuned** (`SR-004`); the weights were chosen so no single term dominates at the
# reference configuration (R = 70 m, rho = 45 m, V = 25 m/s) and are the first
# thing a sweep should vary.

#: Smallest candidate prediction horizon, whole control ticks. 0 means "do not
#: predict", which must stay in the candidate set: if the selector never returns
#: it, that is a result, and if it always returns it, the signal is degenerate
#: (TASK-035 measured exactly that for the Euclidean prediction error).
AH_K_MIN_STEPS = 0

#: Largest candidate prediction horizon, whole control ticks. 40 ticks = 4.0 s at
#: DT_S. Two bounds meet here. The constant-velocity target model (`A-TGT-002`)
#: is not credible much further out — the prototype measured 27.5 m cross-track
#: error at 3.8 s on a 60 m turn — and the selection sweep of 2026-09-03 measured
#: steady-state ring RMS rising from 12.5 m at this bound to 15.2 m at 60 ticks
#: against a 12.5 m/s straight target, because the extra candidates are chosen
#: during the long initial approach and lead the ring past where the aircraft can
#: arrive. See `docs/TASK-039-ISSUES.md`, ISSUE-B2.
AH_K_MAX_STEPS = 40

#: Spacing between candidate horizons, whole control ticks. The candidate set is
#: {AH_K_MIN_STEPS, +AH_K_STEP, ...} up to AH_K_MAX_STEPS inclusive. 5 ticks =
#: 0.5 s, giving 13 candidates over the default range; the cost of a replan is
#: linear in this count and is reported per run (`PR-002`).
AH_K_STEP = 5

#: Weight on the **tangent registration** term J_T — the TASK-038 signal
#: evaluated predictively: how far the planned handover point misses the target's
#: standoff ring at the moment the aircraft is predicted to arrive there. Squared
#: metres, so w = 1.0 makes a 1 m registration miss cost 1.0.
AH_W_TANGENT = 1.0

#: Weight on the **standoff-tube** term J_R — mean squared deviation from the
#: standoff radius sampled along the whole approach curve, not just its endpoint.
#: Distinct from J_T: J_T scores where the plan *ends*, J_R scores where it *goes*.
#: Lower than AH_W_TANGENT because it is a mean over many samples.
AH_W_RADIAL = 0.05

#: Weight on the **curvature-excess** term J_kappa — `max(0, kappa_req -
#: 1/rho)^2` in 1/m^2, where kappa_req is the curvature needed to hold the ring
#: about a centre translating at the estimated target speed, evaluated at the
#: bearing the candidate plan arrives on (the moving-centre turn-rate relation,
#: `TASK-039`). Large because 1/m^2 units make the term numerically tiny
#: otherwise; chosen so a 10% curvature overshoot (0.0022 1/m at rho = 45 m)
#: costs about 10, comparable with a 3 m registration miss. This term does not
#: bite below about 15 m/s of target speed, which is the point: it is the
#: candidate-level expression of the feasibility boundary `TASK-039` records.
AH_W_CURVATURE = 2.0e6

#: Weight on the **switching** term J_S — `(k - k_previous)^2` in ticks^2,
#: damping oscillation of the selected horizon between adjacent candidates. At
#: 2.0 a one-candidate (5-tick) move costs 50, which the tangent term matches
#: only at about a 7 m registration change: the selector moves for a real
#: mistiming and not for measurement noise. **Raised from an initial 0.02 on
#: 2026-09-03** after that value produced a horizon spanning the whole candidate
#: set inside a single steady-state tail (`docs/TASK-039-ISSUES.md`, ISSUE-B1);
#: the sweep behind the value is recorded there.
AH_W_SWITCH = 2.0

#: How many points are sampled along a candidate approach curve for J_R. The
#: curve is a few hundred points at DELTA_D_M; scoring all of them costs more
#: than it informs. 12 is enough to see a bowed approach.
AH_PATH_SAMPLES = 12

# --------------------------------------------------------------------------
# Arm C — receding-horizon geometric planner (TASK-039)
# --------------------------------------------------------------------------
# `rh_geometric` is the draft's own recommended first step toward MPC: predict
# the target over a horizon, roll a small grid of curvature-feasible aircraft
# trajectories forward, reject none (the grid is feasible by construction), score
# them against the predicted standoff tube, apply only the first command and
# re-optimise. **Illustrative weights, not tuned** (`SR-004`).

#: Planning horizon N, whole control ticks. **Not `lookahead_steps` and not
#: `replan_every`** — a third horizon (`TASK-039` naming reconciliation). 45
#: ticks = 4.5 s at DT_S, about 112 m of flight at 25 m/s, which is a little over
#: one and a half standoff radii — far enough that a rollout reaches around the
#: ring rather than only onto it. Chosen from the sweep of 2026-09-03: 17.2 m
#: steady-state ring RMS at N = 30, 9.4 m at N = 45, 14.7 m at N = 60 against a
#: 12.5 m/s straight target. Planning cost is linear in it (`rh_rollout_steps`).
RH_HORIZON_STEPS = 45

#: Number of candidate values for the **first** curvature segment, spanning
#: [-1/rho, +1/rho] inclusive. Odd, so straight flight (kappa = 0) is always a
#: candidate. Planning cost is RH_CANDIDATES * RH_CANDIDATES_2 * RH_HORIZON_STEPS
#: aircraft steps per replan and is measured, not assumed (`PR-002`, `A-SW-002`).
#:
#: **Grid resolution is the dominant tuning parameter for this arm at low target
#: speed**, because a 70 m standoff ring needs kappa = 0.014286 1/m and a coarse
#: grid does not contain it. Measured 2026-09-03 against a stationary target,
#: steady-state ring RMS: 8.70 m at 9 x 5, 0.13 m at 15 x 9, and at 5 m/s
#: 6.17 m against 3.19 m. See `docs/TASK-039-ISSUES.md`, ISSUE-C2.
RH_CANDIDATES = 15

#: Number of candidate values for the **second** curvature segment, over the same
#: span. Two segments are the minimum that can both turn onto a ring and then
#: hold it; one segment can only fly circles.
RH_CANDIDATES_2 = 9

#: How many ticks the first curvature is held before the second takes over.
#: 20 ticks = 2.0 s, a little under half the default horizon. From the same
#: sweep: at N = 45 the ring RMS is 17.1 m at 5 ticks, 12.7 m at 10 and 9.4 m at
#: 20 — too short a first segment leaves the rollout dominated by its second
#: curvature, which is never commanded.
RH_SEGMENT_STEPS = 20

#: Weight on the running standoff cost `(range - R)^2`, summed over the horizon
#: and divided by the number of steps. Squared metres.
RH_W_STANDOFF = 1.0

#: Weight on the **terminal** standoff cost at step N, squared metres. Larger
#: than the running weight so the rollout is judged mainly on where it ends up,
#: which is what makes this receding-horizon rather than greedy.
RH_W_TERMINAL = 3.0

#: How many rollout steps ahead the emitted guidance point is taken, whole
#: control ticks. **1 = command the next planned pose**, which is the receding-
#: horizon "apply the first command" rule and the only setting that makes the
#: aircraft fly the trajectory that was optimised.
#:
#: This arm deliberately does **not** use `LOOK_AHEAD_M`. A carrot placed 50 m
#: along a curving rollout is a chord across it: the harness steers on the
#: bearing to the carrot under a turn-rate limit, and on a 60 m-radius plan a
#: 50 m carrot asks for a 0.42 rad heading change against a 0.056 rad/tick limit,
#: so the limiter saturates every tick and the aircraft flies its MINIMUM radius
#: regardless of the plan. Measured 2026-09-03: a stationary target with a 70 m
#: commanded ring was held at 45.0 m — exactly `turn_radius_m` — under a 50 m
#: carrot, and at 65.0 m under a 2.5 m one. The geometric arms are not exposed to
#: this because their plans curve at `1/R`, not at the aircraft's limit.
#: `docs/TASK-039-ISSUES.md`, ISSUE-C1.
RH_COMMAND_STEPS = 1

#: Weight on control effort, `kappa^2` averaged over the horizon, in m^2. Scaled
#: so a full-deflection rollout (kappa = 1/45) costs about 1.0.
RH_W_EFFORT = 2.0e3

#: Weight on command smoothness, `(kappa_1 - kappa_previous)^2` in m^2. Damps
#: chattering between adjacent candidates on successive replans.
RH_W_SMOOTH = 5.0e3

# --------------------------------------------------------------------------
# Amplitude weave parameters (TASK-004) — harness-only, illustrative
# --------------------------------------------------------------------------
# The names mirror `modules/continuous_weave.lua`. These are **not** tuned from
# airframe data or the shipping `CTRL_*` values; they are illustrative harness
# settings (`SR-004`) chosen so the curvature cap binds near the target and the
# safety-factor comparison is visible.

#: Weave wavelength, m. Sine argument is 2*pi*s/lambda.
WEAVE_LAMBDA_M = 150.0

#: Maximum desired weave amplitude far from the constraint, m.
WEAVE_A_CAP_M = 40.0

#: Distance at which the weave begins, m (d_start > d_full).
WEAVE_D_START_M = 400.0

#: Distance at which the weave reaches full desired amplitude, m.
WEAVE_D_FULL_M = 120.0

#: Curvature safety factor, 0 < eta <= 1. eta = 1 sits at the curvature limit
#: 1/R_min; eta < 1 keeps a margin. The TASK-004 comparison is eta vs eta = 1.
WEAVE_ETA = 0.8

#: Variable-amplitude weave onset lead time, s (TASK-014). The weave begins at
#: `d_full + WEAVE_VAW_LEAD_S * relative_speed`, so faster closing starts it
#: farther out. Only the `var_amplitude`/`vaw` algorithms read it. Provisional
#: and **not approved** (`MTG-2026-07-28-01`, `DEC-2026-07-28-04`).
WEAVE_VAW_LEAD_S = 8.0


class InfeasibleConfiguration(Exception):
    """Raised when a configuration violates the harness bank limit.

    Carries the numbers rather than a bare message, because the whole point of
    refusing is to show what would have to change.
    """


def bank_limited_turn_radius(airspeed_ms, bank_limit_deg, gravity_ms2):
    """Smallest turn radius flyable at ``bank_limit_deg``, metres.

    Coordinated-turn relation ``rho = V^2 / (g * tan(phi))``, the same relation
    used in `py_plots/dubins_path.py:29` and recorded in
    `docs/PROJECT_CONTEXT.md`. See `A-AIR-002`.
    """
    return (airspeed_ms * airspeed_ms) / (
        gravity_ms2 * math.tan(math.radians(bank_limit_deg))
    )


def required_bank_deg(radius_m, airspeed_ms, gravity_ms2):
    """Bank angle needed to hold ``radius_m`` at ``airspeed_ms``, degrees.

    The inverse of :func:`bank_limited_turn_radius`. This is the direction the
    harness works in: airspeed is fixed and bank falls out of the geometry,
    whereas `py_plots/combined.py` fixes bank and derives speed. Same relation,
    inverted — recorded as a behavioural change of the port under `A-VAL-004`.
    """
    return math.degrees(
        math.atan((airspeed_ms * airspeed_ms) / (gravity_ms2 * radius_m))
    )


def load_factor(bank_deg):
    """Load factor ``n = 1 / cos(phi)`` for a level coordinated turn."""
    return 1.0 / math.cos(math.radians(bank_deg))


class HarnessConfig:
    """Immutable flight configuration handed to an algorithm at construction.

    Attributes are set once and cannot be reassigned. Nothing in the harness
    mutates configuration; that is the whole of the state-simplicity rule the
    supervisor asked for.
    """

    __slots__ = (
        "airspeed_ms",
        "turn_radius_m",
        "orbit_radius_m",
        "bank_limit_deg",
        "gravity_ms2",
        "look_ahead_m",
        "delta_psi_rad",
        "delta_d_m",
        "dt_s",
        "weave_lambda_m",
        "weave_a_cap_m",
        "weave_d_start_m",
        "weave_d_full_m",
        "weave_eta",
        "weave_vaw_lead_s",
        "lookahead_steps",
        "orbit_precompensate",
        "replan_every",
        "hold_policy",
        "ah_k_min_steps",
        "ah_k_max_steps",
        "ah_k_step",
        "ah_w_tangent",
        "ah_w_radial",
        "ah_w_curvature",
        "ah_w_switch",
        "ah_path_samples",
        "rh_horizon_steps",
        "rh_candidates",
        "rh_candidates_2",
        "rh_segment_steps",
        "rh_command_steps",
        "rh_w_standoff",
        "rh_w_terminal",
        "rh_w_effort",
        "rh_w_smooth",
        "_frozen",
    )

    def __init__(
        self,
        airspeed_ms=AIRSPEED_MS,
        turn_radius_m=TURN_RADIUS_M,
        orbit_radius_m=ORBIT_RADIUS_M,
        bank_limit_deg=BANK_LIMIT_DEG,
        gravity_ms2=GRAVITY_MS2,
        look_ahead_m=LOOK_AHEAD_M,
        delta_psi_rad=DELTA_PSI_RAD,
        delta_d_m=DELTA_D_M,
        dt_s=DT_S,
        weave_lambda_m=WEAVE_LAMBDA_M,
        weave_a_cap_m=WEAVE_A_CAP_M,
        weave_d_start_m=WEAVE_D_START_M,
        weave_d_full_m=WEAVE_D_FULL_M,
        weave_eta=WEAVE_ETA,
        weave_vaw_lead_s=WEAVE_VAW_LEAD_S,
        lookahead_steps=LOOKAHEAD_STEPS,
        orbit_precompensate=ORBIT_PRECOMPENSATE,
        replan_every=REPLAN_EVERY,
        hold_policy=HOLD_POLICY,
        ah_k_min_steps=AH_K_MIN_STEPS,
        ah_k_max_steps=AH_K_MAX_STEPS,
        ah_k_step=AH_K_STEP,
        ah_w_tangent=AH_W_TANGENT,
        ah_w_radial=AH_W_RADIAL,
        ah_w_curvature=AH_W_CURVATURE,
        ah_w_switch=AH_W_SWITCH,
        ah_path_samples=AH_PATH_SAMPLES,
        rh_horizon_steps=RH_HORIZON_STEPS,
        rh_candidates=RH_CANDIDATES,
        rh_candidates_2=RH_CANDIDATES_2,
        rh_segment_steps=RH_SEGMENT_STEPS,
        rh_command_steps=RH_COMMAND_STEPS,
        rh_w_standoff=RH_W_STANDOFF,
        rh_w_terminal=RH_W_TERMINAL,
        rh_w_effort=RH_W_EFFORT,
        rh_w_smooth=RH_W_SMOOTH,
    ):
        object.__setattr__(self, "_frozen", False)
        self.airspeed_ms = float(airspeed_ms)
        self.turn_radius_m = float(turn_radius_m)
        self.orbit_radius_m = float(orbit_radius_m)
        self.bank_limit_deg = float(bank_limit_deg)
        self.gravity_ms2 = float(gravity_ms2)
        self.look_ahead_m = float(look_ahead_m)
        self.delta_psi_rad = float(delta_psi_rad)
        self.delta_d_m = float(delta_d_m)
        self.dt_s = float(dt_s)
        self.weave_lambda_m = float(weave_lambda_m)
        self.weave_a_cap_m = float(weave_a_cap_m)
        self.weave_d_start_m = float(weave_d_start_m)
        self.weave_d_full_m = float(weave_d_full_m)
        self.weave_eta = float(weave_eta)
        self.weave_vaw_lead_s = float(weave_vaw_lead_s)
        self.lookahead_steps = int(lookahead_steps)
        self.orbit_precompensate = bool(orbit_precompensate)
        self.replan_every = int(replan_every)
        self.hold_policy = str(hold_policy)
        self.ah_k_min_steps = int(ah_k_min_steps)
        self.ah_k_max_steps = int(ah_k_max_steps)
        self.ah_k_step = int(ah_k_step)
        self.ah_w_tangent = float(ah_w_tangent)
        self.ah_w_radial = float(ah_w_radial)
        self.ah_w_curvature = float(ah_w_curvature)
        self.ah_w_switch = float(ah_w_switch)
        self.ah_path_samples = int(ah_path_samples)
        self.rh_horizon_steps = int(rh_horizon_steps)
        self.rh_candidates = int(rh_candidates)
        self.rh_candidates_2 = int(rh_candidates_2)
        self.rh_segment_steps = int(rh_segment_steps)
        self.rh_command_steps = int(rh_command_steps)
        self.rh_w_standoff = float(rh_w_standoff)
        self.rh_w_terminal = float(rh_w_terminal)
        self.rh_w_effort = float(rh_w_effort)
        self.rh_w_smooth = float(rh_w_smooth)
        self._check_parameters()
        object.__setattr__(self, "_frozen", True)

    def _check_parameters(self):
        """Reject a malformed configuration at construction (``TASK-004``).

        These are domain and geometric-ordering violations that would otherwise
        surface as a division by zero, a NaN or a silently inverted envelope. An
        *infeasible* configuration (a turn radius below the bank-limited floor) is
        different — that is a flyability judgement handled by :meth:`validate`,
        which warns rather than refuses to construct.
        """
        if self.turn_radius_m <= 0.0:
            raise ValueError(
                "turn_radius_m must be positive (it is R_min), got %r"
                % self.turn_radius_m
            )
        if self.look_ahead_m <= 0.0:
            raise ValueError(
                "look_ahead_m must be positive, got %r" % self.look_ahead_m
            )
        if self.weave_lambda_m <= 0.0:
            raise ValueError(
                "weave_lambda_m must be positive, got %r" % self.weave_lambda_m
            )
        if self.weave_a_cap_m < 0.0:
            raise ValueError(
                "weave_a_cap_m must be non-negative, got %r" % self.weave_a_cap_m
            )
        if self.weave_d_full_m < 0.0:
            raise ValueError(
                "weave_d_full_m must be non-negative, got %r" % self.weave_d_full_m
            )
        if self.weave_d_start_m <= self.weave_d_full_m:
            raise ValueError(
                "weave_d_start_m must exceed weave_d_full_m (the weave ramps up "
                "as distance falls), got d_start=%r <= d_full=%r"
                % (self.weave_d_start_m, self.weave_d_full_m)
            )
        if not (0.0 < self.weave_eta <= 1.0):
            raise ValueError(
                "weave_eta must be in (0, 1] (eta > 1 exceeds the 1/R_min "
                "curvature limit), got %r" % self.weave_eta
            )
        if self.weave_vaw_lead_s <= 0.0:
            raise ValueError(
                "weave_vaw_lead_s must be positive (it scales the weave onset "
                "with relative speed), got %r" % self.weave_vaw_lead_s
            )
        if self.lookahead_steps < 0:
            raise ValueError(
                "lookahead_steps must be >= 0 (0 disables the look-ahead), got %r"
                % self.lookahead_steps
            )
        if self.replan_every < 1:
            raise ValueError(
                "replan_every must be >= 1 whole control ticks (1 replans every "
                "tick, which is the unchanged behaviour); the replan period is "
                "replan_every * dt_s seconds. Got %r"
                % self.replan_every
            )
        if self.hold_policy not in HOLD_POLICIES:
            raise ValueError(
                "hold_policy must be one of %s, got %r"
                % (", ".join(repr(h) for h in HOLD_POLICIES), self.hold_policy)
            )
        # -- arm B, adaptive prediction horizon (TASK-039) -------------------
        if self.ah_k_min_steps < 0:
            raise ValueError(
                "ah_k_min_steps must be >= 0 whole control ticks (0 keeps 'do "
                "not predict' in the candidate set), got %r"
                % self.ah_k_min_steps
            )
        if self.ah_k_max_steps < self.ah_k_min_steps:
            raise ValueError(
                "ah_k_max_steps must be >= ah_k_min_steps; the candidate "
                "horizons run from the first to the second in ah_k_step ticks. "
                "Got min=%r, max=%r"
                % (self.ah_k_min_steps, self.ah_k_max_steps)
            )
        if self.ah_k_step < 1:
            raise ValueError(
                "ah_k_step must be >= 1 whole control tick, got %r"
                % self.ah_k_step
            )
        if self.ah_path_samples < 2:
            raise ValueError(
                "ah_path_samples must be >= 2 (a curve needs both ends), got %r"
                % self.ah_path_samples
            )
        for weight in ("ah_w_tangent", "ah_w_radial", "ah_w_curvature",
                       "ah_w_switch", "rh_w_standoff", "rh_w_terminal",
                       "rh_w_effort", "rh_w_smooth"):
            if getattr(self, weight) < 0.0:
                raise ValueError(
                    "%s must be >= 0 (a negative weight rewards the thing it "
                    "names), got %r" % (weight, getattr(self, weight))
                )
        # -- arm C, receding-horizon geometric planner (TASK-039) ------------
        if self.rh_horizon_steps < 1:
            raise ValueError(
                "rh_horizon_steps must be >= 1 whole control tick; it is the "
                "planning horizon N, which is neither lookahead_steps nor "
                "replan_every. Got %r" % self.rh_horizon_steps
            )
        if self.rh_candidates < 2 or self.rh_candidates_2 < 2:
            raise ValueError(
                "rh_candidates and rh_candidates_2 must each be >= 2; a single "
                "candidate is not a search. Got %r and %r"
                % (self.rh_candidates, self.rh_candidates_2)
            )
        if not (1 <= self.rh_command_steps <= self.rh_horizon_steps):
            raise ValueError(
                "rh_command_steps must be in [1, rh_horizon_steps] ticks; it "
                "selects which rolled-out pose is emitted as the guidance "
                "point, and 1 is the receding-horizon 'apply the first "
                "command' rule. Got %r against a horizon of %r"
                % (self.rh_command_steps, self.rh_horizon_steps)
            )
        if not (1 <= self.rh_segment_steps <= self.rh_horizon_steps):
            raise ValueError(
                "rh_segment_steps must be in [1, rh_horizon_steps] ticks — the "
                "first curvature is held that long and the second covers the "
                "rest. Got %r against a horizon of %r"
                % (self.rh_segment_steps, self.rh_horizon_steps)
            )

    def __setattr__(self, name, value):
        if getattr(self, "_frozen", False):
            raise AttributeError(
                "HarnessConfig is immutable; build a new one instead of "
                "mutating %r" % name
            )
        object.__setattr__(self, name, value)

    # -- derived quantities -------------------------------------------------

    @property
    def bank_limited_turn_radius_m(self):
        """Governing minimum radius, m. 63.73 m at 25 m/s and 45 degrees."""
        return bank_limited_turn_radius(
            self.airspeed_ms, self.bank_limit_deg, self.gravity_ms2
        )

    @property
    def turn_radius_bank_deg(self):
        """Bank required by the configured turn radius, degrees. 54.78."""
        return required_bank_deg(
            self.turn_radius_m, self.airspeed_ms, self.gravity_ms2
        )

    @property
    def orbit_bank_deg(self):
        """Bank required by the configured orbit radius, degrees. 42.31."""
        return required_bank_deg(
            self.orbit_radius_m, self.airspeed_ms, self.gravity_ms2
        )

    @property
    def replan_period_s(self):
        """Seconds between replan instants, ``replan_every * dt_s``."""
        return self.replan_every * self.dt_s

    @property
    def replan_rate_hz(self):
        """Replan rate, Hz. Reported so a log states the frequency directly."""
        return 1.0 / self.replan_period_s if self.replan_period_s > 0.0 else 0.0

    @property
    def lookahead_horizon_s(self):
        """Prediction horizon in seconds, ``lookahead_steps * dt_s`` (``k * dt``)."""
        return self.lookahead_steps * self.dt_s

    @property
    def ah_candidate_horizons(self):
        """Arm B's candidate prediction horizons, whole ticks (``TASK-039``).

        ``[ah_k_min_steps, ..., ah_k_max_steps]`` in ``ah_k_step`` increments, the
        upper bound included when it lands on the grid. Derived rather than stored
        so the three bounds cannot disagree with the list they generate.
        """
        out = []
        k = self.ah_k_min_steps
        while k <= self.ah_k_max_steps:
            out.append(k)
            k += self.ah_k_step
        return out

    @property
    def rh_horizon_s(self):
        """Arm C's planning horizon in seconds, ``rh_horizon_steps * dt_s``."""
        return self.rh_horizon_steps * self.dt_s

    @property
    def rh_rollout_steps(self):
        """Aircraft steps integrated per arm C replan — the planning-cost figure.

        ``rh_candidates * rh_candidates_2 * rh_horizon_steps``. Reported because
        ``PR-002`` (the planning cycle finishes within its deadline) and
        ``A-SW-002`` (the Lua instruction budget at 10 Hz) are the binding
        constraints on this arm, and an unmeasured optimiser is exactly the risk
        ``TASK-039`` records against it.
        """
        return self.rh_candidates * self.rh_candidates_2 * self.rh_horizon_steps

    @property
    def turn_radius_feasible(self):
        """True when the configured turn radius respects the bank limit."""
        return self.turn_radius_m >= self.bank_limited_turn_radius_m

    @property
    def orbit_feasible(self):
        """True when the configured orbit radius respects the bank limit."""
        return self.orbit_radius_m >= self.bank_limited_turn_radius_m

    @property
    def feasible(self):
        return self.turn_radius_feasible and self.orbit_feasible

    # -- reporting ----------------------------------------------------------

    def infeasibility_report(self):
        """Return a list of human-readable violations. Empty when feasible."""
        problems = []
        floor = self.bank_limited_turn_radius_m
        if not self.turn_radius_feasible:
            problems.append(
                "turn radius %.1f m is below the bank-limited minimum %.2f m "
                "(%.1f m/s at %.1f deg). Holding %.1f m would need %.2f deg of "
                "bank and %.2f g, against %.2f g at the limit."
                % (
                    self.turn_radius_m,
                    floor,
                    self.airspeed_ms,
                    self.bank_limit_deg,
                    self.turn_radius_m,
                    self.turn_radius_bank_deg,
                    load_factor(self.turn_radius_bank_deg),
                    load_factor(self.bank_limit_deg),
                )
            )
        if not self.orbit_feasible:
            problems.append(
                "orbit radius %.1f m is below the bank-limited minimum %.2f m; "
                "it would need %.2f deg of bank."
                % (self.orbit_radius_m, floor, self.orbit_bank_deg)
            )
        return problems

    def validate(self, allow_infeasible=False):
        """Raise unless the configuration respects the harness bank limit.

        Args:
            allow_infeasible: When True, return the violations instead of
                raising. Callers that do this must mark every resulting output
                as infeasible; see ``Harness.history`` and the plot title.

        Returns:
            The list of violations, empty when feasible.

        Raises:
            InfeasibleConfiguration: When violations exist and
                ``allow_infeasible`` is False.
        """
        problems = self.infeasibility_report()
        if problems and not allow_infeasible:
            raise InfeasibleConfiguration(
                "configuration violates the %.1f deg bank limit:\n  - %s\n"
                "Re-run with --allow-infeasible to proceed anyway; output will "
                "be marked infeasible." % (self.bank_limit_deg, "\n  - ".join(problems))
            )
        return problems

    def summary(self):
        """Multi-line description of the configuration and what it implies."""
        return "\n".join(
            [
                "airspeed            %.1f m/s" % self.airspeed_ms,
                "bank limit          %.1f deg (harness, %.2f g; flight code holds 45 deg)"
                % (self.bank_limit_deg, load_factor(self.bank_limit_deg)),
                "bank-limited radius %.2f m" % self.bank_limited_turn_radius_m,
                "turn radius         %.1f m -> %.2f deg, %.2f g  [%s]"
                % (
                    self.turn_radius_m,
                    self.turn_radius_bank_deg,
                    load_factor(self.turn_radius_bank_deg),
                    "OK" if self.turn_radius_feasible else "INFEASIBLE",
                ),
                "orbit radius        %.1f m -> %.2f deg, %.2f g  [%s]"
                % (
                    self.orbit_radius_m,
                    self.orbit_bank_deg,
                    load_factor(self.orbit_bank_deg),
                    "OK" if self.orbit_feasible else "INFEASIBLE",
                ),
                "look-ahead          %.1f m (from CTRL_C_LOOK, not directed)"
                % self.look_ahead_m,
            ]
        )
