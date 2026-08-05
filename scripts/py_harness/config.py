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
