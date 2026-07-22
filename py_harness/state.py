"""Plane and target state, time stepping, recorded history and the plotter.

This is the state module of the two-module harness required by
``DEC-2026-06-25-01``. It owns:

1. the state of the plane and the target at a given time;
2. advancing that state by one fixed time step; and
3. the plotter (``DEC-2026-06-25-03``).

It owns no geometry. It calls a :class:`~interface.GeometricAlgorithm` to
obtain each guidance point and knows nothing about how that point is derived.

Recorded tension (``DEC-2026-06-25-03``)
----------------------------------------
``ARCHITECTURE.md`` requires component responsibilities and data ownership to
stay explicit and independently testable. Co-locating the plotter with state
weakens that. The plotter was placed here by request, and the mitigation is
part of the decision rather than an implementation detail: the plotter reads a
recorded state history and shall not mutate state or call the algorithm.

If the plotter later needs to drive the algorithm — to sweep a parameter
interactively, as the existing ``py_plots/`` sliders do — this separation is
under pressure and must be revisited rather than quietly relaxed.

Modelling limitation (``A-VAL-001``)
------------------------------------
This is a **kinematic** model. It does not model L1, TECS, EKF or airframe
response. It can validate geometry. It cannot validate tracking, and it must
not be cited as evidence for ``SR-001`` to ``SR-003``.

Status: SKELETON. Every function below raises ``NotImplementedError``.
"""


class PlaneState:
    """Aircraft state at one instant, in local North/East metres.

    Attributes are plain floats. Headings are radians clockwise from North
    (``IR-002``); distances metres, speeds m/s (``IR-005``).

    Status: SKELETON.
    """

    def __init__(self, n_m=0.0, e_m=0.0, hdg_rad=0.0, speed_ms=0.0):
        self.n_m = float(n_m)
        self.e_m = float(e_m)
        self.hdg_rad = float(hdg_rad)
        self.speed_ms = float(speed_ms)

    def as_dict(self):
        """Return the plain-float dict form used in an algorithm snapshot."""
        raise NotImplementedError("ACT-2026-06-25-04")


class TargetState:
    """Target state at one instant, in local North/East metres.

    Status: SKELETON.
    """

    def __init__(self, n_m=0.0, e_m=0.0, vn_ms=0.0, ve_ms=0.0):
        self.n_m = float(n_m)
        self.e_m = float(e_m)
        self.vn_ms = float(vn_ms)
        self.ve_ms = float(ve_ms)

    def as_dict(self):
        """Return the plain-float dict form used in an algorithm snapshot."""
        raise NotImplementedError("ACT-2026-06-25-04")


class Harness:
    """Owns plane and target state and advances them by fixed time steps.

    The harness holds the only mutable state in the system. An algorithm
    receives an immutable snapshot and returns a guidance point plus its own
    accumulated state; it never reaches into the harness.

    Status: SKELETON.
    """

    def __init__(self, plane, target, algorithm, dt_s=0.1):
        """
        Args:
            plane: :class:`PlaneState` initial aircraft state.
            target: :class:`TargetState` initial target state.
            algorithm: A :class:`~interface.GeometricAlgorithm` instance.
            dt_s: Fixed time step, seconds. Defaults to 0.1 s to match the
                shipping controller's 100 ms loop (``control_cont.lua``).
        """
        self.plane = plane
        self.target = target
        self.algorithm = algorithm
        self.dt_s = float(dt_s)
        self.t_s = 0.0
        self.history = []

    def snapshot(self):
        """Build the read-only algorithm snapshot for the current instant.

        Returns:
            Plain dict with the keys in ``interface.SNAPSHOT_FIELDS``.
        """
        raise NotImplementedError("ACT-2026-06-25-04")

    def step(self):
        """Advance one time step.

        Intended sequence: build a snapshot, ask the algorithm for a guidance
        point, steer the aircraft toward it under its turn-rate limit, advance
        the target on its own motion model, append the instant to
        :attr:`history`, and increment :attr:`t_s`.

        Raises:
            interface.NoSolution: Propagated unchanged. The harness records the
                failure and stops; it does not substitute a fallback point.
        """
        raise NotImplementedError("ACT-2026-06-25-04")

    def run(self, duration_s):
        """Step until ``duration_s`` of simulated time has elapsed."""
        raise NotImplementedError("ACT-2026-06-25-04")


def plot_history(history, axes=None):
    """Draw a recorded state history.

    This function is **read-only by contract** (``DEC-2026-06-25-03``). It
    shall not mutate ``history``, shall not construct or call a
    :class:`~interface.GeometricAlgorithm`, and shall not advance a
    :class:`Harness`. It takes a completed history and draws it.

    Args:
        history: The list produced by :attr:`Harness.history`.
        axes: Optional matplotlib axes to draw into. A new figure is created
            when omitted.

    Returns:
        The axes drawn into.

    Status: SKELETON.
    """
    raise NotImplementedError("ACT-2026-06-25-04")
