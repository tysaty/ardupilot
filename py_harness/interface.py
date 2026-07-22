"""Geometric algorithm interface contract for the offline validation harness.

This module is the deliverable of meeting action ``ACT-2026-06-25-03``: the
exact snapshot fields, return value, units, frame and no-solution convention
that every geometric algorithm in the harness must honour.

It contains **no geometry**. It defines the boundary across which geometry is
substituted, per ``DEC-2026-06-25-02``.

Contract rationale
------------------
``control_cont.lua`` commands a *single guidance point per cycle* through
``vehicle:set_target_location()`` (line 370). It does not upload a path. The
algorithm contract therefore returns one point, so that porting a validated
Python algorithm to Lua is a transliteration rather than a redesign.

Transliteration restrictions (``DEC-2026-06-25-04``)
----------------------------------------------------
An algorithm implemented against this interface shall:

* accept and return plain ``float`` values inside plain ``dict`` objects,
  which map directly onto Lua tables;
* not use ``numpy`` or any array library internally;
* not rely on Python closures over mutable module state — any state an
  algorithm accumulates across calls is passed in and returned through the
  snapshot's ``algorithm_state`` field, never held in the module;
* not use Python-only constructs (comprehensions over generators, ``yield``,
  decorators, exceptions for control flow) inside the geometry itself.

Conventions (``IR-001`` to ``IR-005``, ``AGENTS.md``)
-----------------------------------------------------
* Positions are local planar North/East in **metres**.
* Headings are **radians**, zero at North, increasing **clockwise**.
* Speeds are **metres per second**; time is **seconds**.
* Angles are wrapped before comparison.

Status: SKELETON. No algorithm is implemented. See
``tasks/active/TASK-001-python-geometric-validation-harness.md``.
"""

# --------------------------------------------------------------------------
# Snapshot contract
# --------------------------------------------------------------------------

#: Fields an algorithm receives. Every value is a plain float unless stated.
#:
#: The snapshot is a coherent view of one instant. An algorithm shall treat it
#: as read-only; mutating it is a contract violation, not an optimisation.
SNAPSHOT_FIELDS = {
    "t_s": "Simulation time, seconds since run start.",
    "plane_n_m": "Aircraft North position, metres.",
    "plane_e_m": "Aircraft East position, metres.",
    "plane_hdg_rad": "Aircraft heading, radians clockwise from North.",
    "plane_speed_ms": "Aircraft ground speed, metres per second.",
    "target_n_m": "Target North position, metres.",
    "target_e_m": "Target East position, metres.",
    "target_vn_ms": "Target North velocity, metres per second.",
    "target_ve_ms": "Target East velocity, metres per second.",
    "algorithm_state": (
        "Plain dict of floats the algorithm returned on the previous call, or "
        "an empty dict on the first call. This is how an algorithm carries "
        "accumulated state — for example the continuous weave's arc-length "
        "accumulator — without holding module-level state. See A-VAL-003."
    ),
}

#: Fields an algorithm returns.
#:
#: ``guidance_n_m``/``guidance_e_m`` are the single commanded guidance point,
#: the Python equivalent of the location passed to
#: ``vehicle:set_target_location()``.
RESULT_FIELDS = {
    "guidance_n_m": "Commanded guidance point, North, metres.",
    "guidance_e_m": "Commanded guidance point, East, metres.",
    "algorithm_state": (
        "Plain dict of floats to be handed back on the next call. May be "
        "empty for a stateless algorithm."
    ),
}

#: Configuration is a plain dict of floats supplied once at construction.
#: Every entry must carry explicit units in the algorithm's docstring;
#: unexplained constants are forbidden (``AGENTS.md``, ``SR-004``).


class NoSolution(Exception):
    """Raised when an algorithm cannot produce a guidance point.

    The no-solution convention is an explicit exception rather than a sentinel
    return value, because a silently returned ``None`` or ``(0.0, 0.0)`` would
    be indistinguishable from a valid point at the local origin.

    The harness records the failure in the state history and stops advancing.
    It does **not** substitute a fallback point: choosing a safe fallback is a
    controller decision governed by ``SR-007``, not a harness decision.

    Transliteration note: Lua has no exceptions in this sense. The Lua
    equivalent of raising ``NoSolution`` is returning ``nil``, which the
    caller must check. An algorithm shall therefore raise this only at points
    where a Lua port could return ``nil`` from the same branch.
    """


class GeometricAlgorithm:
    """Base class every substitutable geometric algorithm implements.

    Subclasses shall be constructible from a plain dict of float configuration
    and shall implement :meth:`guidance_point`. Nothing else is part of the
    contract.

    Status: SKELETON — no subclass implements geometry yet.
    """

    #: Registry name used to select this algorithm. Subclasses must override.
    name = None

    def __init__(self, config):
        """Store configuration.

        Args:
            config: Plain dict of floats. Units are documented per algorithm.
        """
        self.config = dict(config)

    def guidance_point(self, snapshot):
        """Return one guidance point for the given instant.

        Args:
            snapshot: Plain dict with the keys in :data:`SNAPSHOT_FIELDS`.
                Read-only.

        Returns:
            Plain dict with the keys in :data:`RESULT_FIELDS`.

        Raises:
            NoSolution: If no guidance point can be produced.
        """
        raise NotImplementedError(
            "Geometric algorithm not implemented. Porting the continuous "
            "weave is ACT-2026-06-25-04; a second algorithm is "
            "ACT-2026-06-25-05."
        )
