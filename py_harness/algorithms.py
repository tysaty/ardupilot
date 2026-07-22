"""Substitutable geometric algorithms and the registry that selects them.

This is the algorithm module of the two-module harness
(``DEC-2026-06-25-01``). Every algorithm here implements the single interface
defined in :mod:`interface` (``DEC-2026-06-25-02``), so that the state module
never changes when an algorithm is swapped.

The registry exists so that ``VR-014``'s acceptance criterion — "two algorithms
run through the harness unchanged apart from a selection argument" — can be
tested rather than asserted.

Status: SKELETON. Both algorithms below are stubs. Neither contains geometry.
"""

from .interface import GeometricAlgorithm


class ContinuousWeaveAlgorithm(GeometricAlgorithm):
    """Port of the shipping continuous weave law.

    To be transliterated from ``modules/continuous_weave.lua``, whose
    ``straight_weave`` (line 37) is the shipping guidance law and the only one
    the controller currently calls. Porting it in first gives the harness a
    known-behaviour reference against which a second algorithm can be judged
    (``ACT-2026-06-25-04``).

    Two properties of the Lua source must survive the port, or the harness is
    validating something the aircraft does not fly:

    * the weave is **plane-anchored** — the reference line is recomputed from
      the aircraft's current position every cycle, which is the source of the
      achieved-amplitude discrepancy recorded in ``ADR-001`` and
      ``A-DEC-009``. The port must reproduce the defect, not silently fix it.
    * the sinusoid's phase argument is the **arc-length accumulator**, not
      elapsed time. It is accumulated state, so it travels through the
      snapshot's ``algorithm_state`` field rather than a module global
      (``DEC-2026-06-25-04``, ``A-VAL-003``).

    Expected configuration keys, all floats, matching the ``CTRL_CW_*``
    parameters: ``lambda_m``, ``r_min_m``, ``a_cap_m``, ``d_start_m``,
    ``d_full_m``, ``eta``, ``u``, ``phase_rad``.

    Status: SKELETON.
    """

    name = "continuous_weave"

    def guidance_point(self, snapshot):
        raise NotImplementedError(
            "Port of continuous_weave.lua straight_weave is ACT-2026-06-25-04."
        )


class DubinsAlgorithm(GeometricAlgorithm):
    """Second algorithm, demonstrating that substitution actually works.

    Candidate source is the retained Dubins geometry: ``SRC-GEOM``
    (``modules/dubins_weave_full.lua``) and its working Python port
    ``py_plots/dubins_path.py``, which carries the re-derived LSR/RSL geometry
    recorded nowhere else (``ACT-2026-06-25-05``, ``DEC-2026-06-25-06``).

    Two obstacles are known in advance and are not incidental:

    * ``dubins_path.py`` imports ``numpy``, which ``DEC-2026-06-25-04``
      forbids inside an algorithm. The geometry must be re-expressed with
      plain floats before it can sit behind this interface.
    * Dubins generates a *path*, whereas this interface returns a *single
      guidance point*. The adapter must therefore define how a point is picked
      off the generated path each cycle. That choice is a design decision and
      belongs in the task record, not in this docstring.

    Status: SKELETON.
    """

    name = "dubins"

    def guidance_point(self, snapshot):
        raise NotImplementedError(
            "Dubins algorithm behind the common interface is "
            "ACT-2026-06-25-05, blocked on ACT-2026-06-25-04."
        )


#: Name-to-class registry. Selecting an algorithm is a lookup here and nothing
#: else; no other part of the harness may branch on algorithm identity.
REGISTRY = {
    ContinuousWeaveAlgorithm.name: ContinuousWeaveAlgorithm,
    DubinsAlgorithm.name: DubinsAlgorithm,
}


def build(name, config):
    """Construct the algorithm registered under ``name``.

    Args:
        name: Registry key, one of :data:`REGISTRY`.
        config: Plain dict of float configuration.

    Returns:
        A :class:`~interface.GeometricAlgorithm` instance.

    Raises:
        KeyError: If ``name`` is not registered. The message lists the
            available names, because a typo here is otherwise indistinguishable
            from an unimplemented algorithm.
    """
    if name not in REGISTRY:
        raise KeyError(
            "unknown algorithm %r; available: %s"
            % (name, ", ".join(sorted(REGISTRY)))
        )
    return REGISTRY[name](config)
