"""Pure geometry, ported out of ``py_plots/``.

Modules here hold **no state**: no globals mutated at runtime, no matplotlib,
no argparse, no time. They are functions of their arguments. That is what makes
them substitutable behind one interface and portable to Lua
(``DEC-2026-06-25-04``, ``VR-015``).

Frame note. These modules keep the originals' internal convention —
``x = East``, ``y = North``, heading ``psi`` measured from North increasing
clockwise — because changing it would be a rewrite rather than a port and would
invalidate the equivalence tests. Conversion to the harness's ``(north, east)``
ordering happens explicitly in the adapters in ``py_harness.algorithms``, per
``IR-008``.

The originals in ``py_plots/`` are unmodified and remain the reference
(``DEC-2026-06-25-06``).
"""

__all__ = ["dubins", "orbit"]
