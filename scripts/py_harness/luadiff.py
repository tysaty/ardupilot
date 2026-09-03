"""Differential harness: drive the same inputs through Python and Lua (``TASK-006``).

Created 2026-09-03, Tranche 0.

`TASK-006` ports the harness-validated algorithms into the ArduPilot Lua runtime
**in tranches, each proved against the harness before the next begins**. This
module is the mechanism that proves them: it loads a Lua module, calls it with
the same arguments the Python takes, converts the result back, and compares
field by field at the tolerances `ADR-008` records.

Why this exists rather than "read both and check they look the same"
--------------------------------------------------------------------
A numeric disagreement between two implementations of the same geometry has at
least five indistinguishable causes — floating-point ordering, an angle-wrap
boundary, a matrix helper, a coordinate swap, and a genuine logic error. All
five present identically as "the numbers differ". Gating each tranche behind an
executed comparison is what makes a divergence *local* to the tranche that
introduced it.

Runtime selection, in order of preference
-----------------------------------------
1. **``lupa.lua53``** — an embedded **Lua 5.3** interpreter, which is the version
   ArduPilot's scripting engine runs. This is the preferred route: it needs no
   vehicle, no SITL and no system package, so the pure-geometry tranches run in
   an ordinary unit-test suite.
2. ``lupa``'s default runtime — whatever Lua version the installed wheel carries.
   Used with a **recorded warning**, because integer/float semantics and
   ``math.atan``'s two-argument form are exactly the things Tranche 1 exists to
   verify, and verifying them against the wrong version proves nothing about the
   target.
3. A ``lua`` binary on ``PATH``, driven through a temporary script. Slower, and
   the version is whatever is installed.

If none is available, :func:`runtime_or_skip` reports why, and the tranche tests
skip **with the reason visible** rather than passing vacuously. A skipped gate is
not a closed gate, and `TASK-006`'s status must say so.

What this module does not do
----------------------------
It does not import, wrap or modify any harness algorithm. It loads Lua and
compares numbers. The Python side of every comparison is the ordinary harness
module, called normally by the test — so the thing under test is the *port*, and
never a special path built for the port to pass.
"""

import math
import os
import subprocess
import tempfile


#: Absolute path to the Lua module directory the ported modules live in.
MODULE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          "modules")

#: The Lua version ArduPilot's scripting engine runs. Recorded here because
#: Tranche 0 requires the target runtime's version and numeric semantics to be
#: **measured, not assumed**, and because a comparison run against a different
#: version does not discharge `A-SW-005` or `IR-004`.
TARGET_LUA_VERSION = "Lua 5.3"


# --------------------------------------------------------------------------
# Agreement tolerances — ADR-008
# --------------------------------------------------------------------------
# Rationale, per quantity. These are **transliteration** tolerances, not modelling
# tolerances: both sides evaluate the same formulas in IEEE-754 doubles, so the
# only legitimate difference is the order of floating-point operations. They are
# therefore tight by design. A divergence larger than these is a defect to be
# localised, not a rounding difference to be accommodated -- which is the whole
# point of gating.

#: Position, metres. At the ~1e2 m scale these quantities live at, double
#: precision resolves ~1e-14 m; 1e-9 leaves five orders of headroom for
#: reassociated sums while still catching any real divergence, which in practice
#: shows up at 1e-3 m or larger.
POSITION_M = 1e-9

#: Length and distance, metres. Same reasoning as position; named separately
#: because a path length is a sum over many segments and accumulates more.
LENGTH_M = 1e-9

#: Angles, radians. Tighter than position because they are O(1), so the same
#: relative precision buys more absolute headroom.
ANGLE_RAD = 1e-12

#: Curvature, 1/m. O(1e-2), and always a reciprocal of a radius rather than an
#: accumulated sum.
CURVATURE_1PM = 1e-12

#: Speed, m/s.
SPEED_MS = 1e-9

#: Dimensionless quantities: weights, fractions, dot products.
DIMENSIONLESS = 1e-12

#: Objective-function values, in **squared** metres. Looser than everything else
#: for a reason that is arithmetic, not tolerance-shopping: a cost is a sum of
#: squares of ~1e2 m quantities, so it is O(1e4), and 1e-9 absolute is a relative
#: error of 1e-13 -- tighter, in relative terms, than POSITION_M is at 1e2 m.
#: Scoring a cost at DIMENSIONLESS would demand a relative precision of 1e-16,
#: which is below double epsilon and would fail on operation order alone.
COST = 1e-9

#: **Accumulated** state after a multi-tick closed-loop run, metres. Deliberately
#: looser than :data:`POSITION_M`: a 1800-tick run feeds each tick's rounding into
#: the next through the aircraft pose and the filter covariance, so the two sides
#: diverge by a growing amount even when every individual operation agrees. 1e-6 m
#: over 180 s is 5.6 nm per tick, which no logic error can hide under.
ACCUMULATED_M = 1e-6

#: Integer tick counts, phase strings and boolean flags must match **exactly**.
#: There is no tolerance for a discrete quantity: a replan clock that is one tick
#: out, or a phase that switched a tick early, is a divergence however small the
#: positions look.
EXACT = 0.0


#: Tolerance by field-name suffix, so a comparison need not be told the units of
#: every field it is handed (`IR-005`: units are carried in the name).
_SUFFIX_TOLERANCE = (
    ("cost", COST),
    ("j_tangent", COST),
    ("j_radial", COST),
    ("j_standoff", COST),
    ("j_terminal", COST),
    ("_1pm", CURVATURE_1PM),
    ("_rad", ANGLE_RAD),
    ("_deg", ANGLE_RAD),
    ("_ms", SPEED_MS),
    ("_m", POSITION_M),
    ("_s", DIMENSIONLESS),
)


def tolerance_for(name, default=DIMENSIONLESS):
    """The tolerance for a field, from its unit suffix (`IR-005`).

    Falls back to ``default`` for a name carrying no unit — a count, a flag or a
    dimensionless cost.
    """
    for suffix, tol in _SUFFIX_TOLERANCE:
        if name.endswith(suffix):
            return tol
    return default


class LuaUnavailable(Exception):
    """No Lua runtime could be found. Carries the reason, for the skip message."""


class LuaRuntimeInfo:
    """What was actually used to run the Lua, and whether it is the target.

    Recorded on every comparison so a passing tranche cannot be quoted without
    the runtime it passed on. ``is_target`` is False when the available
    interpreter is not `TARGET_LUA_VERSION`; the tranche still runs, but its
    evidence is weaker and says so.
    """

    __slots__ = ("kind", "version", "is_target", "note")

    def __init__(self, kind, version, note=""):
        self.kind = kind
        self.version = version
        self.is_target = (version == TARGET_LUA_VERSION)
        self.note = note

    def __repr__(self):
        return "LuaRuntimeInfo(%s, %r, target=%s)" % (
            self.kind, self.version, self.is_target)

    def describe(self):
        """One line naming the runtime and flagging it when it is not the target."""
        text = "%s via %s" % (self.version, self.kind)
        if not self.is_target:
            text += (" — NOT %s, the version ArduPilot runs; integer/float "
                     "semantics and math.atan behaviour may differ"
                     % TARGET_LUA_VERSION)
        if self.note:
            text += " (%s)" % self.note
        return text


class LuaSandbox:
    """A Lua interpreter with the ported modules on its ``package.path``.

    Holds one runtime for its lifetime. Lua modules are cached by ``require``
    inside that runtime, so a sandbox reused across tests shares module state —
    which is exactly what should be shared, because every ported module is
    required to be **stateless** (`VR-015`, `A-VAL-003`). A module that
    accumulated state would make tests order-dependent, and
    :meth:`assert_stateless` exists to catch that rather than let it hide.
    """

    def __init__(self):
        self.runtime, self.info = _make_runtime()
        #: Reason string from the last refusal (``nil, "why"``), else ``None``.
        self.last_reason = None
        self._lua_path = os.path.join(MODULE_DIR, "?.lua")
        self.runtime.execute(
            'package.path = %s .. ";" .. package.path' % _lua_quote(self._lua_path))

    # -- loading -----------------------------------------------------------

    def require(self, module_name):
        """``require`` a Lua module and return its table."""
        return self.runtime.eval('require(%s)' % _lua_quote(module_name))

    def call(self, module_name, function_name, *args):
        """Call ``module.function(*args)`` and convert the **first** result.

        Numbers, strings and booleans come back as themselves; Lua tables become
        dicts, or lists when they are pure sequences. ``nil`` becomes ``None`` —
        the Lua equivalent of the harness's ``NoSolution``, per the interface
        contract's transliteration note ("raise only where a Lua port could
        return ``nil`` from the same branch").

        A ported function refuses as ``return nil, "reason"``. Only the first
        value is returned here, so a refusal reads as ``None`` at the call site
        and a test can assert it plainly; the reason is kept on
        :attr:`last_reason` so it can be checked too. Returning the raw tuple
        instead made every refusal test compare ``(None, "...")`` against
        ``None`` and fail for the wrong reason.

        A function returning several *values* rather than a value and a reason —
        ``point_state`` returns ``n, e, vn, ve`` — is returned as a list, since
        every such function in this port returns a fixed-arity tuple whose first
        element is never ``nil`` on success.
        """
        module = self.require(module_name)
        function = module[function_name]
        if function is None:
            raise AttributeError("Lua module %r has no function %r"
                                 % (module_name, function_name))
        self.last_reason = None
        result = function(*[to_lua(self.runtime, a) for a in args])
        if isinstance(result, tuple):
            converted = [to_python(v) for v in result]
            if converted and converted[0] is None:
                # A refusal: nil plus a reason.
                self.last_reason = converted[1] if len(converted) > 1 else None
                return None
            return converted
        return to_python(result)

    def eval(self, expression):
        """Evaluate a Lua expression and convert the result."""
        return to_python(self.runtime.eval(expression))

    def execute(self, chunk):
        """Execute a Lua chunk, returning nothing."""
        self.runtime.execute(chunk)

    # -- properties the ported modules must have ---------------------------

    def assert_stateless(self, module_name):
        """Fail unless a required module exposes only functions and constants.

        `VR-015` and `A-VAL-003` forbid module-level mutable state: an algorithm's
        accumulated state travels through ``algorithm_state``, never through a
        module upvalue. A Lua module holding a mutable table at module level would
        make two identical calls return different answers, which a differential
        test comparing single calls would never notice.

        A table of **scalars** is permitted and is a constant, not state — the
        mode-name list, for instance. What is refused is a nested table or a
        table of functions, which is how a cache or an accumulator would look.
        This is a shape check, so it is necessary and not sufficient; the
        determinism the property really means is asserted by the differential
        tests themselves, every one of which calls the same function repeatedly
        and requires the same answer.

        Raises:
            AssertionError: Naming the offending key.
        """
        module = self.require(module_name)
        type_of = self.runtime.eval("type")
        for key in module:
            value = module[key]
            if type_of(value) != "table":
                continue
            for inner_key in value:
                if type_of(value[inner_key]) == "table" or \
                        type_of(value[inner_key]) == "function":
                    raise AssertionError(
                        "Lua module %r exposes a nested table at %r.%r. Ported "
                        "modules must hold no mutable state (VR-015, "
                        "A-VAL-003); state travels through algorithm_state."
                        % (module_name, key, inner_key))


def _lua_quote(text):
    """A Lua string literal for ``text``, with backslashes and quotes escaped."""
    return '"%s"' % text.replace("\\", "\\\\").replace('"', '\\"')


def _make_runtime():
    """Return ``(runtime, LuaRuntimeInfo)``, preferring Lua 5.3.

    Raises:
        LuaUnavailable: With a message naming what was tried and how to fix it.
    """
    tried = []
    try:
        from lupa import lua53                      # noqa: F401 (optional)
        runtime = lua53.LuaRuntime(unpack_returned_tuples=True)
        version = runtime.eval("_VERSION")
        return runtime, LuaRuntimeInfo("lupa.lua53", version)
    except ImportError as exc:
        tried.append("lupa.lua53 (%s)" % exc)

    try:
        import lupa
        runtime = lupa.LuaRuntime(unpack_returned_tuples=True)
        version = runtime.eval("_VERSION")
        return runtime, LuaRuntimeInfo(
            "lupa (default)", version,
            note="lupa.lua53 was not importable, so this is not the target version")
    except ImportError as exc:
        tried.append("lupa (%s)" % exc)

    binary = _find_lua_binary()
    if binary is not None:
        return _BinaryRuntime(binary), LuaRuntimeInfo(
            "lua binary at %s" % binary, _binary_version(binary))
    tried.append("a `lua` binary on PATH")

    raise LuaUnavailable(
        "no Lua runtime available; tried %s. Install one with `pip install lupa` "
        "(preferred — it embeds Lua 5.3, the version ArduPilot runs) or "
        "`brew install lua`. Until then the TASK-006 tranche gates cannot close, "
        "and a skipped gate is not a closed gate." % ", ".join(tried))


def _find_lua_binary():
    for name in ("lua5.3", "lua53", "lua5.4", "lua"):
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            path = os.path.join(directory, name)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return path
    return None


def _binary_version(binary):
    try:
        out = subprocess.run([binary, "-v"], capture_output=True, text=True,
                             timeout=10)
        return (out.stdout or out.stderr).strip().split("\n")[0].split("  ")[0]
    except Exception:                                     # pragma: no cover
        return "unknown"


class _BinaryRuntime:
    """Minimal ``lupa``-shaped wrapper over a standalone ``lua`` binary.

    Supports the small surface :class:`LuaSandbox` uses. Each evaluation is a
    separate process, so it is far slower than the embedded route and cannot hold
    a loaded module between calls — which is acceptable only because every ported
    module is stateless.
    """

    def __init__(self, binary):
        self.binary = binary
        self._prelude = ""

    def execute(self, chunk):
        self._prelude += chunk + "\n"

    def eval(self, expression):
        script = "%s\nlocal __r = (%s)\nio.write(tostring(__r))\n" % (
            self._prelude, expression)
        with tempfile.NamedTemporaryFile("w", suffix=".lua", delete=False) as handle:
            handle.write(script)
            path = handle.name
        try:
            out = subprocess.run([self.binary, path], capture_output=True,
                                 text=True, timeout=60)
            if out.returncode != 0:
                raise RuntimeError("lua failed: %s" % out.stderr.strip())
            return _coerce_scalar(out.stdout.strip())
        finally:
            os.unlink(path)


def _coerce_scalar(text):
    if text == "nil":
        return None
    if text == "true":
        return True
    if text == "false":
        return False
    try:
        return float(text)
    except ValueError:
        return text


# --------------------------------------------------------------------------
# Value conversion
# --------------------------------------------------------------------------

def to_lua(runtime, value):
    """Convert a Python value to Lua: dicts and lists become tables.

    Lists become **1-indexed** Lua sequences, which is the whole of the index
    convention difference between the two languages and the one place a port can
    silently drop or duplicate an element.
    """
    if isinstance(value, dict):
        table = runtime.table()
        for key, item in value.items():
            table[key] = to_lua(runtime, item)
        return table
    if isinstance(value, (list, tuple)):
        table = runtime.table()
        for index, item in enumerate(value):
            table[index + 1] = to_lua(runtime, item)
        return table
    return value


def to_python(value):
    """Convert a Lua value to Python: a sequence table becomes a list, else a dict.

    A table is treated as a sequence when its keys are exactly ``1..n``. An empty
    table is ambiguous — Lua does not distinguish an empty list from an empty
    map — and is returned as an empty **list**, because every empty table this
    harness sees is an empty point sequence.
    """
    if value is None or isinstance(value, (int, float, str, bool)):
        return value
    if hasattr(value, "items"):
        items = dict(value.items())
        keys = list(items)
        if keys and all(isinstance(k, int) for k in keys) and \
                sorted(keys) == list(range(1, len(keys) + 1)):
            return [to_python(items[i]) for i in range(1, len(keys) + 1)]
        if not keys:
            return []
        return dict((k, to_python(v)) for k, v in items.items())
    return value


# --------------------------------------------------------------------------
# Comparison
# --------------------------------------------------------------------------

class Divergence(object):
    """One field where the two implementations disagree, with enough to chase it."""

    __slots__ = ("path", "python", "lua", "delta", "tolerance")

    def __init__(self, path, python, lua, delta, tolerance):
        self.path = path
        self.python = python
        self.lua = lua
        self.delta = delta
        self.tolerance = tolerance

    def __repr__(self):
        if self.delta is None:
            return "%s: python=%r lua=%r (not comparable)" % (
                self.path, self.python, self.lua)
        return "%s: python=%.17g lua=%.17g delta=%.3g > tol=%.3g" % (
            self.path, self.python, self.lua, self.delta, self.tolerance)


def compare(python_value, lua_value, tolerance=None, path="", default_tolerance=None):
    """Compare two converted values, returning a list of :class:`Divergence`.

    Args:
        python_value: The harness's answer.
        lua_value: The port's answer, already through :func:`to_python`.
        tolerance: A single numeric tolerance for every leaf, or ``None`` to
            derive each leaf's tolerance from its field name (`IR-005`).
        path: Field path prefix, used in the message.
        default_tolerance: Tolerance for a leaf whose name carries no unit.

    Strings, booleans and ``None`` must match **exactly** whatever the tolerance:
    a phase name, a direction or a no-solution is discrete, and "close" is not a
    meaningful relation on it.
    """
    out = []
    _compare_into(out, python_value, lua_value, tolerance, path or "value",
                  default_tolerance)
    return out


def _compare_into(out, expected, actual, tolerance, path, default_tolerance):
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            out.append(Divergence(path, expected, actual, None, tolerance))
            return
        for key in sorted(set(expected) | set(actual)):
            if key not in expected or key not in actual:
                out.append(Divergence("%s.%s" % (path, key),
                                      expected.get(key), actual.get(key),
                                      None, tolerance))
                continue
            leaf_tol = tolerance
            if leaf_tol is None:
                leaf_tol = tolerance_for(
                    str(key), default_tolerance if default_tolerance is not None
                    else DIMENSIONLESS)
            _compare_into(out, expected[key], actual[key], leaf_tol,
                          "%s.%s" % (path, key), default_tolerance)
        return

    if isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple)):
            out.append(Divergence(path, expected, actual, None, tolerance))
            return
        if len(expected) != len(actual):
            out.append(Divergence("%s.length" % path, len(expected), len(actual),
                                  None, EXACT))
            return
        for index, (a, b) in enumerate(zip(expected, actual)):
            _compare_into(out, a, b, tolerance, "%s[%d]" % (path, index),
                          default_tolerance)
        return

    if expected is None or isinstance(expected, (str, bool)):
        if expected != actual:
            out.append(Divergence(path, expected, actual, None, EXACT))
        return

    if actual is None or isinstance(actual, (str, bool)):
        out.append(Divergence(path, expected, actual, None, EXACT))
        return

    tol = DIMENSIONLESS if tolerance is None else tolerance
    if math.isnan(expected) or math.isnan(actual):
        # NaN never compares equal, so an unguarded comparison would pass a run
        # in which both sides produced garbage.
        if not (math.isnan(expected) and math.isnan(actual)):
            out.append(Divergence(path, expected, actual, None, tol))
        return
    delta = abs(expected - actual)
    if delta > tol:
        out.append(Divergence(path, expected, actual, delta, tol))


def format_divergences(divergences, limit=12):
    """A readable failure message: the worst offenders first, and how many more."""
    if not divergences:
        return "no divergence"
    ranked = sorted(divergences,
                    key=lambda d: (-1.0 if d.delta is None else -d.delta))
    lines = [repr(d) for d in ranked[:limit]]
    if len(ranked) > limit:
        lines.append("... and %d more" % (len(ranked) - limit))
    return "\n  ".join(lines)


def assert_agrees(python_value, lua_value, tolerance=None, what="value",
                  default_tolerance=None):
    """Raise ``AssertionError`` naming every divergence, or return silently.

    The message carries the field path, both values at full precision and the
    tolerance, because the point of a differential test that fails is to say
    *where*, not that something is wrong somewhere.
    """
    divergences = compare(python_value, lua_value, tolerance, what,
                          default_tolerance)
    if divergences:
        raise AssertionError(
            "Lua and Python disagree on %s:\n  %s"
            % (what, format_divergences(divergences)))


# --------------------------------------------------------------------------
# Runtime semantics — the Tranche 0 record
# --------------------------------------------------------------------------

def runtime_semantics(sandbox):
    """Measure the numeric semantics Tranche 0 requires to be recorded, not assumed.

    Lua 5.3 introduced an integer subtype, which changes division, comparison and
    formatting; and `IR-004`/`A-SW-005` assume ``math.atan(y, x)`` behaves as
    ``atan2``, which is *Provisional* until measured. Both are measured here.

    Returns a plain dict, so it can be printed into a task's evidence section.
    """
    return {
        "version": sandbox.eval("_VERSION"),
        "has_integer_subtype": sandbox.eval("math.type ~= nil"),
        "int_type": sandbox.eval("math.type and math.type(1) or 'n/a'"),
        "float_type": sandbox.eval("math.type and math.type(1.0) or 'n/a'"),
        "true_division_7_2": sandbox.eval("7/2"),
        "floor_division_7_2": sandbox.eval("7//2"),
        "atan_two_arg_quadrant_ii": sandbox.eval("math.atan(1.0, -1.0)"),
        "atan_two_arg_quadrant_iii": sandbox.eval("math.atan(-1.0, -1.0)"),
        "fmod_negative": sandbox.eval("math.fmod(-1.0, 2.0)"),
        "modulo_negative": sandbox.eval("-1.0 % 2.0"),
        "huge_is_inf": sandbox.eval("math.huge > 0 and math.huge == math.huge * 2"),
    }
