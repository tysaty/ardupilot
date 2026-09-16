"""A minimal ArduPilot DataFlash **writer**, for fixtures (``TASK-052`` P7).

The extractor reads ``.bin`` logs through ``pymavlink.DFReader``. To test it
without a vehicle, a build, or a recorded log (logs are private telemetry and
``*.bin`` is git-ignored per ``AGENTS.md``), the tests synthesise a log from a
Python harness history through this writer and check the extractor gives the
history back. That is the strongest test available: the round trip
``history -> .bin -> bundle`` must be the identity to float32 precision.

Format (``libraries/AP_Logger/README.md``): every message is
``0xA3 0x95 <type>`` followed by the packed fields; message type 128 is the
``FMT`` self-description (``BBnNZ``: type, length, name[4], format[16],
columns[64]). ``logger:write`` from Lua prepends ``TimeUS`` (``Q``) to the
format and labels it gives, and so does :meth:`DataFlashWriter.add`.
"""

import struct

HEAD = b"\xa3\x95"
FMT_TYPE = 128

#: DataFlash format character -> struct format (little-endian, packed).
_STRUCT = {
    "b": "b", "B": "B", "h": "h", "H": "H", "i": "i", "I": "I",
    "f": "f", "d": "d", "n": "4s", "N": "16s", "Z": "64s",
    "c": "h", "C": "H", "e": "i", "E": "I", "L": "i", "M": "B",
    "q": "q", "Q": "Q",
}


def _struct_format(fmt):
    return "<" + "".join(_STRUCT[c] for c in fmt)


def _pad(text, n):
    raw = str(text).encode("ascii")[:n]
    return raw + b"\0" * (n - len(raw))


class DataFlashWriter:
    """Write messages to a ``.bin`` in the order they are added."""

    def __init__(self, path):
        self._handle = open(path, "wb")
        self._types = {}
        self._next_type = 129   # 128 is FMT
        self._fmt_struct = struct.Struct(_struct_format("BBnNZ"))

    def add_format(self, name, labels, fmt, with_time=True):
        """Declare a message. With ``with_time`` the ``TimeUS`` column is
        prepended, as the Lua ``logger:write`` binding does."""
        if with_time:
            labels = "TimeUS," + labels
            fmt = "Q" + fmt
        if len(name) > 4 or len(fmt) > 16 or len(labels) > 64:
            raise ValueError("FMT field too long for %s" % name)
        msg_type = self._next_type
        self._next_type += 1
        packer = struct.Struct(_struct_format(fmt))
        length = 3 + packer.size
        self._types[name] = (msg_type, packer, fmt)
        self._handle.write(HEAD + bytes([FMT_TYPE]) + self._fmt_struct.pack(
            msg_type, length, _pad(name, 4), _pad(fmt, 16), _pad(labels, 64)))
        return msg_type

    def add(self, name, time_us, *values):
        """Write one message; ``time_us`` fills ``TimeUS`` when declared."""
        msg_type, packer, fmt = self._types[name]
        fields = (int(time_us),) + tuple(values) if fmt.startswith("Q") else values
        self._handle.write(HEAD + bytes([msg_type]) + packer.pack(*fields))

    def close(self):
        self._handle.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
