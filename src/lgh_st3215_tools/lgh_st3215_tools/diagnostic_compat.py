"""Compatibility helpers for ROS 2 diagnostic message fields.

``diagnostic_msgs/msg/DiagnosticStatus.level`` is declared as ROS ``byte``.
On ROS 2 Humble's generated Python bindings that field is exposed as a
single-byte ``bytes`` object, while some newer bindings and test doubles expose
it as an integer.  Normalize both representations here so every LittleGreen
tool uses the same behavior.
"""
from __future__ import annotations

from typing import Any


def diagnostic_level_to_int(level: Any) -> int:
    """Return a DiagnosticStatus level as an integer in the range 0..255.

    Accepts the single-byte ``bytes`` representation used by ROS 2 Humble as
    well as integer-like values used by other bindings and tests.
    """
    if isinstance(level, (bytes, bytearray, memoryview)):
        raw = bytes(level)
        if len(raw) != 1:
            raise ValueError(
                f"DiagnosticStatus.level must contain exactly one byte; got {len(raw)}"
            )
        return raw[0]

    try:
        value = int(level)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"unsupported DiagnosticStatus.level value: {level!r}"
        ) from exc

    if not 0 <= value <= 255:
        raise ValueError(
            f"DiagnosticStatus.level is outside byte range 0..255: {value}"
        )
    return value
