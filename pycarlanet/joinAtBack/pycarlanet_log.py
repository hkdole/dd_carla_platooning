#!/usr/bin/env python3
"""
pycarlanet_log.py — readable logging for pyCARLANeT.

Design goals:
- No wallclock timestamp
- No log levels
- Multiple colors (consistent by layer)
- No units (numbers only)
- Avoid repetitive filler text: event tags should carry meaning
- Avoid spam: support "print once until reset" for WAITING
- Keep fields human-readable (no cryptic abbreviations in output)

Environment variables:
- PYCARLANET_LOG_COLOR: 1/0 (default 1)
- PYCARLANET_LOG_DEDUPE: 1/0 (default 1)   suppress exact duplicate consecutive lines
- PYCARLANET_LOG_FLUSH: 1/0 (default 1)
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional, Sequence, Set, Tuple, Union

# ---------- ANSI colors (simple names) ----------
red = "\033[31m"
green = "\033[32m"
yellow = "\033[33m"
blue = "\033[34m"
magenta = "\033[35m"
cyan = "\033[36m"
gray = "\033[90m"
bright_yellow  = "\033[93m"
bright_blue    = "\033[94m"
bright_red     = "\033[91m"
bright_green   = "\033[92m"
bright_magenta = "\033[95m"
bright_cyan    = "\033[96m"
reset = "\033[0m"


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name, "1" if default else "0").strip().lower()
    return value in ("1", "true", "yes", "y", "on")


_LOG_COLOR = _env_bool("PYCARLANET_LOG_COLOR", True)
_LOG_DEDUPE = _env_bool("PYCARLANET_LOG_DEDUPE", True)
_LOG_FLUSH = _env_bool("PYCARLANET_LOG_FLUSH", True)

_last_line: Optional[str] = None
_once_keys: Set[str] = set()

# ---------- consistent layer colors (whole line) ----------
_LAYER_COLOR: Dict[str, str] = {
    "ZMQ": cyan,
    "PROTOCOL": green,
    "OMNET": bright_magenta,
    "CARLA": blue,
    "SPAWN": magenta,
    "CONTROL": yellow,
    "BRIDGE": bright_green,
    "STEP": bright_cyan,
    "TELEMETRY": gray,
    "RAW": bright_yellow,
    "SYSTEM": bright_red,
    "ERROR": red,
}

# ---------- field order (what you see first) ----------
_FIELD_ORDER: Tuple[str, ...] = (
    "hop",
    "message_type",
    "msg_type",
    "status",
    "bytes",
    "sequence",
    "source_module",

    "step",
    "simulation_time",
    "timestamp",
    "delta",

    "actor",
    "actors",

    "desired_speed",
    "desired_acceleration",
    "throttle",
    "brake",
    "steering",
    "hold_brake_active",

    "speed",
    "gap_to_leader",
    "relative_speed_to_leader",
    "headway",

    "map_name",
    "carla_fixed_delta",
    "carla_seed",
)


def _is_number(value: Any) -> bool:
    try:
        float(value)
        return True
    except Exception:
        return False


def _format_number(value: Any, *, key: str = "") -> str:
    """
    Compact numeric formatting without units.
    - ints as int
    - floats:
        * throttle/brake/steering: 3 decimals
        * times: 3 decimals
        * everything else: 4 decimals
    """
    try:
        v = float(value)
    except Exception:
        return str(value)

    # Normalize tiny noise to 0 (prevents "-0")
    if abs(v) < 5e-7:
        v = 0.0

    if abs(v - int(v)) < 1e-9:
        return str(int(v))

    if key in ("throttle", "brake", "steering"):
        s = f"{v:.3f}"
    elif key in ("simulation_time", "timestamp", "delta", "headway"):
        s = f"{v:.3f}"
    else:
        s = f"{v:.4f}"

    s = s.rstrip("0").rstrip(".")
    if s == "-0":
        s = "0"
    return s


def _normalize_fields(fields: Dict[str, Any]) -> Dict[str, Any]:
    # No aliases: fields must already use canonical names.
    return dict(fields) if fields else {}


def _format_actors(actor: Optional[str], actors: Optional[Union[str, Sequence[str]]]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    if actor:
        out["actor"] = str(actor).strip()
    if actors:
        if isinstance(actors, str):
            actor_list = [actors.strip()]
        else:
            actor_list = [str(a).strip() for a in actors if str(a).strip()]
        if actor_list:
            out["actors"] = ",".join(actor_list)
    return out


def _format_fields(fields: Dict[str, Any]) -> str:
    if not fields:
        return ""

    ordered_keys = []
    for k in _FIELD_ORDER:
        if k in fields:
            ordered_keys.append(k)
    for k in sorted(fields.keys()):
        if k not in ordered_keys:
            ordered_keys.append(k)

    parts = []
    for k in ordered_keys:
        v = fields.get(k, None)
        if v is None:
            continue

        if isinstance(v, bool):
            text = "1" if v else "0"
        elif _is_number(v):
            text = _format_number(v, key=k)
        else:
            text = str(v)

        parts.append(f"{k}={text}")

    return " " + " ".join(parts) if parts else ""


def _colorize_line(text: str, color_code: Optional[str]) -> str:
    if not _LOG_COLOR or not color_code:
        return text
    return f"{color_code}{text}{reset}"


def reset_once(key: str) -> None:
    _once_keys.discard(str(key))


def log_once(
    key: str,
    layer: str,
    event: str,
    message: str = "",
    *,
    actor: Optional[str] = None,
    actors=None,
    **fields: Any,
) -> None:
    if str(key) in _once_keys:
        return
    _once_keys.add(str(key))
    log(layer, event, message, actor=actor, actors=actors, **fields)


def log(
    layer: str,
    event: str,
    message: str = "",
    *,
    actor: Optional[str] = None,
    actors=None,
    **fields: Any,
) -> None:
    """
    Output:
      [LAYER][EVENT] message key=value key=value
    """
    global _last_line

    layer_text = str(layer or "UNKNOWN").strip().upper()
    event_text = str(event or "EVENT").strip().upper()

    fields = _normalize_fields(fields)
    fields.update(_format_actors(actor, actors))

    tag = f"[{layer_text}][{event_text}]"
    msg = f" {message.strip()}" if message else ""
    field_text = _format_fields(fields)

    line_plain = f"{tag}{msg}{field_text}"

    if _LOG_DEDUPE and _last_line == line_plain:
        return
    _last_line = line_plain

    if event_text in ("ERROR", "FAILED", "EXCEPTION"):
        color = red
    elif event_text == "RAW" or event_text.startswith("RAW_"):
        color = _LAYER_COLOR.get("RAW", bright_blue)
    else:
        color = _LAYER_COLOR.get(layer_text, gray)

    print(_colorize_line(line_plain, color), flush=_LOG_FLUSH)


def banner(layer: str, title: str, **fields: Any) -> None:
    layer_text = str(layer or "SYSTEM").strip().upper()
    title_text = str(title or "").strip()

    fields = _normalize_fields(fields)
    field_text = _format_fields(fields)

    tag = f"[{layer_text}]"
    bar = "=" * max(28, min(90, len(title_text) + 18))

    color = _LAYER_COLOR.get(layer_text, gray)
    print(_colorize_line(f"{tag} {bar}", color), flush=_LOG_FLUSH)
    print(_colorize_line(f"{tag} {title_text}{field_text}", color), flush=_LOG_FLUSH)
    print(_colorize_line(f"{tag} {bar}", color), flush=_LOG_FLUSH)