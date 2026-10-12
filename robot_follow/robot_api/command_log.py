"""JSONL log of every command the app sends to the flight controller.

One line per setpoint / action, written by the drone adapters at the
point where the command goes on the wire (after altitude-P, retreat and
smoothing), so the file shows exactly what the FC received. Works for
every MAVSDK target (H8 / RPi / x86 via robot_follow_app, Hailo15 via
drone_follow_h15) and for the H15 ``--dry-run`` adapter, which logs the
commands it *would* send.

Lifecycle
---------
A process-global singleton, opened once at startup by
:func:`open_from_args` (``--cmd-log``, default on: ``~/fc_command_logs/``)
and closed at exit.
Emits are thread-safe and silently no-op when no log is open, so call
sites don't need to guard.

Wire example (line-delimited JSON)::

    {"t": 1701234567.123, "kind": "session",  "action": "start", "connection": "serial:///dev/ttyACM0:57600", "dry_run": false}
    {"t": ...,             "kind": "action",   "action": "arm"}
    {"t": ...,             "kind": "action",   "action": "offboard_start"}
    {"t": ...,             "kind": "velocity", "forward_m_s": 0.42, "right_m_s": 0.0, "down_m_s": -0.05, "yaw_deg_s": 7.5,
                                               "cmd": {"forward_m_s": 0.5, "down_m_s": 0.0, "yaw_deg_s": 7.5}}
    {"t": ...,             "kind": "search",   "yaw_deg_s": -10.0}
    {"t": ...,             "kind": "mode",     "offboard": false}
    {"t": ...,             "kind": "zero"}
    {"t": ...,             "kind": "action",   "action": "land"}
    {"t": ...,             "kind": "session",  "action": "stop"}

``velocity`` rows carry the sent setpoint (VelocityBodyYawspeed fields)
plus ``cmd``, the controller's command before the adapter's altitude /
retreat / smoothing stages.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from enum import Enum
from typing import Any, Optional

LOGGER = logging.getLogger(__name__)

DEFAULT_LOG_DIR = "~/fc_command_logs"


class CommandKind(str, Enum):
    """Row types written to the command log."""
    SESSION  = "session"   # log/app session start / stop
    VELOCITY = "velocity"  # body-frame velocity + yawspeed setpoint
    SEARCH   = "search"    # lost-target yaw-spin setpoint
    ZERO     = "zero"      # all-zero setpoint (shutdown / quiescent)
    ACTION   = "action"    # arm / takeoff / offboard_start / offboard_stop / land
    MODE     = "mode"      # OFFBOARD lost / regained (pilot took or returned control)


class CommandLog:
    """Thread-safe JSONL writer with a process-global singleton."""

    _instance: Optional["CommandLog"] = None
    _instance_lock = threading.Lock()

    def __init__(self) -> None:
        self._file_lock = threading.Lock()
        self._file = None
        self._path: Optional[str] = None

    @classmethod
    def get(cls) -> "CommandLog":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = CommandLog()
            return cls._instance

    @property
    def path(self) -> Optional[str]:
        return self._path

    @property
    def is_open(self) -> bool:
        return self._file is not None

    def open(self, path: str) -> "CommandLog":
        """Open the log file (append, line-buffered). Closes any existing handle."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with self._file_lock:
            self._close_unlocked()
            self._file = open(path, "a", buffering=1)
            self._path = path
        LOGGER.info("[cmd-log] writing FC command log to %s", path)
        return self

    def close(self) -> None:
        with self._file_lock:
            self._close_unlocked()

    def _close_unlocked(self) -> None:
        if self._file is None:
            return
        try:
            self._file.close()
        except OSError:
            pass
        self._file = None
        self._path = None

    def emit(self, kind: CommandKind | str, **fields: Any) -> None:
        """Write one JSON line; ``t`` (wall clock) and ``kind`` are auto-added.

        No-op when no log is open. Never raises — a lost log handle must
        not take down the control loop.
        """
        kind_str = kind.value if isinstance(kind, CommandKind) else str(kind)
        with self._file_lock:
            if self._file is None:
                return
            row = {"t": round(time.time(), 3), "kind": kind_str, **fields}
            try:
                self._file.write(json.dumps(row) + "\n")
            except (OSError, ValueError, TypeError, AttributeError):
                pass


def _r(v: float) -> float:
    return round(float(v), 4)


# ---------------------------------------------------------------------------
# Site helpers — no-ops when the log is closed
# ---------------------------------------------------------------------------

def log_velocity(forward_m_s: float, right_m_s: float, down_m_s: float, yaw_deg_s: float,
                 cmd: Any = None) -> None:
    """A velocity setpoint was sent. ``cmd`` is the pre-adapter RobotCommand (optional)."""
    fields: dict = {
        "forward_m_s": _r(forward_m_s),
        "right_m_s": _r(right_m_s),
        "down_m_s": _r(down_m_s),
        "yaw_deg_s": _r(yaw_deg_s),
    }
    if cmd is not None:
        fields["cmd"] = {
            "forward_m_s": _r(cmd.forward_m_s),
            "down_m_s": _r(cmd.down_m_s),
            "yaw_deg_s": _r(cmd.yaw_rate),
        }
    CommandLog.get().emit(CommandKind.VELOCITY, **fields)


def log_search(yaw_deg_s: Optional[float]) -> None:
    """Lost-target search setpoint. ``None`` when the yaw isn't computed (dry-run)."""
    CommandLog.get().emit(CommandKind.SEARCH,
                          yaw_deg_s=None if yaw_deg_s is None else _r(yaw_deg_s))


def log_zero() -> None:
    CommandLog.get().emit(CommandKind.ZERO)


def log_action(action: str, **fields: Any) -> None:
    CommandLog.get().emit(CommandKind.ACTION, action=action, **fields)


def log_mode(offboard: bool) -> None:
    CommandLog.get().emit(CommandKind.MODE, offboard=bool(offboard))


# ---------------------------------------------------------------------------
# CLI wiring
# ---------------------------------------------------------------------------

def add_command_log_args(group) -> None:
    """Register ``--cmd-log`` on an argparse group."""
    group.add_argument(
        "--cmd-log", default="auto", metavar="PATH",
        help="Log every command sent to the flight controller as JSONL. "
             f"'auto' (default): {DEFAULT_LOG_DIR}/fc_commands_<timestamp>.jsonl; "
             "a directory: auto-named file inside it; a .jsonl path: that file; "
             "'off': disable.",
    )


def resolve_path(value: Optional[str]) -> Optional[str]:
    """Map the ``--cmd-log`` value to a file path, or None when disabled."""
    if value is None or str(value).lower() in ("off", "none", "no", "false", "0"):
        return None
    name = time.strftime("fc_commands_%Y%m%d_%H%M%S.jsonl")
    if value == "auto":
        value = DEFAULT_LOG_DIR
    value = os.path.expanduser(value)
    if value.endswith(".jsonl"):
        return value
    return os.path.join(value, name)


def open_from_args(args, **session_fields: Any) -> Optional[str]:
    """Open the log per ``args.cmd_log`` and write the session-start row.

    Returns the path, or None when disabled or the file can't be opened
    (logged as a warning — the app keeps running without a command log).
    """
    path = resolve_path(getattr(args, "cmd_log", "auto"))
    if path is None:
        return None
    try:
        log = CommandLog.get().open(path)
    except OSError as e:
        LOGGER.warning("[cmd-log] cannot open %s (%s) — continuing without command log", path, e)
        return None
    log.emit(CommandKind.SESSION, action="start", **session_fields)
    return path


def close() -> None:
    """Write the session-stop row and close the log."""
    log = CommandLog.get()
    log.emit(CommandKind.SESSION, action="stop")
    log.close()
