"""Tests for robot_api.command_log — the FC command JSONL log.

Pins:
1. ``emit`` is a no-op when no log file is open.
2. ``--cmd-log`` path resolution (auto / dir / file / off).
3. ``open_from_args`` writes a session-start row; ``close`` a session-stop row.
4. MavsdkDroneAdapter logs exactly what it sends: the smoothed setpoint
   (plus the pre-adapter cmd), zero, search yaw — and nothing when it
   doesn't send (target lost / OFFBOARD inactive).
"""

from __future__ import annotations

import asyncio
import json
import os
from types import SimpleNamespace

import pytest

from robot_follow.follow_api.config import ControllerConfig
from robot_follow.follow_api.types import Detection, RobotCommand, SafetyContext
from robot_follow.robot_api import command_log
from robot_follow.robot_api.adapters.mavsdk_drone import MavsdkDroneAdapter
from robot_follow.robot_api.command_log import CommandKind, CommandLog


@pytest.fixture(autouse=True)
def _isolate_singleton():
    CommandLog._instance = None
    yield
    CommandLog.get().close()
    CommandLog._instance = None


def _read_rows(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def _det(cx=0.5, cy=0.5, bh=0.2):
    return Detection(label="person", confidence=0.9, center_x=cx, center_y=cy,
                     bbox_height=bh, timestamp=0.0)


# ---------------------------------------------------------------------------
# Writer + path resolution
# ---------------------------------------------------------------------------

def test_emit_without_open_is_noop():
    command_log.log_velocity(1.0, 0.0, 0.0, 5.0)
    command_log.log_zero()
    assert not CommandLog.get().is_open


@pytest.mark.parametrize("value", ["off", "OFF", "none", None])
def test_resolve_path_disabled(value):
    assert command_log.resolve_path(value) is None


def test_resolve_path_auto_uses_home(monkeypatch, tmp_path):
    monkeypatch.setenv("HOME", str(tmp_path))
    path = command_log.resolve_path("auto")
    assert path.startswith(os.path.join(str(tmp_path), "fc_command_logs", "fc_commands_"))
    assert path.endswith(".jsonl")


def test_resolve_path_dir_and_file(tmp_path):
    assert command_log.resolve_path(str(tmp_path / "x.jsonl")) == str(tmp_path / "x.jsonl")
    in_dir = command_log.resolve_path(str(tmp_path / "logs"))
    assert os.path.dirname(in_dir) == str(tmp_path / "logs")


def test_open_from_args_writes_session_rows(tmp_path):
    path = str(tmp_path / "sub" / "cmds.jsonl")
    got = command_log.open_from_args(SimpleNamespace(cmd_log=path),
                                     dry_run=True, connection=None)
    assert got == path
    command_log.log_velocity(0.5, 0.0, -0.1, 7.25)
    command_log.close()
    rows = _read_rows(path)
    assert [r["kind"] for r in rows] == ["session", "velocity", "session"]
    assert rows[0] == {**rows[0], "action": "start", "dry_run": True, "connection": None}
    assert rows[1]["forward_m_s"] == 0.5 and rows[1]["yaw_deg_s"] == 7.25
    assert "cmd" not in rows[1]
    assert rows[2]["action"] == "stop"
    assert all(isinstance(r["t"], float) for r in rows)


def test_open_from_args_off_returns_none():
    assert command_log.open_from_args(SimpleNamespace(cmd_log="off")) is None
    assert not CommandLog.get().is_open


# ---------------------------------------------------------------------------
# MavsdkDroneAdapter integration
# ---------------------------------------------------------------------------

class _MockOffboard:
    def __init__(self):
        self.calls: list = []

    async def set_velocity_body(self, vbys):
        self.calls.append(vbys)


def _build_adapter() -> MavsdkDroneAdapter:
    cfg = ControllerConfig(
        yaw_only=False,
        target_altitude=3.0, kp_alt_hold=0.5,
        min_altitude=2.0, max_altitude=4.0,
        max_climb_speed=1.0, max_down_speed=1.5,
        smooth_forward=False, smooth_down=False, smooth_yaw=False,
        max_forward_accel=0.0,
        top_margin_safety=0.0, bottom_margin_safety=0.0,
        max_yawspeed=90.0,
    )
    args = SimpleNamespace(connection="udp://0:0", takeoff_landing=False,
                           target_altitude=cfg.target_altitude)
    adapter = MavsdkDroneAdapter(args, cfg)
    adapter._drone = SimpleNamespace(offboard=_MockOffboard())
    adapter._altitude_cache["m"] = 3.5  # err 0.5 → down 0.25 after altitude P
    return adapter


@pytest.fixture
def log_path(tmp_path):
    path = str(tmp_path / "cmds.jsonl")
    CommandLog.get().open(path)
    return path


def test_adapter_logs_sent_setpoint_and_controller_cmd(log_path):
    adapter = _build_adapter()
    cmd = RobotCommand(forward_m_s=0.8, yaw_rate=20.0, down_m_s=0.0)
    asyncio.run(adapter.send_command(cmd, SafetyContext.from_detection(_det())))
    CommandLog.get().close()
    (row,) = _read_rows(log_path)
    sent = adapter._drone.offboard.calls[0]
    assert row["kind"] == CommandKind.VELOCITY.value
    assert row["forward_m_s"] == pytest.approx(sent.forward_m_s)
    assert row["down_m_s"] == pytest.approx(sent.down_m_s) == pytest.approx(0.25)
    assert row["yaw_deg_s"] == pytest.approx(sent.yawspeed_deg_s)
    assert row["right_m_s"] == 0.0
    assert row["cmd"] == {"forward_m_s": 0.8, "down_m_s": 0.0, "yaw_deg_s": 20.0}


def test_adapter_logs_nothing_when_not_sending(log_path):
    adapter = _build_adapter()
    cmd = RobotCommand(forward_m_s=0.8, yaw_rate=20.0, down_m_s=0.0)
    asyncio.run(adapter.send_command(cmd, SafetyContext.lost(last_target_x=0.7)))
    adapter._offboard_active = False
    asyncio.run(adapter.send_command(cmd, SafetyContext.from_detection(_det())))
    asyncio.run(adapter.on_target_lost(_det(cx=0.8)))
    CommandLog.get().close()
    assert adapter._drone.offboard.calls == []
    assert _read_rows(log_path) == []


def test_adapter_logs_zero_and_search(log_path):
    adapter = _build_adapter()
    asyncio.run(adapter.on_target_lost(_det(cx=0.8)))
    asyncio.run(adapter.send_zero())
    CommandLog.get().close()
    rows = _read_rows(log_path)
    assert [r["kind"] for r in rows] == ["search", "zero"]
    assert rows[0]["yaw_deg_s"] == pytest.approx(adapter._drone.offboard.calls[0].yawspeed_deg_s)
