"""MAVSDK drone controller — all MAVSDK imports are confined to this module.

Translates the pure RobotCommand domain type to MAVSDK's VelocityBodyYawspeed
internally. No other module needs to import mavsdk. The legacy
VelocityCommand / VelocityCommandAPI / live_control_loop / run_live_drone
were deleted in Phase 3 plan 03-07 — MavsdkDroneAdapter (driven by
robot_api.orchestrator.run_robot_loop) is the sole production path.
"""

import asyncio
import logging
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Optional
from urllib.parse import urlparse

import mavsdk
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.telemetry import FlightMode

from robot_follow.follow_api.types import (
    Axis,
    Capabilities,
    Detection,
    RobotCommand,
    SafetyContext,
)
from robot_follow.follow_api.config import ControllerConfig
from robot_follow.robot_api import command_log

LOGGER = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Robot-protocol capabilities (Q8 lock: lives here, NOT in follow_api/types.py)
# ---------------------------------------------------------------------------

# Drone-axis capability constant — declared here per Q8 lock
# (CONTEXT 2026-05-17). follow_api/types.py stays types-only;
# per-adapter constants live with their adapter.
DRONE_CAPS: Capabilities = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW, Axis.ALTITUDE}),
    yaw_unit="deg/s",
)


# ---------------------------------------------------------------------------
# SmoothingState — per-adapter EMA / slew-rate filter state
# ---------------------------------------------------------------------------

@dataclass
class SmoothingState:
    """Per-adapter smoothing/slew state.

    Mutated in place by _apply_smoothing. Lives on the
    MavsdkDroneAdapter instance; recreated on shutdown/restart.
    """
    filtered_yaw: float = 0.0
    filtered_forward: float = 0.0
    filtered_down: float = 0.0
    prev_forward: float = 0.0


# ---------------------------------------------------------------------------
# Pure-function extracts (R5): altitude P, retreat-from-tilt, smoothing,
# search-yawspeed. Module-level so they are testable without instantiating
# the adapter or talking to MAVSDK.
# ---------------------------------------------------------------------------

def _apply_altitude_p(
    down_m_s: float,
    altitude_cache: dict,
    config: ControllerConfig,
) -> float:
    """Altitude-hold P correction on the down axis.

    Behavior byte-equivalent to live_control_loop's altitude block
    (mavsdk_drone.py lines 509-524). Pure: no I/O, no asyncio, no MAVSDK.

    Returns the (possibly clamped) down command:
      - No altitude reading available → passthrough input.
      - yaw_only mode → passthrough input.
      - At or below floor with positive down (descend) → 0.
      - At or above ceiling with negative down (climb) → 0.
      - Otherwise: down = kp_alt_hold * (current_alt - target_altitude),
        clamped to [-max_climb_speed, max_down_speed].
    """
    current_alt = altitude_cache.get("m")
    if current_alt is None:
        return down_m_s
    if config.yaw_only:
        down = down_m_s
    else:
        alt_err = current_alt - config.target_altitude  # +ve = too high
        down = config.kp_alt_hold * alt_err
        down = max(-config.max_climb_speed, min(config.max_down_speed, down))
    if current_alt <= config.min_altitude and down > 0:
        down = 0.0
    elif current_alt >= config.max_altitude and down < 0:
        down = 0.0
    return down


def _apply_retreat_from_tilt(
    forward_m_s: float,
    safety_ctx: SafetyContext,
    config: ControllerConfig,
) -> float:
    """Drone-specific bottom/top frame-edge fade + safety push + deadband.

    Lifted from controller._apply_frame_edge_safety (the legacy
    pre-Phase-3 controller body) but reads bbox_top / bbox_bottom from
    SafetyContext rather than Detection. The legacy controller applied
    a final ``|forward| < forward_velocity_deadband → 0`` step AFTER
    the fade/push gradient — that step lives here in the adapter now
    so the order (raw → fade/push → deadband) stays equivalent.
    Behavior byte-equivalent to the pre-rewrite path.

    Q6 lock: when ``safety_ctx.target_lost == True``, returns
    ``forward_m_s`` unchanged. Sentinel bbox values in the lost
    SafetyContext are intentionally not inspected.
    """
    if safety_ctx.target_lost:
        return forward_m_s
    if config.yaw_only:
        return forward_m_s

    # Emergency safety: when the bbox is past the panic threshold the
    # controller already emitted -max_backward. The fade/push gradient
    # MUST NOT overwrite that — the legacy compute_velocity_command
    # returned VelocityCommand(-max_backward, 0, yawspeed) and never
    # called _apply_frame_edge_safety in this branch. Bypassing the
    # gradient + deadband here preserves that semantics; the test
    # ``TestApplyRetreatFromTilt.test_emergency_safety_skipped`` locks
    # this behavior.
    if (config.max_bbox_height_safety is not None
            and safety_ctx.bbox_size_normalized > config.max_bbox_height_safety):
        return forward_m_s

    bbox_bottom = safety_ctx.bbox_bottom_normalized
    # SafetyContext stores bottom + size; reconstruct top as
    # bottom - size to match the original controller math
    # (top = center_y - height/2 = bottom - height).
    bbox_top = bbox_bottom - safety_ctx.bbox_size_normalized

    forward = forward_m_s

    if config.bottom_margin_safety > 0:
        margin = config.bottom_margin_safety
        # Fade positive (approach) natural command across [1-2m, 1-m].
        if forward > 0:
            fade_depth = bbox_bottom - (1.0 - 2.0 * margin)
            if fade_depth > 0:
                fade = max(0.0, 1.0 - fade_depth / margin)
                forward *= fade
        # Safety push inside the margin.
        depth = bbox_bottom - (1.0 - margin)
        if depth > 0:
            ratio = min(depth / margin, 1.0)
            forward = min(forward, -ratio * config.max_backward)

    if config.top_margin_safety > 0:
        margin = config.top_margin_safety
        # Fade negative (retreat) natural command across [m, 2m].
        if forward < 0:
            fade_depth = (2.0 * margin) - bbox_top
            if fade_depth > 0:
                fade = max(0.0, 1.0 - fade_depth / margin)
                forward *= fade
        # Safety push inside the margin.
        depth = margin - bbox_top
        if depth > 0:
            ratio = min(depth / margin, 1.0)
            forward = max(forward, ratio * config.max_forward)

    # Post-fade deadband (matches legacy controller's final step). Applied
    # here in the adapter so the controller can stay robot-agnostic (rover
    # doesn't need a forward-velocity deadband tied to drone tilt physics).
    if abs(forward) < config.forward_velocity_deadband:
        forward = 0.0

    return forward


def _apply_smoothing(
    cmd: RobotCommand,
    state: SmoothingState,
    config: ControllerConfig,
) -> RobotCommand:
    """Per-axis clamp → per-axis EMA → forward-axis slew-rate cap.

    Mutates ``state`` in place (filtered_* + prev_forward). Returns a
    NEW RobotCommand with the clamped/filtered values. Behavior
    byte-equivalent to VelocityCommandAPI.send (mavsdk_drone.py:130-181).
    """
    # Clamp each axis to configured maximums
    forward = max(-config.max_backward, min(config.max_forward, cmd.forward_m_s))
    down = max(-config.max_down_speed, min(config.max_down_speed, cmd.down_m_s))
    yaw_raw = max(-config.max_yawspeed, min(config.max_yawspeed, cmd.yaw_rate))

    # Per-axis EMA filtering
    if config.smooth_forward:
        state.filtered_forward = (
            config.forward_alpha * forward
            + (1.0 - config.forward_alpha) * state.filtered_forward
        )
        forward = state.filtered_forward
    else:
        state.filtered_forward = forward

    if config.smooth_down:
        state.filtered_down = (
            config.down_alpha * down
            + (1.0 - config.down_alpha) * state.filtered_down
        )
        down = state.filtered_down
    else:
        state.filtered_down = down

    if config.smooth_yaw:
        state.filtered_yaw = (
            config.yaw_alpha * yaw_raw
            + (1.0 - config.yaw_alpha) * state.filtered_yaw
        )
        yaw_out = state.filtered_yaw
    else:
        state.filtered_yaw = yaw_raw
        yaw_out = yaw_raw

    # Forward-axis slew-rate cap (after EMA): hard m/s² bound on |Δforward/Δt|.
    # Tames PX4 pitch transients on target acquisition / abrupt distance changes.
    # Independent of cfg.max_forward and of cfg.forward_alpha.
    if config.max_forward_accel > 0 and config.control_loop_hz > 0:
        max_step = config.max_forward_accel / config.control_loop_hz
        delta = forward - state.prev_forward
        if delta > max_step:
            forward = state.prev_forward + max_step
        elif delta < -max_step:
            forward = state.prev_forward - max_step
    state.prev_forward = forward
    # Keep the EMA filter state in sync so it doesn't fight the slew limiter
    state.filtered_forward = forward

    return RobotCommand(forward_m_s=forward, yaw_rate=yaw_out, down_m_s=down)


def _compute_search_yawspeed(
    last_detection: Optional[Detection],
    config: ControllerConfig,
) -> float:
    """Yaw speed for the drone's search-mode spin.

    Direction follows the last seen side: ``last_detection.center_x > 0.5``
    → spin right (+); else spin left (−). Magnitude:
    ``config.search_yawspeed_slow``. No last detection → default right
    (positive), matching controller.compute_velocity_command's
    ``search_direction = 1.0`` default.
    """
    if last_detection is None:
        return config.search_yawspeed_slow
    sign = 1.0 if last_detection.center_x > 0.5 else -1.0
    return sign * config.search_yawspeed_slow


def add_drone_args(parser) -> None:
    """Register drone connection and flight-lifecycle CLI flags on *parser*."""
    group = parser.add_argument_group("drone-connection")

    group.add_argument("--connection", default="udpin://0.0.0.0:14540",
                       help="MAVLink connection string (default: udpin://0.0.0.0:14540)")
    group.add_argument("--serial", nargs="?", const="/dev/ttyACM0", default=None,
                       metavar="DEVICE",
                       help="Connect to CubeOrange via serial cable instead of UDP. "
                            "Optionally specify device path (default: /dev/ttyACM0)")
    group.add_argument("--serial-baud", type=int, default=57600,
                       help="Baud rate for serial connection (default: 57600)")
    group.add_argument("--takeoff-landing", action="store_true",
                       help="Enable auto arm/takeoff/land (default: off — drone must already be airborne)")
    group.add_argument("--auto-offboard", action="store_true",
                       help="Activate OFFBOARD mode programmatically at startup (drone.offboard.start()). "
                            "Default: off — stream zero setpoints and wait for the operator to switch "
                            "to OFFBOARD via the GCS. Manual handshake is the safer choice for real-drone "
                            "flights; auto-offboard is appropriate for SITL / bench tests.")
    group.add_argument("--target-altitude", type=float, default=3.0,
                       help="Target altitude in metres (default: 3.0). Also used as takeoff height with --takeoff-landing.")
    group.add_argument("--mission-duration", type=float, default=300.0,
                       help="Maximum mission duration in seconds. Default 300.0 (5 min). "
                            "With --takeoff-landing: triggers an automatic land at expiry. "
                            "Without --takeoff-landing: triggers a control-loop restart "
                            "(the app keeps running; the loop reconnects). "
                            "Set to a large value (e.g. 86400) for unbounded missions.")
    command_log.add_command_log_args(group)


# ---------------------------------------------------------------------------
# mavsdk_server process management
# ---------------------------------------------------------------------------

def _reap_mavsdk_server() -> None:
    """Kill any straggler mavsdk_server processes scoped to current uid.

    Used in two places:
      1. DetachedMavsdkServer.__enter__ — reap stale server from a prior run
         before binding UDP 14540 / TCP 50051 for a fresh start.
      2. robot_follow_app.py finally-block — reap a survivor when the drone
         thread hangs (typically a MAVSDK land/offboard timeout against a
         dead sim) and __exit__ never runs.

    Scoped to the current uid so shared hosts don't see cross-user kills.
    """
    try:
        subprocess.run(
            ["pkill", "-9", "-u", str(os.getuid()), "-f", "mavsdk_server"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        pass


# ---------------------------------------------------------------------------
# Detached MAVSDK Server (for graceful shutdown)
# ---------------------------------------------------------------------------

class DetachedMavsdkServer:
    """
    Manages a mavsdk_server process that is detached from the current session,
    so it doesn't die on Ctrl+C (SIGINT). This allows the Python script to
    catch SIGINT and perform a graceful landing sequence using the server.
    """
    def __init__(self, connection_url, port=50051):
        self.connection_url = connection_url
        self.port = port
        self.process = None

    def _grpc_address_from_connection(self):
        """Derive gRPC address from connection URL (host from connection, port from self.port)."""
        try:
            parsed = urlparse(self.connection_url)
            host = (parsed.hostname or "127.0.0.1").strip() or "127.0.0.1"
            if host == "0.0.0.0":
                host = "127.0.0.1"
            return f"grpc://{host}:{self.port}"
        except (ValueError, AttributeError):
            return f"grpc://127.0.0.1:{self.port}"

    def __enter__(self):
        # If already using grpc, no need to start a server
        if self.connection_url.startswith("grpc://"):
            return self.connection_url

        # Try to find mavsdk_server binary
        try:
            server_path = os.path.join(os.path.dirname(mavsdk.__file__), 'bin', 'mavsdk_server')
        except (AttributeError, TypeError):
            server_path = None

        if not server_path or not os.path.exists(server_path):
            LOGGER.warning("[drone] mavsdk_server not found at %s, using default System() behavior", server_path)
            return self.connection_url  # Fallback to default behavior

        # Reap any stale mavsdk_server from a prior run. start_new_session=True
        # means the server survives Ctrl+C, and if our previous shutdown timed
        # out (e.g. the drone_thread join expired during a stuck land/offboard
        # call), __exit__ never ran. The leftover keeps UDP 14540 + TCP 50051
        # bound, which blocks the next run from connecting to PX4.
        _reap_mavsdk_server()
        # Brief settle so the OS releases the bound ports before we respawn.
        time.sleep(0.3)

        cmd = [server_path, "-p", str(self.port), self.connection_url]
        LOGGER.info("[drone] Starting detached mavsdk_server: %s", " ".join(cmd))

        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        # Wait for server to start and verify it's still running
        time.sleep(1.0)
        if self.process.poll() is not None:
            LOGGER.warning("[drone] Detached mavsdk_server exited immediately (code=%s), "
                           "falling back to direct connection", self.process.returncode)
            self.process = None
            return self.connection_url
        return self._grpc_address_from_connection()

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()


# ---------------------------------------------------------------------------
# Offboard mode helpers
# ---------------------------------------------------------------------------

async def _wait_for_offboard_mode(drone: System, shutdown: asyncio.Event) -> None:
    """Block until the drone enters OFFBOARD mode, streaming zero setpoints as keep-alive.

    In the default (no --takeoff-landing) mode the user switches to OFFBOARD externally (e.g. via
    a GCS).  We stream zero-velocity setpoints so PX4 accepts the transition, and
    wait patiently instead of killing the process.
    """
    zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    setpoint_period = 0.05

    async def _stream_setpoints():
        while not shutdown.is_set():
            try:
                await drone.offboard.set_velocity_body(zero)
            except (OffboardError, ConnectionError):
                pass
            await asyncio.sleep(setpoint_period)

    async def _watch_for_offboard():
        async for mode in drone.telemetry.flight_mode():
            if shutdown.is_set():
                return
            if mode == FlightMode.OFFBOARD:
                LOGGER.info("[drone] OFFBOARD mode detected.")
                return
            LOGGER.info("[drone] Current mode: %s -- waiting for OFFBOARD...", mode.name)

    setpoint_task = asyncio.create_task(_stream_setpoints())
    watch_task = asyncio.create_task(_watch_for_offboard())
    shutdown_task = asyncio.create_task(shutdown.wait())
    try:
        LOGGER.info("[drone] Waiting for OFFBOARD mode (switch via GCS)...")
        done, pending = await asyncio.wait(
            [watch_task, shutdown_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
        for t in pending:
            await _cancel_task(t)
    finally:
        await _cancel_task(setpoint_task)


async def _watch_offboard_mode(drone: System, shutdown: asyncio.Event,
                               offboard_lost: asyncio.Event) -> None:
    """Background task: set *offboard_lost* when flight mode leaves OFFBOARD."""
    async for mode in drone.telemetry.flight_mode():
        if shutdown.is_set():
            return
        if mode != FlightMode.OFFBOARD:
            LOGGER.warning("[drone] Drone left OFFBOARD mode (current: %s). "
                           "Pausing control loop, waiting for OFFBOARD again...", mode.name)
            offboard_lost.set()
            return


def _print_connection_error(prefix: str, e: Exception, hint: bool = False) -> None:
    """Print a short message when failure is due to lost connection (e.g. sim closed)."""
    msg = str(e).lower()
    if "unavailable" in msg or "connection refused" in msg or "connection reset" in msg:
        LOGGER.warning("%s: connection lost (sim or MAVSDK backend closed).", prefix)
        if hint:
            LOGGER.warning("[drone] Tip: press Ctrl+C once and wait for landing before closing the sim.")
    else:
        LOGGER.warning("%s: %s", prefix, e)


def _ignore_sigint_during_landing(ignore: bool) -> None:
    """Ignore or restore SIGINT so a second Ctrl+C does not kill the process during landing."""
    try:
        if ignore:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        else:
            signal.signal(signal.SIGINT, signal.SIG_DFL)
    except (ValueError, OSError):
        pass  # signal only works in main thread; ignore


# ---------------------------------------------------------------------------
# Live Control Loop
# ---------------------------------------------------------------------------

async def _telemetry_velocity_task(drone, telemetry_cache: dict, shutdown: asyncio.Event) -> None:
    """Background task: stream velocity NED and store in telemetry_cache."""
    try:
        async for vel in drone.telemetry.velocity_ned():
            if shutdown.is_set():
                return
            telemetry_cache["vel_north"] = vel.north_m_s
            telemetry_cache["vel_east"] = vel.east_m_s
            telemetry_cache["vel_down"] = vel.down_m_s
    except (ConnectionError, asyncio.CancelledError):
        pass


async def _telemetry_position_task(drone, telemetry_cache: dict, altitude_cache: dict,
                                   shutdown: asyncio.Event) -> None:
    """Background task: stream position once, write both telemetry + altitude caches.

    Single subscription on the MAVSDK position stream that fans out into:
      - telemetry_cache: lat/lon/abs_alt/rel_alt (used by the 1 Hz telem log task)
      - altitude_cache: 'm' (relative altitude, used by live_control_loop's
        altitude-hold P-loop)

    Both dicts carry pos.relative_altitude_m so existing consumers stay
    untouched; cache consolidation (Shape B in 02-RESEARCH § CLEAN-13) is
    deferred to Phase 3.
    """
    try:
        async for pos in drone.telemetry.position():
            if shutdown.is_set():
                return
            telemetry_cache["lat"] = pos.latitude_deg
            telemetry_cache["lon"] = pos.longitude_deg
            telemetry_cache["abs_alt"] = pos.absolute_altitude_m
            telemetry_cache["rel_alt"] = pos.relative_altitude_m
            altitude_cache["m"] = pos.relative_altitude_m
    except (ConnectionError, asyncio.CancelledError):
        pass


async def _telemetry_log_task(drone, altitude_cache: dict, telemetry_cache: dict,
                               shutdown: asyncio.Event, ui_state=None) -> None:
    """Background task: log drone telemetry at 1 Hz for flight debugging."""
    telem_logger = logging.getLogger("robot_follow.telemetry")
    interval = 1.0
    while not shutdown.is_set():
        await asyncio.sleep(interval)
        alt = altitude_cache.get("m")
        rel_alt = telemetry_cache.get("rel_alt")
        vn = telemetry_cache.get("vel_north")
        ve = telemetry_cache.get("vel_east")
        vd = telemetry_cache.get("vel_down")
        if alt is None and vn is None:
            continue
        parts = []
        if rel_alt is not None:
            parts.append(f"alt={rel_alt:.2f}m")
        if vn is not None:
            horiz_speed = math.sqrt(vn**2 + ve**2)
            parts.append(f"Vn={vn:+.2f} Ve={ve:+.2f} Vd={vd:+.2f} hSpd={horiz_speed:.2f}m/s")
        lat = telemetry_cache.get("lat")
        lon = telemetry_cache.get("lon")
        if lat is not None:
            parts.append(f"pos=({lat:.6f},{lon:.6f})")
        msg = "[TELEM] " + " | ".join(parts)
        # DEBUG-level — per-tick telemetry spams default INFO. Operators
        # opt in via `--log-verbosity debug` when they actually want it.
        telem_logger.debug(msg)
        if ui_state is not None:
            ui_state.push_log(msg)


# ---------------------------------------------------------------------------
# MavsdkDroneAdapter — Robot protocol implementation
# ---------------------------------------------------------------------------
# This is the sole production drone path. Driven by
# orchestrator.run_robot_loop and constructed in
# robot_follow_app.run_drone (which wraps the loop with the
# args.mission_duration deadline per B1).
# ---------------------------------------------------------------------------


class MavsdkDroneAdapter:
    """MAVSDK drone implementation of the Robot protocol.

    Wraps DetachedMavsdkServer + MAVSDK System + telemetry tasks
    + offboard handshake. ``send_command`` applies the 4 pure functions
    (altitude P, retreat-from-tilt, smoothing) and forwards to
    ``drone.offboard.set_velocity_body``.

    Q6 lock: ``send_command`` short-circuits when
    ``safety_ctx.target_lost`` is True. The orchestrator drives
    search-mode behavior via ``on_target_lost``.

    Phase 3 plan 03-07 atomically swapped out the legacy production
    path (live_control_loop + VelocityCommandAPI) in favor of this
    adapter driven by ``orchestrator.run_robot_loop``.
    """

    caps: Capabilities = DRONE_CAPS  # class-level for type-checker

    def __init__(self, args, config: ControllerConfig):
        self.caps = DRONE_CAPS  # per CONTEXT instance-attr rule
        self._args = args
        self._config = config
        self._drone: Optional[System] = None
        self._mavsdk_server_cm: Optional[DetachedMavsdkServer] = None
        self._connection_url: Optional[str] = None
        self._smoothing = SmoothingState()
        self._telemetry_cache: dict = {}
        self._altitude_cache: dict = {}
        self._telemetry_tasks: list = []
        # Lazy-initialised in start_session to avoid binding to the
        # wrong event loop when the adapter is constructed in tests.
        self._shutdown_event: Optional[asyncio.Event] = None
        # True while PX4 is in OFFBOARD. send_command and on_target_lost
        # gate on this so the wire stays idle when the pilot has manual
        # control. Only the manual-handshake path spawns the monitor that
        # flips this flag; the auto-offboard path leaves it True.
        self._offboard_active: bool = True
        self._offboard_monitor_task: Optional[asyncio.Task] = None

    async def connect(self) -> None:
        """Open MAVSDK gRPC. Raises ConnectionError on timeout.

        Wraps DetachedMavsdkServer + drone.connect + _wait_for_connection
        (15 s timeout). Idempotent: if already connected, no-op.
        """
        if self._drone is not None:
            return
        self._mavsdk_server_cm = DetachedMavsdkServer(self._args.connection)
        connection_url = self._mavsdk_server_cm.__enter__()
        self._connection_url = connection_url
        if connection_url.startswith("grpc://"):
            parsed = urlparse(connection_url)
            drone = System(
                mavsdk_server_address=parsed.hostname or "127.0.0.1",
                port=parsed.port or 50051,
            )
            await drone.connect()
        else:
            drone = System()
            await drone.connect(system_address=connection_url)
        try:
            connected = await asyncio.wait_for(
                _wait_for_connection(drone), timeout=_CONNECTION_TIMEOUT_S,
            )
        except asyncio.TimeoutError:
            connected = False
        if not connected:
            raise ConnectionError(
                f"No drone detected on {self._args.connection} after "
                f"{_CONNECTION_TIMEOUT_S}s. Pipeline continues without drone control."
            )
        self._drone = drone

    async def start_session(self) -> None:
        """Spawn telemetry tasks, run offboard handshake.

        Raises ``RuntimeError`` if called before ``connect()`` succeeded.
        The offboard handshake (``_start_offboard``) streams zero
        VelocityBodyYawspeed setpoints directly to ``self._drone``;
        the legacy VelocityCommandAPI helper was deleted in 03-07.
        """
        if self._drone is None:
            raise RuntimeError("start_session called before successful connect")
        if self._shutdown_event is None:
            self._shutdown_event = asyncio.Event()
        # Spawn telemetry tasks. The position task fans out into both
        # telemetry_cache and altitude_cache (CLEAN-13: single position
        # subscription, two consumers).
        self._telemetry_tasks.append(asyncio.create_task(
            _telemetry_velocity_task(
                self._drone, self._telemetry_cache, self._shutdown_event,
            )
        ))
        self._telemetry_tasks.append(asyncio.create_task(
            _telemetry_position_task(
                self._drone, self._telemetry_cache, self._altitude_cache,
                self._shutdown_event,
            )
        ))
        # Optional arm+takeoff when --takeoff-landing is set.
        if getattr(self._args, "takeoff_landing", False):
            await self._drone.action.set_takeoff_altitude(self._args.target_altitude)
            for attempt in range(_ARM_MAX_ATTEMPTS):
                if self._shutdown_event.is_set():
                    return
                try:
                    await self._drone.action.arm()
                    command_log.log_action("arm")
                    break
                except mavsdk.action.ActionError as e:
                    if attempt == _ARM_MAX_ATTEMPTS - 1:
                        raise
                    LOGGER.warning(
                        "[drone] arm() failed (%s), retrying in %ds... (%d/%d)",
                        e, _ARM_RETRY_DELAY_S, attempt + 1, _ARM_MAX_ATTEMPTS - 1,
                    )
                    for _ in range(int(_ARM_RETRY_DELAY_S / _ARM_SHUTDOWN_POLL_S)):
                        if self._shutdown_event.is_set():
                            return
                        await asyncio.sleep(_ARM_SHUTDOWN_POLL_S)
            await self._drone.action.takeoff()
            command_log.log_action("takeoff", altitude_m=self._args.target_altitude)
            await asyncio.sleep(5)
        # OFFBOARD handshake. --auto-offboard tells the app to set the
        # mode itself (SITL / bench). Without it, stream zero setpoints
        # and wait for the operator to flip OFFBOARD on the GCS, then
        # spawn the resilience monitor that pauses send_command if the
        # pilot leaves OFFBOARD mid-flight.
        if getattr(self._args, "auto_offboard", False):
            await _start_offboard(self._drone, self._shutdown_event)
            command_log.log_action("offboard_start")
        else:
            await _wait_for_offboard_mode(self._drone, self._shutdown_event)
            if self._shutdown_event.is_set():
                return
            command_log.log_mode(offboard=True)
            self._offboard_monitor_task = asyncio.create_task(
                self._offboard_resilience_loop())

    async def _offboard_resilience_loop(self) -> None:
        """Pause control while the pilot is in manual mode, resume when OFFBOARD returns.

        When the flight mode leaves OFFBOARD mid-flight (RC override,
        GCS mode switch), clear _offboard_active so send_command and
        on_target_lost short-circuit. When OFFBOARD comes back, reset
        the smoothing filter and re-arm the flag so the next command
        starts from a clean state.
        """
        assert self._shutdown_event is not None
        assert self._drone is not None
        while not self._shutdown_event.is_set():
            offboard_lost = asyncio.Event()
            watcher = asyncio.create_task(
                _watch_offboard_mode(self._drone, self._shutdown_event, offboard_lost))
            try:
                done, _pending = await asyncio.wait(
                    [asyncio.create_task(offboard_lost.wait()),
                     asyncio.create_task(self._shutdown_event.wait())],
                    return_when=asyncio.FIRST_COMPLETED,
                )
            finally:
                await _cancel_task(watcher)
            if self._shutdown_event.is_set():
                return
            # OFFBOARD lost; stop driving the wire until pilot flips back.
            self._offboard_active = False
            command_log.log_mode(offboard=False)
            await _wait_for_offboard_mode(self._drone, self._shutdown_event)
            if self._shutdown_event.is_set():
                return
            # Re-acquired. Reset smoothing so we don't carry stale state
            # into the new control session.
            self._smoothing = SmoothingState()
            self._offboard_active = True
            command_log.log_mode(offboard=True)

    async def send_command(
        self,
        cmd: RobotCommand,
        safety_ctx: SafetyContext,
    ) -> None:
        """Per-tick command. Applies altitude P + retreat-from-tilt + smoothing.

        Q6 lock: early-returns when ``safety_ctx.target_lost`` is True.
        OFFBOARD-loss: early-returns when self._offboard_active is False
        (pilot has manual control); keeps the wire idle until the pilot
        re-engages OFFBOARD.
        """
        if safety_ctx.target_lost:
            return
        if not self._offboard_active:
            return  # pilot has manual control; do not fight them
        if self._drone is None:
            return  # connect failed; silently skip
        down = _apply_altitude_p(cmd.down_m_s, self._altitude_cache, self._config)
        forward = _apply_retreat_from_tilt(cmd.forward_m_s, safety_ctx, self._config)
        staged = RobotCommand(
            forward_m_s=forward,
            yaw_rate=cmd.yaw_rate,
            down_m_s=down,
        )
        smoothed = _apply_smoothing(staged, self._smoothing, self._config)
        vbys = VelocityBodyYawspeed(
            smoothed.forward_m_s,
            0.0,  # right_m_s — drone is non-holonomic
            smoothed.down_m_s,
            smoothed.yaw_rate,  # already deg/s per DRONE_CAPS.yaw_unit (Q5 lock)
        )
        await self._drone.offboard.set_velocity_body(vbys)
        command_log.log_velocity(vbys.forward_m_s, vbys.right_m_s, vbys.down_m_s,
                                 vbys.yawspeed_deg_s, cmd=cmd)

    async def send_zero(self) -> None:
        """Quiescent zero setpoint + reset smoothing state.

        Called once in the orchestrator's finally block on shutdown.
        """
        self._smoothing = SmoothingState()
        if self._drone is None:
            return
        await self._drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        command_log.log_zero()

    async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
        """Drone's lost-target behavior: yaw-spin in last bbox direction."""
        if not self._offboard_active:
            return  # pilot has manual control; do not fight them
        yawspeed_deg_s = _compute_search_yawspeed(last_detection, self._config)
        if self._drone is None:
            return
        await self._drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, yawspeed_deg_s)
        )
        command_log.log_search(yawspeed_deg_s)

    async def shutdown(self) -> None:
        """Idempotent. Cancel telemetry, land if armed, exit context."""
        if self._shutdown_event is not None:
            self._shutdown_event.set()
        if self._offboard_monitor_task is not None:
            await _cancel_task(self._offboard_monitor_task)
            self._offboard_monitor_task = None
        for task in self._telemetry_tasks:
            await _cancel_task(task)
        self._telemetry_tasks.clear()
        if (
            getattr(self._args, "takeoff_landing", False)
            and self._drone is not None
        ):
            command_log.log_action("land")
            try:
                await _land_safely(self._drone)
            except Exception:
                pass  # logged inside _land_safely
        if self._mavsdk_server_cm is not None:
            try:
                self._mavsdk_server_cm.__exit__(None, None, None)
            except Exception:
                pass
        self._drone = None
        self._mavsdk_server_cm = None



# ---------------------------------------------------------------------------
# Offboard start / land / cancel helpers
# ---------------------------------------------------------------------------

async def _start_offboard(drone: System, shutdown: asyncio.Event) -> None:
    """Stream zero setpoints then start offboard mode with retries.

    PX4 requires setpoints to be streamed before offboard.start()
    (NO_SETPOINT_SET otherwise). Streams at ~20 Hz for 2 s, then
    retries offboard.start() up to 3 times.

    Operates on the MAVSDK drone directly — no VelocityCommandAPI
    (deleted in 03-07). The zero setpoint is sent via the same
    set_velocity_body interface MavsdkDroneAdapter.send_command uses.
    """
    zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    setpoint_period_s = 0.05

    for _ in range(int(2.0 / setpoint_period_s)):
        if shutdown.is_set():
            return
        try:
            await drone.offboard.set_velocity_body(zero)
        except (OffboardError, ConnectionError):
            pass
        await asyncio.sleep(setpoint_period_s)

    max_retries = 3
    for attempt in range(max_retries):
        try:
            await drone.offboard.start()
            return
        except OffboardError as e:
            if attempt == max_retries - 1:
                raise
            LOGGER.warning("[drone] Failed to start offboard (%s), retrying...", e)
            for _ in range(int(1.0 / setpoint_period_s)):
                if shutdown.is_set():
                    return
                try:
                    await drone.offboard.set_velocity_body(zero)
                except (OffboardError, ConnectionError):
                    pass
                await asyncio.sleep(setpoint_period_s)


async def _land_safely(drone: System) -> None:
    """Stop offboard mode and land, ignoring SIGINT during the sequence.

    Operates on the MAVSDK drone directly (no VelocityCommandAPI).
    """
    zero = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    try:
        await drone.offboard.set_velocity_body(zero)
        await drone.offboard.stop()
    except Exception as e:
        _print_connection_error("[drone] Offboard stop", e)

    LOGGER.warning("[drone] Landing safely - please wait (ignoring further Ctrl+C until done)...")
    try:
        _ignore_sigint_during_landing(ignore=True)
        LOGGER.info("[drone] Landing...")
        try:
            await drone.action.land()
            await asyncio.sleep(8)
        except Exception as e:
            _print_connection_error("[drone] Land", e)
    finally:
        _ignore_sigint_during_landing(ignore=False)


async def _cancel_task(task: asyncio.Task) -> None:
    """Cancel an asyncio task and suppress CancelledError."""
    task.cancel()
    try:
        await task
    except (asyncio.CancelledError, Exception):
        pass


# ---------------------------------------------------------------------------
# Connection helpers + module constants
# ---------------------------------------------------------------------------

_ARM_MAX_ATTEMPTS = 6
_ARM_RETRY_DELAY_S = 5
_ARM_SHUTDOWN_POLL_S = 0.5
_CONNECTION_TIMEOUT_S = 15


async def _wait_for_connection(drone: System) -> bool:
    """Wait until MAVSDK reports the drone is connected. Returns True on success."""
    async for state in drone.core.connection_state():
        if state.is_connected:
            return True
    return False
