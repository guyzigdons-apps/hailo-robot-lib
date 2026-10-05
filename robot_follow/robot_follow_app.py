#!/usr/bin/env python3
"""
Robot Follow — composition root and CLI entrypoint.

Wires together follow_api (pure domain logic), drone_api (MAVSDK adapter),
and pipeline_adapter (Hailo/GStreamer) into a running application.

The parser is assembled here from each domain's add_*_args() function,
so no module sees arguments it doesn't own.

Usage:
    python robot_follow_app.py --input rpi  # live mode with camera + drone

Pipeline options (--input, --input-codec, etc.) are passed through to the tiling pipeline.
"""

import faulthandler
faulthandler.enable()

import os
os.environ.setdefault("HAILO_MONITOR", "1")

import argparse
import asyncio
import logging
import math
import signal
import sys
import threading
import time
from robot_follow.follow_api import ControllerConfig, SharedDetectionState
from robot_follow.follow_api.state import FollowTargetState
from robot_follow.robot_api.adapters.mavsdk_drone import (
    MavsdkDroneAdapter,
    _reap_mavsdk_server,
    add_drone_args,
)
from robot_follow.robot_api.orchestrator import run_robot_loop
from robot_follow.pipeline_adapter.vision_branches import decide_branches
from robot_follow.servers import FollowServer, OpenHDBridge

LOGGER = logging.getLogger(__name__)

_DEFAULT_REID_HEF = "/usr/local/hailo/resources/models/hailo8/repvgg_a0_person_reid_512.hef"


def _configure_logging(verbosity: str) -> None:
    level = {
        "quiet": logging.WARNING,
        "normal": logging.INFO,
        "debug": logging.DEBUG,
    }.get(verbosity, logging.INFO)
    logging.basicConfig(level=level, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    logging.getLogger().setLevel(level)


def _resolve_serial_connection(args):
    """If --serial is given, override --connection with a serial:// URI."""
    if getattr(args, "serial", None) is not None:
        baud = args.serial_baud
        args.connection = f"serial://{args.serial}:{baud}"
        LOGGER.info("[drone] Serial mode: connection = %s", args.connection)


def _add_app_args(parser: argparse.ArgumentParser) -> None:
    """Register application-level CLI flags (servers, UI).

    Output branches are organised in two orthogonal groups:

    * UI group — outbound network video, mutually exclusive:
        --openhd : RTP H.264 to an OpenHD ground station
        --webui  : MJPEG to the local web UI

    * Local group — on-device output, may coexist:
        --display : X11 window with overlay (tile rectangles stripped, target
                    person's bbox emphasised by class-id remap)
        --record  : pure-GStreamer recording (x264enc -> matroskamux -> filesink)

    If neither --openhd nor --webui is passed, --display defaults to True.

    Note: this helper is invoked from add_common_args(); per ABS-09 it is
    robot-agnostic (every flag here is shared between drone and rover).
    """
    group = parser.add_argument_group("app")

    group.add_argument("--follow-server-port", type=int, default=8080,
                       help="HTTP server port for target selection")

    # --- UI group (mutually exclusive) ---
    group.add_argument("--webui", action="store_true",
                       help="Enable web UI with live MJPEG and clickable bounding boxes")
    group.add_argument("--webui-port", type=int, default=5001,
                       help="Web UI server port (default: 5001)")
    group.add_argument("--webui-fps", type=int, default=10,
                       help="MJPEG stream frame rate (default: 10)")
    group.add_argument("--openhd", action="store_true",
                       help="Send overlay video to OpenHD via UDP RTP (mutually exclusive with --webui)")

    # --- Local group ---
    group.add_argument("--display", action="store_true",
                       help="Show local X11 display window with overlay (tile rectangles "
                            "stripped, target person's bbox emphasised). Default: enabled "
                            "when neither --openhd nor --webui is set; disabled otherwise.")
    group.add_argument("--record", action="store_true",
                       help="Build the pure-GStreamer recording branch. Each session "
                            "writes a bundle dir containing clean.mkv (no overlay — "
                            "feeds the offline renderer in scripts/render_overlay.py) "
                            "and overlay.mkv (operator-visible). Auto-starts on launch; "
                            "can be toggled mid-run from the web UI / OpenHD.")
    group.add_argument("--record-output", type=str, default=None,
                       help="splitmuxsink ``location`` placeholder. The actual paths "
                            "are recordings/<YYYY-MM-DD_HH-MM-SS>/{clean,overlay}.mkv "
                            "under robot_follow/recordings/.")
    group.add_argument("--record-bitrate", type=int, default=5000,
                       help="x264enc bitrate in kbps for both recording branches "
                            "(clean + overlay) (default: 5000)")

    group.add_argument("--log-perf", action="store_true",
                       help="Log pipeline and tracker performance metrics periodically")

    group.add_argument("--test-log", type=str, default=None,
                       help="Write per-frame detection log as JSONL to this path "
                            "(used by simulation tests)")

    # ReID re-identification
    group.add_argument("--reid-model", type=str, default=_DEFAULT_REID_HEF,
                       help="Path to ReID HEF model for appearance-based re-identification "
                            f"(default: {_DEFAULT_REID_HEF}). Use --no-reid to disable.")
    group.add_argument("--no-reid", action="store_true",
                       help="Disable ReID re-identification")
    group.add_argument("--update-interval", type=int, default=10,
                       help="Frames between ReID gallery embedding updates while following (default: 10)")
    group.add_argument("--reid-threshold", type=float, default=0.75,
                       help="Cosine similarity threshold for ReID match (0.0–1.0, default: 0.75)")
    group.add_argument("--reid-timeout", type=float, default=20.0,
                       help="Seconds to search for a lost locked target via ReID before returning "
                            "to auto mode (default: 20.0)")
    group.add_argument("--reid-drift-threshold", type=float, default=0.6,
                       help="Below this similarity vs gallery, an in-track embedding is treated "
                            "as drift; gallery is not updated and re-acquisition is triggered "
                            "(0.0–1.0, default: 0.6)")
    group.add_argument("--reid-duplicate-threshold", type=float, default=0.9,
                       help="Above this similarity, the embedding is redundant and skipped, "
                            "with periodic refresh via --reid-refresh-every (0.0–1.0, default: 0.9)")
    group.add_argument("--reid-refresh-every", type=int, default=5,
                       help="On every Nth consecutive duplicate-band decision, replace the oldest "
                            "gallery vector to keep the gallery fresh (default: 5)")
    group.add_argument("--reid-min-gallery-for-drift-check", type=int, default=6,
                       help="Number of seed embeddings to collect before the drift gate engages. "
                            "Below this, samples are added unconditionally to seed the gallery. "
                            "Lower values catch swaps sooner but make the gallery more brittle "
                            "to a single bad early crop (default: 6)")
    group.add_argument("--reid-bootstrap-consistency", type=float, default=None,
                       help="If set (0.0-1.0), reject a candidate seed during the bootstrap "
                            "window whose similarity to existing seeds is below this floor. "
                            "Catches a wrong-actor crop being baked into the gallery before "
                            "the drift gate engages. No reacquire is triggered — with a tiny "
                            "gallery we can't tell which side is the outlier. "
                            "Suggested value: 0.4 (default: disabled)")
    group.add_argument("--reid-dump-embeddings", type=str, default=None,
                       help="Save every embedding accepted into the ReID gallery for the active "
                            "target to this .npy path at shutdown (used by tests to verify "
                            "all stored embeddings describe the same person)")

    # OpenHD integration
    group.add_argument("--openhd-port", type=int, default=5500,
                       help="OpenHD UDP input port (default: 5500)")
    group.add_argument("--openhd-bitrate", type=int, default=3917,
                       help="H264 encoding bitrate in kbps for OpenHD stream (default: 3917)")


def add_common_args(parser: argparse.ArgumentParser) -> None:
    """Register all CLI flags shared by every Robot type.

    Pipeline-app extensions are NOT registered here — the upstream
    ``hailo_apps.python.core.common.core.get_pipeline_parser()`` creates
    a fresh parser pre-loaded with its own flags, so the pipeline flags
    are installed by ``_build_parser`` before this function is called.
    What lives in this common set:

      - ``ControllerConfig.add_args``  (controller gains, framing, search,
        smoothing, safety — robot-agnostic)
      - ``_add_app_args``              (UI, output branches, ReID, OpenHD,
        recording — all robot-agnostic per ABS-09)
      - ``add_tracker_args``           (tracker choice + tracker tunables)

    Altitude flags (``--min-altitude``, ``--max-altitude``,
    ``--kp-alt-hold``, ``--max-climb-speed``, ``--smooth-down``,
    ``--down-alpha``) stay in this common set per ABS-07: they are
    Optional types in ``ControllerConfig`` and the rover adapter simply
    ignores them.

    This is invoked from ``_build_parser`` BEFORE the robot-specific
    dispatch (``add_drone_args`` or ``add_rover_args``) so the shared
    flags appear identically in ``--robot drone --help`` and
    ``--robot rover --help``.
    """
    from robot_follow.pipeline_adapter import add_tracker_args

    ControllerConfig.add_args(parser)
    _add_app_args(parser)
    add_tracker_args(parser)


def add_rover_args(parser: argparse.ArgumentParser) -> None:
    """Register rover-only CLI flags.

    Lives here (not in ros2_rover.py) so ``--help`` doesn't trigger the
    rover adapter's lazy ``import rclpy``.
    """
    group = parser.add_argument_group("rover-ros2")
    group.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel",
        help="ROS topic to publish geometry_msgs/Twist setpoints on "
             "(default: /cmd_vel).",
    )
    group.add_argument(
        "--ros-namespace",
        default="",
        help="ROS namespace prefix for the rover node (default: empty — "
             "node publishes on the configured --cmd-vel-topic without "
             "namespacing).",
    )
    group.add_argument(
        "--ros-domain-id",
        type=int,
        default=0,
        help="ROS_DOMAIN_ID for network isolation (default: 0). Non-zero "
             "values isolate the rover from other ROS nodes on the same "
             "LAN. Passed to the adapter; rclpy honors ROS_DOMAIN_ID at "
             "init via environment variable.",
    )


def _build_parser() -> argparse.ArgumentParser:
    """Two-pass argparse with ``--robot`` dispatch.

    Pass 1: a tiny pre-parser extracts ``--robot {drone,rover}`` (default
    ``drone``) from argv without consuming any other flags.
    Pass 2: the full parser is assembled with ``add_common_args`` and,
    based on the pre-parsed robot value, either ``add_drone_args`` (from
    ``robot_api.adapters.mavsdk_drone``) or ``add_rover_args``.

    --help shape::

        robot-follow --robot drone --help → common + drone flags
        robot-follow --robot rover --help → common + rover flags only
                                            (Phase 3: rover-specific is empty)

    Common flags (``--webui``, ``--display``, ``--input``, etc.) appear
    under both. Drone-only flags (``--takeoff-landing``,
    ``--target-altitude``, ``--serial``, ``--serial-baud``,
    ``--connection``, ``--mission-duration``) appear ONLY under
    ``--robot drone``.

    Each domain only registers arguments it owns:
      - follow_api:        controller gains, framing, search, smoothing, safety
      - drone_api:         MAVLink connection, flight lifecycle
      - app (this file):   UI/server ports, ReID, output branches
    """
    from hailo_apps.python.core.common.core import get_pipeline_parser

    # Pre-parser pass 1: extract --robot. Uses parse_known_args so it
    # doesn't error on the full argv (which contains many flags it
    # doesn't know about). add_help=False so this pre-parser doesn't
    # consume --help — that's the full parser's job.
    pre_robot = argparse.ArgumentParser(add_help=False)
    pre_robot.add_argument("--robot", choices=("drone", "rover"), default="drone")
    robot_args, _ = pre_robot.parse_known_args()

    # Pass 2: full parser with robot-conditional dispatch. Start from
    # the upstream pipeline parser (which already registers --input,
    # --tiles-x, --hef-path, etc.), then layer the rest on top via
    # ``add_common_args`` and the robot-specific add_*_args.
    parser = get_pipeline_parser()

    add_common_args(parser)
    if robot_args.robot == "drone":
        add_drone_args(parser)  # canonical, from robot_api.adapters.mavsdk_drone
    else:
        add_rover_args(parser)

    # Re-add --robot to the full parser so --help renders it as a known
    # option and the final parse_args() call accepts it. default mirrors
    # the pre-parse so args.robot agrees with whichever branch ran.
    parser.add_argument(
        "--robot",
        choices=("drone", "rover"),
        default=robot_args.robot,
        help="Robot adapter to use (default: drone). Drone path runs MAVSDK "
             "via MavsdkDroneAdapter; rover path lands in Phase 4.",
    )

    # Pin the program name in --help output so the `robot-follow` and
    # `drone-follow` console-script aliases produce byte-identical
    # `--help` output (Phase 1 success criterion 2). Without this,
    # argparse derives prog from sys.argv[0] and the two aliases diverge.
    parser.prog = "robot-follow"

    # Tile / multi-scale defaults are injected AFTER the upstream
    # ``GStreamerTilingApp._add_tiling_arguments`` runs (it registers
    # ``--tiles-x`` etc. and would clobber any ``set_defaults`` made
    # here). See ``DroneFollowTilingApp._add_tiling_arguments`` in
    # ``hailo_drone_detection_manager.py`` for the override; the values
    # live in ``robot_follow/pipeline_defaults.py``.

    # Camera is mounted right-side up: no mirroring needed.
    # The library defines --horizontal-mirror/--vertical-mirror (store_true, default=False).
    # Pass both flags on the command line if the camera is upside-down.
    return parser


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # So `--help | head` exits cleanly instead of leaking BrokenPipeError.
    # Only for --help: at runtime SIG_DFL makes any write to a closed socket
    # (e.g. a browser refreshing the web UI mid-MJPEG) kill the process.
    if any(a in ("-h", "--help") for a in sys.argv[1:]):
        signal.signal(signal.SIGPIPE, signal.SIG_DFL)

    shared_state = SharedDetectionState()
    shutdown = asyncio.Event()
    eos_reached = threading.Event()

    # Create target state for follow server
    target_state = FollowTargetState()

    # Pre-parse output-branch / ReID / tracker flags in one shot so we can
    # wire the web UI / recording state objects and the ReIDManager before
    # create_app() runs the full parser. Display follows the implicit
    # rule: enabled when neither --openhd nor --webui is set.
    from robot_follow.pipeline_adapter.tracker_factory import TRACKER_CHOICES, DEFAULT_TRACKER
    pre = argparse.ArgumentParser(add_help=False)
    # Robot adapter (pre-parsed so main() can do main-thread rover preinit
    # before any Hailo pipeline construction — see rover_preinit below.)
    pre.add_argument("--robot", choices=("drone", "rover"), default="drone")
    # UI / output branches
    pre.add_argument("--webui", action="store_true")
    pre.add_argument("--webui-port", type=int, default=5001)
    pre.add_argument("--webui-fps", type=int, default=10)
    pre.add_argument("--openhd", action="store_true")
    pre.add_argument("--display", action="store_true")
    pre.add_argument("--record", action="store_true")
    pre.add_argument("--log-perf", action="store_true")
    # ReID
    pre.add_argument("--reid-model", type=str, default=_DEFAULT_REID_HEF)
    pre.add_argument("--no-reid", action="store_true")
    pre.add_argument("--update-interval", type=int, default=10)
    pre.add_argument("--reid-threshold", type=float, default=0.75)
    pre.add_argument("--reid-timeout", type=float, default=20.0)
    pre.add_argument("--reid-drift-threshold", type=float, default=0.6,
        help="Below this similarity vs gallery, an in-track embedding is "
             "treated as drift; gallery is not updated and re-acquisition "
             "is triggered.")
    pre.add_argument("--reid-duplicate-threshold", type=float, default=0.9,
        help="Above this similarity, the embedding is redundant and skipped "
             "(with periodic refresh — see --reid-refresh-every).")
    pre.add_argument("--reid-refresh-every", type=int, default=5,
        help="On every Nth consecutive duplicate-band decision, replace the "
             "oldest gallery vector to keep the gallery fresh.")
    pre.add_argument("--reid-min-gallery-for-drift-check", type=int, default=6)
    pre.add_argument("--reid-bootstrap-consistency", type=float, default=None)
    pre.add_argument("--reid-overlap-skip-iou", type=float, default=0.15,
        help="When another tracked person overlaps the target's bbox by more "
             "than this IoU, skip the gallery update for this frame to avoid "
             "storing a bridge crop. Set to 1.0 to disable.")
    pre.add_argument("--reid-dump-embeddings", type=str, default=None)
    # Tracker
    pre.add_argument("--tracker", default=DEFAULT_TRACKER, choices=TRACKER_CHOICES)
    # --config is registered here on the pre-parser so ControllerConfig.from_args(pre_args)
    # honors --config at the Path A wiring point (RINT-03 / RESEARCH Pitfall A guard).
    # The full parser also registers --config via ControllerConfig.add_args(); argparse
    # tolerates the duplicate registration when default=None and dest matches (verified
    # at import — if a future argparse change rejects duplicates, remove the --config
    # entry from ControllerConfig.add_args and keep this one as the sole registration).
    pre.add_argument("--config", type=str, default=None)
    pre_args, _ = pre.parse_known_args()

    # Single source of truth for output-branch policy
    # (mutex + implicit-display + record-branch gating in one helper).
    # ``decide_branches`` raises ValueError on the --openhd/--webui mutex;
    # surface it as a SystemExit so the CLI fails fast at pre-parse time
    # before any heavy pipeline construction.
    try:
        decision = decide_branches(
            openhd=pre_args.openhd,
            webui=pre_args.webui,
            display=pre_args.display,
            record=pre_args.record,
        )
    except ValueError as exc:
        raise SystemExit(f"error: {exc}")
    # Write the resolved values back to pre_args so downstream code that
    # reads pre_args.display / pre_args.webui / pre_args.openhd sees the
    # post-implicit-rule state.
    pre_args.display = decision.display

    # Build the recording branch whenever there's a control surface that
    # can toggle it remotely (--record auto-start, --webui mid-flight
    # toggle, --openhd mid-flight toggle via QOpenHD). The valve gates
    # frames at runtime, so building the branch when no toggle source
    # exists wastes CPU; building it when one does lets operators flip
    # recording on/off without restarting robot-follow.
    record_branch_enabled = decision.record_branch_enabled

    # Always create SharedUIState — the OpenHD bridge needs it for bbox
    # messages even when the web UI is disabled.
    from robot_follow.servers import SharedUIState
    ui_state = SharedUIState()

    web_server = None
    if pre_args.webui:
        from robot_follow.servers import WebServer
        # Check that the UI has been built
        _ui_build_index = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "ui", "build", "index.html")
        if not os.path.isfile(_ui_build_index):
            LOGGER.error("Web UI has not been built yet.")
            LOGGER.error("  cd robot_follow/ui")
            LOGGER.error("  npm install")
            LOGGER.error("  npm run build")
            raise SystemExit(1)
    # Build the full parser from all domains, then pass to pipeline adapter
    parser = _build_parser()

    reid_manager = None
    if not pre_args.no_reid and pre_args.reid_model:
        from robot_follow.pipeline_adapter.reid_manager import ReIDManager
        reid_manager = ReIDManager(
            hef_path=pre_args.reid_model,
            update_interval=pre_args.update_interval,
            reid_match_threshold=pre_args.reid_threshold,
            drift_threshold=pre_args.reid_drift_threshold,
            duplicate_threshold=pre_args.reid_duplicate_threshold,
            refresh_every=pre_args.reid_refresh_every,
            min_gallery_for_drift_check=pre_args.reid_min_gallery_for_drift_check,
            bootstrap_consistency_threshold=pre_args.reid_bootstrap_consistency,
            overlap_skip_iou=pre_args.reid_overlap_skip_iou,
            dump_embeddings_path=pre_args.reid_dump_embeddings,
        )

    # Phase 6 workaround: when --robot rover, initialize rclpy + Node on
    # the MAIN thread BEFORE the Hailo gstreamer plugins load. Worker-thread
    # Node creation races with Hailo plugin loading and SIGABRT/SIGSEGVs the
    # process. See Ros2RoverAdapter.init_on_main_thread for details.
    rover_preinit = None
    if pre_args.robot == "rover":
        try:
            from robot_follow.robot_api.adapters.ros2_rover import (
                Ros2RoverAdapter,
            )
            # Placeholder ControllerConfig — the adapter only uses config during
            # send_command (later, on the worker thread). connect/start_session
            # only read self._topic/_namespace/_domain_id from args; safe defaults
            # are baked into __init__'s getattr() calls.
            rover_preinit = Ros2RoverAdapter(pre_args, ControllerConfig())
            rover_preinit.init_on_main_thread()
            LOGGER.info("[rover] rclpy + Node initialized on main thread")
        except RuntimeError as e:
            LOGGER.error(
                "[rover] main-thread rclpy init failed: %s\n"
                "Pipeline continues without robot control.",
                e,
            )
            rover_preinit = None

    from robot_follow.pipeline_adapter import create_app

    recordings_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
    # RINT-03 (Path A wiring): first-pass ControllerConfig built from pre-parsed args,
    # passed into create_app for ByteTracker init. The full-args ControllerConfig
    # below (line ~448) stays the source of truth for the rest of the app
    # (web server, OpenHD bridge, callback live-config). Both honor --config because
    # --config is on both the pre-parser (added above) and the full parser
    # (via ControllerConfig.add_args).
    app = create_app(shared_state, target_state=target_state, eos_reached=eos_reached,
                     ui_state=ui_state, ui_fps=pre_args.webui_fps, parser=parser,
                     record_enabled=record_branch_enabled, record_dir=recordings_dir,
                     reid_manager=reid_manager,
                     reid_search_timeout=pre_args.reid_timeout,
                     tracker_name=pre_args.tracker,
                     log_perf=pre_args.log_perf,
                     controller_config=ControllerConfig.from_args(pre_args))
    args = app.options_menu
    # Propagate the pre-parse branch decision onto the full-parser args
    # namespace so the pipeline-string builder reads the post-implicit-rule
    # display state. The mutex check + implicit-display rule already ran in
    # ``decide_branches`` above; no duplicate logic here.
    args.display = decision.display
    _configure_logging(getattr(args, "log_verbosity", "normal"))
    _resolve_serial_connection(args)

    # Create controller config once so it can be shared (and mutated via web UI)
    controller_config = ControllerConfig.from_args(args)

    # Make the live config visible to the detection callback so it can read auto_select
    # and write target_bbox_height when a target is locked.
    app.user_data.controller_config = controller_config

    test_log_path = getattr(args, "test_log", None)
    if test_log_path:
        app.user_data.open_test_log(test_log_path)

    # --save-config: dump effective config to JSON and exit
    save_path = getattr(args, "save_config", None)
    if save_path:
        controller_config.save_json(save_path)
        LOGGER.info("[app] Config saved to %s", save_path)
        raise SystemExit(0)

    # Start follow server (always available)
    follow_server = FollowServer(target_state, shared_state, port=args.follow_server_port,
                                 reid_manager=reid_manager,
                                 ui_state=ui_state, controller_config=controller_config)
    follow_server.start()

    # Start OpenHD parameter bridge (allows QOpenHD to control follow params,
    # bitrate, and air-side recording start/stop).
    openhd_bridge = OpenHDBridge(controller_config, target_state=target_state,
                                 detection_state=shared_state, ui_state=ui_state,
                                 gst_app=app, recording_ctl=app)
    openhd_bridge.start()

    # Start web UI server
    if pre_args.webui:
        static_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ui", "build")
        web_server = WebServer(ui_state, target_state, shared_state,
                               controller_config=controller_config,
                               port=args.webui_port, static_dir=static_dir,
                               follow_server_port=args.follow_server_port,
                               recording_ctl=app)

        web_server.start()

    def _quit_pipeline():
        """Tell GStreamer to quit (safe to call multiple times)."""
        try:
            app.loop.quit()
        except (AttributeError, RuntimeError):
            pass

    def _eos_to_shutdown():
        eos_reached.wait()
        shutdown.set()
        _quit_pipeline()
    threading.Thread(target=_eos_to_shutdown, daemon=True).start()

    def run_robot():
        """Run robot control in a background thread with its own asyncio loop."""
        if args.robot == "drone":
            adapter = MavsdkDroneAdapter(args, controller_config)
        elif args.robot == "rover":
            if rover_preinit is not None:
                # Reuse the main-thread preinit (avoids the Hailo-plugin race).
                # Swap the placeholder config for the real one.
                adapter = rover_preinit
                adapter._config = controller_config
            else:
                # Lazy import so `--robot drone` works on a no-rclpy box and the
                # friendly RuntimeError only fires when the user actually picks rover.
                try:
                    from robot_follow.robot_api.adapters.ros2_rover import (
                        Ros2RoverAdapter,
                    )
                    adapter = Ros2RoverAdapter(args, controller_config)
                except RuntimeError as e:
                    LOGGER.error(
                        "[rover] adapter construction failed: %s\n"
                        "Pipeline continues without robot control.",
                        e,
                    )
                    return
        else:
            raise ValueError(f"Unknown --robot value: {args.robot}")

        async def _main():
            duration = getattr(args, "mission_duration", math.inf)
            loop_task = asyncio.create_task(
                run_robot_loop(adapter, shared_state, controller_config, shutdown, ui_state=ui_state)
            )
            deadline_task = asyncio.create_task(asyncio.sleep(duration))
            try:
                done, pending = await asyncio.wait(
                    [loop_task, deadline_task],
                    return_when=asyncio.FIRST_COMPLETED,
                )
            finally:
                for t in (loop_task, deadline_task):
                    if not t.done():
                        t.cancel()
                        try:
                            await t
                        except asyncio.CancelledError:
                            pass
                        except Exception:
                            LOGGER.warning(
                                "[robot] background task raised on shutdown",
                                exc_info=True,
                            )

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(_main())
        except Exception:
            LOGGER.warning(
                "[robot] Control loop failed — pipeline continues without robot control.",
                exc_info=True,
            )
        finally:
            loop.close()

    robot_thread = threading.Thread(target=run_robot,
                                    name="robot-follow-control",
                                    daemon=True)
    robot_thread.start()
    LOGGER.info("[app] Robot control started in background thread")

    def on_signal(*_):
        if not shutdown.is_set():
            shutdown.set()
            LOGGER.warning("[drone] Ctrl+C received, shutting down...")
            _quit_pipeline()

    signal.signal(signal.SIGINT, on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, on_signal)

    # Start recording from CLI flag after pipeline is running
    if pre_args.record:
        def _start_recording_delayed():
            time.sleep(1.0)  # wait for pipeline to reach PLAYING
            app.start_recording()
        threading.Thread(target=_start_recording_delayed, daemon=True).start()

    # Run the GStreamer pipeline on the main thread (UI + Hailo start immediately)
    LOGGER.info("[app] Starting Hailo pipeline and UI on main thread")
    try:
        app.run()
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if not shutdown.is_set():
            shutdown.set()
        if app.is_recording:
            app.stop_recording()
        else:
            app.cleanup_recording_branch()
        # Wait for robot thread to finish cleanly
        robot_thread.join(timeout=5.0)
        if robot_thread.is_alive():
            # Robot thread is stuck (typically a MAVSDK land/offboard timeout
            # against a sim that's already gone). Its `with DetachedMavsdkServer`
            # __exit__ won't run, and start_new_session=True means mavsdk_server
            # would survive us — leaving UDP 14540 + TCP 50051 bound and blocking
            # the next run. Reap it by name now while we still own a shell.
            _reap_mavsdk_server()
        if reid_manager is not None:
            reid_manager.dump_embeddings()
            reid_manager.release()
        app.user_data.close_test_log()
        if web_server is not None:
            web_server.stop()
        openhd_bridge.stop()
        follow_server.stop()


if __name__ == "__main__":
    main()
