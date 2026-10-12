# H15 Drone Follow — Service & Launch Reference

Reference for running `drone_follow_h15` on the Hailo15 target.

## Files on the H15 (after `install.sh`)

| File | Purpose |
|---|---|
| `/etc/init.d/drone-follow` | SysV init script |
| `/etc/default/drone-follow` | CLI args passed to the app (edit this to change flags) |
| `/etc/rcS.d/S99drone-follow` | Symlink for auto-start at boot |
| `/var/log/drone-follow.log` | Service stdout/stderr (appended each run) |
| `/var/run/drone-follow.pid` | PID file while service is running |
| `/home/root/robot_follow/` | Synced Python package |
| `/home/root/robot_follow/ui/build/` | Built React UI |
| `/home/root/recordings/` | Video recordings (`.mkv`) + detection logs (`.jsonl`) |
| `/home/root/df_config.json` | Saved controller config (if "Save Config" pressed in UI) |

## Installation (from build host)

```bash
cd hailo-drone-follow/scripts/h15_boot

./install.sh                          # full install: UI build + app + service (target=root@10.0.0.1)
./install.sh root@<ip>                # custom target
TARGET=root@<ip> ./install.sh         # via env var

./build_ui.sh                         # just rebuild React UI (robot_follow/ui/build/)
./install_app.sh                      # just sync robot_follow/ code (assumes UI already built)
./install_service.sh                  # just install/refresh init service
./uninstall.sh                        # remove service files (keeps log)
./install_usb_automount.sh            # one-time setup of USB auto-mount at /mnt/usb
```

The full `install.sh` runs `build_ui.sh`, then `install_app.sh`, then `install_service.sh`.
Run `build_ui.sh` standalone whenever you change `robot_follow/ui/src/`.

## Service control (on target)

```bash
/etc/init.d/drone-follow start
/etc/init.d/drone-follow stop
/etc/init.d/drone-follow restart
/etc/init.d/drone-follow status
```

After reboot the service starts automatically via `/etc/rcS.d/S99drone-follow`.

To **disable** auto-start without uninstalling:
```bash
rm /etc/rcS.d/S99drone-follow
```

To **re-enable**:
```bash
ln -sf /etc/init.d/drone-follow /etc/rcS.d/S99drone-follow
```

## Changing CLI args

Edit `/etc/default/drone-follow` on the device:
```bash
nano /etc/default/drone-follow
/etc/init.d/drone-follow restart
```

The file sets `DRONE_FOLLOW_ARGS`. Examples:

```bash
# Real drone via Pixhawk serial (waits for OFFBOARD via GCS — safest):
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0"

# Bench testing with Pixhawk (auto-OFFBOARD, no GCS):
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --auto-offboard"

# SITL on the same machine (auto-OFFBOARD over UDP):
DRONE_FOLLOW_ARGS="--auto-offboard"

# PX4 SITL on PC over UDP:
DRONE_FOLLOW_ARGS=""

# Sim with auto takeoff + tracking + landing:
DRONE_FOLLOW_ARGS="--takeoff-landing"

# Dry-run (no drone connection):
DRONE_FOLLOW_ARGS="--dry-run"

# Add local recording:
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --record"

# Custom recording path (specific file):
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --record /home/root/recordings/test.mkv"

# Recording to USB (directory — drone_<ts>.mkv auto-named inside it):
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --record /mnt/usb"

# Load a saved config:
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --config /home/root/df_config.json"
```

## Manual launch (without service)

If you want to run the app manually (e.g., to see live logs):

```bash
# Stop the service first
/etc/init.d/drone-follow stop

# Run manually
PYTHONPATH=/home/root python3 -m robot_follow.drone_follow_h15 --serial /dev/ttyACM0
```

## Viewing live

| Endpoint | What |
|---|---|
| `http://10.0.0.1:5001/` | Web UI (React) — video + detection overlay + config sliders |
| `http://10.0.0.1:5001/api/video` | Raw MJPEG stream (works without React build) |
| `http://10.0.0.1:5001/api/config` | Current config as JSON |

The web UI shows the FHD-inference branch downscaled to 640×360 MJPEG at 15fps.
There is no UDP H264 stream — the VPU encoder budget is reserved for the 4K
on-device recording (see Recordings below).

## SSH tunnel (browser on a different host)

If the browser machine can't reach `10.0.0.1` directly but can reach an intermediate PC:
```bash
ssh -L 5001:10.0.0.1:5001 user@<intermediate-PC>
```
Then open `http://localhost:5001` in the browser.

## Recordings

**Location on target:** `/home/root/recordings/`

When `--record` is set, the service writes two files per run:
- `drone_<YYYYMMDD_HHMMSS>.mkv` — 4K (3840×2160) H264 video, no overlays, ~16 Mbps
- `drone_<YYYYMMDD_HHMMSS>.jsonl` — detection data per inference frame (normalized coords, resolution-independent)

At 4K@30 ~16 Mbps the recording is roughly **120 MB/minute** — check `df -h /home/root` before long flights. The `.jsonl` overlay script works regardless of video resolution since detection coordinates are normalized [0, 1].

### Recording to USB

Internal storage is 8 GB total — about an hour of 4K. For longer sessions, record to a USB stick.

**One-time setup** (installs udev rule + mount helper):
```bash
cd hailo-drone-follow/scripts/h15_boot
./install_usb_automount.sh
```

After that, plug any USB stick in and it auto-mounts at `/mnt/usb`. Confirm with:
```bash
ssh root@10.0.0.1 "df -h /mnt/usb"
tail /var/log/drone-usb-mount.log    # on the target, for diagnostics
```

**Direct the app to record there.** `--record` accepts a directory and auto-names the file inside it:
```bash
# In /etc/default/drone-follow on the target:
DRONE_FOLLOW_ARGS="--serial /dev/ttyACM0 --record /mnt/usb"
# Then: /etc/init.d/drone-follow restart
```

Each run writes `/mnt/usb/drone_<YYYYMMDD_HHMMSS>.mkv` + matching `.jsonl`.

**vfat (FAT32) caveat:** single file capped at 4 GB. At 4K@16 Mbps that's about **33 minutes per file** — fine for typical flights but plan around it for longer ones. Reformat the stick as exFAT or ext4 to remove the cap (note: ext4 sticks can't be read by Windows without extra drivers).

**List recordings on target:**
```bash
ssh root@10.0.0.1 "ls -lt /home/root/recordings/"
```

**Copy recordings off the target:**
```bash
scp 'root@10.0.0.1:/home/root/recordings/drone_*' ./
```

**Disk usage / free space on target:**
```bash
ssh root@10.0.0.1 "df -h /home/root && du -sh /home/root/recordings/"
```

**Delete old recordings on target:**
```bash
ssh root@10.0.0.1 "rm /home/root/recordings/drone_*"
```

**Generate annotated video (post-flight, on a machine with OpenCV):**

Requires `pip install opencv-python` on the machine running the script.

```bash
# Pull both files (the .jsonl must be next to the .mkv)
scp root@10.0.0.1:/home/root/recordings/drone_xxx.{mkv,jsonl} .

# Default: auto-finds drone_xxx.jsonl next to drone_xxx.mkv
# Writes drone_xxx_annotated.mp4
./scripts/post_process/overlay_detections.py drone_xxx.mkv

# Explicit detection log file
./scripts/post_process/overlay_detections.py drone_xxx.mkv path/to/dets.jsonl

# Custom output path
./scripts/post_process/overlay_detections.py drone_xxx.mkv -o my_overlay.mp4

# Different codec (mp4v default; avc1 for H264; XVID for AVI)
./scripts/post_process/overlay_detections.py drone_xxx.mkv --codec avc1

# Help
./scripts/post_process/overlay_detections.py --help
```

Output: green boxes around the followed target, white boxes around the rest, with `ID N XX%` labels.

## Replay mode (re-run inference on a recorded video)

Run a recorded `.mkv` back through the H15's live inference + tracker pipeline. Useful for testing tracker changes against real flight footage, or generating an updated detection log without flying again.

**Run replay:**
```bash
PYTHONPATH=/home/root python3 -m robot_follow.drone_follow_h15 \
    -i /home/root/recordings/drone_xxx.mkv

# Loop the file continuously
PYTHONPATH=/home/root python3 -m robot_follow.drone_follow_h15 \
    -i /home/root/recordings/drone_xxx.mkv --loop
```

The replay runs at native source speed (throttled by an `identity sync=true` element), feeds frames through the inference path, and exposes the new detections on the web UI (`http://10.0.0.1:5001/`). `--record` is auto-disabled for video output (no encoder branch), and drone control is forced to dry-run.

**Save the new detections to a `.jsonl` sidecar:**
```bash
# Auto-named: /home/root/recordings/drone_<ts>_replay.jsonl
PYTHONPATH=/home/root python3 -m robot_follow.drone_follow_h15 \
    -i /home/root/recordings/drone_xxx.mkv --record

# Then overlay the new detections on the ORIGINAL video (on PC w/ OpenCV):
./scripts/post_process/overlay_detections.py drone_xxx.mkv drone_<ts>_replay.jsonl
```

This is useful for comparing tracker output between code versions — run the same flight twice with different tracker settings, get two `.jsonl` files, and produce two annotated videos to diff.

## Logs

```bash
# Service log (appended across runs)
tail -f /var/log/drone-follow.log

# Last 200 lines
tail -200 /var/log/drone-follow.log
```

### FC command log

Every command sent to the flight controller (and, with `--dry-run`, every command that
would be sent) is written to `/home/root/fc_command_logs/fc_commands_<timestamp>.jsonl`.
This directory survives reboots, unlike `/var/log`. Use `--cmd-log <dir|file.jsonl>` to change
the location, or `--cmd-log off` to disable it.

```bash
ls -t /home/root/fc_command_logs | head -3
tail -f /home/root/fc_command_logs/$(ls -t /home/root/fc_command_logs | head -1)
```

## Quick health check on target

```bash
/etc/init.d/drone-follow status
ps aux | grep robot_follow
netstat -tln | grep 5001                  # WebServer
tail /var/log/drone-follow.log
```
