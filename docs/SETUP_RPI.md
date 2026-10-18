# TaPS CameraNode - Raspberry Pi Bring-Up Guide (CM5 / Debian 13)

Bench-verified procedure (2026-10-03, camera + autostart sections updated
2026-10-18) for running CameraNode on a Raspberry Pi CM5 (Debian 13,
aarch64). Bench device: hostname CameraNode1, user dan, FINAL configuration
(1x Arducam UC-852 global-shutter camera + 1x GPS 1PPS; no further devices
planned). These steps ARE the production bring-up; inline notes distinguish
bench-specific hardware quirks (camera power, GPS) from the standing setup.

## 1. Base OS and Access

- Raspberry Pi OS Lite (Debian 13, 64-bit), SSH enabled, key-based login.
  The bench uses a dedicated operator key with BatchMode/IdentitiesOnly.
- Set the hostname once:

```
sudo hostnamectl set-hostname CameraNode1
```

- Consumers on the Windows side (ServerNode yaml) must use the LOWERCASE
  hostname cameranode1.local - the local mDNS resolution there is
case-sensitive and the capitalized form fails every fetch silently.

## 2. GPIO Permissions (required for PPS)

/dev/gpiochip0 is root:gpio mode 660. Put the run user in the gpio group
so libgpiod line requests need no sudo:

```
sudo usermod -aG gpio dan
# log out/in (or reboot) for the group change to apply
ls -l /dev/gpiochip0        # crw-rw---- root gpio
id dan                      # groups include gpio
```

## 3. Disable Unused Radios

Production links over wired Ethernet/robot radio. Save power and reduce
RF noise - append to /boot/firmware/config.txt:

```
dtoverlay=disable-wifi
dtoverlay=disable-bt
```

Reboot, then verify: ip link shows no wlan0; rfkill list is empty.
(Overlay lines must each sit on their own line - check with cat -A after
editing; a missing newline silently concatenates overlays.)

## 4. Build Dependencies

```
sudo apt update
sudo apt install -y build-essential cmake pkg-config git \
    libspdlog-dev libgpiod-dev python3
```

- cmake 3.25+ (Debian 13 ships a sufficient version).
- args.hxx (C++ arg parsing): NOT packaged on Debian 13 - it is VENDORED
  in the repo; no apt package exists or is needed.
- OpenCV 4.5+ with videoio/V4L2 - built FROM SOURCE on the bench (distro
  build lagged):

```
git clone --depth 1 https://github.com/opencv/opencv.git
cmake -B opencv/build -S opencv -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_LIST=videoio,imgproc,imgcodecs,calib3d \
    -DWITH_V4L=ON -DWITH_GTK=OFF -DWITH_QT=OFF
cmake --build opencv/build -j4
sudo cmake --install opencv/build
```

- AprilTag library - also built from source:

```
git clone --depth 1 https://github.com/AprilRobotics/apriltag.git
cmake -B apriltag/build -S apriltag -DCMAKE_BUILD_TYPE=Release
cmake --build apriltag/build -j4
sudo cmake --install apriltag/build
```

## 5. Build CameraNode

Builds run ON the Pi (no maintained cross-toolchain). Bench source tree:
~/taps/camera_node.

```
cd ~/taps/camera_node
cmake -B build
cmake --build build -j4
```

## 6. Run - Bench-Verified Command Line

From ~/taps/camera_node (the web UI templates are served relative to the
working directory - always launch from the tree root):

```
./build/TaPS_CameraNode -c 0 -w 1280 -h 800 -f 100 -F MJPG \
    --apriltag --apriltag-threads=2 --apriltag-quad-decimation=3 \
    --camera-alias CameraNode1 -o output --pps-gpio 24
```

- -c 0 = /dev/video0 for the USB camera (confirm: v4l2-ctl --list-devices)
- -F MJPG is MANDATORY on this camera: YUYV caps at 10 fps (section 7)
- --apriltag enables local detection + .taps pose records; the threads and
  decimation flags are measured tuning, not optional (section 7.2)
- --pps-gpio 24 attaches the 1PPS time-sync input (section 9)

## 7. Camera: Arducam UC-852 (OmniVision OV9782 global shutter)

Final-config camera, replacing the earlier Logitech C925e bench unit.
USB UVC device, VID:PID 0c45:6366, serial string UC852, driver uvcvideo,
present at /dev/video0. Identified and mode-probed 2026-10-18.

### 7.1 Supported modes (measured via v4l2-ctl)

| Pix fmt | Resolutions | Frame rates |
|---------|-------------|-------------|
| MJPG | 1280x800, 1280x720, 800x600 | 100 / 90 / 60 / 30 / 15 fps |
| MJPG | 640x480, 320x240 | 100 / 90 / 60 / 30 fps |
| YUYV | 1280x800, 1280x720 | 10 fps only |

- USE MJPG. YUYV is limited to 10 fps by USB2 bandwidth and is not useful.
- 1280x800 is the native full sensor frame; 1280x720 is a vertical crop
  (narrower FOV). Prefer 800p so the calibrated FOV matches the archive.
- 1280x800 MJPG at -f 100 delivers a true 100.0 Hz: median frame interval
  9.8 ms, p95 12.6 ms. Exposure is capped at ~10 ms at 100 fps (16.7 ms at
  60 fps); at 60 fps the decision margin measured higher (166 vs 137), which
  matters if the field lighting is dim.
- USB inrush when plugging the camera can brown-out the CM5 power path
  (bench observed: undervoltage latched in vcgencmd get_throttled as
  bit 0x10000, which reset the board). Use a SELF-POWERED USB hub.
- After bring-up: vcgencmd get_throttled must read 0x0.
- A camera powered via Pi 5V can also brown-out mid-session on load
  peaks; keep it on the external rail.
- If a second identical camera is ever added, both enumerate as
  /dev/video0 and /dev/video2 with the same VID:PID - pin device nodes by
  serial (UC852) with a udev rule instead of trusting -c 0.

### 7.2 AprilTag detector is the throughput ceiling

Bandwidth, network (1 Gbps eth0) and disk are all far ahead of the sensor.
The limit is AprilTag pose generation on the Pi:

| Mode | Detector tuning | Frames with valid pose | Poses/s |
|------|-----------------|------------------------|---------|
| 100 fps | 1 thread, decimation 2 | 32% | ~33 |
| 100 fps | 2 threads, decimation 2 | 40% | ~41 |
| 100 fps | 2 threads, decimation 3 | 72% | ~73 |
| 60 fps | 2 threads, decimation 3 | 100% | 60 |
| 30 fps | 1 thread, decimation 2 | 100% | 30 |

- Decimation 2 -> 3 is the large lever (quarter-resolution pyramid for the
  quad search); decision margin stayed at 137, so detection quality was not
  measurably hurt for the bench tag size and distance.
- 2 detector threads raises CPU from ~2.8 to ~3.0 cores of 4. Do NOT raise
  it further on this single-camera build; the remaining core carries the
  HTTP/SSE, PPS and NT threads.
- Recorded video never slows when detection falls behind: detection skips
  frames, capture keeps full rate (so coverage is the metric to watch, not
  fps). Detection quality on each recorded pose is unaffected.
- Storage: 100 fps at 800p MJPG writes ~1.1-1.3 GB/min. Offload or trim.
- Thermal: ~45 C steady state at 100 fps on the CM5 with no throttle flags.
  If the enclosure runs hotter, -f 60 with the same tuning gives 100% pose
  coverage at ~2 cores.

## 8. Thermal

Sustained MJPG capture + AprilTag detection throttles a fanless CM5
(throttle bits 0xe0000 appear once the soft-temp limit engages). A fan
is strongly recommended; target < 45 C at full load. The node's web page
(Power & Thermal card) reports live temperature and throttle bits.

For offline periods the node offers a LOW POWER mode (section 10.1): the
camera device is released and AprilTag processing idles, which measured
0% CPU and 27-30 C on the fanless CM5 (vs ~75% and 36-45 C capturing).
This is the boot default, so an unattended node runs cool until capture
is deliberately enabled.

## 9. 1PPS Time-Sync Input (PPS)

- Carrier port User_IO1 is GPIO24. The carrier schematic labels this
  GPIO20 - the schematic is WRONG; wire and configure GPIO24.
- Signal: active-LOW, ~100 ms low pulse; the RISING edge marks the
  second-start. The app requests the line with an internal PULL_UP
  (signal idles HIGH).
- Connect GPS 1PPS output to User_IO1 (GPIO24) with common ground.
- CLI: --pps-gpio 24. Bench-verified: period 1000.0 ms, jitter 5-55 us,
  zero dropped edges over long runs; stats visible on /system.
- Before the first edge arrives the app timestamps with the system
  clock; the web page shows NO SIGNAL until lock.
- Production plan: feed 1PPS through an RS485 line receiver for robust
  long runs - the same GPIO discipline applies (rising edge, pull-up).

## 10. systemd Autostart (production - ENABLED on CameraNode1)

The unit file in this repo is systemd/camera_node.service; it installs as
taps-camera-node.service. Install name and service name are therefore
taps-camera-node, NOT camera-node:

```
sudo install -m 644 ~/taps/camera_node/systemd/camera_node.service \
    /etc/systemd/system/taps-camera-node.service
sudo systemctl daemon-reload
sudo systemctl enable --now taps-camera-node
systemctl is-enabled taps-camera-node     # enabled
journalctl -u taps-camera-node -f
```

Enabled on CameraNode1 and reboot-verified 2026-10-18 (again after the low
power feature landed, same day): after a full reboot the unit starts at
boot, comes up in LOW POWER mode (section 10.1), and PPS re-locks within
~10 s of the first GPS edges. The MJPEG stream only flows once capture is
enabled. There is no rc.local or other second autostart path - do not add
one (it would race the unit for /dev/video0).

Never run taps-camera-node and pps-watch simultaneously - GPIO line requests
are exclusive (section 12).

### 10.1 Low Power Mode (boot default)

The node boots with the camera CLOSED and AprilTag processing idle. The
journal says so explicitly:

```
LOW POWER mode at boot: /dev/video0 stays closed, AprilTag idle...
```

- Always alive: web UI, SSE events, /status /system /lowpower, PPS time
  sync and NetworkTables. PPS keeps locking and edge-stamping normally.
- Refused while low power (HTTP 409): recording start, calibration start,
  NT match-start trigger (logged as a warning).
- Toggle from the web UI button ("Enter Normal Mode" / "Low Power Mode",
  with a banner over the stream), or:

```
curl -s localhost:8080/lowpower                                # state
curl -s -X POST localhost:8080/lowpower -d '{"enable":false}'  # capture on
curl -s -X POST localhost:8080/lowpower -d '{"enable":true}'   # back to LP
```

- Enabling low power while recording stops the recorder first and flushes
  the .taps cleanly, then releases the camera (~1 s).
- Reopening after a long idle is fast (~1-2 s) and needs no replug; the
  SIGTERM-only rule (section 11) still applies to the process itself.
- Measured: low power 0% CPU / 27-30 C; capturing 100 fps ~75% CPU /
  36-45 C. Six back-to-back LP <-> capture cycles recorded cleanly with
  no UVC wedge.
- Opt out of the low-power boot default with --start-capturing on the
  command line (the production unit deliberately does NOT use it).

## 11. Maintenance Rule - SIGTERM Only

Stop the node ONLY with SIGTERM: systemctl stop taps-camera-node, or kill
<PID>. A SIGKILL (kill -9) mid-stream wedges the UVC session - the device
enumerates but streams zero bytes, and the ONLY recovery is a physical USB
replug or a reboot (proven on the bench with both cameras). The unit sets
TimeoutStopSec=30 so an open .taps can flush on graceful stop. Wait >= 4 s
after a stop before relaunching; RestartSec=5 covers the automatic case.

## 12. Bench PPS Watcher (diagnostic only)

scripts/pps_watch.sh logs rising-edge epochs on GPIO24 with gpiomon,
for debugging a new GPS antenna/cable while the node is NOT running
(systemctl is-active pps-watch). It yields to the node automatically
while the flag file /tmp/pps_pause exists, and never kills anything.

```
touch /tmp/pps_pause               # pause/yield the watcher
journalctl -u pps-watch -f
systemctl stop pps-watch           # when done; node owns the line
```

## 13. Verification Checklist

| # | Check | Command | Expected |
|---|-------|---------|----------|
| 1 | Status | curl -s localhost:8080/status | JSON: state Idle; lp:true at boot (low power) |
| 2 | System+PPS | curl -s localhost:8080/system | pps en/g set; e/d/s/p/j advance; l true once locked |
| 3 | Recording refused in LP | curl -X POST localhost:8080/recording -d '{"action":"start"}' | HTTP 409 + low power error |
| 3b | Enter normal mode | curl -X POST localhost:8080/lowpower -d '{"enable":false}' | {"low_power":false}; camera opens in ~1-2 s |
| 4 | Stream flows | curl -s --max-time 4 localhost:8080/stream -o /tmp/s; wc -c /tmp/s | hundreds of KB |
| 5 | Record 6 s | curl -X POST localhost:8080/recording -d '{"action":"start"}'; sleep 6; curl -X POST localhost:8080/recording -d '{"action":"stop"}' | state returns Idle; log shows Wrote N frames |
| 6 | File exists | ls -l ~/taps/camera_node/output/*.taps | nonzero-size .taps file |
| 7 | Power clean | vcgencmd get_throttled | 0x0 |
| 8 | PPS lock | curl -s localhost:8080/system | "l":true with GPS fix (NO SIGNAL badge clears) |

Rows 4-6 presume capture is enabled (do 3b first); in low power /stream
serves the placeholder and recording stays refused.

/system pps fields: en=enabled, g=pin, l=locked, el=ever-locked,
e=edges, d=dropouts, s=seconds since last edge, p=period ms,
j=jitter us. sys fields: t=SoC C, raw=throttle hex, f=fault list.

## 14. Gotchas

- "unable to open camera": a previous node instance may still hold
  /dev/video0 (pgrep -af TaPS_CameraNode) - or the SIGKILL wedge of
  section 11; physically replug the camera.
- AprilTag poses use intrinsics from output/calibration.json once the
  wizard (http://<camera>:8080/calibrate) has run; recalibrate whenever
  capture resolution changes. The current OV9782 800p set is archived in
  this repo at config/calibrations/CameraNode1_OV9782_1280x800_2026-10-18.json
  (fx 926.4, RMS 1.68 px) - copy it back to output/calibration.json after a
  rebuild or SD reflash to avoid a recalibration detour.
- Keep recordings offloaded; ~1.2 GB/min at 100 fps fills the SD card in
  under an hour.
- mDNS is unreliable from the Windows ServerNode host: use the node's
  static IP in server_node.yaml, not cameranode1.local (section 1).
- Switching capture resolution requires recalibration before poses are
  trustworthy (the calibration is only loaded when its image size matches
  the running resolution).