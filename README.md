# TaPS Consolidated System

Tracking and Playback System — consolidated CameraNode and ServerNode applications.

## Overview

TaPS provides high-precision robot tracking and multi-camera video recording/playback for FRC-style robot matches. The system consists of:

- **CameraNode** (C++) — Runs on Raspberry Pi devices, captures video from global shutter cameras, records to `.taps` format, streams MJPEG, and responds to NetworkTables triggers
- **ServerNode** (Python) — Runs on a Linux workstation, aggregates camera streams, manages recording sessions, receives log files, and provides a web-based playback interface

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        SERVER NODE (Linux Workstation)               │
│                                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│  │ CamNode1 │→│ CamNode2 │→│ CamNode3 │→│ CamNode4 │            │
│  │ (C++)    │  │ (C++)    │  │ (C++)    │  │ (C++)    │            │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘            │
│       │              │              │              │                 │
│       └──────────────┴──────────────┴──────────────┘                 │
│                          ↑ 2x2 composited MJPEG                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                    ServerNode (Python)                        │   │
│  │  CameraAggregator │ SessionManager │ LogPoller │ Playback    │   │
│  │  WebServer (aiohttp) + WebSocket + HTML Frontend             │   │
│  └──────────────────────────────────────────────────────────────┘   │
│                          ↑ SMB poll                                   │
│              ┌───────────────────────┐                               │
│              │ Robot Control PC      │                               │
│              │ (Windows SMB share)   │                               │
│              └───────────────────────┘                               │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              NetworkTables (RoboRIO Client)                   │   │
│  │         Listens for matchStart/matchEnd boolean               │   │
│  └──────────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
TaPS_Consolidated/
├── common/                         # Shared Python code
│   ├── __init__.py
│   ├── log_parser.py              # FRC DriverStation log parser
│   └── config.py                  # YAML config loader
│
├── camera_node/                    # C++ CameraNode application
│   ├── CMakeLists.txt              # Build configuration
│   ├── src/                        # Source files (enhanced from existing)
│   ├── templates/                  # Web frontend templates
│   └── systemd/                    # systemd service files
│
├── server_node/                    # Python ServerNode application
│   ├── main.py                    # Entry point
│   ├── camera_aggregator.py       # MJPEG stream aggregation
│   ├── session_manager.py         # Recording session management
│   ├── log_poller.py              # SMB share polling
│   ├── log_sync.py                # Log file synchronization
│   ├── playback_engine.py         # .taps file playback
│   ├── web_server.py              # aiohttp web server
│   ├── static/index.html          # Web frontend
│   └── requirements.txt
│
├── shared_config/                  # Configuration templates
│   ├── camera_node_template.yaml
│   └── server_node_template.yaml
│
└── README.md                       # This file
```

## Quick Start

### Prerequisites

**CameraNode (Raspberry Pi):**
- Ubuntu for Raspberry Pi (headless)
- CMake 3.28+, GCC with C++20 support
- OpenCV 5.0+ with V4L2 support
- spdlog, libhttpserver
- libgpiod-dev (for GPIO/PPS)
- Python 3 with pyntcore package (for C++ ntcore bindings)

**ServerNode (Linux Workstation):**
- Ubuntu 22.04+
- Python 3.10+
- OpenCV 4.8+

### CameraNode Setup

1. Copy the template and edit for your camera:
```bash
cp shared_config/camera_node_template.yaml camera_node.yaml
```

2. Build:
```bash
cd camera_node
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

3. Install systemd service:
```bash
sudo cp systemd/camera_node.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable camera_node
sudo systemctl start camera_node
```

### ServerNode Setup

1. Copy the template and edit for your setup:
```bash
cp shared_config/server_node_template.yaml server_node.yaml
```

2. Install Python dependencies:
```bash
cd server_node
pip install -r requirements.txt
pip install -e ../common  # If using as a package
```

3. Run:
```bash
python main.py server_node.yaml
```

4. Open a browser to `http://<server-ip>:8080`

## .taps File Format

Full specification: [`../common/taps_format.md`](../common/taps_format.md)

The `.taps` binary format stores video frames with GPS-aligned nanosecond
timestamps (PPS + chrony discipline; PTP optional). Version `0x02` is the
baseline; version `0x03` adds per-frame AprilTag detections (id, pose, and
quality/uncertainty metrics for fusion) plus camera intrinsics in the header.

```
Header:
  Magic:     'TaPS\x02' or 'TaPS\x03' (5 bytes)
  Encoder:   uint8 (0=JPEG, 1=RAW)
  Width:     uint64
  Height:    uint64
  FPS:       double
  ArgsLen:   uint32
  Args:      string (variable length)
  FrameCount:uint64
  [0x03 only] Fx,Fy,Cx,Cy: 4×double (intrinsics) · TagSizeM: double
            · CameraAlias: u32+string · TagFamily: u32+string

Frame (repeated):
  FrameIdx:  uint64
  PTP_Ns:    int64        (GPS-aligned nanoseconds; see spec)
  Size:      uint32
  [0x03 only] MetaSize:   uint32
  Data:      bytes (variable length)
  [0x03 only] Meta:       bytes (AprilTag records; see spec)
```

## AprilTag Detection (CameraNode)

Each CameraNode can perform local AprilTag detection with 3D pose estimation
(AprilTag 3 + OpenCV solvePnP). Detections are stamped with the frame's
GPS-aligned timestamp and travel two paths:

1. **Live**: published on NetworkTables table `AprilTag` — `tag_{id}_pose_translation`
   (3 doubles, meters), `tag_{id}_pose_rotation` (9 doubles, row-major), `tag_ids`,
   `tags_visible`, `ptp_ns`, `camera` (alias). Matches the robot_tracker topic layout.
2. **Recorded**: .taps v0x03 per-frame metadata (id, pose, decision margin, hamming,
   reprojection RMS, apparent tag size — see `common/taps_format.md`). The ServerNode
   fuses these across cameras for higher-confidence robot localization.

Enabling and tuning:

```bash
camera_node ... \
    --apriltag \
    --apriltag-family tag36h11 \
    --apriltag-tag-size 0.165 \
    --apriltag-fx 610 --apriltag-fy 610 --apriltag-cx 640 --apriltag-cy 360 \
    --apriltag-quad-decimation 2 \
    --apriltag-threads 1 \
    --camera-alias CameraNode1
```

Notes:
- Intrinsics load automatically from `calibration.json` (see **Camera Calibration
  Wizard** below); `--apriltag-fx/fy/cx/cy` explicitly passed on the CLI override it.
  With neither, poses use a documented default (f = frame size, c = center).
- `--apriltag-pose false` records detection-only metadata (no 3D pose).
- Detection runs on a background thread with latest-frame handoff; if it falls
  behind capture, frames are skipped for detection but video is never slowed.
- Build: CMake fetches AprilTag 3.4.3 automatically (`-DTAPS_ENABLE_APRILTAG=OFF`
  builds without it; the binary then warns if `--apriltag` is requested).

## Camera Calibration Wizard (one-time, headless)

Open `http://<camera-node>:8080/calibrate` — a guided four-step wizard that
replaces the old desktop `camera_calibration.py` helper and works entirely in
the browser against the live camera:

1. **Target** — shows whether intrinsics are already stored and when they were
   captured. Choose the checkerboard inner-corner count (pulldowns, default
   9×6 = a printed 10×7-square board) and the square size in mm (pulldowns +
   custom). A "Print board" button generates a scale-checked printable board.
   **Start calibration** begins even if intrinsics already exist (recalibrate).
2. **Focus test** — live view with a real-time *spatial-frequency sharpness
   score* (variance of the Laplacian measured on the digitally rectified
   checkerboard, so it is invariant to distance/position). Shows the current
   score, the session best, and a PASS indicator (≥60 % of the session best).
   Instructions: place the board ≥1 ft away, hold still, turn the focus ring to
   maximize the score, then lock it. **Next** proceeds.
3. **Capture coverage** — presents the live view with a green checkerboard
   overlay and a 3×3 FOV grid. Frames are stored only when the pattern is
   detected, the board is held still, and the view is *new* (≥40 px corner
   displacement from any stored frame). Coverage is enforced by nine FOV
   regions (`n/5` each — e.g. "lower-left 3/5") **and** orientation variety
   (front / left / right / up / down tilts, `n/4` each) — shown as fill meters
   on the page and drawn on the video itself. Auto-advances when full; a
   **Finish early** button unlocks after 24 well-distributed frames.
4. **Result** — runs `cv::calibrateCamera`, shows reprojection error, fx/fy/
   cx/cy, distortion coefficients and frames used, **writes `calibration.json`
   atomically** (timestamped, overwrites the previous calibration), deletes the
   temporary images, and live-updates the running AprilTag detector — no
   restart needed. *Return to main page* / *Repeat calibration* buttons.

An **"✕ Abort & return to main"** button is available in every active step
(focus, capture, and computing): it cancels the device-side session, deletes
the temporary images, and returns to the status page. Leaving the wizard via
the header link asks for confirmation and aborts as well — closing or
reloading the tab mid-session also aborts (beacon on page hide), so a session
can never be left running unattended.

Storage & precedence:

- Default file: `<--output>/calibration.json` (override with `--calibration-file`).
- Loaded at startup; applied only when `image_width/height` matches the current
  capture resolution (changing resolution invalidates it — recalibrate).
- Precedence: explicit CLI `--apriltag-fx/fy/cx/cy` > calibration.json >
  frame-derived defaults.
- The wizard works whether or not `--apriltag` is enabled (the intrinsics are
  still stored for later). Recording must be stopped before starting a session.
- `/` (status page) shows a calibration badge: present + captured date +
  reprojection error, or "missing".

Endpoints: `GET /calibrate` (wizard page), `GET /calib/status` (state/focus/
coverage JSON, polled by the page), `POST /calib/control`
(`start|next|finish|abort|reset`), `GET /calib/stream` (overlay MJPEG).

## Web Frontend

The ServerNode provides a web interface with:

- **Live Mode**: Real-time 2x2 grid view of all cameras
- **Playback Mode**: Frame-by-frame playback of recorded sessions with:
  - Play/Pause/Step controls
  - Speed control (0.25x–4x)
  - Per-stream rotation
  - Log message overlay synchronized with playback time
  - Session browser

## Configuration

### CameraNode (YAML)
```yaml
camera:
  device_id: 0          # /dev/video0
  width: 1920
  height: 1080
  fps: 60
  fourcc: "MJPG"

recording:
  output_dir: "/recordings"
  encoder: "jpeg"
  quality: 85

streaming:
  http_port: 8080
  compose_grid: true

pps:
  gpio_pin: 17

network_tables:
  server: "10.0.1.X"
  match_start_topic: "RoboRIO/matchStart"
  match_end_topic: "RoboRIO/matchEnd"

identity:
  name: "CameraNode1"
  index: 1
```

### ServerNode (YAML)
```yaml
cameras:
  - name: "CameraNode1"
    host: "10.0.1.11"
    http_port: 8080

network_tables:
  server: "10.0.1.X"
  match_start_topic: "RoboRIO/matchStart"
  match_end_topic: "RoboRIO/matchEnd"

smb_share:
  path: "\\\\10.0.2.X\\share\\logs"
  poll_interval: 30

web:
  host: "0.0.0.0"
  port: 8080
```

## API Reference

### CameraNode Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/stream` | GET | MJPEG single camera stream |
| `/grid-stream` | GET | MJPEG 2x2 composited grid |
| `/status` | GET | Current status JSON |
| `/recording` | POST | Start/stop recording (`{"action": "start/stop"}`) |
| `/events` | GET | SSE events (status, disk, cpu, mem) |
| `/files/` | GET | Browse recorded files |
| `/calibrate` | GET | Guided calibration wizard page |
| `/calib/status` | GET | Calibration state / focus score / coverage JSON |
| `/calib/control` | POST | `{"action":"start|next|finish|abort|reset", ...}` |
| `/calib/stream` | GET | MJPEG calibration view with overlays |

### ServerNode Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web frontend |
| `/api/cameras` | GET | Camera status list |
| `/api/grid/stream` | GET | Composited grid JPEG |
| `/api/sessions` | GET | Available sessions |
| `/api/session/{id}/frame` | GET | Current playback frame |
| `/api/session/{id}/logs` | GET | Log entries at current time |
| `/api/session/{id}/play` | POST | Start playback |
| `/api/session/{id}/pause` | POST | Pause/resume |
| `/api/session/{id}/step` | POST | Frame step |
| `/api/session/{id}/speed` | POST | Set speed |
| `/api/session/{id}/rotate` | POST | Set rotation |
| `/api/session/{id}/seek` | POST | Seek to time |
| `/ws` | WS | Real-time WebSocket |

## Development Notes

### Reused Code
- `.taps` reader/writer from C++ CameraNode project
- Log parser from playback_server project
- NetworkTables trigger patterns from robot_tracker project
- HTTP server patterns from C++ http_server.h
- Web frontend inspired by C++ templates/index.html

### Future Work
- [ ] CameraNode: Implement PPS handler (pps_handler.h)
- [ ] CameraNode: Implement NetworkTables client (nt_client.h)
- [ ] CameraNode: Implement grid composition endpoint
- [ ] ServerNode: Full SMB polling implementation (requires smbprotocol setup)
- [ ] ServerNode: WebSocket session loading
- [ ] Testing: End-to-end integration tests
- [ ] Documentation: Deployment guides for CameraNode and ServerNode
