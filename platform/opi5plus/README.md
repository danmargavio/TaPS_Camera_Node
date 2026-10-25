# CameraNode on Orange Pi 5 Plus (RK3588)

Board-specific assets and bring-up notes. **The daemon itself is unchanged
and board-agnostic** - one binary, one repo; this folder holds only what
differs: PPS GPIO mapping guidance, setup notes, and the bench checkout
script.

> STATUS 2026-10-25: FULLY VERIFIED on the bench (Armbian 26.8 vendor,
> RK3588, OV9782 USB, GPS 1PPS wired to pin 11). Native build boots, cold
> power-cycle survives, live 1PPS locks with ~7 us jitter (1000.0 ms
> period), low-power toggle + record cycle writes valid .taps, web UI on
> 8080. Hostname `cameranode1` + mDNS: reach as `cameranode1.local`.

## Base image

Bench reality: **Armbian 26.8.2 vendor image** (kernel 6.1.115-vendor-rk35xx)
works and everything below is verified on it. Vendor vs mainline does not
matter for TaPS: both expose one gpiochip per bank (gpiochipN = GPIO bank N,
32 lines each; the extra gpiochip5 is the rk806 PMIC - not the header).

## PPS pin choice

The debug console (ttyS2) is on a separate onboard header, NOT on the
40-pin. Functional pins to avoid on the 40-pin: I2C2 (3/5), PWM14 (7),
SPI0 (19/21/23/24/26), and pins 8/10 (GPIO1_A1/A0 - UART3 mux group).
Free 3.3 V candidates (`--pps-gpio` values):

| Header pin | RK name | Spec | Notes |
|---|---|---|---|
| **11** | GPIO1_A4 | `gpiochip1:4` | **recommended - GND at adjacent pin 9** |
| 13 | GPIO1_A7 | `gpiochip1:7` | |
| 15 | GPIO1_B0 | `gpiochip1:8` | |
| 22 | GPIO1_A2 | `gpiochip1:2` | |
| 27 / 28 | GPIO1_B7 / B6 | `gpiochip1:15` / `:14` | |
| 29 | GPIO1_D7 | `gpiochip1:31` | |
| 32 | GPIO1_A3 | `gpiochip1:3` | |
| 31/33/35/37 | GPIO3_A0/C2/A2/C1 | `gpiochip3:0` / `:18` / `:2` / `:17` | |
| 12/16/18/36/38/40 | GPIO3_A1/B5/B6/A5/A4/A3 | `gpiochip3:1` / `:13` / `:14` / `:5` / `:4` / `:3` | |

Line offset within a bank = letter*8 + number (GPIO1_A4 -> A(0)*8 + 4 = 4).
Never wire 5 V to any header GPIO. Wire: 1PPS OUT -> pin 11, GND -> pin 9.

Permissions: udev rule `KERNEL=="gpiochip[0-9]*", SUBSYSTEM=="gpio",
GROUP="gpio", MODE="0660"` + service user in group gpio.

## PPS - libgpiod, NOT kernel pps-gpio (audited 2026-10-24)

`src/camera/pps_handler.cpp` captures 1PPS with **libgpiod v2 edge events**
(RISING, `CLOCK_REALTIME` timestamps). No `/dev/ppsN`, no device-tree
overlay - do not bother enabling the kernel pps subsystem.

- On the Pi every BCM GPIO lives on gpiochip0, so `--pps-gpio 24` works.
- On RK3588 each bank is its own gpiochip; the header exposes GPIO1-GPIO4
  only.
- `--pps-gpio` accepts `gpiochipN:LINE` or a bare number (legacy: gpiochip0
  line N). Verified on board: `gpiochip1:4`, `24`, `-1`, and garbage specs
  all behave correctly (help text, init log, fallback warnings).

## Build dependencies - verified recipe

**Never install `libopencv-dev`** on a constrained root: it is a metapackage
that drags VTK, Qt5, GDAL, HDF5, even AMD GPU runtimes (~1 GB). Install the
module-scoped set instead:

```bash
sudo apt install -y build-essential cmake pkg-config git automake libtool
sudo apt install -y libspdlog-dev libgpiod-dev libmicrohttpd-dev v4l-utils
sudo apt install -y libopencv-core-dev libopencv-imgproc-dev \
  libopencv-imgcodecs-dev libopencv-videoio-dev \
  libopencv-calib3d-dev libopencv-objdetect-dev
```

Debian ships `OpenCVConfig.cmake` only inside the metapackage, so add a
small shim - both files under `/usr/local/lib/cmake/OpenCV/`:

`OpenCVConfig.cmake`:

```cmake
set(OpenCV_FOUND TRUE)
set(OpenCV_VERSION 4.10.0)
set(OpenCV_INCLUDE_DIRS /usr/include/opencv4)
set(_taps_libs "")
foreach(m core imgproc imgcodecs videoio calib3d features2d objdetect)
  find_library(OpenCV_${m}_LIBRARY NAMES opencv_${m})
  if(OpenCV_${m}_LIBRARY)
    if(NOT TARGET opencv_${m})
      add_library(opencv_${m} UNKNOWN IMPORTED)
      set_target_properties(opencv_${m} PROPERTIES
        IMPORTED_LOCATION "${OpenCV_${m}_LIBRARY}")
    endif()
    list(APPEND _taps_libs opencv_${m})
  endif()
endforeach()
set(OpenCV_LIBS ${_taps_libs})
```

`OpenCVConfigVersion.cmake`:

```cmake
set(PACKAGE_VERSION 4.10.0)
if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_FIND_VERSION STREQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
```

Source libraries:

```bash
# AprilTag - examples need the umbrella header, build without them
git clone --depth 1 https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF
cmake --build build -j8
sudo cmake --install build && sudo ldconfig

# libhttpserver 2.0.0 - no autogen.sh in the tag; out-of-tree build required
git clone --depth 1 --branch 2.0.0 https://github.com/etr/libhttpserver.git
cd libhttpserver
autoreconf -i
mkdir b && cd b
../configure --disable-examples --disable-doc
make -j8 && sudo make install && sudo ldconfig
```

Disk note: on a 3.5 GB root this is tight but sufficient (keep ~400 MB free
via `sudo apt-get clean` after installs and deleting the source build trees
post-install). Recordings must NOT land on a card this size.

Then the usual `cmake -B build && cmake --build build -j8`. The CMakeLists
now lists `${OpenCV_INCLUDE_DIRS}` explicitly on TaPS_CameraNode - a latent
bug: it previously linked only against metapackage-provided global include
paths, which the shim (correctly) does not provide.

## Service and ports

camera_node serves its web UI on **8080** (8090 is ServerNode). Reuse the
systemd service unchanged except ExecStart `--pps-gpio gpiochip1:4` (or the
selected pin). Low-power boot default confirmed active on this board.

## Checkout script

`opi_checkout.sh` in this folder runs the read-only SD-phase checks
(kernel, libgpiod chips/lines, UVC enumeration + MJPG formats, thermal
zones, disk, network). Run as a normal user after flashing:
`bash opi_checkout.sh`.

## Wiring summary

| Signal | Orange Pi 5 Plus |
|---|---|
| GPS 1PPS | header pin 11 = GPIO1_A4 = `gpiochip1:4` (3.3 V ONLY) |
| GPS GND | header pin 9 |
| Camera | USB camera direct into a native USB port (no hub) |
| Power | USB-C PD, adequate wattage (bench: 65 W) |
| Cooling | active cooler advised under sustained AprilTag load; idle ~28 C |
