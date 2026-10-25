#!/usr/bin/env bash
# TaPS CameraNode - Orange Pi 5 Plus basic functional checkout (SD-card phase).
# READ-ONLY checks; safe to run any time. Missing tools report SKIP, not fail.
# Intended for the Armbian current branch (mainline >= 6.6).
set -u

ok()   { printf '  [ OK ] %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; FAILS=$((FAILS+1)); }
skip() { printf '  [SKIP] %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; }
FAILS=0

echo "== 1. Board / kernel =="
[ -r /proc/device-tree/compatible ] && echo "  board: $(tr '\0' ' ' < /proc/device-tree/compatible)"
k="$(uname -r)"; echo "  kernel: $k"
case "$k" in 6.*|7.*) ok "mainline-era kernel";; *) bad "expected >= 6.6 mainline, got $k";; esac

echo "== 2. libgpiod (PPS capture mechanism) =="
if command -v gpiodetect >/dev/null 2>&1; then
    ok "gpiod tools present"
    gpiodetect 2>/dev/null | sed 's/^/         /'
    if command -v gpioinfo >/dev/null 2>&1; then
        NCHIP="$(gpiodetect 2>/dev/null | grep -c gpiochip)"
        if [ "$NCHIP" -ge 2 ]; then
            ok "$NCHIP gpiochips - RK3588 banks are separate chips; camera_node currently opens gpiochip0 only - see platform/opi5plus/README.md"
        else
            warn "only $NCHIP gpiochip visible - unusual for RK3588 mainline"
        fi
        echo "         edge test once GPS is wired: gpionotify <gpiochipN> <line>  (expect 1 event/s)"
    fi
else
    bad "gpiod tools missing (apt install gpiod libgpiod2 libgpiod-dev) - PPS cannot be verified"
fi

echo "== 3. libgpiod runtime for the build =="
LD="$(command -v ldconfig || echo /usr/sbin/ldconfig)"
if "$LD" -p 2>/dev/null | grep -q 'libgpiod.*\.so\.2'; then
    ok "libgpiod v2 runtime present (pps_handler needs the v2 C++ bindings)"
elif ldconfig -p 2>/dev/null | grep -q libgpiod; then
    bad "libgpiod present but NOT v2 - pps_handler (gpiod.hpp) will not build/run"
else
    bad "no libgpiod (apt install libgpiod-dev)"
fi

echo "== 4. Camera (UC-852) enumeration =="
if command -v v4l2-ctl >/dev/null 2>&1; then
    if v4l2-ctl --list-devices 2>/dev/null | grep -qi 'usb'; then
        ok "at least one video device present:"
        v4l2-ctl --list-devices 2>/dev/null | sed 's/^/         /'
        VID="$(v4l2-ctl --list-devices 2>/dev/null | grep -o '/dev/video[0-9]*' | head -1)"
        if [ -n "$VID" ] && v4l2-ctl -d "$VID" --list-formats-ext 2>/dev/null \
             | grep -A20 -i 'MJPG' | grep -qi '1280x800'; then
            ok "$VID lists MJPG 1280x800"
            v4l2-ctl -d "$VID" --list-formats-ext 2>/dev/null | grep -B1 -A12 -i 'MJPG' | grep -E '1280x800|Interval|Discrete' | sed 's/^/         /'
        else
            bad "$VID does NOT list MJPG 1280x800 - check USB port / power"
        fi
    else
        bad "no video devices - is the UC-852 plugged in (native port, no hub)?"
    fi
else
    skip "v4l-utils not installed (apt install v4l-utils)"
fi

echo "== 5. USB link of camera =="
if command -v lsusb >/dev/null 2>&1; then
    # UC-852 = 0c45:6366, a USB 2.0 high-speed device; native port, no hub
    if lsusb | grep -qi '0c45:6366'; then ok "UC-852 on lsusb (0c45:6366)"; else warn "UC-852 not on lsusb yet"; fi
fi

echo "== 6. Thermals =="
Z=0
for t in /sys/class/thermal/thermal_zone*/temp; do
    [ -r "$t" ] || continue
    TYPE="$(cat "$(dirname "$t")/type" 2>/dev/null)"
    V="$(cat "$t")"
    printf '         %-16s ' "$TYPE"; awk -v v="$V" 'BEGIN{printf "%.1f C\n", v/1000}'
    Z=$((Z+1))
done
if [ "$Z" -gt 0 ]; then ok "$Z thermal zones readable (expect ~40-65 C idle/light with active cooler)"; else skip "no thermal zones"; fi

echo "== 7. Storage =="
df -h / | awk 'NR==2{printf "         root: %s used of %s (%s free)\n", $3, $2, $4}'

echo "== 8. Network =="
for i in /sys/class/net/*/operstate; do
    n="$(basename "$(dirname "$i")")"; [ "$n" = lo ] && continue
    printf '         %-8s %s\n' "$n" "$(cat "$i")"
done

echo
if [ "$FAILS" -eq 0 ]; then
    echo "CHECKOUT: no hard failures (WARNs above point at next steps)"
else
    echo "CHECKOUT: $FAILS hard failure(s) - fix before building camera_node"
fi
exit "$FAILS"
