#!/usr/bin/env bash
# pps_watch.sh - BENCH DIAGNOSTIC for the GPS 1PPS input on GPIO24.
#
# Prints one line per rising edge: "<epoch.ns> edge <gpiomon line>".
# The rising edge marks the second start (active-LOW ~100 ms pulse;
# the line idles HIGH, so we watch RISING).
#
# IMPORTANT - GPIO line requests are EXCLUSIVE: a CameraNode running
# with --pps-gpio 24 owns the line and gpiomon fails with
# "Device or resource busy". This watcher therefore:
#   - checks the flag file /tmp/pps_pause every pass and sleeps 5 s
#     while it exists (touch /tmp/pps_pause to make it yield), and
#   - backs off 5 s whenever gpiomon reports the line busy.
# It never signals other processes. Preferred usage: run only while
# the camera node is stopped (systemctl stop camera-node - SIGTERM,
# never kill -9).

set -u

CHIP="${PPS_CHIP:-gpiochip0}"
PIN="${PPS_GPIO:-24}"
EVENTS="${PPS_EVENTS:-120}"
PAUSE_FLAG=/tmp/pps_pause
BUSY_MARK=/tmp/pps_watch_busy
trap 'rm -f "$BUSY_MARK"' EXIT

echo "pps_watch: chip=$CHIP line=$PIN rising-edge x$EVENTS (yield while $PAUSE_FLAG exists)"

while true; do
    if [ -e "$PAUSE_FLAG" ]; then
        sleep 5
        continue
    fi

    rm -f "$BUSY_MARK"
    timeout 135 gpiomon --rising-edge --num-events "$EVENTS" "$CHIP" "$PIN" 2>&1 |
    while IFS= read -r line; do
        case "$line" in
            *busy*)
                echo "$(date +%s.%N) pps_watch: line busy (camera node running?) - backing off 5 s"
                touch "$BUSY_MARK"
                ;;
            *RISING*)
                echo "$(date +%s.%N) edge $line"
                ;;
        esac
    done

    if [ -e "$BUSY_MARK" ]; then
        sleep 5
    else
        sleep 1
    fi
done
