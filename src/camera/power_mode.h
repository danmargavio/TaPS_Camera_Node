//
// Low power mode: gates camera capture + AprilTag processing so an idle /
// offline node stays thermally quiet (capture loop releases /dev/video0 and
// parks on a condvar: near-zero CPU, sensor and USB link idle).
//
// Always kept alive so the node stays reachable: HTTP server + web UI, SSE,
// PPS time sync, NetworkTables.
//
// Default at boot is LOW POWER (per operator decision 2026-10-18: reboot
// lands quiet). Pass --start-capturing to opt out, or toggle from the web UI
// / POST {"enable":false} to /lowpower.
//

#ifndef TAPS_CAMERANODE_POWER_MODE_H
#define TAPS_CAMERANODE_POWER_MODE_H

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace PowerMode {

inline std::mutex g_mtx;
inline std::condition_variable g_cv;
inline std::atomic<bool> g_lowPower{true};

inline bool isLowPower() { return g_lowPower.load(std::memory_order_acquire); }

// Changing the flag and notifying under the mutex avoids the lost-wakeup
// race between the capture loop's predicate check and its wait.
inline void request(const bool lowPower) {
    {
        std::lock_guard lock(g_mtx);
        g_lowPower.store(lowPower, std::memory_order_release);
    }
    g_cv.notify_all();
}

// Blocks the calling thread while low power mode is active.
inline void waitWhileLowPower() {
    std::unique_lock lock(g_mtx);
    g_cv.wait(lock, [] { return !g_lowPower.load(std::memory_order_acquire); });
}

}  // namespace PowerMode

#endif  // TAPS_CAMERANODE_POWER_MODE_H
