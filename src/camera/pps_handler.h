//
// PPS (Pulse Per Second) GPIO Handler
//
// Reads GPIO pin for 1 PPS signal using libgpiod.
// On each rising edge, records a hardware timestamp.
// Provides frame-accurate PPS-synced timestamps.
//

#ifndef TAPS_CAMERANODE_PPS_HANDLER_H
#define TAPS_CAMERANODE_PPS_HANDLER_H

#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>
#include <optional>

#ifdef __linux__
#include <gpiod.hpp>
#endif

class PPSHandler {
public:
    explicit PPSHandler(int gpio_pin);
    ~PPSHandler();

    // Initialize the GPIO pin for PPS input
    // Returns true if PPS is available, false if using fallback
    bool initialize();

    // Returns nanosecond timestamp from most recent PPS edge.
    // Falls back to system clock if PPS not available.
    std::chrono::nanoseconds get_timestamp();

    void shutdown();

    // Check if PPS hardware is available
    bool is_pps_available() const { return pps_available_; }

private:
    // Background thread that monitors PPS rising edge
    void pps_monitor_loop();

    int gpio_pin_;
    std::atomic<bool> running_;
    std::atomic<bool> pps_available_;

    // Latest PPS timestamp
    std::chrono::nanoseconds latest_pps_time_;
    std::mutex pps_mutex_;

#ifdef __linux__
    // libgpiod objects
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line> line_;
#endif

    // Fallback: system clock with PPS correction offset
    std::chrono::nanoseconds system_clock_offset_;
};

#endif // TAPS_CAMERANODE_PPS_HANDLER_H
