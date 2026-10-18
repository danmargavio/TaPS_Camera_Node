//
// PPS (Pulse Per Second) GPIO Handler
//
// Reads GPIO pin for 1 PPS signal using libgpiod v2 (character device API).
// On each rising edge, records the kernel REALTIME timestamp, which is
// disciplined by chrony against the GPS PPS source.
//

#ifndef TAPS_CAMERANODE_PPS_HANDLER_H
#define TAPS_CAMERANODE_PPS_HANDLER_H

#include <chrono>
#include <atomic>
#include <thread>
#include <memory>
#include <mutex>

#ifdef HAVE_GPIOD
#include <gpiod.hpp>
#endif

class PPSHandler {
public:
    explicit PPSHandler(int gpio_pin);
    ~PPSHandler();

    // Request the GPIO line for rising-edge events.
    // Returns true if PPS is available, false if falling back to system clock.
    bool initialize();

    // PPS-disciplined nanosecond CLOCK_REALTIME timestamp: the real-time time
    // of the most recent 1PPS edge, plus the elapsed time since that edge
    // measured on the monotonic clock. This interpolates smooth, strictly
    // increasing timestamps between edges (raw edge times repeat once/sec).
    // Falls back to the plain system clock before the first edge arrives.
    std::chrono::nanoseconds get_timestamp();

    void shutdown();

    // Check if PPS hardware is available
    bool is_pps_available() const { return pps_available_; }

    // Snapshot for the web frontend (/system endpoint + SSE)
    struct Stats {
        bool configured = false;   // a --pps-gpio pin was requested
        bool available = false;    // GPIO line acquired
        bool locked = false;       // an edge arrived within the last 2.5 s
        bool ever_locked = false;  // at least one edge since startup
        uint64_t edges = 0;        // total rising edges seen
        uint64_t dropouts = 0;     // signal-loss events (2.5 s gaps)
        double since_edge_s = -1;  // seconds since last edge (-1 = none yet)
        double period_ms = 0;      // smoothed measured 1PPS period (0 = no data)
        double jitter_us = 0;      // smoothed |period - 1 s| in microseconds
        int pin = -1;
    };
    Stats stats() const;

private:
    // Background thread that monitors PPS rising edges
    void pps_monitor_loop();

    int gpio_pin_;
    std::atomic<bool> running_{false};
    std::atomic<bool> pps_available_{false};
    std::atomic<uint64_t> edge_count_{0};
    std::atomic<uint64_t> dropout_count_{0};
    std::atomic<double> ema_period_ms_{0};
    std::atomic<double> ema_jitter_us_{0};

    // Latest PPS timestamp (CLOCK_REALTIME ns)
    std::chrono::nanoseconds latest_pps_time_{0};
    // CLOCK_MONOTONIC (steady_clock) reading taken when the edge was handled.
    // get_timestamp() extrapolates real-time forward from latest_pps_time_ by
    // the monotonic delta, yielding smooth per-frame timestamps between edges.
    std::chrono::nanoseconds edge_monotonic_{0};
    std::atomic<bool> have_edge_{false};
    mutable std::mutex pps_mutex_;
    std::thread monitor_thread_;

#ifdef HAVE_GPIOD
    // libgpiod v2 objects
    std::unique_ptr<gpiod::chip> chip_;
    std::unique_ptr<gpiod::line_request> request_;
#endif
};

#endif //TAPS_CAMERANODE_PPS_HANDLER_H
