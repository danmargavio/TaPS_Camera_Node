//
// PPS Handler Implementation (libgpiod v2 C++ bindings)
//

#include "pps_handler.h"
#include <spdlog/spdlog.h>
#include <cmath>

#ifdef HAVE_GPIOD
#include <gpiod.hpp>
#endif

PPSHandler::PPSHandler(std::string spec)
    : spec_(std::move(spec)) {
    latest_pps_time_ = std::chrono::system_clock::now().time_since_epoch();
}

PPSHandler::~PPSHandler() {
    shutdown();
}

bool PPSHandler::initialize() {
    // Parse the --pps-gpio spec. Accepted forms:
    //   "24"            -> gpiochip0, line 24 (legacy, Raspberry Pi BCM numbering)
    //   "gpiochipN:L"   -> gpiochipN, line L  (multi-chip boards: RK3588 exposes
    //                       one gpiochip per bank; the 40-pin header lives on
    //                       gpiochip1..4, so a bare Pi-style number is wrong
    //                       there and the explicit form must be used)
    //   "-1" / empty    -> disabled
    std::string spec = spec_;
    spec.erase(0, spec.find_first_not_of(" \t\r\n"));
    spec.erase(spec.find_last_not_of(" \t\r\n") + 1);

    bool parsed = false;
    if (spec.empty()) {
        spec = "-1";
    }
    if (spec == "-1") {
        spdlog::warn("PPS GPIO not configured (--pps-gpio -1), using system clock");
        return false;
    }

    const auto colon = spec.find(':');
    try {
        if (colon == std::string::npos) {
            const int line = std::stoi(spec);
            if (line >= 0) {
                chip_name_ = "gpiochip0";
                gpio_line_ = line;
                parsed = true;
            }
        } else {
            const std::string chip = spec.substr(0, colon);
            const int line = std::stoi(spec.substr(colon + 1));
            if (line >= 0 && chip.rfind("gpiochip", 0) == 0) {
                chip_name_ = chip;
                gpio_line_ = line;
                parsed = true;
            }
        }
    } catch (const std::exception &) {
        parsed = false;
    }

    if (!parsed) {
        chip_name_.clear();
        gpio_line_ = -1;
        spdlog::warn("Invalid --pps-gpio spec '{}' - expected 'N' (gpiochip0 line) "
                     "or 'gpiochipN:LINE' (-1 = disabled). Using system clock.",
                     spec_);
        return false;
    }

#ifdef HAVE_GPIOD
    try {
        // Open the GPIO chip selected by the spec.
        chip_ = std::make_unique<gpiod::chip>("/dev/" + chip_name_);

        // Rising-edge events timestamped by the kernel CLOCK_REALTIME —
        // disciplined by chrony once the GPS PPS source is configured.
        // Internal pull-up: GPS PPS outputs are commonly open-collector and
        // float otherwise (confirmed needed on this carrier).
        gpiod::line_settings settings;
        settings.set_direction(gpiod::line::direction::INPUT)
                .set_edge_detection(gpiod::line::edge::RISING)
                .set_event_clock(gpiod::line::clock::REALTIME)
                .set_bias(gpiod::line::bias::PULL_UP);

        request_ = std::make_unique<gpiod::line_request>(
            chip_->prepare_request()
                .set_consumer("taps_camera")
                .set_event_buffer_size(1024)
                .add_line_settings(static_cast<unsigned int>(gpio_line_), settings)
                .do_request());

        spdlog::info("PPS handler initialized on {} line {}", chip_name_, gpio_line_);
        pps_available_ = true;
        running_ = true;
        monitor_thread_ = std::thread(&PPSHandler::pps_monitor_loop, this);
        return true;

    } catch (const std::exception &e) {
        spdlog::warn("PPS init error: {}. Falling back to system clock.", e.what());
        request_.reset();
        chip_.reset();
        pps_available_ = false;
        return false;
    }
#else
    spdlog::warn("Built without libgpiod — using system clock for timestamps");
    return false;
#endif
}

std::chrono::nanoseconds PPSHandler::get_timestamp() {
    if (pps_available_ && have_edge_.load()) {
        std::lock_guard<std::mutex> lock(pps_mutex_);
        // Real-time of the last edge + monotonic elapsed since it. Smooth and
        // strictly increasing at any frame rate; immune to clock steps between
        // edges. Fresh edges pull it back onto the GPS second boundary.
        const auto mono_now =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch());
        return latest_pps_time_ + (mono_now - edge_monotonic_);
    }
    return std::chrono::system_clock::now().time_since_epoch();
}

void PPSHandler::shutdown() {
    running_ = false;
    if (monitor_thread_.joinable())
        monitor_thread_.join();

#ifdef HAVE_GPIOD
    try {
        if (request_) request_->release();
    } catch (...) { /* already released by destructor */ }
    request_.reset();
    chip_.reset();
#endif
    pps_available_ = false;
    spdlog::debug("PPS handler shutdown");
}

void PPSHandler::pps_monitor_loop() {
    spdlog::debug("PPS monitor thread started");

#ifdef HAVE_GPIOD
    gpiod::edge_event_buffer buffer(32);
    bool signal_lost_logged = false;
    while (running_) {
        try {
            if (!request_ || !request_->wait_edge_events(std::chrono::milliseconds(100))) {
                // Watchdog: PPS expected every second. Warn once when it stops
                // (weak GPS fix), report again when it returns.
                if (have_edge_.load() && !signal_lost_logged) {
                    std::lock_guard<std::mutex> lock(pps_mutex_);
                    const auto since =
                            std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch()) -
                            edge_monotonic_;
                    if (since > std::chrono::milliseconds(2500)) {
                        signal_lost_logged = true;
                        dropout_count_.fetch_add(1, std::memory_order_relaxed);
                        spdlog::warn("PPS signal lost ({:.1f}s since last edge); timestamps "
                                     "hold local-clock pacing until it returns",
                                     std::chrono::duration<double>(since).count());
                    }
                }
                continue;
            }

            const std::size_t n = request_->read_edge_events(buffer);
            for (std::size_t i = 0; i < n; ++i) {
                const gpiod::edge_event &ev = buffer.get_event(static_cast<unsigned int>(i));
                if (ev.type() != gpiod::edge_event::event_type::RISING_EDGE)
                    continue;

                // Kernel REALTIME timestamp of the exact edge, in ns.
                const auto edge_ns = std::chrono::nanoseconds(ev.timestamp_ns().ns());
                // Same-instant monotonic reading for between-edge interpolation.
                const auto mono =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch());
                {
                    std::lock_guard<std::mutex> lock(pps_mutex_);
                    // Period/jitter tracking: only gaps that look like genuine
                    // consecutive seconds (dropouts are excluded by the window).
                    if (have_edge_.load()) {
                        const double gap_s =
                                std::chrono::duration<double>(edge_ns - latest_pps_time_).count();
                        if (gap_s > 0.9 && gap_s < 1.1) {
                            const double prev_p = ema_period_ms_.load(std::memory_order_relaxed);
                            const double prev_j = ema_jitter_us_.load(std::memory_order_relaxed);
                            const double period_ms = gap_s * 1000.0;
                            ema_period_ms_.store(prev_p > 0 ? prev_p * 0.8 + period_ms * 0.2 : period_ms,
                                                 std::memory_order_relaxed);
                            const double jit_us = std::abs(period_ms - 1000.0) * 1000.0;
                            ema_jitter_us_.store(prev_j > 0 ? prev_j * 0.8 + jit_us * 0.2 : jit_us,
                                                 std::memory_order_relaxed);
                        }
                    }
                    latest_pps_time_ = edge_ns;
                    edge_monotonic_ = mono;
                }
                edge_count_.fetch_add(1, std::memory_order_relaxed);
                if (!have_edge_.exchange(true))
                    spdlog::info("PPS LOCKED: first 1PPS edge at realtime ns {}", edge_ns.count());
                else if (signal_lost_logged) {
                    signal_lost_logged = false;
                    spdlog::info("PPS reacquired after dropout");
                }
            }
        } catch (const std::exception &e) {
            spdlog::error("PPS monitor error: {}", e.what());
            if (!running_) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
#endif

    spdlog::debug("PPS monitor thread stopped");
}

PPSHandler::Stats PPSHandler::stats() const {
    Stats st;
    st.configured = gpio_line_ >= 0;
    st.available = pps_available_.load();
    st.edges = edge_count_.load(std::memory_order_relaxed);
    st.dropouts = dropout_count_.load(std::memory_order_relaxed);
    st.period_ms = ema_period_ms_.load(std::memory_order_relaxed);
    st.jitter_us = ema_jitter_us_.load(std::memory_order_relaxed);
    st.pin = gpio_line_;
    st.chip = chip_name_;
    st.ever_locked = have_edge_.load();
    if (have_edge_.load()) {
        std::lock_guard<std::mutex> lock(pps_mutex_);
        const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch());
        const auto since = now - edge_monotonic_;
        st.since_edge_s = std::chrono::duration<double>(since).count();
        st.locked = st.since_edge_s < 2.5;
    }
    return st;
}
