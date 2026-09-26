//
// PPS Handler Implementation
//

#include "pps_handler.h"
#include <spdlog/spdlog.h>

#ifdef __linux__
#include <gpiod.hpp>
#endif

PPSHandler::PPSHandler(int gpio_pin)
    : gpio_pin_(gpio_pin), running_(false), pps_available_(false) {
    latest_pps_time_ = std::chrono::high_resolution_clock::now().time_since_epoch();
    system_clock_offset_ = std::chrono::nanoseconds(0);
}

PPSHandler::~PPSHandler() {
    shutdown();
}

bool PPSHandler::initialize() {
    if (gpio_pin_ < 0) {
        spdlog::warn("PPS GPIO pin not configured (pin < 0), using system clock");
        return false;
    }

#ifdef __linux__
    try {
        // Open GPIO chip (typically /dev/gpiochip0)
        chip_ = std::make_unique<gpiod::chip>("gpiochip0");
        if (!chip_) {
            spdlog::warn("Could not open gpiochip0, falling back to system clock");
            return false;
        }

        // Request the line as input with rising edge event
        line_ = chip_->get_line(gpio_pin_);
        if (!line_) {
            spdlog::warn("Could not get GPIO line {}, falling back to system clock", gpio_pin_);
            return false;
        }

        // Configure as input
        line_.release();
        line_ = chip_->request_line(gpiod::line_request{
            .function = "pps",
            .consumer = "taps_camera",
            .request_type = gpiod::line_request::REQUEST_TYPE_EVENT,
            .event_settings = gpiod::line_request::event_settings{
                .base = gpio_pin_,
                .flags = gpiod::line_request::event_flags::EDGE_RISING,
                .buffer_size = 1024,
            }
        });

        if (!line_) {
            spdlog::warn("Could not request GPIO line {} for PPS, falling back", gpio_pin_);
            return false;
        }

        spdlog::info("PPS handler initialized on GPIO {}", gpio_pin_);
        pps_available_ = true;
        running_ = true;

        // Start background monitoring thread
        std::thread(&PPSHandler::pps_monitor_loop, this).detach();
        return true;

    } catch (const std::exception& e) {
        spdlog::warn("PPS init error: {}. Falling back to system clock.", e.what());
        return false;
    }
#else
    // Not Linux - use system clock
    spdlog::warn("PPS handler only supported on Linux, using system clock");
    return false;
#endif
}

std::chrono::nanoseconds PPSHandler::get_timestamp() {
    // If PPS is available, return the latest PPS-synced timestamp
    // Otherwise return system clock time
    if (pps_available_) {
        std::lock_guard<std::mutex> lock(pps_mutex_);
        return latest_pps_time_;
    }
    return std::chrono::high_resolution_clock::now().time_since_epoch();
}

void PPSHandler::shutdown() {
    running_ = false;
#ifdef __linux__
    if (line_) {
        line_.release();
    }
    chip_.reset();
    line_.reset();
#endif
    spdlog::debug("PPS handler shutdown");
}

void PPSHandler::pps_monitor_loop() {
    spdlog::debug("PPS monitor thread started");

#ifdef __linux__
    while (running_) {
        try {
            // Wait for rising edge event with timeout
            auto event = line_.wait(std::chrono::milliseconds(100));

            if (!event) {
                continue; // Timeout, continue waiting
            }

            if (event.type() == gpiod::line_event::TYPE_EDGE_RISING) {
                // Record the PPS timestamp
                auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
                {
                    std::lock_guard<std::mutex> lock(pps_mutex_);
                    latest_pps_time_ = now;
                }

                // Calculate offset from system clock for precision timing
                // The PPS edge marks the exact start of a second
                auto ns_in_second = now.count() % 1'000'000'000;
                if (ns_in_second < 10'000'000) { // Within 10ms of second boundary
                    system_clock_offset_ = std::chrono::nanoseconds(-static_cast<int64_t>(ns_in_second));
                }
            }
        } catch (const std::exception& e) {
            spdlog::error("PPS monitor error: {}", e.what());
            if (!running_) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
#endif

    spdlog::debug("PPS monitor thread stopped");
}
