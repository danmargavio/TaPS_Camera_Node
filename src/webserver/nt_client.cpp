//
// NetworkTables Client Implementation (ntcore)
//
// Uses ntcore C++ API to communicate with RoboRIO.
// Polls for boolean values on match start/end topics.
// Falls back gracefully if ntcore library is not available.
//

#include "nt_client.h"
#include <spdlog/spdlog.h>

#ifdef HAVE_NT_CORE
#include <ntcore.h>
#include <ntcore_c.h>
#endif

#include <thread>

NTClient::NTClient()
    : connected_(false), started_(false) {
}

NTClient::~NTClient() {
    stop();
}

void NTClient::initialize(const std::string& server_address,
                          const std::string& match_start_topic,
                          const std::string& match_end_topic,
                          const std::string& local_alias) {
    server_address_ = server_address;
    match_start_topic_ = match_start_topic;
    match_end_topic_ = match_end_topic;
    local_alias_ = local_alias;

    spdlog::info("NTClient initialized: server={}, start={}, end={}, alias={}",
                 server_address_, match_start_topic_, match_end_topic_, local_alias_);
}

void NTClient::start() {
    if (started_) return;

#ifdef HAVE_NT_CORE
    try {
        // Initialize ntcore instance
        m_instance = nt_Initialize(nullptr, nullptr);
        if (!m_instance) {
            spdlog::warn("nt_Initialize failed, running without NT triggers");
            return;
        }

        // Set local name/alias
        nt_SetLocalName(m_instance, local_alias_.c_str());

        // Connect to server
        nt_StartClient(m_instance, server_address_.c_str(), nullptr);
        spdlog::info("NT client starting, connecting to {}", server_address_);

        // Get the RoboRIO table
        m_table = nt_GetTable(m_instance, "RoboRIO");
        if (!m_table) {
            spdlog::warn("Could not get RoboRIO table");
            nt_Deinitialize(m_instance);
            m_instance = nullptr;
            return;
        }

        connected_ = true;
        started_ = true;

        // Start polling thread
        m_poll_thread = std::thread(&NTClient::poll_loop, this);
        spdlog::info("NT polling thread started");

    } catch (const std::exception& e) {
        spdlog::warn("NT init error: {}. Running without NT triggers.", e.what());
        connected_ = false;
        started_ = false;
    }
#else
    spdlog::warn("ntcore C++ library not available. Running without NT triggers.");
    connected_ = false;
    started_ = false;
#endif
}

void NTClient::stop() {
    started_ = false;

#ifdef HAVE_NT_CORE
    if (m_poll_thread.joinable()) {
        m_poll_thread.join();
    }

    if (m_instance) {
        nt_StopClient(m_instance);
        nt_Deinitialize(m_instance);
        m_instance = nullptr;
    }
    m_table = nullptr;
#endif

    connected_ = false;
    spdlog::info("NTClient stopped");
}

void NTClient::set_recording_callback(std::function<void(bool)> cb) {
    recording_callback_ = std::move(cb);
}

void NTClient::publish_health(const std::string& state, int frame_count, double fps) {
#ifdef HAVE_NT_CORE
    if (m_table && connected_) {
        nt_PutString(m_table, "status", state.c_str());
        nt_PutDouble(m_table, "frame_count", static_cast<double>(frame_count));
        nt_PutDouble(m_table, "fps", fps);
    }
#else
    (void)state;
    (void)frame_count;
    (void)fps;
#endif
}

void NTClient::poll_loop() {
    spdlog::debug("NT poll loop started");

#ifdef HAVE_NT_CORE
    while (started_ && m_instance) {
        try {
            // Flush outgoing writes
            nt_Flush(m_instance);

            // Poll for incoming data
            nt_Poll(m_instance, 0.1);  // 100ms poll interval

            // Check match start
            bool match_start = false;
            if (m_table) {
                match_start = nt_GetBoolean(m_table, match_start_topic_.c_str(), false);
            }
            if (match_start && recording_callback_) {
                spdlog::debug("NT matchStart detected");
                recording_callback_(true);
            }

            // Check match end
            bool match_end = false;
            if (m_table) {
                match_end = nt_GetBoolean(m_table, match_end_topic_.c_str(), false);
            }
            if (match_end && recording_callback_) {
                spdlog::debug("NT matchEnd detected");
                recording_callback_(false);
            }

        } catch (const std::exception& e) {
            spdlog::warn("NT poll error: {}", e.what());
            if (!started_) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
#endif

    spdlog::debug("NT poll loop stopped");
}
