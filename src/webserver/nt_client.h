//
// NetworkTables Client for CameraNode (ntcore)
//
// Listens for matchStart/matchEnd boolean topics from RoboRIO.
// Triggers recording start/stop via VideoRecordThread.
// Publishes camera health status.
// Uses ntcore (pyntcore) C++ API.
//

#ifndef TAPS_CAMERANODE_NT_CLIENT_H
#define TAPS_CAMERANODE_NT_CLIENT_H

#include <string>
#include <functional>
#include <atomic>

// Forward declaration for ntcore types
struct nt_instance;
struct nt_table;
struct nt_listener;

class NTClient {
public:
    NTClient();
    ~NTClient();

    // Initialize with RoboRIO address and topic names
    void initialize(const std::string& server_address,
                    const std::string& match_start_topic,
                    const std::string& match_end_topic,
                    const std::string& local_alias);

    // Start listening for events
    void start();

    // Stop and cleanup
    void stop();

    // Set callback for recording start/stop
    void set_recording_callback(std::function<void(bool)> cb);

    // Publish camera health status
    void publish_health(const std::string& state, int frame_count, double fps);

    // Check if NT is connected
    bool is_connected() const { return connected_; }

private:
    // Polling thread function
    void poll_loop();

    std::string server_address_;
    std::string match_start_topic_;
    std::string match_end_topic_;
    std::string local_alias_;

    std::atomic<bool> connected_;
    std::atomic<bool> started_;

    std::function<void(bool)> recording_callback_;

    // ntcore handles (opaque pointers)
    nt_instance* m_instance = nullptr;
    nt_table* m_table = nullptr;

    // Polling thread
    std::thread m_poll_thread;
};

#endif // TAPS_CAMERANODE_NT_CLIENT_H
