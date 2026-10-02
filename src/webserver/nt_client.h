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
#include <cstdint>
#include <thread>
#include <vector>

#include "../camera/apriltag_detector.h"

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

    // Publish AprilTag detections with poses on the "AprilTag" table.
    // Topic layout matches robot_tracker.py: tag_{id}_pose_translation (3 doubles),
    // tag_{id}_pose_rotation (9 doubles, row-major); plus tag_ids summary,
    // tags_visible count, ptp_ns capture timestamp, and camera alias.
    void publish_tag_poses(const std::vector<TagDetectionResult>& tags, int64_t ptp_ns);

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
    nt_table* m_tag_table = nullptr;

    // Resolved trigger topics: "RoboRIO/matchStart" -> table "RoboRIO", key
    // "matchStart". Reading a prefixed key through a table handle would double
    // the prefix (/RoboRIO/RoboRIO/matchStart), so topics are split at start().
    nt_table* m_start_table = nullptr;
    nt_table* m_end_table = nullptr;
    std::string m_start_key;
    std::string m_end_key;

    // Polling thread
    std::thread m_poll_thread;
};

#endif // TAPS_CAMERANODE_NT_CLIENT_H
