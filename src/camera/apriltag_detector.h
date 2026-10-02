//
// AprilTag detector for CameraNode — local target location determination.
//
// Runs AprilTag 3 detection on a background thread (latest-frame handoff, same
// pattern as robot_tracker.py's AprilTagPipeline), estimates each tag's 3D pose
// in the camera frame via solvePnP, and attaches quality/uncertainty metrics
// used by the ServerNode's multi-camera fusion:
//   - decision_margin : decode confidence (NOT pose uncertainty)
//   - hamming         : corrected code bits (0 = perfect)
//   - reproj_error_rms_px : RMS reprojection error of the 4 tag corners —
//                     primary pose-uncertainty proxy for fusion weighting
//   - tag_px_diag     : apparent tag diagonal in pixels (distance cue)
//
// Results are keyed by capture sequence number so the video recorder can
// serialize the detections belonging to each recorded frame into the .taps
// v0x03 per-frame metadata block (see common/taps_format.md).
//
// Compiles to a no-op stub when HAVE_APRILTAG is undefined.
//

#ifndef TAPS_CAMERANODE_APRILTAG_DETECTOR_H
#define TAPS_CAMERANODE_APRILTAG_DETECTOR_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

struct TagDetectionResult {
    uint16_t tagId = 0;
    float decisionMargin = 0.0f;   // AprilTag decode confidence
    uint8_t hamming = 0;           // code bits corrected
    bool poseValid = false;
    double translation[3] = {0.0, 0.0, 0.0};  // meters, camera frame (+x right, +y down, +z forward)
    double rotation[9] = {0.0};               // row-major 3x3 camera<-tag rotation matrix
    float reprojErrorRmsPx = 0.0f; // pose-uncertainty proxy
    float tagPxDiag = 0.0f;        // apparent tag diagonal (px)
};

struct TagBatch {
    uint64_t seq = 0;              // capture sequence of the detected frame
    int64_t ptpNs = 0;             // GPS-aligned timestamp of that frame
    std::vector<TagDetectionResult> tags;
};

class AprilTagDetector {
public:
    struct Config {
        bool enabled = false;
        std::string family = "tag36h11"; // tag36h11|tag25h9|tag16h5|tagStandard41h12|tagStandard52h13
        int threads = 1;
        double quadDecimation = 2.0;     // 1..4: higher = faster, coarser
        double blur = 0.0;               // -1 = auto, 0 = none
        bool estimatePose = true;
        double tagSizeMeters = 0.165;
        double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0; // 0 => derive defaults from frame size
    };

    static constexpr size_t kMaxTagsPerFrame = 16; // .taps v0x03 meta hard cap (u8 count)
    static constexpr size_t kResultHistory = 512;  // pending/completed seq results retained

    explicit AprilTagDetector(Config cfg);

    AprilTagDetector(const AprilTagDetector &) = delete;

    AprilTagDetector &operator=(const AprilTagDetector &) = delete;

    ~AprilTagDetector();

    // Returns false when disabled or when AprilTag support was not compiled in.
    bool start();

    void stop();

    bool is_enabled() const { return m_cfg.enabled; }

    // True only when the detection thread actually started (enabled AND
    // AprilTag support compiled in AND start() succeeded).
    bool is_running() const { return m_running; }

    // Capture-thread call: hands the newest frame to the detection thread.
    // Cheap when disabled. Clones the frame once.
    void submit(const cv::Mat &frameBgr, uint64_t seq, int64_t ptpNs);

    // Recorder-thread call: detections for a specific capture sequence, waiting
    // up to waitMs for the detection thread to reach it. Empty batch on timeout
    // or when the entry was already evicted — never blocks shutdown.
    TagBatch resultsForSeq(uint64_t seq, int waitMs);

    // Live-path call (NT publishing): most recently completed batch.
    TagBatch latestBatch() const;

    // Thread-safe live intrinsic update (web calibration wizard). Values <= 0
    // mean "derive from frame size". Takes effect on the next detection frame
    // and the next .taps session header.
    void updateIntrinsics(double fx, double fy, double cx, double cy);

    // Currently effective intrinsic values (raw; <= 0 = not calibrated).
    void intrinsics(double &fx, double &fy, double &cx, double &cy) const;

    uint64_t batchesCompleted() const { return m_batchesCompleted.load(); }

    uint64_t tagsDetectedTotal() const { return m_tagsDetectedTotal.load(); }

private:
    void processLoop();

    TagBatch detectFrame(const cv::Mat &frameBgr, uint64_t seq, int64_t ptpNs);

    Config m_cfg;
    std::atomic<double> m_fx, m_fy, m_cx, m_cy; // live intrinsics (calibration wizard)
    void *m_det = nullptr;    // apriltag_detector_t* when HAVE_APRILTAG
    void *m_family = nullptr; // apriltag_family_t*
    std::thread m_thread;
    std::atomic<bool> m_stopFlag{false};
    std::atomic<bool> m_running{false};

    mutable std::mutex m_frameMutex;
    std::condition_variable m_frameCv;
    cv::Mat m_pendingFrame; // BGR clone, guarded by m_frameMutex
    uint64_t m_pendingSeq = 0;
    int64_t m_pendingPtpNs = 0;
    bool m_pendingReady = false;

    mutable std::mutex m_resultMutex;
    std::condition_variable m_resultCv;
    std::map<uint64_t, TagBatch> m_results;
    std::deque<uint64_t> m_resultOrder;
    uint64_t m_lastCompletedSeq = 0; // monotonic (single capture thread, FIFO)
    TagBatch m_latest;

    std::atomic<uint64_t> m_batchesCompleted{0};
    std::atomic<uint64_t> m_tagsDetectedTotal{0};
};

#endif //TAPS_CAMERANODE_APRILTAG_DETECTOR_H
