//
// Guided one-time camera calibration for the headless CameraNode webpage.
//
// Ports robot_tracker/camera_calibration.py into an interactive state machine:
//
//   Idle → Focus → Collect → (auto) Processing → Done → Idle
//
// Focus:      spatial-frequency sharpness (Laplacian variance measured on the
//             rectified checkerboard so it is distance/orientation invariant),
//             current + session-best + pass/fail vs the session best.
// Collect:    multi-fallback checkerboard detection, stillness/cooldown/
//             uniqueness gating, coverage tracking over a 3x3 FOV grid and
//             tilt/orientation categories, frames saved to a temp dir.
// Process:    cv::calibrateCamera, summary, atomic write of calibration.json,
//             temp images deleted.
//
// calibration.json is the non-volatile store read at startup to seed the
// AprilTag detector intrinsics (CLI flags still win if explicitly passed).
//

#ifndef TAPS_CAMERANODE_CALIBRATION_H
#define TAPS_CAMERANODE_CALIBRATION_H

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

struct SavedCalibration {
    bool valid = false;
    std::string calibratedAt; // ISO8601 UTC
    int imageWidth = 0;
    int imageHeight = 0;
    int patternW = 0; // inner corners
    int patternH = 0;
    double squareMm = 0.0;
    int framesUsed = 0;
    double reprojErrorPx = 0.0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    std::vector<double> distortion; // k1 k2 p1 p2 k3

    nlohmann::json toJson() const;
};

class CalibrationSession {
public:
    // Coverage goals (tunable): each of 9 FOV cells needs kCellGoal samples;
    // each of 5 orientation categories needs kTiltGoal samples. Early finish
    // requires at least kMinTotal frames with every cell >= 1 and >= 2 of the
    // non-frontal categories represented.
    static constexpr int kCellGoal = 5;
    static constexpr int kTiltGoal = 4;
    static constexpr int kMinTotal = 24;

    enum class Stage { Idle, Focus, Collect, Processing, Done, Failed };

    // Load any existing calibration.json (status display + startup seeding).
    static void init(const fs::path &calibrationFilePath, const std::string &cameraAlias);
    static void shutdown();

    // Read the persisted calibration (thread-safe copy).
    static SavedCalibration saved();

    static bool active(); // Focus/Collect/Processing running
    static void submitFrame(const cv::Mat &frameBgr); // cheap no-op when idle

    // GET /calib/status payload
    static nlohmann::json statusJson();

    // POST /calib/control payload handler.
    // actions: start {pattern_w, pattern_h, square_mm} | next | finish | abort | reset
    static nlohmann::json control(const nlohmann::json &body);

    // Called after a successful calibration.json write (live intrinsic update).
    static void setOnSaved(std::function<void(double fx, double fy, double cx, double cy)> cb);

    // Stream plumbing (read by HttpServer's MJPEG producer, same pattern as s_latestJpeg)
    static std::mutex s_jpegMutex;
    static std::vector<uint8_t> s_latestJpeg;
    static std::atomic<uint64_t> s_jpegSequence;

private:
    static const char *stageName(Stage s);
    static Stage stage() { return static_cast<Stage>(s_stage.load()); }
    static void setStage(Stage s) { s_stage.store(static_cast<int>(s)); }

    static void workerLoop();
    // Per-stage processing of one frame; draws overlay into `preview`.
    static void focusPass(const cv::Mat &gray, cv::Mat &preview);
    static void collectPass(const cv::Mat &gray, const cv::Mat &frameBgr, cv::Mat &preview);
    static void processCollected(); // runs on worker thread when requested

    // Checkerboard detection with fallbacks (scaled → standard → equalized → SB).
    static bool detectBoard(const cv::Mat &gray, const cv::Size &pattern,
                            std::vector<cv::Point2f> &corners);
    // Spatial-frequency sharpness of the rectified board (distance invariant).
    static double boardFocusScore(const cv::Mat &gray, const std::vector<cv::Point2f> &corners);

    static void clearCollectedLocked(); // RAM points + temp dir

    static nlohmann::json configJsonLocked();

    static inline std::atomic<int> s_stage{static_cast<int>(Stage::Idle)};
    static inline std::atomic<bool> s_stop{false};
    static inline std::atomic<bool> s_processRequested{false};

    static inline std::thread s_worker;

    // frame handoff (latest wins)
    static inline std::mutex s_frameMutex;
    static inline std::condition_variable s_frameCv;
    static inline cv::Mat s_pendingFrame;
    static inline bool s_frameReady = false;

    // config + state (guarded by s_stateMutex)
    static inline std::mutex s_stateMutex;
    static inline int s_patternW = 9;
    static inline int s_patternH = 6;
    static inline double s_squareMm = 25.0;
    static inline std::string s_error;

    // focus stage
    static inline double s_focusCurrent = 0.0;
    static inline double s_focusBest = 0.0;
    static inline bool s_focusBoardSeen = false;

    // collect stage
    static inline std::vector<std::vector<cv::Point3f>> s_objPoints;
    static inline std::vector<std::vector<cv::Point2f>> s_imgPoints;
    static inline std::vector<std::vector<cv::Point2f>> s_acceptedCorners; // for uniqueness
    static inline int s_cells[9] = {0};
    static inline int s_tilts[5] = {0}; // front, left, right, up, down
    static inline std::array<std::chrono::steady_clock::time_point, 9> s_cellCooldown{};
    static inline std::vector<cv::Point2f> s_prevCorners;
    static inline bool s_hadPrev = false;
    static inline int s_lastTotal = 0;

    // results / persistence
    static inline fs::path s_calibPath;
    static inline fs::path s_tmpDir;
    static inline std::string s_alias;
    static inline SavedCalibration s_saved;
    static inline nlohmann::json s_resultJson; // Done payload
    static inline std::mutex s_savedMutex;
    static inline std::mutex s_resultMutex;

    static inline std::mutex s_cbMutex;
    static inline std::function<void(double, double, double, double)> s_onSaved;

    static inline cv::Size s_frameSize; // full-resolution capture size
};

#endif //TAPS_CAMERANODE_CALIBRATION_H
