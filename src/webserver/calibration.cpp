//
// CalibrationSession implementation — see calibration.h for the flow.
//

#include "calibration.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>

#include <pthread.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

// MJPEG producer buffer (shared with HttpServer's calib stream resource)
std::mutex CalibrationSession::s_jpegMutex;
std::vector<uint8_t> CalibrationSession::s_latestJpeg;
std::atomic<uint64_t> CalibrationSession::s_jpegSequence{0};

// Extra session-local state (kept here to keep the header lean)
namespace {
    std::mutex s_msgMutex;
    std::string s_lastMsg;
    std::chrono::steady_clock::time_point s_lastMsgAt{};

    void setMsg(const std::string &m) {
        std::lock_guard lock(s_msgMutex);
        s_lastMsg = m;
        s_lastMsgAt = std::chrono::steady_clock::now();
    }

    // Message + whether it was set within the last `ttl` (for overlay display).
    std::pair<std::string, bool> lastMsgStatus(const std::chrono::seconds &ttl) {
        std::lock_guard lock(s_msgMutex);
        return {s_lastMsg, std::chrono::steady_clock::now() - s_lastMsgAt < ttl};
    }

    std::string lastMsgCopy() {
        std::lock_guard lock(s_msgMutex);
        return s_lastMsg;
    }

    std::string isoNowUtc() {
        const std::time_t t = std::time(nullptr);
        std::tm tmv{};
        gmtime_r(&t, &tmv);
        char buf[32];
        std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tmv);
        return buf;
    }

    double meanCornerDisp(const std::vector<cv::Point2f> &a, const std::vector<cv::Point2f> &b) {
        if (a.size() != b.size() || a.empty()) return 1e9;
        double sum = 0;
        for (size_t i = 0; i < a.size(); ++i)
            sum += std::hypot(a[i].x - b[i].x, a[i].y - b[i].y);
        return sum / static_cast<double>(a.size());
    }

    double lapVar(const cv::Mat &grayRoi) {
        cv::Mat lap;
        cv::Laplacian(grayRoi, lap, CV_64F);
        cv::Scalar m, sd;
        cv::meanStdDev(lap, m, sd);
        return sd[0] * sd[0];
    }

    // Orientation categories for coverage tracking.
    enum TiltCat { TILT_FRONT = 0, TILT_LEFT, TILT_RIGHT, TILT_UP, TILT_DOWN, TILT_COUNT };

    int tiltCategory(const std::vector<cv::Point2f> &c, int patternW, int patternH) {
        const auto dist = [](const cv::Point2f &a, const cv::Point2f &b) {
            return std::hypot(a.x - b.x, a.y - b.y);
        };
        const double top = dist(c[0], c[1]);
        const double bottom = dist(c[3], c[2]);
        const double left = dist(c[0], c[3]);
        const double right = dist(c[1], c[2]);
        const double perCol = (top + bottom) * 0.5 / std::max(1, patternW - 1);
        const double perRow = (left + right) * 0.5 / std::max(1, patternH - 1);
        const double hSkew = std::fabs(top - bottom) / std::max({top, bottom, 1e-3});
        const double vSkew = std::fabs(left - right) / std::max({left, right, 1e-3});
        if (hSkew < 0.12 && vSkew < 0.12 && std::fabs(std::log(perCol / std::max(perRow, 1e-3))) < 0.35)
            return TILT_FRONT;
        if (hSkew >= vSkew)
            return top < bottom ? TILT_UP : TILT_DOWN;
        return left < right ? TILT_LEFT : TILT_RIGHT;
    }

    int regionCell(const std::vector<cv::Point2f> &c, int w, int h) {
        double sx = 0, sy = 0;
        for (const auto &p: c) { sx += p.x; sy += p.y; }
        const double cx = sx / c.size(), cy = sy / c.size();
        const int col = std::clamp(static_cast<int>(cx * 3.0 / w), 0, 2);
        const int row = std::clamp(static_cast<int>(cy * 3.0 / h), 0, 2);
        return row * 3 + col;
    }

    bool focusPassThreshold(const double current, const double best, const bool board) {
        return board && best > 1.0 && current >= 0.6 * best;
    }
} // namespace

json SavedCalibration::toJson() const {
    if (!valid) return nullptr;
    json j;
    j["calibrated_at"] = calibratedAt;
    j["image_width"] = imageWidth;
    j["image_height"] = imageHeight;
    j["pattern_width"] = patternW;
    j["pattern_height"] = patternH;
    j["square_mm"] = squareMm;
    j["frames_used"] = framesUsed;
    j["reprojection_error_px"] = reprojErrorPx;
    j["fx"] = fx;
    j["fy"] = fy;
    j["cx"] = cx;
    j["cy"] = cy;
    j["distortion"] = distortion;
    return j;
}

// ----------------------------------------------------------------------
//  Lifecycle
// ----------------------------------------------------------------------

void CalibrationSession::init(const fs::path &calibrationFilePath, const std::string &cameraAlias) {
    s_calibPath = calibrationFilePath;
    s_alias = cameraAlias;

    std::lock_guard lock(s_savedMutex);
    s_saved = SavedCalibration{};
    std::ifstream in(s_calibPath, std::ios::binary);
    if (!in.is_open()) {
        spdlog::info("No persisted camera calibration at {}", s_calibPath.string());
        return;
    }
    try {
        json j;
        in >> j;
        s_saved.valid = true;
        s_saved.calibratedAt = j.value("calibrated_at", "");
        s_saved.imageWidth = j.value("image_width", 0);
        s_saved.imageHeight = j.value("image_height", 0);
        s_saved.patternW = j.value("pattern_width", 0);
        s_saved.patternH = j.value("pattern_height", 0);
        s_saved.squareMm = j.value("square_mm", 0.0);
        s_saved.framesUsed = j.value("frames_used", 0);
        s_saved.reprojErrorPx = j.value("reprojection_error_px", 0.0);
        s_saved.fx = j.value("fx", 0.0);
        s_saved.fy = j.value("fy", 0.0);
        s_saved.cx = j.value("cx", 0.0);
        s_saved.cy = j.value("cy", 0.0);
        s_saved.distortion = j.value("distortion", std::vector<double>{});
        spdlog::info("Loaded camera calibration from {} (captured {}, reproj err {:.3f} px)",
                     s_calibPath.string(), s_saved.calibratedAt, s_saved.reprojErrorPx);
    } catch (const std::exception &e) {
        spdlog::warn("Failed to parse {}: {}", s_calibPath.string(), e.what());
        s_saved = SavedCalibration{};
    }
}

void CalibrationSession::shutdown() {
    s_stop.store(true);
    s_frameCv.notify_all();
    if (s_worker.joinable())
        s_worker.join();
    setStage(Stage::Idle);
}

SavedCalibration CalibrationSession::saved() {
    std::lock_guard lock(s_savedMutex);
    return s_saved;
}

bool CalibrationSession::active() {
    const Stage s = stage();
    return s == Stage::Focus || s == Stage::Collect || s == Stage::Processing;
}

void CalibrationSession::submitFrame(const cv::Mat &frameBgr) {
    if (!active() || frameBgr.empty())
        return;
    std::lock_guard lock(s_frameMutex);
    if (s_stop.load()) return;
    frameBgr.copyTo(s_pendingFrame); // latest-frame handoff
    s_frameReady = true;
    s_frameCv.notify_one();
}

void CalibrationSession::setOnSaved(std::function<void(double, double, double, double)> cb) {
    std::lock_guard lock(s_cbMutex);
    s_onSaved = std::move(cb);
}

const char *CalibrationSession::stageName(const Stage s) {
    switch (s) {
        case Stage::Focus: return "focus";
        case Stage::Collect: return "collect";
        case Stage::Processing: return "processing";
        case Stage::Done: return "done";
        case Stage::Failed: return "failed";
        default: return "idle";
    }
}

json CalibrationSession::configJsonLocked() {
    json cfg;
    cfg["pattern_w"] = s_patternW;
    cfg["pattern_h"] = s_patternH;
    cfg["square_mm"] = s_squareMm;
    return cfg;
}

void CalibrationSession::clearCollectedLocked() {
    s_objPoints.clear();
    s_imgPoints.clear();
    s_acceptedCorners.clear();
    for (int &c: s_cells) c = 0;
    for (int &t: s_tilts) t = 0;
    s_hadPrev = false;
    s_prevCorners.clear();
    s_lastRegion = s_lastTilt = -1;
    s_lastTotal = 0;
    if (!s_tmpDir.empty()) {
        std::error_code ec;
        fs::remove_all(s_tmpDir, ec);
    }
}

// ----------------------------------------------------------------------
//  Detection / focus
// ----------------------------------------------------------------------

bool CalibrationSession::detectBoard(const cv::Mat &gray, const cv::Size &pattern,
                                     std::vector<cv::Point2f> &corners) {
    if (pattern.width < 4 || pattern.height < 4) return false;

    cv::Mat small;
    cv::resize(gray, small, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    const int base = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    bool found = cv::findChessboardCorners(small, pattern, corners, base | cv::CALIB_CB_FAST_CHECK);
    if (!found) {
        cv::Mat eq;
        cv::equalizeHist(small, eq);
        found = cv::findChessboardCorners(eq, pattern, corners, base);
        if (!found) {
#if CV_VERSION_MAJOR >= 4
            // 3-arg call: the optional flags parameter only exists on OpenCV
            // >= 4.8; the built-in default (CALIB_CB_NORMALIZE_IMAGE) matches.
            found = cv::findChessboardCornersSB(small, pattern, corners);
            if (!found)
                found = cv::findChessboardCornersSB(eq, pattern, corners);
#endif
        }
    }
    if (!found) return false;

    for (auto &p: corners) p *= 2.0f; // undo 0.5 detection scale
    return true;
}

double CalibrationSession::boardFocusScore(const cv::Mat &gray,
                                           const std::vector<cv::Point2f> &corners) {
    // Rectify the inner grid to a fixed 24 px/square image so the score
    // reflects optical sharpness, not distance or position.
    const float s = 24.0f;
    const int W = s_patternW > 1 ? s_patternW : 9;
    const int H = s_patternH > 1 ? s_patternH : 6;
    const std::vector<cv::Point2f> dst = {
            {0.0f, 0.0f},
            {(W - 1) * s, 0.0f},
            {(W - 1) * s, (H - 1) * s},
            {0.0f, (H - 1) * s}
    };
    try {
        const cv::Mat M = cv::getPerspectiveTransform(corners, dst);
        cv::Mat warp;
        cv::warpPerspective(gray, warp, M, cv::Size(static_cast<int>((W - 1) * s) + 1,
                                                    static_cast<int>((H - 1) * s) + 1));
        const int margin = static_cast<int>(s * 0.5f);
        if (warp.cols <= 2 * margin || warp.rows <= 2 * margin) return lapVar(gray);
        const cv::Mat roi = warp(cv::Rect(margin, margin, warp.cols - 2 * margin, warp.rows - 2 * margin));
        return lapVar(roi);
    } catch (const cv::Exception &) {
        return lapVar(gray);
    }
}

void CalibrationSession::focusPass(const cv::Mat &gray, cv::Mat &preview) {
    std::vector<cv::Point2f> corners;
    const bool found = detectBoard(gray, cv::Size(s_patternW, s_patternH), corners);

    double raw;
    if (found) {
        raw = boardFocusScore(gray, corners);
    } else {
        const cv::Rect r(gray.cols / 6, gray.rows / 6, (gray.cols * 2) / 3, (gray.rows * 2) / 3);
        raw = lapVar(gray(r));
    }

    {
        std::lock_guard lock(s_stateMutex);
        s_focusCurrent = s_focusCurrent <= 0.0 ? raw : 0.8 * s_focusCurrent + 0.2 * raw;
        if (found) {
            s_focusBoardSeen = true;
            s_focusBest = std::max(s_focusBest, s_focusCurrent);
        }
    }

    // Overlay: board outline + score bar
    if (found) {
        const double sf = static_cast<double>(preview.cols) / gray.cols;
        std::vector<cv::Point2f> pc(corners.size());
        for (size_t i = 0; i < corners.size(); ++i) pc[i] = corners[i] * static_cast<float>(sf);
        cv::drawChessboardCorners(preview, cv::Size(s_patternW, s_patternH), pc, found);
    }

    double cur, best;
    bool boardSeen;
    {
        std::lock_guard lock(s_stateMutex);
        cur = s_focusCurrent;
        best = s_focusBest;
        boardSeen = s_focusBoardSeen;
    }
    const bool pass = focusPassThreshold(cur, best, boardSeen);

    // Score bar (bottom)
    const int barH = 34;
    cv::rectangle(preview, cv::Point(0, preview.rows - barH), cv::Point(preview.cols, preview.rows),
                  cv::Scalar(15, 15, 15), cv::FILLED);
    if (best > 0) {
        const int w = static_cast<int>(std::clamp(cur / best, 0.0, 1.0) * (preview.cols - 20));
        cv::rectangle(preview, cv::Point(10, preview.rows - barH + 8),
                      cv::Point(10 + w, preview.rows - 8),
                      pass ? cv::Scalar(60, 200, 60) : cv::Scalar(60, 160, 250), cv::FILLED);
    }
    cv::putText(preview, cv::format("Sharpness %.0f   Session best %.0f", cur, best),
                cv::Point(12, preview.rows - barH - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 2);
    cv::putText(preview, pass ? "PASS - lock focus ring" : "ADJUST FOCUS to maximize",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                pass ? cv::Scalar(60, 220, 60) : cv::Scalar(60, 160, 250), 2);
    if (!boardSeen)
        cv::putText(preview, "Place the checkerboard in view (>= 30 cm / 1 ft away)",
                    cv::Point(12, 56), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(200, 200, 200), 1);
}

// ----------------------------------------------------------------------
//  Collection
// ----------------------------------------------------------------------

void CalibrationSession::collectPass(const cv::Mat &gray, const cv::Mat &frameBgr, cv::Mat &preview) {
    std::lock_guard lock(s_stateMutex);
    std::vector<cv::Point2f> corners;
    bool found = detectBoard(gray, cv::Size(s_patternW, s_patternH), corners);
    if (found) {
        const cv::TermCriteria crit(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01);
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), crit);
    }

    const auto now = std::chrono::steady_clock::now();
    int region = -1, tilt = -1;
    bool accepted = false;
    std::string reject;

    if (found) {
        region = regionCell(corners, gray.cols, gray.rows);
        tilt = tiltCategory(corners, s_patternW, s_patternH);

        const double move = s_hadPrev ? meanCornerDisp(corners, s_prevCorners) : 1e9;
        const bool still = !s_hadPrev || move < 2.5;
        // Accept even in a full region when this frame still adds a missing
        // orientation category (otherwise a full FOV with a missing tilt is a
        // dead end).
        const bool tiltNeeded = s_tilts[tilt] < kTiltGoal;

        if (!still) {
            reject = "hold the board steady";
            s_hadPrev = false;
        } else if (s_cells[region] >= kCellGoal && !tiltNeeded) {
            reject = "region full";
        } else if (now - s_cellCooldown[region] < std::chrono::milliseconds(1000)) {
            reject = "too soon after last capture here - try a new spot";
        } else if ([&] {
                       for (const auto &acc: s_acceptedCorners)
                           if (meanCornerDisp(corners, acc) < 40.0) return true;
                       return false;
                   }()) {
            reject = "already sampled here - move to a new spot";
        } else if (static_cast<int>(s_objPoints.size()) >= 120) {
            reject = "sample cap reached";
        } else {
            // Accept: build object points for this pattern (mm, z=0).
            std::vector<cv::Point3f> obj;
            obj.reserve(s_patternW * s_patternH);
            for (int r = 0; r < s_patternH; ++r)
                for (int c = 0; c < s_patternW; ++c)
                    obj.emplace_back(c * s_squareMm, r * s_squareMm, 0.0f);
            s_objPoints.push_back(std::move(obj));
            s_imgPoints.push_back(corners);
            s_acceptedCorners.push_back(corners);
            s_cells[region]++;
            s_tilts[tilt]++;
            s_cellCooldown[region] = now;
            accepted = true;

            // Persist the raw image for audit (deleted after processing).
            std::error_code ec;
            fs::create_directories(s_tmpDir, ec);
            cv::imwrite((s_tmpDir /
                         cv::format("frame_%03zu.jpg", s_objPoints.size()))
                                .string(), frameBgr,
                        {cv::IMWRITE_JPEG_QUALITY, 92});
            setMsg("CAPTURED frame " + std::to_string(s_objPoints.size()));
        }
        if (!accepted && !reject.empty()) setMsg(reject);
        s_prevCorners = corners;
        s_hadPrev = found;
    } else {
        s_hadPrev = false;
    }

    s_lastTotal = static_cast<int>(s_objPoints.size());

    // Goals met? Auto-advance.
    bool goalsMet = true;
    for (const int c: s_cells) if (c < kCellGoal) goalsMet = false;
    for (const int t: s_tilts) if (t < kTiltGoal) goalsMet = false;
    if (goalsMet && !s_processRequested.load()) {
        setMsg("All coverage goals met - computing calibration...");
        setStage(Stage::Processing);
        s_processRequested.store(true);
        return;
    }

    // --- Overlay ---
    const double sf = static_cast<double>(preview.cols) / gray.cols;
    if (found) {
        std::vector<cv::Point2f> pc(corners.size());
        for (size_t i = 0; i < corners.size(); ++i) pc[i] = corners[i] * static_cast<float>(sf);
        cv::drawChessboardCorners(preview, cv::Size(s_patternW, s_patternH), pc, found);
    }

    // FOV region grid with per-cell fill
    const int pw = preview.cols, ph = preview.rows;
    for (int r = 0; r <= 3; ++r) {
        const int y = r * ph / 3;
        cv::line(preview, cv::Point(0, y), cv::Point(pw, y), cv::Scalar(255, 255, 255), 1);
    }
    for (int c = 0; c <= 3; ++c) {
        const int x = c * pw / 3;
        cv::line(preview, cv::Point(x, 0), cv::Point(x, ph), cv::Scalar(255, 255, 255), 1);
    }
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const int idx = r * 3 + c;
            const int n = s_cells[idx];
            const cv::Scalar color = n >= kCellGoal
                                         ? cv::Scalar(80, 220, 80)
                                         : (n > 0 ? cv::Scalar(80, 180, 250) : cv::Scalar(120, 120, 120));
            cv::putText(preview, cv::format("%d/%d", n, kCellGoal),
                        cv::Point(c * pw / 3 + 8, r * ph / 3 + 22),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2);
        }
    }

    // Bottom bar: totals + tilt coverage + message
    const int barH = 34;
    cv::rectangle(preview, cv::Point(0, ph - barH), cv::Point(pw, ph), cv::Scalar(15, 15, 15), cv::FILLED);
    cv::putText(preview, cv::format("Collected %d (need %d) | front %d L %d R %d U %d D %d (need %d each)",
                                    s_lastTotal, kMinTotal,
                                    s_tilts[TILT_FRONT], s_tilts[TILT_LEFT], s_tilts[TILT_RIGHT],
                                    s_tilts[TILT_UP], s_tilts[TILT_DOWN], kTiltGoal),
                cv::Point(10, ph - 12), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    const auto [msg, msgFresh] = lastMsgStatus(std::chrono::seconds(2));
    if (msgFresh && !msg.empty()) {
        const cv::Scalar color = accepted ? cv::Scalar(80, 220, 80) : cv::Scalar(80, 180, 250);
        cv::putText(preview, msg, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
    }
}

// ----------------------------------------------------------------------
//  Processing / persistence
// ----------------------------------------------------------------------

void CalibrationSession::processCollected() {
    s_processRequested.store(false);
    setStage(Stage::Processing);

    if (s_objPoints.size() < 12) {
        std::lock_guard lock(s_stateMutex);
        s_error = "Not enough frames collected";
        setStage(Stage::Failed);
        spdlog::warn("Calibration failed: {}", s_error);
        return;
    }

    cv::Mat cameraMatrix, dist;
    std::vector<cv::Mat> rvecs, tvecs;
    double err = -1;
    try {
        // Defaults (flags=0, standard TermCriteria) — identical semantics on
        // OpenCV 4.x and 5.x, avoids signature-order differences there.
        err = cv::calibrateCamera(s_objPoints, s_imgPoints, s_frameSize,
                                  cameraMatrix, dist, rvecs, tvecs);
    } catch (const cv::Exception &e) {
        std::lock_guard lock(s_stateMutex);
        s_error = std::string("calibrateCamera threw: ") + e.what();
        setStage(Stage::Failed);
        return;
    }

    if (!std::isfinite(err) || err <= 0.0 || cameraMatrix.empty()) {
        std::lock_guard lock(s_stateMutex);
        s_error = "Calibration produced invalid results";
        setStage(Stage::Failed);
        return;
    }

    const double fx = cameraMatrix.at<double>(0, 0);
    const double fy = cameraMatrix.at<double>(1, 1);
    const double cx = cameraMatrix.at<double>(0, 2);
    const double cy = cameraMatrix.at<double>(1, 2);
    std::vector<double> distV;
    distV.reserve(dist.total());
    for (size_t i = 0; i < dist.total(); ++i) distV.push_back(dist.at<double>(static_cast<int>(i)));

    SavedCalibration calib;
    calib.valid = true;
    calib.calibratedAt = isoNowUtc();
    calib.imageWidth = s_frameSize.width;
    calib.imageHeight = s_frameSize.height;
    {
        std::lock_guard lock(s_stateMutex);
        calib.patternW = s_patternW;
        calib.patternH = s_patternH;
        calib.squareMm = s_squareMm;
    }
    calib.framesUsed = static_cast<int>(s_objPoints.size());
    calib.reprojErrorPx = err;
    calib.fx = fx;
    calib.fy = fy;
    calib.cx = cx;
    calib.cy = cy;
    calib.distortion = distV;

    // Non-volatile write: tmp + atomic rename.
    json j;
    j["version"] = 1;
    j["camera_alias"] = s_alias;
    j["calibrated_at"] = calib.calibratedAt;
    j["image_width"] = calib.imageWidth;
    j["image_height"] = calib.imageHeight;
    j["pattern_width"] = calib.patternW;
    j["pattern_height"] = calib.patternH;
    j["square_mm"] = calib.squareMm;
    j["frames_used"] = calib.framesUsed;
    j["reprojection_error_px"] = err;
    j["fx"] = fx;
    j["fy"] = fy;
    j["cx"] = cx;
    j["cy"] = cy;
    j["camera_matrix"] = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    j["distortion"] = distV;

    const fs::path tmpPath = s_calibPath.string() + ".tmp";
    std::error_code ec;
    fs::create_directories(s_calibPath.parent_path(), ec);
    {
        std::ofstream out(tmpPath, std::ios::binary | std::ios::trunc);
        out << j.dump(2);
    }
    fs::rename(tmpPath, s_calibPath, ec);
    if (ec) {
        std::lock_guard lock(s_stateMutex);
        s_error = "Failed to write " + s_calibPath.string() + ": " + ec.message();
        setStage(Stage::Failed);
        return;
    }

    {
        std::lock_guard lock(s_savedMutex);
        s_saved = calib;
    }
    {
        std::lock_guard lock(s_resultMutex);
        s_resultJson = calib.toJson();
        s_resultJson["success"] = true;
        if (err > 1.0)
            s_resultJson["warning"] = "Reprojection error is above 1 px - consider repeating (check board flatness/focus)";
    }

    {
        std::lock_guard lock(s_stateMutex);
        clearCollectedLocked();
    }
    setStage(Stage::Done);
    spdlog::info("Camera calibration saved to {} (fx {:.2f} fy {:.2f} cx {:.2f} cy {:.2f}, err {:.3f} px)",
                 s_calibPath.string(), fx, fy, cx, cy, err);

    std::function<void(double, double, double, double)> cb;
    { std::lock_guard lock(s_cbMutex); cb = s_onSaved; }
    if (cb) cb(fx, fy, cx, cy);
}

// ----------------------------------------------------------------------
//  Worker loop
// ----------------------------------------------------------------------

void CalibrationSession::workerLoop() {
    pthread_setname_np(pthread_self(), "calibrate");
    spdlog::info("Calibration worker started");

    while (!s_stop.load()) {
        cv::Mat frame;
        {
            std::unique_lock lock(s_frameMutex);
            s_frameCv.wait_for(lock, std::chrono::milliseconds(60),
                               [] { return s_stop.load() || s_frameReady; });
            if (s_stop.load()) break;
            if (s_frameReady) {
                frame = std::move(s_pendingFrame);
                s_frameReady = false;
            }
        }

        const Stage st = stage();
        // Processing is checked even when no frame arrives (wait_for timeout),
        // so a stalled camera cannot delay calibration compute.
        if (st == Stage::Processing) {
            processCollected();
            continue;
        }
        if (frame.empty() || (st != Stage::Focus && st != Stage::Collect))
            continue;

        s_frameSize = frame.size();

        cv::Mat gray;
        if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        else gray = frame;

        // Preview for overlay + streaming
        cv::Mat preview;
        const double pw = std::min(960.0, static_cast<double>(frame.cols));
        const double psf = pw / frame.cols;
        cv::resize(frame, preview, cv::Size(), psf, psf, cv::INTER_AREA);

        if (st == Stage::Focus) focusPass(gray, preview);
        else collectPass(gray, frame, preview);

        std::vector<uint8_t> buf;
        if (cv::imencode(".jpg", preview, buf, {cv::IMWRITE_JPEG_QUALITY, 80})) {
            std::lock_guard lock(s_jpegMutex);
            s_latestJpeg = std::move(buf);
            s_jpegSequence.fetch_add(1);
        }
    }
    spdlog::info("Calibration worker stopped");
}

// ----------------------------------------------------------------------
//  Status / control
// ----------------------------------------------------------------------

json CalibrationSession::statusJson() {
    json j;
    {
        std::lock_guard lock(s_stateMutex);
        j["stage"] = stageName(stage());
        j["error"] = s_error;
        j["config"] = configJsonLocked();
        j["focus"] = {
                {"current", s_focusCurrent},
                {"best", s_focusBest},
                {"board", s_focusBoardSeen},
                {"pass", focusPassThreshold(s_focusCurrent, s_focusBest, s_focusBoardSeen)}
        };
        json cells = json::array();
        for (const int c: s_cells) cells.push_back(c);
        json tilts = json::array();
        for (const int t: s_tilts) tilts.push_back(t);
        int total = 0;
        bool goalsMet = true, earlyOk = s_lastTotal >= kMinTotal;
        int nonFrontCats = 0;
        for (int i = 0; i < 9; ++i) {
            total += s_cells[i];
            if (s_cells[i] < kCellGoal) goalsMet = false;
            if (s_cells[i] < 1) earlyOk = false;
        }
        for (int i = 0; i < 5; ++i) {
            if (s_tilts[i] < kTiltGoal) goalsMet = false;
            if (i != TILT_FRONT && s_tilts[i] > 0) nonFrontCats++;
        }
        earlyOk = earlyOk && nonFrontCats >= 2;
        j["collect"] = {
                {"cells", cells},
                {"cell_goal", kCellGoal},
                {"tilts", tilts},
                {"tilt_goal", kTiltGoal},
                {"total", s_lastTotal},
                {"min_total", kMinTotal},
                {"goals_met", goalsMet},
                {"early_ok", earlyOk}
        };
        j["last_message"] = lastMsgCopy();
    }
    {
        std::lock_guard lock(s_resultMutex);
        j["result"] = s_resultJson.is_null() ? json(nullptr) : s_resultJson;
    }
    j["calibration"] = saved().toJson();
    j["calibration_file"] = s_calibPath.string();
    j["active"] = active();
    return j;
}

json CalibrationSession::control(const json &body) {
    const std::string action = body.value("action", "");

    if (action == "start") {
        if (s_worker.joinable()) {
            s_stop.store(true);
            s_frameCv.notify_all();
            s_worker.join();
        }
        s_stop.store(false);
        {
            std::lock_guard jl(s_jpegMutex);
            std::vector<uint8_t>().swap(s_latestJpeg);
        }

        std::lock_guard lock(s_stateMutex);
        s_error.clear();
        s_patternW = std::clamp(body.value("pattern_w", 9), 4, 20);
        s_patternH = std::clamp(body.value("pattern_h", 6), 4, 20);
        s_squareMm = std::clamp(body.value("square_mm", 25.0), 3.0, 250.0);
        s_focusCurrent = s_focusBest = 0.0;
        s_focusBoardSeen = false;
        clearCollectedLocked();
        s_tmpDir = s_calibPath.parent_path() / "calib_tmp";
        { std::lock_guard rl(s_resultMutex); s_resultJson = nullptr; }
        spdlog::info("Calibration started: pattern {}x{} inner corners, square {} mm",
                     s_patternW, s_patternH, s_squareMm);
        setStage(Stage::Focus);
        s_worker = std::thread(workerLoop);
        return {{"ok", true}};
    }

    if (action == "next") {
        if (stage() != Stage::Focus)
            return {{"ok", false}, {"error", "not in focus stage"}};
        setStage(Stage::Collect);
        return {{"ok", true}};
    }

    if (action == "finish") {
        if (stage() != Stage::Collect)
            return {{"ok", false}, {"error", "not in collect stage"}};
        bool ok;
        {
            std::lock_guard lock(s_stateMutex);
            int total = 0;
            int nonFrontCats = 0;
            for (int i = 0; i < 9; ++i) total += s_cells[i];
            for (int i = 1; i < 5; ++i) if (s_tilts[i] > 0) nonFrontCats++;
            ok = total >= kMinTotal && nonFrontCats >= 2;
            for (int i = 0; i < 9 && ok; ++i) if (s_cells[i] < 1) ok = false;
        }
        if (!ok)
            return {{"ok", false}, {"error", "too few frames / insufficient coverage for early finish"}};
        setStage(Stage::Processing);
        s_processRequested.store(true);
        return {{"ok", true}};
    }

    if (action == "abort") {
        s_stop.store(true);
        s_frameCv.notify_all();
        if (s_worker.joinable()) s_worker.join();
        s_stop.store(false);
        {
            std::lock_guard lock(s_stateMutex);
            clearCollectedLocked();
            s_error.clear();
        }
        setStage(Stage::Idle);
        return {{"ok", true}};
    }

    if (action == "reset") { // Done/Failed → Idle
        if (stage() == Stage::Focus || stage() == Stage::Collect)
            return {{"ok", false}, {"error", "session running - abort first"}};
        setStage(Stage::Idle);
        return {{"ok", true}};
    }

    return {{"ok", false}, {"error", "unknown action"}};
}
