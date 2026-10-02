//
// AprilTag detector implementation. See apriltag_detector.h for design notes.
//

#include "apriltag_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include <pthread.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>

#if defined(HAVE_APRILTAG)

// AprilTag 3 headers: include/apriltag/*.h layout (3.3+) with fallback to the
// historic flat layout. Requires the AprilTag 3.4 API (apriltag_detections_t).
#if __has_include(<apriltag/apriltag.h>)

#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>

#elif __has_include(<apriltag.h>)

#include <apriltag.h>
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

#else
#error "HAVE_APRILTAG defined but no AprilTag headers found. Install AprilTag 3.4+ or configure with TAPS_ENABLE_APRILTAG=OFF."
#endif

namespace {
    apriltag_family_t *createFamily(const std::string &name) {
        if (name == "tag36h11") return tag36h11_create();
        if (name == "tag25h9") return tag25h9_create();
        if (name == "tag16h5") return tag16h5_create();
        if (name == "tagStandard41h12") return tagStandard41h12_create();
        if (name == "tagStandard52h13") return tagStandard52h13_create();
        return nullptr;
    }

    void destroyFamily(const std::string &name, apriltag_family_t *fam) {
        if (!fam) return;
        if (name == "tag36h11") tag36h11_destroy(fam);
        else if (name == "tag25h9") tag25h9_destroy(fam);
        else if (name == "tag16h5") tag16h5_destroy(fam);
        else if (name == "tagStandard41h12") tagStandard41h12_destroy(fam);
        else if (name == "tagStandard52h13") tagStandard52h13_destroy(fam);
    }

    // Pose estimation matching the AprilTag/pyapriltags convention: detection
    // corners p[0..3] correspond to the tag model corners at (+-s/2, +-s/2, 0)
    // with model y-down, giving camera frame +x right, +y down, +z forward.
    void estimatePose(const apriltag_detection_t *det, const cv::Matx33d &K,
                      double tagSize, TagDetectionResult &out) {
        const double h = tagSize * 0.5;
        const std::vector<cv::Point3f> obj = {
                {-h, -h, 0.f},
                { h, -h, 0.f},
                { h,  h, 0.f},
                {-h,  h, 0.f}
        };
        std::vector<cv::Point2f> img;
        img.reserve(4);
        for (int i = 0; i < 4; ++i)
            img.emplace_back(det->p[i][0], det->p[i][1]);

        try {
            cv::Mat rvec, tvec;
            const bool ok = cv::solvePnP(obj, img, cv::Mat(K), cv::noArray(),
                                         rvec, tvec, false, cv::SOLVEPNP_IPPE);
            if (!ok || tvec.empty() || !std::isfinite(tvec.at<double>(0)))
                return;

            cv::Mat R;
            cv::Rodrigues(rvec, R);

            for (int i = 0; i < 3; ++i)
                out.translation[i] = tvec.at<double>(i);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    out.rotation[i * 3 + j] = R.at<double>(i, j);

            // RMS reprojection error of the 4 corners: pose-uncertainty proxy.
            std::vector<cv::Point2f> reproj;
            cv::projectPoints(obj, rvec, tvec, cv::Mat(K), cv::noArray(), reproj);
            double sum = 0.0;
            for (int i = 0; i < 4; ++i) {
                const double dx = img[i].x - reproj[i].x;
                const double dy = img[i].y - reproj[i].y;
                sum += dx * dx + dy * dy;
            }
            out.reprojErrorRmsPx = static_cast<float>(std::sqrt(sum / 4.0));

            const double dx = img[0].x - img[2].x;
            const double dy = img[0].y - img[2].y;
            out.tagPxDiag = static_cast<float>(std::sqrt(dx * dx + dy * dy));

            out.poseValid = true;
        } catch (const cv::Exception &e) {
            spdlog::debug("AprilTag solvePnP failed for tag {}: {}", det->id, e.what());
        }
    }
} // namespace

#endif // HAVE_APRILTAG

AprilTagDetector::AprilTagDetector(Config cfg) : m_cfg(std::move(cfg)),
                                                 m_fx(m_cfg.fx), m_fy(m_cfg.fy),
                                                 m_cx(m_cfg.cx), m_cy(m_cfg.cy) {
}

void AprilTagDetector::updateIntrinsics(const double fx, const double fy,
                                        const double cx, const double cy) {
    m_fx.store(fx);
    m_fy.store(fy);
    m_cx.store(cx);
    m_cy.store(cy);
}

void AprilTagDetector::intrinsics(double &fx, double &fy, double &cx, double &cy) const {
    fx = m_fx.load();
    fy = m_fy.load();
    cx = m_cx.load();
    cy = m_cy.load();
}

AprilTagDetector::~AprilTagDetector() {
    stop();
}

bool AprilTagDetector::start() {
    if (!m_cfg.enabled)
        return false;

#if !defined(HAVE_APRILTAG)
    spdlog::error("AprilTag requested (--apriltag) but built WITHOUT AprilTag support. "
                  "Configure with -DTAPS_ENABLE_APRILTAG=ON (default) and rebuild.");
    return false;
#else
    m_det = apriltag_detector_create();
    auto *det = static_cast<apriltag_detector_t *>(m_det);
    auto *fam = createFamily(m_cfg.family);
    if (!fam) {
        spdlog::error("Unknown AprilTag family '{}'. Supported: tag36h11, tag25h9, tag16h5, "
                      "tagStandard41h12, tagStandard52h13", m_cfg.family);
        apriltag_detector_destroy(det);
        m_det = nullptr;
        return false;
    }
    m_family = fam;

    det->nthreads = std::max(1, m_cfg.threads);
    det->quad_decimation = static_cast<unsigned>(std::clamp(m_cfg.quadDecimation, 1.0, 4.0));
    det->blur = m_cfg.blur;

    apriltag_detector_add_family(det, fam);

    if (m_cfg.estimatePose && (m_fx.load() <= 0.0 || m_cx.load() <= 0.0)) {
        spdlog::warn("AprilTag pose enabled without camera intrinsics; using f=max(w,h), c=center "
                     "defaults. Run the web calibration wizard (/calibrate) for accurate fusion.");
    }

    m_stopFlag.store(false);
    m_thread = std::thread([this] {
        pthread_setname_np(pthread_self(), "apriltag");
        processLoop();
    });
    m_running = true;

    spdlog::info("AprilTag detection enabled: family={}, threads={}, quad_decimation={}, pose={}, tag_size={}m",
                 m_cfg.family, det->nthreads, det->quad_decimation, m_cfg.estimatePose, m_cfg.tagSizeMeters);
    return true;
#endif
}

void AprilTagDetector::stop() {
    if (!m_running)
        return;
    m_running = false;

    {
        std::lock_guard<std::mutex> lk(m_frameMutex);
        m_stopFlag.store(true);
        m_pendingFrame.release();
        m_pendingReady = false;
    }
    m_frameCv.notify_all();
    m_resultCv.notify_all();

    if (m_thread.joinable())
        m_thread.join();

#if defined(HAVE_APRILTAG)
    if (m_det) {
        auto *det = static_cast<apriltag_detector_t *>(m_det);
        if (m_family)
            apriltag_detector_remove_family(det, static_cast<apriltag_family_t *>(m_family));
        apriltag_detector_destroy(det);
        m_det = nullptr;
    }
    if (m_family) {
        destroyFamily(m_cfg.family, static_cast<apriltag_family_t *>(m_family));
        m_family = nullptr;
    }
#endif

    spdlog::info("AprilTag detector stopped: {} batches processed, {} tag detections",
                 m_batchesCompleted.load(), m_tagsDetectedTotal.load());
}

void AprilTagDetector::submit(const cv::Mat &frameBgr, uint64_t seq, int64_t ptpNs) {
    if (!m_running || frameBgr.empty())
        return;

    // Latest-frame handoff: replace whatever is queued so detections stay fresh
    // even when detection is slower than capture.
    {
        std::lock_guard<std::mutex> lk(m_frameMutex);
        if (m_stopFlag.load()) return;
        frameBgr.copyTo(m_pendingFrame);
        m_pendingSeq = seq;
        m_pendingPtpNs = ptpNs;
        m_pendingReady = true;
    }
    m_frameCv.notify_one();
}

TagBatch AprilTagDetector::resultsForSeq(uint64_t seq, const int waitMs) {
    std::unique_lock<std::mutex> lk(m_resultMutex);
    if (waitMs > 0) {
        m_resultCv.wait_for(lk, std::chrono::milliseconds(waitMs), [this, seq] {
            return m_stopFlag.load() || m_lastCompletedSeq >= seq;
        });
    }
    if (const auto it = m_results.find(seq); it != m_results.end())
        return it->second;
    return {};
}

TagBatch AprilTagDetector::latestBatch() const {
    std::lock_guard<std::mutex> lk(m_resultMutex);
    return m_latest;
}

void AprilTagDetector::processLoop() {
    for (;;) {
        cv::Mat frame;
        uint64_t seq;
        int64_t ptp;
        {
            std::unique_lock<std::mutex> lk(m_frameMutex);
            m_frameCv.wait(lk, [this] { return m_stopFlag.load() || m_pendingReady; });
            if (m_stopFlag.load())
                break;
            frame = std::move(m_pendingFrame);
            seq = m_pendingSeq;
            ptp = m_pendingPtpNs;
            m_pendingReady = false;
        }

        TagBatch batch = detectFrame(frame, seq, ptp);
        frame.release();

        {
            std::lock_guard<std::mutex> lk(m_resultMutex);
            m_results[seq] = batch;
            m_resultOrder.push_back(seq);
            while (m_results.size() > kResultHistory) {
                m_results.erase(m_resultOrder.front());
                m_resultOrder.pop_front();
            }
            m_lastCompletedSeq = seq; // capture seqs are monotonic; loop is FIFO
            m_latest = batch;
            m_batchesCompleted.fetch_add(1);
        }
        m_resultCv.notify_all();
    }
}

TagBatch AprilTagDetector::detectFrame(const cv::Mat &frameBgr, uint64_t seq, int64_t ptpNs) {
    TagBatch batch;
    batch.seq = seq;
    batch.ptpNs = ptpNs;

#if defined(HAVE_APRILTAG)
    if (!m_det || frameBgr.empty())
        return batch;

    cv::Mat gray;
    if (frameBgr.channels() == 3)
        cv::cvtColor(frameBgr, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frameBgr;
    if (!gray.isContinuous())
        gray = gray.clone();

    // Camera intrinsics: configured values or documented defaults.
    const double fxL = m_fx.load(), fyL = m_fy.load(), cxL = m_cx.load(), cyL = m_cy.load();
    const double fx = fxL > 0.0 ? fxL : static_cast<double>(gray.cols);
    const double fy = fyL > 0.0 ? fyL : static_cast<double>(gray.rows);
    const double cx = cxL > 0.0 ? cxL : gray.cols * 0.5;
    const double cy = cyL > 0.0 ? cyL : gray.rows * 0.5;
    const cv::Matx33d K(fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0);

    image_u8_t img{};
    img.width = gray.cols;
    img.height = gray.rows;
    img.stride = static_cast<int>(gray.step);
    img.buf = gray.data;

    auto *det = static_cast<apriltag_detector_t *>(m_det);
    apriltag_detections_t dets{};
    apriltag_detector_detect(det, &img, &dets);

    const size_t count = apriltag_detections_count(&dets);
    batch.tags.reserve(std::min(count, kMaxTagsPerFrame));
    for (size_t i = 0; i < count && batch.tags.size() < kMaxTagsPerFrame; ++i) {
        const apriltag_detection_t *d = apriltag_detections_deref(&dets, i);
        if (!d) continue;

        TagDetectionResult r;
        r.tagId = static_cast<uint16_t>(d->id);
        r.decisionMargin = static_cast<float>(d->decision_margin);
        r.hamming = static_cast<uint8_t>(std::min(255, static_cast<int>(std::fabs(d->hamming))));

        if (m_cfg.estimatePose && m_cfg.tagSizeMeters > 0.0)
            estimatePose(d, K, m_cfg.tagSizeMeters, r);

        batch.tags.push_back(r);
    }
    apriltag_detections_destroy(&dets);

    m_tagsDetectedTotal.fetch_add(batch.tags.size());
#else
    (void) frameBgr;
#endif

    return batch;
}
