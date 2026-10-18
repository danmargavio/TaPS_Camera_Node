//
// Synthetic V4L2 camera feeder — TEST HARNESS ONLY.
//
// Emits animated RGB (BGR-ordered) frames on stdout, paced to the target
// frame rate. Pipe into a v4l2loopback device via GStreamer:
//
//   sudo modprobe v4l2loopback devices=1 video_nr=0 card_label="TaPS Virtual"
//   ./TaPS_TestFeed 1280 720 30 [tag.png] | \
//     gst-launch-1.0 fdsrc \
//       ! "video/x-raw,format=BGR,width=1280,height=720,framerate=30/1" \
//       ! videoconvert ! "video/x-raw,format=YUY2" ! v4l2sink device=/dev/video0
//
// An optional AprilTag PNG is composited into the scene so the AprilTag
// detection path can be exercised end to end.
//
// Build:  g++ -O2 -std=c++23 TaPS_TestFeed.cpp $(pkg-config --cflags --libs opencv4) -o TaPS_TestFeed
//

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char **argv) {
    const int width = argc > 1 ? std::atoi(argv[1]) : 1280;
    const int height = argc > 2 ? std::atoi(argv[2]) : 720;
    const int fps = argc > 3 ? std::atoi(argv[3]) : 30;

    // Optional tag overlay (e.g. /tmp/tag41.png rendered via
    // apriltag_to_image) pasted right-of-center.
    cv::Mat tag;
    if (argc > 4) {
        tag = cv::imread(argv[4], cv::IMREAD_COLOR);
        if (tag.empty())
            fprintf(stderr, "warning: could not load tag image %s\n", argv[4]);
        else
            cv::resize(tag, tag, {height / 3, height / 3}, 0, 0, cv::INTER_NEAREST);
    }

    cv::Mat frame(height, width, CV_8UC3);
    const auto t0 = std::chrono::steady_clock::now();
    uint64_t n = 0;

    for (;; ++n) {
        const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

        // Background gradient + border so motion and framing are obvious.
        for (int y = 0; y < height; ++y) {
            auto *row = frame.ptr<cv::Vec3b>(y);
            const uint8_t g = static_cast<uint8_t>(y * 255 / height);
            for (int x = 0; x < width; ++x)
                row[x] = {static_cast<uint8_t>(40 + g / 4), g, static_cast<uint8_t>(160 - g / 2)};
        }
        cv::rectangle(frame, {2, 2}, {width - 3, height - 3}, {255, 255, 255}, 2);

        // AprilTag pasted right-of-center (static, like a tag on a wall).
        if (!tag.empty()) {
            const cv::Rect roi(width - tag.cols - 80, (height - tag.rows) / 2,
                               tag.cols, tag.rows);
            tag.copyTo(frame(roi));
        }

        // Crosshair at the optical center (useful for calibration sanity).
        cv::line(frame, {width / 2 - 40, height / 2}, {width / 2 + 40, height / 2}, {0, 0, 0}, 1);
        cv::line(frame, {width / 2, height / 2 - 40}, {width / 2, height / 2 + 40}, {0, 0, 0}, 1);

        // Two orbiting balls (motion for fps/stream checks).
        const double r = height / 5.0;
        cv::circle(frame,
                   {static_cast<int>(width / 2.0 + r * std::cos(t * 1.3)),
                    static_cast<int>(height / 2.0 + r * std::sin(t * 1.3))},
                   30, {0, 255, 255}, -1);
        cv::circle(frame,
                   {static_cast<int>(width / 2.0 + r * std::cos(t * 1.3 + CV_PI)),
                    static_cast<int>(height / 2.0 + r * std::sin(t * 1.3 + CV_PI))},
                   30, {255, 80, 0}, -1);

        // Timestamps burned in: frame count + monotonic + wall clock seconds.
        const auto wall = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        char buf[160];
        std::snprintf(buf, sizeof(buf), "frame %llu  t=%.3fs  wall=%lld",
                      static_cast<unsigned long long>(n), t, static_cast<long long>(wall));
        cv::putText(frame, buf, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {255, 255, 255}, 2);
        std::snprintf(buf, sizeof(buf), "%dx%d @ %d fps (synthetic)", width, height, fps);
        cv::putText(frame, buf, {20, height - 25}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {255, 255, 255}, 2);

        std::fwrite(frame.data, 1, static_cast<size_t>(width) * height * 3, stdout);
        std::fflush(stdout);

        // Pace to the target frame rate (backpressure from v4l2-ctl will
        // naturally slow us down if the device is full).
        const auto target = t0 + std::chrono::microseconds((n + 1) * 1000000 / fps);
        std::this_thread::sleep_until(target);
    }
}
