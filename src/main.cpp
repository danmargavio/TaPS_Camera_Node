#include <chrono>
#include <csignal>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

#include <args.hxx>
#include <deque>
#include <fstream>
#include <mutex>
#include <vector>
#include <stdexcept>

#include "camera/camera_enumeration.h"
#include "camera/cv_cap.h"
#include "camera/cv_logger.h"
#include "camera/pps_handler.h"
#include "camera/fourcc.h"
#include "runtime_args.h"
#include "video_queue.h"
#include "video_recorder.h"
#include "webserver/http_server.h"
#include "webserver/nt_client.h"

static VideoBuffer<TimestampedFrame> *g_frameBuffer = nullptr;
static VideoBuffer<TimestampedFrame> *g_frameBufferStream = nullptr;
static PPSHandler *g_ppsHandler = nullptr;
static NTClient *g_ntClient = nullptr;

using namespace cv;
using namespace std;

int main(const int argc, char *argv[]) {
    // ReSharper disable once CppUseStructuredBinding
    auto flags = RuntimeArgs{};
    args::ArgumentParser parser("Camera Node for FRC Tracking and Playback System");
    args::HelpFlag help(parser, "help", "Display this help menu", {'?', "help"});

    args::Flag enumerateOnly(parser, "enumerate", "Enumerate supported V4L2 camera modes, and exit",
                             {"enum", "enumerate"});
    args::Flag verboseFlag(parser, "verbose", "Enable verbose logging (debug level)", {'v', "verbose"});
    args::Flag traceFlag(parser, "trace", "Enable trace logging (highest detail)", {'t', "trace"});

    args::ValueFlag cameraIdFlag(parser, "camera", "Camera ID as in /dev/videoX", {'c', "camera"}, flags.cameraId);
    args::ValueFlag fpsFlag(parser, "fps", "Target frame rate", {'f', "fps"}, flags.fps);
    args::ValueFlag widthFlag(parser, "px", "Target frame width", {'w', "width"}, flags.width);
    args::ValueFlag heightFlag(parser, "px", "Target frame height", {'h', "height"}, flags.height);
    args::ValueFlag fourccFlag(parser, "fourcc", "Target FourCC string", {'F', "fourcc"},
                               fourcc_to_string(flags.fourcc));

    args::ValueFlag rollingFpsFrameCountFlag(parser, "frames", "Frame count for fps averaging", {"rolling-fps-frames"},
                                             flags.rollingFpsFrameCount);
    args::ValueFlag fpsReportingIntervalFlag(parser, "frames", "How often to report FPS, 0 to disable",
                                             {"fps-interval"}, flags.fpsReportingInterval);

    args::ValueFlag bufferMaxSizeFlag(parser, "buffer-max-size", "Maximum number of frames to buffer (0 for unlimited)",
                                      {"buffer-max-size"}, flags.bufferMaxSize);

    args::ValueFlag encoderFlag(parser, "encoder", "Encoder type: jpeg or raw", {'e', "encoder"}, string("jpeg"));

    args::ValueFlag encoderArgsFlag(parser, "encoder-args",
                                    "Encoder arguments (e.g., quality:90 for jpeg, order:rgb/bgr/gray/bgr565/bgr555 for raw)",
                                    {'a', "encoder-args"}, flags.encoderArgs);

    args::ValueFlag encoderThreadsFlag(parser, "threads", "Number of threads for the encoder", {'j', "encoder-threads"},
                                       flags.encoderThreads);

    args::ValueFlag outputDirFlag(parser, "dir", "Output directory, defaults to output/", {'o', "output"},
                                  flags.outputDir);

    args::ValueFlag httpPortFlag(parser, "http-port", "HTTP server port", {'p', "http-port"}, flags.httpPort);

    args::ValueFlag streamJpegQualityFlag(parser, "quality", "MJPEG streaming quality", {'q', "stream-quality"},
                                          flags.jpegStreamQuality);
    args::ValueFlag streamJpegWidthFlag(parser, "px", "MJPEG streaming width", {'W', "stream-width"},
                                        flags.jpegStreamWidth);
    args::ValueFlag streamJpegHeightFlag(parser, "px", "MJPEG streaming height", {'H', "stream-height"},
                                         flags.jpegStreamHeight);
    args::ValueFlag streamJpegFpsFlag(parser, "fps", "MJPEG streaming FPS", {"stream-fps"}, flags.jpegStreamFps);

    // NEW: Grid composition settings
    args::ValueFlag gridComposeFlag(parser, "compose", "Enable 2x2 grid composition", {"grid-compose"},
                                    flags.composeGrid);
    args::ValueFlag gridWidthFlag(parser, "px", "Grid preview width", {"grid-width"},
                                  flags.gridPreviewWidth);
    args::ValueFlag gridHeightFlag(parser, "px", "Grid preview height", {"grid-height"},
                                   flags.gridPreviewHeight);

    // NEW: PPS/GPIO flag
    args::ValueFlag ppsGpioFlag(parser, "pin", "GPIO pin for 1 PPS signal (-1 = disabled)", {"pps-gpio"},
                                flags.ppsGpioPin);

    // NEW: NetworkTables flags
    args::ValueFlag ntServerFlag(parser, "address", "NetworkTables server address", {"nt-server"},
                                 flags.ntServerAddress);
    args::ValueFlag ntStartFlag(parser, "topic", "NetworkTables match start topic", {"nt-match-start"},
                                flags.ntMatchStartTopic);
    args::ValueFlag ntEndFlag(parser, "topic", "NetworkTables match end topic", {"nt-match-end"},
                              flags.ntMatchEndTopic);
    args::ValueFlag ntAliasFlag(parser, "alias", "NetworkTables local alias", {"nt-local"},
                                flags.ntLocalAlias);

    args::CompletionFlag completion(parser, {"complete"});

    try {
        parser.ParseCLI(argc, argv);
    } catch (const args::Completion &e) {
        std::cout << e.what();
        return 0;
    } catch (const args::Help &) {
        std::cout << parser;
        return 0;
    } catch (const args::ParseError &e) {
        cerr << e.what() << endl;
        cerr << parser;
        return 1;
    }

    if (traceFlag) {
        spdlog::set_level(spdlog::level::trace);
    } else if (verboseFlag) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }

    flags.cameraId = args::get(cameraIdFlag);
    flags.fps = args::get(fpsFlag);
    flags.width = args::get(widthFlag);
    flags.height = args::get(heightFlag);
    flags.fourcc = string_to_fourcc(args::get(fourccFlag));
    flags.rollingFpsFrameCount = args::get(rollingFpsFrameCountFlag);
    flags.fpsReportingInterval = args::get(fpsReportingIntervalFlag);
    flags.bufferMaxSize = args::get(bufferMaxSizeFlag);

    // Handle encoder flag
    if (std::string encoderStr = args::get(encoderFlag); encoderStr == "jpeg") {
        flags.encoderType = EncoderType::JPEG;
    } else if (encoderStr == "raw") {
        flags.encoderType = EncoderType::RAW;
    } else {
        throw std::invalid_argument("Encoder must be 'jpeg' or 'raw'");
    }

    flags.encoderArgs = args::get(encoderArgsFlag);
    flags.encoderThreads = args::get(encoderThreadsFlag);
    flags.outputDir = args::get(outputDirFlag);

    flags.httpPort = args::get(httpPortFlag);
    flags.jpegStreamQuality = args::get(streamJpegQualityFlag);
    flags.jpegStreamWidth = args::get(streamJpegWidthFlag);
    flags.jpegStreamHeight = args::get(streamJpegHeightFlag);
    flags.jpegStreamFps = args::get(streamJpegFpsFlag);

    // NEW: Grid composition settings
    flags.composeGrid = args::get(gridComposeFlag);
    flags.gridPreviewWidth = args::get(gridWidthFlag);
    flags.gridPreviewHeight = args::get(gridHeightFlag);

    // NEW: PPS GPIO
    flags.ppsGpioPin = args::get(ppsGpioFlag);

    // NEW: NetworkTables
    flags.ntServerAddress = args::get(ntServerFlag);
    flags.ntMatchStartTopic = args::get(ntStartFlag);
    flags.ntMatchEndTopic = args::get(ntEndFlag);
    flags.ntLocalAlias = args::get(ntAliasFlag);

    if (enumerateOnly) {
        enumerate_camera_modes(flags.cameraId);
        return 0;
    }

    spdlog_opencv_init();
    spdlog::info("Starting TaPS Camera Node");

    // Initialize PPS handler
    PPSHandler ppsHandler(flags.ppsGpioPin);
    g_ppsHandler = &ppsHandler;
    bool pps_available = ppsHandler.initialize();
    if (pps_available) {
        spdlog::info("PPS hardware timestamping enabled");
    } else {
        spdlog::warn("PPS not available, using system clock timestamps");
    }

    // Initialize NetworkTables client
    NTClient ntClient;
    g_ntClient = &ntClient;
    if (!flags.ntServerAddress.empty()) {
        ntClient.initialize(flags.ntServerAddress,
                           flags.ntMatchStartTopic.empty() ? "RoboRIO/matchStart" : flags.ntMatchStartTopic,
                           flags.ntMatchEndTopic.empty() ? "RoboRIO/matchEnd" : flags.ntMatchEndTopic,
                           flags.ntLocalAlias.empty() ? "CameraNode" : flags.ntLocalAlias);

        // Set up recording callback
        ntClient.set_recording_callback([](bool start) {
            spdlog::info("NT recording command: {}", start ? "START" : "STOP");
            VideoRecordThread::setRecording(start);
        });

        ntClient.start();
        if (ntClient.is_connected()) {
            spdlog::info("NetworkTables connected to {}", flags.ntServerAddress);
        }
    } else {
        spdlog::info("NetworkTables not configured, running without NT triggers");
    }

    Mat frame;
    VideoCapture cap;
    cv_cap_setup(&cap, flags);

    auto frameBuffer = VideoBuffer<TimestampedFrame>(flags.bufferMaxSize);
    auto frameBufferStream = VideoBuffer<TimestampedFrame>(flags.bufferMaxSize);
    g_frameBuffer = &frameBuffer;
    g_frameBufferStream = &frameBufferStream;

    // Start video recorder
    VideoRecordThread::begin(&frameBuffer, flags.outputDir, flags);

    // Wire PPS timestamps into video recorder (enhanced recorder uses g_ppsHandler)
    if (pps_available) {
        VideoRecordThread::setPPSHandler(&ppsHandler);
        spdlog::info("Video recorder configured with PPS timestamps");
    }

    // Start HTTP server
    HttpServer::begin(&frameBufferStream, flags);

    std::signal(SIGINT, [](const int sig) {
        std::cout << "\n";
        spdlog::warn("SIGINT received");
        g_frameBuffer->shutdown();
        g_frameBufferStream->shutdown();
        VideoRecordThread::shutdown();
        HttpServer::stop();
        if (g_ntClient) g_ntClient->stop();
        if (g_ppsHandler) g_ppsHandler->shutdown();
        std::exit(sig);
    });

    vector<double> delta_times;
    delta_times.reserve(flags.rollingFpsFrameCount);

    unsigned long long frame_count = 0;

    auto start_time = chrono::high_resolution_clock::now();

    if (cap.get(CAP_PROP_FRAME_HEIGHT) != flags.height || cap.get(CAP_PROP_FRAME_WIDTH) != flags.width) {
        spdlog::critical("frame size from camera {}x{} != expected {}x{}", frame.cols, frame.rows, flags.width,
                         flags.height);
        VideoRecordThread::shutdown();
        HttpServer::stop();
        std::exit(1);
    }

    for (;;) {
        cap.read(frame);

        if (frame.empty()) {
            spdlog::error("blank frame grabbed");
            break;
        }

        auto end_time = chrono::high_resolution_clock::now();

        chrono::duration<double, milli> elapsed =
                end_time - start_time;

        double delta_ms = elapsed.count();

        start_time = end_time;

        delta_times.push_back(delta_ms);
        frame_count++;

        // Get PPS timestamp if available, otherwise use system clock
        std::chrono::nanoseconds ptp_ts;
        if (pps_available && g_ppsHandler) {
            ptp_ts = g_ppsHandler->get_timestamp();
        } else {
            ptp_ts = std::chrono::high_resolution_clock::now().time_since_epoch();
        }

        if (VideoRecordThread::getState() != VideoRecordThread::RecorderState::Saving) {
            if (!frameBuffer.tryPush(TimestampedFrame{frame, ptp_ts})) {
                spdlog::warn("record buffer full, dropping frame #{}", frame_count);
            }
        }

        if (!frameBufferStream.tryPush(TimestampedFrame{frame, ptp_ts})) {
            spdlog::warn("stream buffer full, dropping frame #{}", frame_count);
        }

        // Publish health via NT if available
        if (g_ntClient && g_ntClient->is_connected()) {
            static int health_interval = 0;
            if (frame_count % 30 == 0) { // Every 30 frames
                double rate = (delta_ms > 0) ? (1000.0 / delta_ms) : 0;
                std::string state_str;
                switch (VideoRecordThread::getState()) {
                    case VideoRecordThread::RecorderState::Idle:
                        state_str = "idle"; break;
                    case VideoRecordThread::RecorderState::Recording:
                        state_str = "recording"; break;
                    case VideoRecordThread::RecorderState::Saving:
                        state_str = "saving"; break;
                }
                g_ntClient->publish_health(state_str, static_cast<int>(frame_count), rate);
            }
        }

        if (frame_count >= flags.rollingFpsFrameCount) {
            double sum_dt = 0.0;

            for (const double dt: delta_times)
                sum_dt += dt;

            const double mean_dt =
                    sum_dt / flags.rollingFpsFrameCount;

            double rate = 1000.0 / mean_dt;

            if (frame_count % flags.fpsReportingInterval == 0) {
                spdlog::debug(
                    "rolling avg fps of last {} frames: {:.2f}fps",
                    flags.rollingFpsFrameCount, rate);
                spdlog::debug("buffer health: {}/{}", frameBuffer.size(), flags.bufferMaxSize);
            }

            delta_times.erase(delta_times.begin());
        }
    }

    // Cleanup
    VideoRecordThread::shutdown();
    HttpServer::stop();
    if (g_ntClient) g_ntClient->stop();
    if (g_ppsHandler) g_ppsHandler->shutdown();
    return 0;
}
