//
// Created by kevin on 7/31/26.
//

#ifndef TAPS_CAMERANODE_VIDEO_RECORDER_H
#define TAPS_CAMERANODE_VIDEO_RECORDER_H

#include <chrono>
#include <functional>
#include <thread>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <map>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include "runtime_args.h"
#include "video_queue.h"

// Forward declaration
class PPSHandler;

namespace fs = std::filesystem;

struct TimestampedFrame {
    cv::Mat frame;
    std::chrono::nanoseconds ptpTimestamp;
};

class VideoRecordThread {
public:
    enum class RecorderState {
        Idle,
        Recording,
        Saving,
    };

    static void begin(VideoBuffer<TimestampedFrame> *frameBuffer,
                      const fs::path &outputDir,
                      const RuntimeArgs &flags) {
        s_buffer = frameBuffer;
        s_outputDir = outputDir;
        s_width = flags.width;
        s_height = flags.height;
        s_fps = flags.fps;
        s_encoderType = flags.encoderType;
        s_encoderArgs = flags.encoderArgs;
        s_numEncoders = flags.encoderThreads;

        state = RecorderState::Idle;
        s_dispatcherThread = std::thread(dispatcher);
    }

    // NEW: Set PPS handler for hardware timestamps
    static void setPPSHandler(PPSHandler *handler) {
        s_ppsHandler = handler;
    }

    static void setRecording(const bool record) {
        std::unique_lock lock(s_cmdMutex);
        if ((record && state == RecorderState::Recording) || (!record && state == RecorderState::Idle) || (
                !record && state == RecorderState::Saving)) return;
        s_pendingCommand = record ? Command::Start : Command::Stop;
        s_cmdCv.notify_all();
    }

    static RecorderState getState() { return state; }

    static void setStateCallback(std::function<void(RecorderState)> cb) {
        std::lock_guard lock(s_callbackMutex);
        s_stateCallback = std::move(cb);
    }

    static void shutdown() {
        // Make sure any active session is stopped and closed cleanly first.
        setRecording(false);

        if (s_buffer) {
            s_buffer->shutdown();
        }

        {
            std::unique_lock<std::mutex> lock(s_cmdMutex);
            s_shuttingDown = true;
            s_cmdCv.notify_all();
        }

        if (s_dispatcherThread.joinable()) {
            s_dispatcherThread.join();
        }
    }

private:
    enum class Command { None, Start, Stop };

    struct Job {
        uint64_t frameIdx{};
        int64_t ptpNs{};
        cv::Mat frame;
    };

    struct Result {
        uint64_t frameIdx{};
        int64_t ptpNs{};
        std::vector<unsigned char> jpegData; // holds either JPEG or raw bytes
    };

    class WorkerInbox {
    public:
        explicit WorkerInbox(const size_t capacity = 2) : m_capacity(capacity) {
        }

        void push(Job &&job) {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&] { return m_closed || m_queue.size() < m_capacity; });
            if (m_closed) return;
            m_queue.push_back(std::move(job));
            m_cv.notify_all();
        }

        std::optional<Job> pop() {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&] { return m_closed || !m_queue.empty(); });
            if (m_queue.empty()) return std::nullopt;
            Job job = std::move(m_queue.front());
            m_queue.pop_front();
            m_cv.notify_all();
            return job;
        }

        void close() {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_closed = true;
            m_cv.notify_all();
        }

    private:
        std::deque<Job> m_queue;
        std::mutex m_mutex;
        std::condition_variable m_cv;
        bool m_closed = false;
        size_t m_capacity;
    };

    class ResultBuffer {
    public:
        explicit ResultBuffer(const size_t capacity = 8) : m_capacity(capacity) {}

        void push(Result &&r) {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&] { return m_closed || m_queue.size() < m_capacity; });
            if (m_closed) return;
            m_queue.push_back(std::move(r));
            m_cv.notify_all();
        }

        std::optional<Result> pop() {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&] { return m_closed || !m_queue.empty(); });
            if (m_queue.empty()) return std::nullopt;
            Result r = std::move(m_queue.front());
            m_queue.pop_front();
            m_cv.notify_all();
            return r;
        }

        void close() {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_closed = true;
            m_cv.notify_all();
        }

    private:
        std::deque<Result> m_queue;
        std::mutex m_mutex;
        std::condition_variable m_cv;
        bool m_closed = false;
        size_t m_capacity;
    };

    // A single recording session: one output file, one header, one encoder pool.
    struct Session {
        std::ofstream output;
        fs::path outputFile;
        std::streampos frameCountFieldPos{};
        uint64_t writtenCount = 0;

        int jpegQuality = 85;
        int colorOrder = 0; // 0: RGB, 1: BGR, 2: GRAY, 3: BGR565, 4: BGR555

        std::vector<WorkerInbox> inboxes;
        std::vector<ResultBuffer> results;
        std::vector<std::thread> encoders;
        std::thread writerThread;

        uint64_t nextFrameIdx = 0;
        unsigned nextWorker = 0;
        unsigned numEncoders = 0;
        EncoderType encoderType{};
    };

    static std::map<std::string, std::string> parseEncoderArgs(const std::string &argsStr) {
        std::map<std::string, std::string> argsMap;
        if (argsStr.empty()) {
            return argsMap;
        }

        std::stringstream ss(argsStr);
        std::string pair;
        while (std::getline(ss, pair, ',')) {
            if (const size_t colonPos = pair.find(':'); colonPos != std::string::npos) {
                std::string key = pair.substr(0, colonPos);
                std::string value = pair.substr(colonPos + 1);
                // Trim whitespace
                size_t start = key.find_first_not_of(" \t");
                size_t end = key.find_last_not_of(" \t");
                if (start != std::string::npos && end != std::string::npos) {
                    key = key.substr(start, end - start + 1);
                } else {
                    key.clear();
                }

                start = value.find_first_not_of(" \t");
                end = value.find_last_not_of(" \t");
                if (start != std::string::npos && end != std::string::npos) {
                    value = value.substr(start, end - start + 1);
                } else {
                    value.clear();
                }

                if (!key.empty()) {
                    argsMap[key] = value;
                }
            }
        }
        return argsMap;
    }

    static fs::path makeOutputPath(const fs::path &outputDir) {
        const auto now = std::chrono::system_clock::now();
        const auto t = std::chrono::system_clock::to_time_t(now);
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                            now.time_since_epoch()) % 1'000'000;

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&t), "%Y-%m-%d_%H-%M-%S")
                << '.' << std::setfill('0') << std::setw(6) << us.count();

        return outputDir / ("rec_" + oss.str() + ".taps");
    }

    static std::unique_ptr<Session> startSession(const fs::path &outputDir,
                                                 const unsigned long width, const unsigned long height,
                                                 const double targetFps,
                                                 EncoderType encoderType,
                                                 const std::string &encoderArgsStr,
                                                 const unsigned numEncoders) {
        auto session = std::make_unique<Session>();
        session->outputFile = makeOutputPath(outputDir);
        session->encoderType = encoderType;
        session->numEncoders = numEncoders;

        session->output.open(session->outputFile, std::ios::binary);
        if (!session->output.is_open()) {
            spdlog::error("Failed to open output file: {}", session->outputFile.c_str());
            return nullptr;
        }
        spdlog::info("Recording to {}", session->outputFile.c_str());

        // Write header
        std::string header = "TaPS";
        header.push_back(0x02); // bumped: header now includes frame count
        header.push_back(static_cast<char>(encoderType));
        session->output.write(header.c_str(), static_cast<std::streamsize>(header.length()));
        session->output.write(reinterpret_cast<const char *>(&width), sizeof(width));
        session->output.write(reinterpret_cast<const char *>(&height), sizeof(height));
        session->output.write(reinterpret_cast<const char *>(&targetFps), sizeof(targetFps));
        const unsigned int argsLength = encoderArgsStr.length();
        session->output.write(reinterpret_cast<const char *>(&argsLength), sizeof(unsigned int));
        session->output.write(encoderArgsStr.c_str(), static_cast<std::streamsize>(encoderArgsStr.length()));

        session->frameCountFieldPos = session->output.tellp();
        constexpr uint64_t zeroCount = 0;
        session->output.write(reinterpret_cast<const char *>(&zeroCount), sizeof(zeroCount));
        session->output.flush();

        // parse encoder args
        for (auto argsMap = parseEncoderArgs(encoderArgsStr); const auto &[arg, val]: argsMap) {
            if (encoderType == EncoderType::JPEG && arg == "quality") {
                try {
                    if (int q = std::stoi(val); q >= 0 && q <= 100) {
                        session->jpegQuality = q;
                    } else {
                        spdlog::warn("JPEG quality {} out of range [0,100], using default {}", q, session->jpegQuality);
                    }
                } catch (const std::exception &e) {
                    spdlog::warn("Invalid JPEG value '{}': {}", val, e.what());
                }
            } else if (encoderType == EncoderType::RAW && arg == "order") {
                if (val == "rgb") {
                    session->colorOrder = 0;
                } else if (val == "bgr") {
                    session->colorOrder = 1;
                } else if (val == "gray") {
                    session->colorOrder = 2;
                } else if (val == "bgr565") {
                    session->colorOrder = 3;
                } else if (val == "bgr555") {
                    session->colorOrder = 4;
                } else {
                    spdlog::warn("Unknown color order '{}', using default RGB", val);
                }
            }
        }

        session->inboxes = std::vector<WorkerInbox>(numEncoders);
        session->results = std::vector<ResultBuffer>(numEncoders);
        session->encoders.reserve(numEncoders);

        for (unsigned i = 0; i < numEncoders; ++i) {
            session->encoders.emplace_back([s = session.get(), i, encoderType]() {
                pthread_setname_np(pthread_self(), std::format("encode_{}", i).c_str());
                const std::vector params = {cv::IMWRITE_JPEG_QUALITY, s->jpegQuality};
                while (const auto job = s->inboxes[i].pop()) {
                    Result r;
                    r.frameIdx = job->frameIdx;
                    r.ptpNs = job->ptpNs;
                    if (encoderType == EncoderType::JPEG) {
                        cv::imencode(".jpg", job->frame, r.jpegData, params);
                    } else if (encoderType == EncoderType::RAW) {
                        cv::Mat img;
                        if (job->frame.isContinuous()) {
                            img = job->frame;
                        } else {
                            img = job->frame.clone();
                        }

                        // Process based on color order
                        switch (s->colorOrder) {
                            default:
                            case 0: // RGB
                                if (img.channels() == 3) {
                                    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
                                }
                                break;
                            case 1: // BGR (default OpenCV)
                                // No conversion needed
                                break;
                            case 2: // GRAY
                                if (img.channels() == 3) {
                                    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                                }
                                break;
                            case 3: // BGR565
                                if (img.channels() == 3) {
                                    cv::cvtColor(img, img, cv::COLOR_BGR2BGR565);
                                }
                                break;
                            case 4: // BGR555
                                if (img.channels() == 3) {
                                    cv::cvtColor(img, img, cv::COLOR_BGR2BGR555);
                                }
                                break;
                        }

                        // Copy the raw data
                        r.jpegData.assign(img.datastart, img.dataend);
                    }
                    s->results[i].push(std::move(r));
                }
                s->results[i].close();
            });
        }

        // --- Writer thread: round-robins the same order frames were dispatched in ---
        session->writerThread = std::thread([s = session.get()]() {
            pthread_setname_np(pthread_self(), "vid_writer");
            unsigned rr = 0;
            while (true) {
                const auto r = s->results[rr].pop();
                if (!r) break; // that worker is done and drained -> pipeline finished
                auto size = static_cast<uint32_t>(r->jpegData.size());
                s->output.write(reinterpret_cast<const char *>(&r->frameIdx), sizeof(r->frameIdx));
                s->output.write(reinterpret_cast<const char *>(&r->ptpNs), sizeof(r->ptpNs));
                s->output.write(reinterpret_cast<const char *>(&size), sizeof(size));
                s->output.write(reinterpret_cast<const char *>(r->jpegData.data()), size);
                ++s->writtenCount;

                rr = (rr + 1) % s->numEncoders;
            }
        });

        return session;
    }

    static void patchFrameCount(Session &session) {
        const auto endPos = session.output.tellp();
        session.output.seekp(session.frameCountFieldPos);
        session.output.write(reinterpret_cast<const char *>(&session.writtenCount), sizeof(session.writtenCount));
        session.output.seekp(endPos);
        session.output.flush();
    }

    static void notifyStateChange(const RecorderState newState) {
        state = newState;
        std::function<void(RecorderState)> cb;
        {
            std::lock_guard lock(s_callbackMutex);
            cb = s_stateCallback;
        }
        if (cb) cb(newState);
    }

    static void stopSession(const std::unique_ptr<Session> &session) {
        if (!session) return;
        notifyStateChange(RecorderState::Saving);

        for (auto &inbox: session->inboxes) inbox.close();
        for (auto &encoder: session->encoders) encoder.join();
        session->writerThread.join();

        patchFrameCount(*session);
        session->output.close();

        spdlog::info("Wrote {} frames to {}",
                     session->writtenCount, session->outputFile.c_str());
    }

    static void dispatcher() {
        pthread_setname_np(pthread_self(), "vid_dispatch");
        std::unique_ptr<Session> session;

        while (true) {
            {
                std::unique_lock lock(s_cmdMutex);
                if (s_pendingCommand == Command::Start && !session) {
                    lock.unlock();
                    auto newSession = startSession(s_outputDir, s_width, s_height, s_fps,
                                                   s_encoderType, s_encoderArgs, s_numEncoders);
                    lock.lock();
                    session = std::move(newSession);
                    notifyStateChange(session != nullptr ? RecorderState::Recording : RecorderState::Idle);
                    s_pendingCommand = Command::None;
                    s_cmdCv.notify_all();
                } else if (s_pendingCommand == Command::Stop && session) {
                    lock.unlock();
                    stopSession(session);
                    lock.lock();
                    session.reset();
                    notifyStateChange(RecorderState::Idle);
                    s_pendingCommand = Command::None;
                    s_cmdCv.notify_all();
                } else if (s_pendingCommand != Command::None) {
                    s_pendingCommand = Command::None;
                    s_cmdCv.notify_all();
                }

                if (s_shuttingDown && !session) {
                    break;
                }
            }

            // Pull one frame with a short wait so we can keep checking for commands.
            const auto item = s_buffer->pop();
            if (!item) {
                // Buffer has been shut down.
                std::unique_lock lock(s_cmdMutex);
                if (session) {
                    lock.unlock();
                    stopSession(session);
                    lock.lock();
                    session.reset();
                    notifyStateChange(RecorderState::Idle);
                }
                break;
            }

            if (item->frame.empty()) continue;
            if (!session) continue; // not currently recording; drop the frame

            Job job;
            job.frameIdx = session->nextFrameIdx++;
            job.ptpNs = item->ptpTimestamp.count();
            job.frame = item->frame;
            session->inboxes[session->nextWorker].push(std::move(job));
            session->nextWorker = (session->nextWorker + 1) % session->numEncoders;
        }
    }

    static inline std::atomic<RecorderState> state = RecorderState::Idle;

    static inline VideoBuffer<TimestampedFrame> *s_buffer = nullptr;
    static inline fs::path s_outputDir;
    static inline unsigned long s_width = 0;
    static inline unsigned long s_height = 0;
    static inline double s_fps = 0;
    static inline EncoderType s_encoderType{};
    static inline std::string s_encoderArgs;
    static inline unsigned char s_numEncoders = 1;

    // NEW: PPS handler for hardware timestamps
    static inline PPSHandler *s_ppsHandler = nullptr;

    static inline std::thread s_dispatcherThread;

    static inline std::mutex s_cmdMutex;
    static inline std::condition_variable s_cmdCv;
    static inline auto s_pendingCommand = Command::None;
    static inline bool s_shuttingDown = false;

    static inline std::mutex s_callbackMutex;
    static inline std::function<void(RecorderState)> s_stateCallback;
};

#endif //TAPS_CAMERANODE_VIDEO_RECORDER_H
