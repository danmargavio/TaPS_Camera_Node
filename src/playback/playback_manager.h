//
// Dan added this file
//

#ifndef TAPS_CAMERANODE_PLAYBACK_MANAGER_H
#define TAPS_CAMERANODE_PLAYBACK_MANAGER_H

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <filesystem>

#include "../taps_reader/taps_reader.h"
#include "../video_queue.h"

class PlaybackManager {
public:
    struct PlaybackConfig {
        std::string output_dir;
        int grid_width = 1920;
        int grid_height = 1200;
        int video_grid_height = 1100; // Reserve space for log display
        bool use_opencv = true; // Use OpenCV for playback instead of external player
    };

    struct VideoStream {
        std::string path;
        TaPS_Reader reader;
        cv::Mat frame_buffer;
        int width = 0;
        int height = 0;
        double fps = 30.0;
        bool valid = false;
        int rotation = 0; // 0, 90, 180, 270 degrees
        uint64_t current_frame_idx = 0;
        uint64_t total_frames = 0;
    };

    PlaybackManager(const PlaybackConfig& config);
    ~PlaybackManager();

    bool loadVideos(const std::vector<std::string>& video_paths);
    void startPlayback();
    void stopPlayback();
    void setRotation(int stream_index, int rotation_degrees);
    void setSpeed(double speed);
    void togglePause();
    void seekTo(double seconds);
    void seekToFrame(uint64_t frame_idx);

    // Getters for UI updates
    double getCurrentTime() const;
    double getDuration() const;
    bool isPlaying() const;
    bool isPaused() const;
    uint64_t getCurrentFrame() const;

private:
    void playbackLoop();
    void renderGridFrame(cv::Mat& output_frame);
    cv::Mat processFrame(const cv::Mat& input, int rotation);
    void updateDisplay();
    bool readNextFrame(int stream_index);
    void resetStreams();

    PlaybackConfig config_;
    std::vector<std::unique_ptr<VideoStream>> streams_;
    std::atomic<bool> running_;
    std::atomic<bool> paused_;
    std::atomic<double> playback_speed_;
    std::atomic<double> current_time_;
    std::atomic<uint64_t> current_frame_;
    std::atomic<double> duration_;
    std::thread playback_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
};

#endif //TAPS_CAMERANODE_PLAYBACK_MANAGER_H