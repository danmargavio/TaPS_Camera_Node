//
// Dan added this file
//

#include "playback_manager.h"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <iostream>

PlaybackManager::PlaybackManager(const PlaybackConfig& config) 
    : config_(config), running_(false), paused_(false), 
      playback_speed_(1.0), current_time_(0.0), current_frame_(0), duration_(0.0) {
}

PlaybackManager::~PlaybackManager() {
    stopPlayback();
}

bool PlaybackManager::loadVideos(const std::vector<std::string>& video_paths) {
    streams_.clear();
    
    if (video_paths.size() < 4) {
        spdlog::error("At least 4 videos required for playback grid");
        return false;
    }
    
    // Limit to 4 videos
    size_t count = std::min(video_paths.size(), static_cast<size_t>(4));
    
    for (size_t i = 0; i < count; ++i) {
        try {
            auto stream = std::make_unique<VideoStream>();
            stream->path = video_paths[i];
            
            // Open the taps file
            stream->reader = TaPS_Reader(stream->path);
            const auto& header = stream->reader.get_header();
            
            stream->width = static_cast<int>(header.width);
            stream->height = static_cast<int>(header.height);
            stream->fps = header.target_fps;
            stream->valid = true;
            stream->total_frames = header.frame_count;
            
            // Calculate duration based on frame count and fps
            double duration_seconds = static_cast<double>(header.frame_count) / header.target_fps;
            spdlog::info("Loaded video {} with {} frames at {:.2f} FPS, duration: {:.2f}s", 
                        stream->path, header.frame_count, header.target_fps, duration_seconds);
            
            streams_.push_back(std::move(stream));
        } catch (const std::exception& e) {
            spdlog::error("Failed to load video {}: {}", video_paths[i], e.what());
            return false;
        }
    }
    
    // Set total duration based on the longest video
    if (!streams_.empty()) {
        double max_duration = 0.0;
        for (const auto& stream : streams_) {
            if (stream->valid) {
                double duration = static_cast<double>(stream->total_frames) / stream->fps;
                max_duration = std::max(max_duration, duration);
            }
        }
        duration_ = max_duration;
    }
    
    return true;
}

void PlaybackManager::startPlayback() {
    if (running_.load()) {
        return;
    }
    
    running_ = true;
    paused_ = false;
    current_frame_ = 0;
    current_time_ = 0.0;
    
    // Reset all streams to beginning
    resetStreams();
    
    playback_thread_ = std::thread(&PlaybackManager::playbackLoop, this);
}

void PlaybackManager::stopPlayback() {
    running_ = false;
    paused_ = false;
    
    if (playback_thread_.joinable()) {
        cv_.notify_all();
        playback_thread_.join();
    }
}

void PlaybackManager::setRotation(int stream_index, int rotation_degrees) {
    if (stream_index >= 0 && static_cast<size_t>(stream_index) < streams_.size()) {
        // Normalize rotation to 0, 90, 180, or 270
        int normalized = ((rotation_degrees % 360) + 360) % 360;
        if (normalized == 0 || normalized == 90 || normalized == 180 || normalized == 270) {
            streams_[stream_index]->rotation = normalized;
        }
    }
}

void PlaybackManager::setSpeed(double speed) {
    playback_speed_ = std::max(0.0, speed);
}

void PlaybackManager::togglePause() {
    paused_ = !paused_;
}

void PlaybackManager::seekTo(double seconds) {
    if (seconds < 0) seconds = 0;
    if (seconds > duration_.load()) seconds = duration_.load();
    
    current_time_ = seconds;
    
    // Convert time to frame number
    double target_frame = seconds * streams_[0]->fps; // Use first stream's fps
    current_frame_ = static_cast<uint64_t>(target_frame);
    
    // Reset all streams to this position
    resetStreams();
}

void PlaybackManager::seekToFrame(uint64_t frame_idx) {
    current_frame_ = frame_idx;
    current_time_ = static_cast<double>(frame_idx) / streams_[0]->fps; // Use first stream's fps
    
    // Reset all streams to this position
    resetStreams();
}

double PlaybackManager::getCurrentTime() const {
    return current_time_.load();
}

double PlaybackManager::getDuration() const {
    return duration_.load();
}

bool PlaybackManager::isPlaying() const {
    return running_.load() && !paused_.load();
}

bool PlaybackManager::isPaused() const {
    return paused_.load();
}

uint64_t PlaybackManager::getCurrentFrame() const {
    return current_frame_.load();
}

void PlaybackManager::resetStreams() {
    for (auto& stream : streams_) {
        if (stream->valid) {
            try {
                // Seek to the beginning
                stream->reader.seek_to_first_frame();
                stream->current_frame_idx = 0;
            } catch (const std::exception& e) {
                spdlog::error("Failed to reset stream: {}", e.what());
            }
        }
    }
}

bool PlaybackManager::readNextFrame(int stream_index) {
    if (stream_index < 0 || static_cast<size_t>(stream_index) >= streams_.size()) {
        return false;
    }
    
    auto& stream = streams_[stream_index];
    if (!stream->valid) {
        return false;
    }
    
    try {
        TaPS_Reader::Frame frame;
        if (stream->reader.read_next_frame(frame)) {
            // Convert raw data to OpenCV Mat
            // This is a simplified approach - you'd need to handle different encodings properly
            stream->frame_buffer = cv::Mat();
            return true;
        }
    } catch (const std::exception& e) {
        spdlog::error("Failed to read frame from stream {}: {}", stream_index, e.what());
    }
    
    return false;
}

void PlaybackManager::playbackLoop() {
    spdlog::info("Starting playback loop");
    
    // Create output frame
    cv::Mat output_frame(config_.grid_height, config_.grid_width, CV_8UC3, cv::Scalar(0, 0, 0));
    
    // Calculate frame interval based on target FPS
    double target_fps = streams_.empty() ? 30.0 : streams_[0]->fps;
    double frame_interval_ms = 1000.0 / target_fps;
    
    auto last_frame_time = std::chrono::high_resolution_clock::now();
    
    while (running_.load()) {
        if (paused_.load()) {
            // Brief pause to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        auto frame_start_time = std::chrono::high_resolution_clock::now();
        
        // Render the grid frame
        renderGridFrame(output_frame);
        
        // Update current time
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(now - last_frame_time).count();
        
        if (elapsed_ms > 0) {
            double time_delta = elapsed_ms / 1000.0 * playback_speed_.load();
            current_time_ += time_delta;
            
            // Update frame number
            current_frame_ = static_cast<uint64_t>(current_time_ * target_fps);
            
            // Reset to beginning if we've gone past duration
            if (current_time_.load() >= duration_.load()) {
                current_time_ = 0.0;
                current_frame_ = 0;
                resetStreams();
            }
        }
        
        last_frame_time = now;
        
        // Sleep to maintain target frame rate
        auto frame_end_time = std::chrono::high_resolution_clock::now();
        auto frame_duration_ms = std::chrono::duration<double, std::milli>(frame_end_time - frame_start_time).count();
        double sleep_time_ms = frame_interval_ms - frame_duration_ms;
        
        if (sleep_time_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time_ms)));
        }
    }
    
    spdlog::info("Playback loop stopped");
}

void PlaybackManager::renderGridFrame(cv::Mat& output_frame) {
    if (streams_.empty()) {
        // Just clear the frame
        output_frame = cv::Scalar(0, 0, 0);
        return;
    }
    
    // Clear output frame
    output_frame = cv::Scalar(0, 0, 0);
    
    // Calculate grid dimensions
    int tile_width = config_.grid_width / 2;
    int tile_height = config_.video_grid_height / 2;
    
    // Process each stream (4 streams in a 2x2 grid)
    for (int i = 0; i < 4 && i < static_cast<int>(streams_.size()); ++i) {
        const auto& stream = streams_[i];
        if (!stream->valid) continue;
        
        // Calculate tile position
        int col = i % 2;
        int row = i / 2;
        int x_start = col * tile_width;
        int y_start = row * tile_height;
        
        // Create a placeholder frame with video info
        cv::Mat tile(tile_height, tile_width, CV_8UC3, cv::Scalar(50, 50, 50));
        
        // Add video info text
        std::string info_text = "Video " + std::to_string(i + 1);
        cv::putText(tile, info_text, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        
        // Add frame count and FPS
        std::string stats_text = "Frames: " + std::to_string(stream->total_frames) + 
                                " | FPS: " + std::to_string(stream->fps);
        cv::putText(tile, stats_text, cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // Add current frame info
        std::string frame_text = "Frame: " + std::to_string(current_frame_.load());
        cv::putText(tile, frame_text, cv::Point(10, 90), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // Add rotation info
        std::string rot_text = "Rot: " + std::to_string(stream->rotation) + "°";
        cv::putText(tile, rot_text, cv::Point(10, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // Copy tile to output frame
        cv::Rect roi(x_start, y_start, tile_width, tile_height);
        tile.copyTo(output_frame(roi));
    }
}

cv::Mat PlaybackManager::processFrame(const cv::Mat& input, int rotation) {
    cv::Mat result = input.clone();
    
    switch (rotation) {
        case 90:
            cv::rotate(result, result, cv::ROTATE_90_CLOCKWISE);
            break;
        case 180:
            cv::rotate(result, result, cv::ROTATE_180);
            break;
        case 270:
            cv::rotate(result, result, cv::ROTATE_90_COUNTERCLOCKWISE);
            break;
        default:
            // No rotation needed
            break;
    }
    
    return result;
}

void PlaybackManager::updateDisplay() {
    // This would be where you update the actual display in a real implementation
    // For now, it's just a placeholder
}