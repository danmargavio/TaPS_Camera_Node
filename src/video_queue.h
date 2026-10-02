//
// Created by kevin on 7/31/26.
//

#ifndef TAPS_CAMERANODE_VIDEO_QUEUE_H
#define TAPS_CAMERANODE_VIDEO_QUEUE_H

#pragma once

#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>

template <typename T>
class VideoBuffer {
public:
    explicit VideoBuffer(const size_t maxSize = 0) : maxSize_(maxSize) {}

    bool tryPush(T item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shuttingDown_) return false;
            if (maxSize_ != 0 && queue_.size() >= maxSize_) {
                return false; // full
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
        return true;
    }

    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || shuttingDown_; });

        if (queue_.empty() && shuttingDown_) {
            return std::nullopt;
        }

        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shuttingDown_ = true;
        }
        cv_.notify_all();
    }

    bool pushClosed() const {
        return shuttingDown_;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    size_t capacity() const { return maxSize_; }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> queue_;
    size_t maxSize_;
    bool shuttingDown_ = false;
};

#endif //TAPS_CAMERANODE_VIDEO_QUEUE_H
