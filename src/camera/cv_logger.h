//
// Created by kevin on 7/31/26.
//

#ifndef TAPS_CAMERANODE_CV_LOGGER_H
#define TAPS_CAMERANODE_CV_LOGGER_H

#include <opencv2/core/utils/logger.hpp>

void spdlog_opencv_callback(
    cv::utils::logging::LogLevel logLevel,
    const char *tag,
    const char *file,
    int line,
    const char *func,
    const char *message);

inline void spdlog_opencv_init() {
#if CV_VERSION_MAJOR >= 5
    // Custom sink hook exists only in OpenCV >= 5.
    cv::utils::logging::internal::replaceWriteLogMessageEx(
        spdlog_opencv_callback);
#endif
    // On OpenCV 4.x, cv::log output goes to stderr (captured by journald).
    cv::utils::logging::setLogLevel(
        cv::utils::logging::LOG_LEVEL_VERBOSE);
}

#endif //TAPS_CAMERANODE_CV_LOGGER_H
