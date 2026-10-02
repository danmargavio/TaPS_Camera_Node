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
    cv::utils::logging::internal::replaceWriteLogMessageEx(
        spdlog_opencv_callback);

    cv::utils::logging::setLogLevel(
        cv::utils::logging::LOG_LEVEL_VERBOSE);
}

#endif //TAPS_CAMERANODE_CV_LOGGER_H
