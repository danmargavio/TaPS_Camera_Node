//
// Created by kevin on 7/31/26.
//

#include "cv_logger.h"

#include <opencv2/core/utils/logger.hpp>
#include <spdlog/spdlog.h>


void spdlog_opencv_callback(
    const cv::utils::logging::LogLevel logLevel,
    const char *tag,
    const char *file,
    const int line,
    const char *func,
    const char *message) {
    spdlog::level::level_enum level;

    switch (logLevel) {
        case cv::utils::logging::LOG_LEVEL_FATAL:
            level = spdlog::level::critical;
            break;
        case cv::utils::logging::LOG_LEVEL_ERROR:
            level = spdlog::level::err;
            break;
        case cv::utils::logging::LOG_LEVEL_WARNING:
            level = spdlog::level::warn;
            break;
        case cv::utils::logging::LOG_LEVEL_INFO:
            level = spdlog::level::info;
            break;
        case cv::utils::logging::LOG_LEVEL_DEBUG:
        case cv::utils::logging::LOG_LEVEL_VERBOSE:
        default:
            level = spdlog::level::trace;
            break;
    }

    if (tag)
        spdlog::log(
            spdlog::source_loc{file, line, func},
            level,
            "[OpenCV/{}] {}",
            tag,
            message);
    else
        spdlog::log(
            spdlog::source_loc{file, line, func},
            level,
            "[OpenCV] {}",
            message);
}
