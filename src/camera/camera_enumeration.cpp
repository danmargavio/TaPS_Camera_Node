//
// Created by kevin on 7/31/26.
//

#include "camera_enumeration.h"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>

#include <opencv2/core.hpp>
#include <spdlog/spdlog.h>

#include "fourcc.h"

void enumerate_camera_modes(const int cameraId) {
    const std::string device = "/dev/video" + std::to_string(cameraId);

    const int fd = open(device.c_str(), O_RDWR);

    if (fd < 0) {
        spdlog::error("unable to open V4L2 device {}", device);
        return;
    }

    spdlog::info("Supported camera modes for {}", device);

    v4l2_fmtdesc fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) == 0) {
        const std::string pixel_format = fourcc_to_string(fmt.pixelformat);

        spdlog::info(
            "  {} - {}",
            pixel_format,
            reinterpret_cast<const char *>(fmt.description));

        v4l2_frmsizeenum size{};
        size.pixel_format = fmt.pixelformat;

        while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) == 0) {
            if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                const uint32_t width = size.discrete.width;
                const uint32_t height = size.discrete.height;

                spdlog::info(
                    "    {}x{}",
                    width,
                    height);

                v4l2_frmivalenum interval{};
                interval.pixel_format = fmt.pixelformat;
                interval.width = width;
                interval.height = height;

                while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval) == 0) {
                    if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                        const double fps =
                                static_cast<double>(interval.discrete.denominator) /
                                static_cast<double>(interval.discrete.numerator);

                        spdlog::info(
                            "      {:.3f} FPS",
                            fps);
                    } else if (interval.type == V4L2_FRMIVAL_TYPE_STEPWISE) {
                        const double min_fps =
                                static_cast<double>(interval.stepwise.max.denominator) /
                                static_cast<double>(interval.stepwise.max.numerator);

                        const double max_fps =
                                static_cast<double>(interval.stepwise.min.denominator) /
                                static_cast<double>(interval.stepwise.min.numerator);

                        spdlog::info(
                            "      {:.3f}-{:.3f} FPS (stepwise)",
                            min_fps,
                            max_fps);
                    } else if (interval.type == V4L2_FRMIVAL_TYPE_CONTINUOUS) {
                        const double min_fps =
                                static_cast<double>(interval.stepwise.max.denominator) /
                                static_cast<double>(interval.stepwise.max.numerator);

                        const double max_fps =
                                static_cast<double>(interval.stepwise.min.denominator) /
                                static_cast<double>(interval.stepwise.min.numerator);

                        spdlog::info(
                            "      {:.3f}-{:.3f} FPS (continuous)",
                            min_fps,
                            max_fps);
                    }

                    ++interval.index;
                }
            } else if (size.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
                spdlog::info(
                    "    {}x{} to {}x{} (step {}x{})",
                    size.stepwise.min_width,
                    size.stepwise.min_height,
                    size.stepwise.max_width,
                    size.stepwise.max_height,
                    size.stepwise.step_width,
                    size.stepwise.step_height);
            } else if (size.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
                spdlog::info(
                    "    {}x{} to {}x{} (continuous)",
                    size.stepwise.min_width,
                    size.stepwise.min_height,
                    size.stepwise.max_width,
                    size.stepwise.max_height);
            }

            ++size.index;
        }

        ++fmt.index;
    }

    close(fd);
}
