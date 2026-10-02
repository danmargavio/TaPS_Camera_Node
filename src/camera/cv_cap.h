//
// Created by kevin on 7/31/26.
//

#ifndef TAPS_CAMERANODE_CV_CAP_H
#define TAPS_CAMERANODE_CV_CAP_H

#include <opencv2/videoio.hpp>
#include <spdlog/spdlog.h>

#include "../runtime_args.h"

inline int cv_cap_setup(cv::VideoCapture *cap, RuntimeArgs flags) {
    cap->open(flags.cameraId, cv::CAP_V4L2);
    cap->set(cv::CAP_PROP_FOURCC, flags.fourcc);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, flags.width);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, flags.height);
    cap->set(cv::CAP_PROP_FPS, flags.fps);

    if (!cap->isOpened()) {
        spdlog::error("unable to open camera {}", flags.cameraId);
        return -1;
    }

    spdlog::info(
        "configured: {}x{} @ {} fps, FOURCC {}",
        cap->get(cv::CAP_PROP_FRAME_WIDTH),
        cap->get(cv::CAP_PROP_FRAME_HEIGHT),
        cap->get(cv::CAP_PROP_FPS),
        flags.fourcc
    );

    return 0;
}

#endif //TAPS_CAMERANODE_CV_CAP_H
