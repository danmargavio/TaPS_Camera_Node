//
// Created by kevin on 7/31/26.
//

#ifndef TAPS_CAMERANODE_FOURCC_H
#define TAPS_CAMERANODE_FOURCC_H

#include <stdexcept>
#include <string>

static std::string fourcc_to_string(const unsigned long fourcc) {
    std::string result(4, ' ');

    result[0] = static_cast<char>(fourcc & 0xff);
    result[1] = static_cast<char>(fourcc >> 8 & 0xff);
    result[2] = static_cast<char>(fourcc >> 16 & 0xff);
    result[3] = static_cast<char>(fourcc >> 24 & 0xff);

    return result;
}

static unsigned long string_to_fourcc(const std::string &fourcc) {
    if (fourcc.size() != 4)
        throw std::invalid_argument("FOURCC must be exactly 4 characters");

    return static_cast<unsigned long>(fourcc[0]) |
           (static_cast<unsigned long>(fourcc[1]) << 8) |
           (static_cast<unsigned long>(fourcc[2]) << 16) |
           (static_cast<unsigned long>(fourcc[3]) << 24);
}

#endif //TAPS_CAMERANODE_FOURCC_H
