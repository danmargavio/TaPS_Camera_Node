//
// Runtime Arguments for CameraNode
// Enhanced with PPS, NetworkTables, and grid composition options
//

#ifndef TAPS_CAMERANODE_RUNTIME_ARGS_H
#define TAPS_CAMERANODE_RUNTIME_ARGS_H

#include <filesystem>
#include <string>

enum class EncoderType {
    JPEG,
    RAW
};

struct RuntimeArgs {
    // Camera settings
    int cameraId = 0;
    unsigned short fps = 60;
    unsigned long width = 640;
    unsigned long height = 480;
    unsigned long fourcc = 0x47504a4d; // MJPG
    unsigned short rollingFpsFrameCount = 100;
    unsigned short fpsReportingInterval = 100;

    // Buffer settings
    unsigned long bufferMaxSize = 0; // 0 means unlimited

    // Encoder settings
    unsigned char encoderThreads = 4;
    EncoderType encoderType = EncoderType::JPEG; // default to JPEG
    std::filesystem::path outputDir = "output/";
    std::string encoderArgs; // encoder-specific arguments like "quality:90" or "order:rgb"

    // HTTP streaming settings
    unsigned short httpPort = 8080;
    unsigned short jpegStreamQuality = 85;
    unsigned short jpegStreamFps = 60;
    unsigned long jpegStreamWidth = 640;
    unsigned long jpegStreamHeight = 480;

    // PPS / GPIO settings (NEW)
    int ppsGpioPin = -1; // GPIO pin for 1 PPS signal (-1 = disabled)

    // NetworkTables settings (NEW)
    std::string ntServerAddress; // e.g., "10.0.1.X"
    std::string ntMatchStartTopic; // e.g., "RoboRIO/matchStart"
    std::string ntMatchEndTopic; // e.g., "RoboRIO/matchEnd"
    std::string ntLocalAlias; // e.g., "CameraNode1"

    // Grid composition settings (NEW)
    bool composeGrid = true; // Compose 2x2 grid for /grid-stream endpoint
    unsigned short gridPreviewWidth = 640;
    unsigned short gridPreviewHeight = 480;

    // AprilTag detection settings (NEW)
    bool apriltagEnabled = false;
    std::string apriltagFamily = "tag36h11";
    int apriltagThreads = 1;
    double apriltagQuadDecimation = 2.0;   // 1..4, higher = faster/coarser
    double apriltagBlur = 0.0;             // -1 = auto, 0 = none
    bool apriltagEstimatePose = true;
    double apriltagTagSizeMeters = 0.165;  // physical tag edge length
    double apriltagFx = 0.0;               // 0 = derive default (f = width)
    double apriltagFy = 0.0;
    double apriltagCx = 0.0;               // 0 = derive default (image center)
    double apriltagCy = 0.0;
    std::string cameraAlias;               // identity written to .taps header (falls back to ntLocalAlias)

    // Camera calibration persistence (NEW)
    std::string calibrationFile;           // calibration.json path (empty = <outputDir>/calibration.json)
};

#endif // TAPS_CAMERANODE_RUNTIME_ARGS_H
