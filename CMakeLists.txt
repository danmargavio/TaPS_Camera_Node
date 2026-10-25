cmake_minimum_required(VERSION 3.28)
project(TaPS_CameraNode)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
find_package(PkgConfig QUIET)

# Existing dependencies
FetchContent_Declare(
        args
        GIT_REPOSITORY https://github.com/Taywee/args.git
        GIT_TAG 6.6.0
)
FetchContent_MakeAvailable(args)

FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# Optional: ntcore (WPILib NetworkTables core)
option(TAPS_ENABLE_NT "Enable NetworkTables (ntcore) support" ON)
if(TAPS_ENABLE_NT)
    message(STATUS "ntcore support enabled")
    find_package(ntcore QUIET)
    if(ntcore_FOUND)
        add_definitions(-DHAVE_NT_CORE)
        set(NT_LIB ntcore)
    else()
        message(WARNING "ntcore C++ library not found. NT support will be disabled at runtime.")
        set(NT_LIB "")
    endif()
else()
    message(STATUS "ntcore support disabled")
    set(NT_LIB "")
endif()

# Optional: libgpiod (GPIO/PPS) — v2 C++ bindings via pkg-config
option(TAPS_ENABLE_PPS "Enable PPS/GPIO support" ON)
if(TAPS_ENABLE_PPS AND PkgConfig_FOUND)
    pkg_check_modules(GPIOD libgpiodcxx)
    if(GPIOD_FOUND)
        add_definitions(-DHAVE_GPIOD)
        set(GPIOD_LIB ${GPIOD_LINK_LIBRARIES})
    else()
        message(WARNING "libgpiodcxx not found. PPS support will be disabled at runtime.")
        set(GPIOD_LIB "")
    endif()
else()
    message(STATUS "PPS/GPIO support disabled")
    set(GPIOD_LIB "")
endif()

# AprilTag 3 (target detection / pose estimation)
# Built from source via FetchContent (requires AprilTag >= 3.4 API).
option(TAPS_ENABLE_APRILTAG "Enable AprilTag detection support" ON)
if(TAPS_ENABLE_APRILTAG)
    message(STATUS "AprilTag support enabled")
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
            apriltag
            GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
            GIT_TAG v3.4.3
            GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(apriltag)
    find_package(Threads REQUIRED)
    add_definitions(-DHAVE_APRILTAG)
    set(APRILTAG_LIB apriltag Threads::Threads)
else()
    message(STATUS "AprilTag support disabled")
    set(APRILTAG_LIB "")
endif()

# OpenCV >= 4.5 (Raspberry Pi OS trixie ships 4.10; 5.x also accepted).
find_package(OpenCV 4.5 REQUIRED)
find_package(spdlog REQUIRED)
# libhttpserver (>= 2.0 API: create_webserver/register_path/value-typed
# http_response). Autotools project: discovered via pkg-config, typically
# built from source (tag 2.0.0) and installed to /usr/local.
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBHTTPSERVER REQUIRED libhttpserver)

add_executable(
        TaPS_CameraNode
        src/main.cpp
        src/camera/cv_logger.cpp
        src/camera/cv_logger.h
        src/camera/camera_enumeration.cpp
        src/camera/camera_enumeration.h
        src/camera/fourcc.h
        src/camera/cv_cap.h
        src/camera/pps_handler.cpp
        src/camera/pps_handler.h
        src/camera/apriltag_detector.cpp
        src/camera/apriltag_detector.h
        src/runtime_args.h
        src/video_queue.h
        src/video_recorder.h
        src/taps_reader/taps_reader.cpp
        src/taps_reader/taps_reader.h
        src/playback/playback_manager.cpp
        src/playback/playback_manager.h
        src/webserver/http_server.h
        src/webserver/calibration.cpp
        src/webserver/calibration.h
        src/webserver/nt_client.cpp
        src/webserver/nt_client.h
)

target_link_libraries(TaPS_CameraNode PRIVATE
        ${OpenCV_LIBS}
        spdlog::spdlog
        taywee::args
        ${LIBHTTPSERVER_LINK_LIBRARIES}
        nlohmann_json::nlohmann_json
        ${NT_LIB}
        ${GPIOD_LIB}
        ${APRILTAG_LIB}
)

target_include_directories(TaPS_CameraNode PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${LIBHTTPSERVER_INCLUDE_DIRS}
)

target_link_directories(TaPS_CameraNode PRIVATE
        ${LIBHTTPSERVER_LIBRARY_DIRS}
)

# Install
install(TARGETS TaPS_CameraNode DESTINATION bin)
# Web UI templates (served relative to the process working directory —
# set systemd WorkingDirectory to ${CMAKE_INSTALL_PREFIX}/share/taps_camera_node)
install(DIRECTORY templates/ DESTINATION share/taps_camera_node/templates)

# -----------------------------------------------------------------------------
# Optional: synthetic V4L2 test feeder (needs v4l2loopback at runtime)
# -----------------------------------------------------------------------------
option(TAPS_BUILD_TESTFEED "Build the synthetic V4L2 test feeder tool" OFF)
if(TAPS_BUILD_TESTFEED)
    add_executable(TaPS_TestFeed tools/TaPS_TestFeed.cpp)
    target_link_libraries(TaPS_TestFeed PRIVATE ${OpenCV_LIBS})
    target_include_directories(TaPS_TestFeed PRIVATE ${OpenCV_INCLUDE_DIRS})
    install(TARGETS TaPS_TestFeed DESTINATION bin)
endif()
