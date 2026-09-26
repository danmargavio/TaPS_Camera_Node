cmake_minimum_required(VERSION 3.28)
project(TaPS_CameraNode)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)

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

# Optional: libgpiod (GPIO/PPS)
option(TAPS_ENABLE_PPS "Enable PPS/GPIO support" ON)
if(TAPS_ENABLE_PPS)
    message(STATUS "PPS/GPIO support enabled")
    find_package(gpiod QUIET)
    if(gpiod_FOUND)
        add_definitions(-DHAVE_GPIOD)
        set(GPIOD_LIB gpiod)
    else()
        message(WARNING "libgpiod not found. PPS support will be disabled at runtime.")
        set(GPIOD_LIB "")
    endif()
else()
    message(STATUS "PPS/GPIO support disabled")
    set(GPIOD_LIB "")
endif()

find_package(OpenCV 5.0 REQUIRED)
find_package(spdlog REQUIRED)
find_package(LibHttpServer REQUIRED)

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
        src/runtime_args.h
        src/video_queue.h
        src/video_recorder.h
        src/taps_reader/taps_reader.cpp
        src/taps_reader/taps_reader.h
        src/playback/playback_manager.cpp
        src/playback/playback_manager.h
        src/webserver/http_server.h
        src/webserver/nt_client.cpp
        src/webserver/nt_client.h
)

target_link_libraries(TaPS_CameraNode PRIVATE
        ${OpenCV_LIBS}
        spdlog::spdlog
        taywee::args
        httpserver
        nlohmann_json::nlohmann_json
        ${NT_LIB}
        ${GPIOD_LIB}
)

target_include_directories(TaPS_CameraNode PRIVATE
        ${libhttpserver_SOURCE_DIR}/src
        ${libhttpserver_BINARY_DIR}/src
)

# Install
install(TARGETS TaPS_CameraNode DESTINATION bin)
