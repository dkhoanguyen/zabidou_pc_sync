cmake_minimum_required(VERSION 3.20)
project(lucid_calibration VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Build options
# ---------------------------------------------------------------------------
option(BUILD_APPS            "Build runnable applications"   ON)
option(BUILD_TESTS           "Build unit/integration tests"  ON)
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python module"  OFF)
option(BUILD_ROS2_NODE       "Build ROS2 node wrapper"       OFF)
option(ENABLE_ARENA_SDK      "Enable Lucid Arena SDK integration" ON)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(cmake/dependencies.cmake)

# ---------------------------------------------------------------------------
# Core calibration library
# ---------------------------------------------------------------------------
add_library(calibration SHARED
    src/calibration/opencv_calibration.cpp
    src/calibration/rgbd_calibration.cpp
    src/io/common/lucid_device_utils.cpp
    src/io/live_frame_source.cpp
    src/io/dataset_frame_source.cpp
    src/io/hardware/helios2.cpp
    src/io/hardware/phoenix.cpp
)

target_include_directories(calibration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(calibration
    PUBLIC
        opencv_core
        opencv_calib3d
        opencv_imgproc
        opencv_highgui
        opencv_imgcodecs
        Eigen3::Eigen
)

if(ArenaSDK_FOUND)
    target_include_directories(calibration
        PRIVATE
            ${ArenaSDK_INCLUDE_DIR}
            ${ArenaSDK_GENICAM_INCLUDE_DIR}
    )
    target_link_libraries(calibration PRIVATE ArenaSDK::ArenaSDK)
    target_compile_definitions(calibration PUBLIC CALIBRATION_HAS_ARENA_SDK=1)
    # libarena.so has a DT_NEEDED on Metavision libs whose transitive deps
    # (libxerces → ICU 70, libpq → OpenLDAP 2.5) may not be present on the
    # build host. Those are runtime-only concerns handled by the dynamic linker;
    # suppress the link-time ABI-chain check for downstream executables.
    target_link_options(calibration PUBLIC -Wl,--allow-shlib-undefined)
else()
    target_compile_definitions(calibration PUBLIC CALIBRATION_HAS_ARENA_SDK=0)
endif()

# ---------------------------------------------------------------------------
# Sub-directories
# ---------------------------------------------------------------------------
if(BUILD_APPS)
    add_subdirectory(apps)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()

if(BUILD_ROS2_NODE)
    add_subdirectory(ros2)
endif()
