# dependencies.cmake — wraps all find_package calls for the calibration project

# ---------------------------------------------------------------------------
# OpenCV (required)
# ---------------------------------------------------------------------------
find_package(OpenCV 4.0 REQUIRED
    COMPONENTS core calib3d imgproc highgui imgcodecs
)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# ---------------------------------------------------------------------------
# Eigen3 (required)
# ---------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 version: ${EIGEN3_VERSION_STRING}")

# ---------------------------------------------------------------------------
# Arena SDK (optional for dataset-only builds, required for live Lucid hardware)
# ---------------------------------------------------------------------------
if(ENABLE_ARENA_SDK)
    find_package(ArenaSDK QUIET)
    if(ArenaSDK_FOUND)
        message(STATUS "ArenaSDK include: ${ArenaSDK_INCLUDE_DIR}")
        message(STATUS "ArenaSDK library: ${ArenaSDK_LIBRARY}")
        if(ArenaSDK_HAS_METAVISION)
            message(STATUS "ArenaSDK: Metavision event-camera support enabled")
        endif()
    else()
        message(WARNING
            "ArenaSDK was not found. Live Lucid camera support will be unavailable; "
            "dataset-only builds can continue. Set ARENA_ROOT to your SDK install path "
            "or configure with -DENABLE_ARENA_SDK=OFF to silence this warning.")
    endif()
else()
    message(STATUS "ArenaSDK support disabled by user (-DENABLE_ARENA_SDK=OFF)")
endif()

# ---------------------------------------------------------------------------
# Optional: Ceres Solver (needed by RgbdCalibration in the future)
# ---------------------------------------------------------------------------
find_package(Ceres QUIET)
if(Ceres_FOUND)
    message(STATUS "Ceres Solver found: ${CERES_VERSION} — RgbdCalibration will be linkable")
else()
    message(STATUS "Ceres Solver NOT found — RgbdCalibration will remain a stub")
endif()

# ---------------------------------------------------------------------------
# Optional: pybind11 (only needed when BUILD_PYTHON_BINDINGS=ON)
# ---------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
endif()
