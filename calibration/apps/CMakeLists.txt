add_executable(run_calibration run_calibration.cpp)
add_executable(camera_test camera_test.cpp)
add_executable(helios_test helios_test.cpp)
add_executable(dual_camera_test dual_camera_test.cpp)
add_executable(dual_capture_app dual_capture_app.cpp)
add_executable(mono_calibration_app mono_calibration_app.cpp)
add_executable(stereo_calibration_app stereo_calibration_app.cpp)
add_executable(helios_intrinsics_dump helios_intrinsics_dump.cpp)
add_executable(colorize_point_cloud_app colorize_point_cloud_app.cpp)

target_link_libraries(run_calibration PRIVATE calibration)
target_link_libraries(camera_test PRIVATE calibration)
target_link_libraries(helios_test PRIVATE calibration)
target_link_libraries(dual_camera_test PRIVATE calibration)
target_link_libraries(dual_capture_app PRIVATE calibration)
target_link_libraries(mono_calibration_app PRIVATE calibration)
target_link_libraries(stereo_calibration_app PRIVATE calibration)
target_link_libraries(helios_intrinsics_dump PRIVATE calibration)
target_link_libraries(colorize_point_cloud_app PRIVATE calibration)

set_target_properties(run_calibration PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(camera_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(helios_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(dual_camera_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(dual_capture_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(mono_calibration_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(stereo_calibration_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(helios_intrinsics_dump PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)

set_target_properties(colorize_point_cloud_app PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps
)
