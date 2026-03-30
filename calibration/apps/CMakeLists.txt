add_executable(run_calibration run_calibration.cpp)
add_executable(camera_test camera_test.cpp)
add_executable(helios_test helios_test.cpp)
add_executable(dual_camera_test dual_camera_test.cpp)
add_executable(dual_capture_app dual_capture_app.cpp)

target_link_libraries(run_calibration PRIVATE calibration)
target_link_libraries(camera_test PRIVATE calibration)
target_link_libraries(helios_test PRIVATE calibration)
target_link_libraries(dual_camera_test PRIVATE calibration)
target_link_libraries(dual_capture_app PRIVATE calibration)

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
