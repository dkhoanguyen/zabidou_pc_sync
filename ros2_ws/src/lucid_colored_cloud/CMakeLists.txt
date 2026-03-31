cmake_minimum_required(VERSION 3.16)
project(lucid_colored_cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(CALIBRATION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../calibration")
set(CALIBRATION_BUILD_DIR "${CALIBRATION_ROOT}/build")
set(CALIBRATION_LIB_PATH "${CALIBRATION_BUILD_DIR}/libcalibration.so")

if(NOT EXISTS "${CALIBRATION_LIB_PATH}")
  message(FATAL_ERROR
    "Expected prebuilt calibration library at ${CALIBRATION_LIB_PATH}. "
    "Build the calibration project first."
  )
endif()

add_library(calibration SHARED IMPORTED)
set_target_properties(calibration PROPERTIES
  IMPORTED_LOCATION "${CALIBRATION_LIB_PATH}"
  INTERFACE_INCLUDE_DIRECTORIES "${CALIBRATION_ROOT}/include"
  INTERFACE_LINK_OPTIONS "-Wl,--allow-shlib-undefined"
)

add_executable(colored_cloud_publisher src/colored_cloud_publisher.cpp)
target_include_directories(colored_cloud_publisher PRIVATE "${CALIBRATION_ROOT}/include")
target_link_libraries(colored_cloud_publisher calibration ${OpenCV_LIBS})
target_link_options(colored_cloud_publisher PRIVATE -Wl,--allow-shlib-undefined)
set_target_properties(colored_cloud_publisher PROPERTIES
  BUILD_RPATH "${CALIBRATION_BUILD_DIR}"
  INSTALL_RPATH "${CALIBRATION_BUILD_DIR}"
)
ament_target_dependencies(colored_cloud_publisher rclcpp sensor_msgs std_msgs)

install(TARGETS colored_cloud_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
