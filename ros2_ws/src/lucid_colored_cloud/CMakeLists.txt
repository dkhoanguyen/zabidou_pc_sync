cmake_minimum_required(VERSION 3.16)
project(lucid_colored_cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

set(CALIBRATION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../calibration")
set(CALIBRATION_BUILD_DIR "${CALIBRATION_ROOT}/build")
set(CALIBRATION_LIB_PATH "${CALIBRATION_BUILD_DIR}/libcalibration.so")
set(ORB_SLAM3_ROOT "/home/khoa/Projects/ORB_SLAM3" CACHE PATH "Path to ORB-SLAM3 checkout")
set(ORB_SLAM3_LIB_PATH "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so")
set(DBOW2_LIB_PATH "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
set(G2O_LIB_PATH "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so")

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

if(EXISTS "${ORB_SLAM3_LIB_PATH}")
  add_library(ORB_SLAM3 SHARED IMPORTED)
  set_target_properties(ORB_SLAM3 PROPERTIES
    IMPORTED_LOCATION "${ORB_SLAM3_LIB_PATH}"
    INTERFACE_INCLUDE_DIRECTORIES "${ORB_SLAM3_ROOT}/include;${ORB_SLAM3_ROOT}/include/CameraModels;${ORB_SLAM3_ROOT}/Thirdparty/Sophus;${EIGEN3_INCLUDE_DIRS};${Pangolin_INCLUDE_DIRS}"
    INTERFACE_LINK_OPTIONS "-Wl,--allow-shlib-undefined"
  )
else()
  message(WARNING
    "ORB-SLAM3 shared library not found at ${ORB_SLAM3_LIB_PATH}; "
    "building ORB-SLAM3 sources inside this ROS package with C++14."
  )
  add_library(ORB_SLAM3 SHARED
    "${ORB_SLAM3_ROOT}/src/System.cc"
    "${ORB_SLAM3_ROOT}/src/Tracking.cc"
    "${ORB_SLAM3_ROOT}/src/LocalMapping.cc"
    "${ORB_SLAM3_ROOT}/src/LoopClosing.cc"
    "${ORB_SLAM3_ROOT}/src/ORBextractor.cc"
    "${ORB_SLAM3_ROOT}/src/ORBmatcher.cc"
    "${ORB_SLAM3_ROOT}/src/FrameDrawer.cc"
    "${ORB_SLAM3_ROOT}/src/Converter.cc"
    "${ORB_SLAM3_ROOT}/src/MapPoint.cc"
    "${ORB_SLAM3_ROOT}/src/KeyFrame.cc"
    "${ORB_SLAM3_ROOT}/src/Atlas.cc"
    "${ORB_SLAM3_ROOT}/src/Map.cc"
    "${ORB_SLAM3_ROOT}/src/MapDrawer.cc"
    "${ORB_SLAM3_ROOT}/src/Optimizer.cc"
    "${ORB_SLAM3_ROOT}/src/Frame.cc"
    "${ORB_SLAM3_ROOT}/src/KeyFrameDatabase.cc"
    "${ORB_SLAM3_ROOT}/src/Sim3Solver.cc"
    "${ORB_SLAM3_ROOT}/src/Viewer.cc"
    "${ORB_SLAM3_ROOT}/src/ImuTypes.cc"
    "${ORB_SLAM3_ROOT}/src/G2oTypes.cc"
    "${ORB_SLAM3_ROOT}/src/CameraModels/Pinhole.cpp"
    "${ORB_SLAM3_ROOT}/src/CameraModels/KannalaBrandt8.cpp"
    "${ORB_SLAM3_ROOT}/src/OptimizableTypes.cpp"
    "${ORB_SLAM3_ROOT}/src/MLPnPsolver.cpp"
    "${ORB_SLAM3_ROOT}/src/GeometricTools.cc"
    "${ORB_SLAM3_ROOT}/src/TwoViewReconstruction.cc"
    "${ORB_SLAM3_ROOT}/src/Config.cc"
    "${ORB_SLAM3_ROOT}/src/Settings.cc"
  )
  target_compile_definitions(ORB_SLAM3 PUBLIC COMPILEDWITHC11)
  set_target_properties(ORB_SLAM3 PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )
  target_include_directories(ORB_SLAM3 PUBLIC
    "${ORB_SLAM3_ROOT}"
    "${ORB_SLAM3_ROOT}/include"
    "${ORB_SLAM3_ROOT}/include/CameraModels"
    "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
  )
  target_link_libraries(ORB_SLAM3
    ${DBOW2_LIB_PATH}
    ${G2O_LIB_PATH}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    Boost::serialization
    OpenSSL::Crypto
  )
endif()

add_executable(colored_cloud_publisher src/colored_cloud_publisher.cpp)
target_include_directories(colored_cloud_publisher PRIVATE "${CALIBRATION_ROOT}/include")
target_link_libraries(colored_cloud_publisher calibration ${OpenCV_LIBS})
target_link_options(colored_cloud_publisher PRIVATE -Wl,--allow-shlib-undefined)
set_target_properties(colored_cloud_publisher PROPERTIES
  BUILD_RPATH "${CALIBRATION_BUILD_DIR}"
  INSTALL_RPATH "${CALIBRATION_BUILD_DIR}"
)
ament_target_dependencies(colored_cloud_publisher rclcpp sensor_msgs std_msgs)

add_executable(phoenix_image_publisher src/phoenix_image_publisher.cpp)
target_include_directories(phoenix_image_publisher PRIVATE "${CALIBRATION_ROOT}/include")
target_link_libraries(phoenix_image_publisher calibration ${OpenCV_LIBS})
target_link_options(phoenix_image_publisher PRIVATE -Wl,--allow-shlib-undefined)
set_target_properties(phoenix_image_publisher PROPERTIES
  BUILD_RPATH "${CALIBRATION_BUILD_DIR}"
  INSTALL_RPATH "${CALIBRATION_BUILD_DIR}"
)
ament_target_dependencies(phoenix_image_publisher rclcpp sensor_msgs std_msgs)

add_executable(phoenix_orbslam3_node src/phoenix_orbslam3_node.cpp)
target_include_directories(phoenix_orbslam3_node PRIVATE
  "${ORB_SLAM3_ROOT}"
  "${ORB_SLAM3_ROOT}/include"
  "${ORB_SLAM3_ROOT}/include/CameraModels"
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(phoenix_orbslam3_node
  ORB_SLAM3
  ${DBOW2_LIB_PATH}
  ${G2O_LIB_PATH}
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  OpenSSL::Crypto
)
set_target_properties(phoenix_orbslam3_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib"
)
ament_target_dependencies(phoenix_orbslam3_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS colored_cloud_publisher phoenix_image_publisher phoenix_orbslam3_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
