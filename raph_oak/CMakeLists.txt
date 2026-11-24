cmake_minimum_required(VERSION 3.5)
project(raph_oak)

find_package(ament_cmake REQUIRED)
find_package(depthai_v3 REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(
  oak_wrapper_parameters
  src/oak_wrapper_parameters.yaml
)

add_library(raph_oak SHARED
  src/camera_info.cpp
  src/conversions.cpp
  src/demand.cpp
  src/oak_wrapper.cpp
  src/parameters.cpp
  src/pipeline.cpp
)
target_link_libraries(raph_oak
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  oak_wrapper_parameters
  opencv_core
  opencv_imgproc
  depthai_v3::core
)
target_include_directories(raph_oak
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
rclcpp_components_register_node(raph_oak
  PLUGIN "raph_oak::OakWrapper"
  EXECUTABLE "oak_wrapper"
)

install(
  TARGETS raph_oak
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  option(ENABLE_CLANG_TIDY "Enable ament_clang_tidy test" OFF)
  if(ENABLE_CLANG_TIDY)
    include(ProcessorCount)
    processorcount(N)
    set(ament_cmake_clang_tidy_JOBS ${N})
    # Only the headers of this package. clang-tidy's regex has no lookahead, and the generated
    # parameter header lives in the build directory, so it is left out this way
    set(ament_cmake_clang_tidy_HEADER_FILTER "${CMAKE_CURRENT_SOURCE_DIR}/include/.*")
  else()
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_clang_tidy
    )
  endif()

  ament_lint_auto_find_test_dependencies()

  # The logic that decides what the camera has to do does not depend on DepthAI or ROS
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_demand test/test_demand.cpp src/demand.cpp)
  target_include_directories(test_demand PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  # Conversions of camera info, IMU reports and point clouds, with DepthAI but without a camera
  ament_add_gtest(test_conversions
    test/test_camera_info.cpp
    test/test_conversions.cpp
    src/camera_info.cpp
    src/conversions.cpp
  )
  target_include_directories(test_conversions PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_conversions
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    opencv_core
    opencv_imgproc
    depthai_v3::core
  )
endif()

ament_package()
