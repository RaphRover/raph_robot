cmake_minimum_required(VERSION 3.5)
project(raph_oak)

find_package(ament_cmake REQUIRED)
find_package(depthai REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(
  oak_wrapper_parameters
  src/oak_wrapper_parameters.yaml
)

add_library(raph_oak SHARED
  src/camera_info.cpp
  src/oak_wrapper.cpp
  src/parameters.cpp
  src/pipeline.cpp
)
target_link_libraries(raph_oak
  depthai::core
  depthai_bridge::depthai_bridge
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  oak_wrapper_parameters
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

  include(ProcessorCount)
  processorcount(N)
  set(ament_cmake_clang_tidy_JOBS ${N})
  set(ament_cmake_clang_tidy_HEADER_FILTER "^(?!oak_wrapper_parameters\.hpp).*$")

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
