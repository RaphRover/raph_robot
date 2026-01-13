cmake_minimum_required(VERSION 3.5)
project(raph_oak)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(depthai REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  oak_wrapper_parameters
  src/oak_wrapper_parameters.yaml
)

add_library(raph_oak SHARED
  src/oak_wrapper.cpp
)
target_link_libraries(raph_oak
  rclcpp::rclcpp
  rclcpp_components::component
  depthai::core
  depthai_bridge::depthai_bridge
  oak_wrapper_parameters
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()