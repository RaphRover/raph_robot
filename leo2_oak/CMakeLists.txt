cmake_minimum_required(VERSION 3.8)
project(leo2_oak)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(depthai REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_library(leo2_oak SHARED
  src/oak_wrapper.cpp
)
target_link_libraries(leo2_oak
  rclcpp::rclcpp
  rclcpp_components::component
  depthai::core
  depthai_bridge::depthai_bridge
  image_transport::image_transport
)
rclcpp_components_register_node(leo2_oak
  PLUGIN "leo2_oak::OakWrapper"
  EXECUTABLE "oak_wrapper"
)

install(
  TARGETS leo2_oak
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
