cmake_minimum_required(VERSION 3.5)
project(leo2_fw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(leo_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(leo2_fw SHARED
  src/firmware_message_converter.cpp
)
target_link_libraries(leo2_fw
  rclcpp::rclcpp
  ${YAML_CPP_LIBRARIES}
)
ament_target_dependencies(leo2_fw
  sensor_msgs
  leo_msgs
  nav_msgs
  rclcpp_components
)
rclcpp_components_register_node(leo2_fw
  PLUGIN "leo2_fw::FirmwareMessageConverter"
  EXECUTABLE "firmware_message_converter"
)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS leo2_fw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  PROGRAMS
    scripts/flash
    scripts/update
    scripts/test_hw
    scripts/calibrate_imu
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
