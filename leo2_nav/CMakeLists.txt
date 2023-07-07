cmake_minimum_required(VERSION 3.5)
project(leo2_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/cmd_vel_to_ackermann_drive
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
