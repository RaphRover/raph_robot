cmake_minimum_required(VERSION 3.5)
project(raph_fw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/flash.py
    scripts/flash
    scripts/hardware_test.py
    scripts/hardware_test
    scripts/motor_check.py
    scripts/motor_check
    scripts/parameter_bridge.py
    scripts/parameter_bridge
    scripts/update.py
    scripts/update
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
