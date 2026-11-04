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
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_firmware_status test/test_status.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
