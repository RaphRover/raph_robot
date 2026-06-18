cmake_minimum_required(VERSION 3.5)
project(raph_fw)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/flash.py
    scripts/flash
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

  option(ENABLE_CLANG_TIDY "Enable ament_clang_tidy test" OFF)
  if(ENABLE_CLANG_TIDY)
    include(ProcessorCount)
    processorcount(N)
    set(ament_cmake_clang_tidy_JOBS ${N})
  else()
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_clang_tidy
    )
  endif()

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
