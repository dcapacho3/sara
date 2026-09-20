cmake_minimum_required(VERSION 3.8)
project(sara_perception)

find_package(ament_cmake REQUIRED)

file(GLOB SCRIPT_FILES "scripts/*.py")
if(SCRIPT_FILES)
  install(PROGRAMS ${SCRIPT_FILES} DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
