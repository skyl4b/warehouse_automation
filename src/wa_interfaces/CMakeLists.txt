cmake_minimum_required(VERSION 3.8)
project(wa_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress warnings about policy CMP0148
# "Policy CMP0148 is not set: The FindPythonInterp and FindPythonLibs modules are removed." 
# Needs to be fixed internally by ROS2 developers
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

file(GLOB msg_files RELATIVE "${CMAKE_SOURCE_DIR}" "msg/*.msg")
file(GLOB srv_files RELATIVE "${CMAKE_SOURCE_DIR}" "srv/*.srv")
file(GLOB action_files RELATIVE "${CMAKE_SOURCE_DIR}" "action/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}

  DEPENDENCIES
  geometry_msgs
)
ament_package()
