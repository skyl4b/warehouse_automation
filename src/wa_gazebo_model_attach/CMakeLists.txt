# Builder for the model attacher plugin for Gazebo
#
# A simple plugin to add a service to Gazebo that allows models to be attached
# to each other. This is useful for creating simple movement in Gazebo,
# such as when a robot must move / attach to a box.
#
# Based on: https://github.com/Boeing/gazebo_model_attachment_plugin
# License: Apache License 2.0

cmake_minimum_required(VERSION 3.8)
project(wa_gazebo_model_attach)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(wa_interfaces)

#
# Install python module and scripts
#
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_lib SHARED src/${PROJECT_NAME}.cpp)

target_include_directories(
    ${PROJECT_NAME}_lib PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
                            rclcpp
                            gazebo_ros
                            wa_interfaces)

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT ModelAttachment
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include
)

# Add Tests
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

ament_package()
