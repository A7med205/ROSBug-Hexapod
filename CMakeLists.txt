cmake_minimum_required(VERSION 3.8)
project(hexapod_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(gait_controller src/gait_controller.cpp)

target_link_libraries(gait_controller PUBLIC
  ${control_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS
    gait_controller
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
