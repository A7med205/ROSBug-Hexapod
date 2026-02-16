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
find_package(trajectory_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

add_library(hexapod_gait_core
  src/gait_state.cpp
  src/master_path.cpp
  src/kinematics.cpp
  src/path_builder.cpp
  src/executor.cpp
  src/coordinator.cpp
)
target_include_directories(hexapod_gait_core PUBLIC include)

ament_target_dependencies(hexapod_gait_core
  control_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs
)

add_executable(gait_controller
  src/gait_controller_node.cpp
  src/main.cpp
)
target_include_directories(gait_controller PUBLIC include)
target_link_libraries(gait_controller hexapod_gait_core)
ament_target_dependencies(gait_controller
  rclcpp
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

## Install
install(
  TARGETS
    hexapod_gait_core
    gait_controller
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/trajectory_type_keyboard.py
    scripts/joint_goal_sequence_sender.py
    scripts/lite_gait_controller.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
