cmake_minimum_required(VERSION 3.8)
project(hexapod_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  rviz
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
    scripts/sim_interface.py
    scripts/simulation_batch_executor.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# The direct-keyboard simulation adapter and the hardware adapter import the
# same transport-independent Python gait implementation.  Installing these
# directories beside the script keeps that import valid in a colcon install.
install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../gait_core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../posture_core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../robot_core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../common
  DESTINATION
    lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
