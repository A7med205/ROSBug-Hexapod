#pragma once

#include <array>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "hexapod_sim/gait_config.hpp"

namespace hexapod_sim
{

struct Point3D { double x; double y; double z; };
struct BasePose2D { double x; double y; double theta; };
struct LocalDisplacement2D { double dx_local; double dy_local; };
struct WorldNeutralPose { double x; double y; double theta; };
struct LocalPose2D { double x; double y; };

struct LegFramePose { double x; double y; double theta; };

struct Leg
{
  int leg_id;
  std::array<std::string, 3> joint_names;
  LegFramePose frame_pose;
  Point3D current_tip_position;
  std::array<double, 3> current_joint_angles;
};

struct GaitState
{
  double master_path_time{0.0};
  std::array<std::vector<Point3D>, 6> path_3d;
  std::array<Point3D, 6> path_tip_state;
  std::vector<Leg> legs;
  trajectory_msgs::msg::JointTrajectory joints_trajectory;
  TrajectoryType current_trajectory_type{TrajectoryType::Stationary};
  TrajectoryType requested_trajectory_type{TrajectoryType::Stationary};
  bool stop_requested{false};
};

std::vector<Leg> create_default_legs();
void initialize_path_tip_state(GaitState & state);

}  // namespace hexapod_sim
