#pragma once

#include <chrono>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/gait_state.hpp"

namespace hexapod_sim
{

class TrajectoryExecutor
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

  TrajectoryExecutor(rclcpp::Node & node, const GaitConfig & config, GaitState & state);

  bool wait_for_action_server(const std::chrono::milliseconds & timeout);
  bool execute_current_paths();

private:
  bool send_joints_trajectory();

  rclcpp::Node & node_;
  const GaitConfig & config_;
  GaitState & state_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

}  // namespace hexapod_sim
