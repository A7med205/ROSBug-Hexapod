#include "hexapod_sim/executor.hpp"

#include <array>
#include <cmath>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/kinematics.hpp"

namespace hexapod_sim
{

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node & node, const GaitConfig & config, GaitState & state)
: node_(node), config_(config), state_(state)
{
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(&node_, config_.action_name);
}

// Waits for the joint trajectory action server.
bool TrajectoryExecutor::wait_for_action_server(const std::chrono::milliseconds & timeout)
{
  return action_client_->wait_for_action_server(timeout);
}

// Converts local tip paths to filtered joint goals and sends a single trajectory.
bool TrajectoryExecutor::execute_current_paths()
{
  if (state_.path_3d[0].empty()) {
    RCLCPP_ERROR(node_.get_logger(), "[executor] cannot execute empty path set");
    return false;
  }

  const std::size_t points_per_leg = state_.path_3d[0].size();
  for (std::size_t i = 1; i < state_.path_3d.size(); ++i) {
    if (state_.path_3d[i].size() != points_per_leg) {
      RCLCPP_ERROR(node_.get_logger(), "[executor] path size mismatch across legs");
      return false;
    }
  }

  const double min_angle_rad = config_.min_angle_deg * (kPi / 180.0);
  std::array<std::array<double, 3>, 6> candidate_joint_angles{};
  for (std::size_t leg_idx = 0; leg_idx < state_.legs.size(); ++leg_idx) {
    candidate_joint_angles[leg_idx] = state_.legs[leg_idx].current_joint_angles;
  }

  state_.joints_trajectory.joint_names.clear();
  state_.joints_trajectory.points.clear();
  state_.joints_trajectory.joint_names.reserve(state_.legs.size() * 3);
  state_.joints_trajectory.points.reserve(points_per_leg * state_.legs.size());
  for (const auto & leg : state_.legs) {
    state_.joints_trajectory.joint_names.insert(
      state_.joints_trajectory.joint_names.end(),
      leg.joint_names.begin(),
      leg.joint_names.end());
  }

  for (std::size_t point_idx = 0; point_idx < points_per_leg; ++point_idx) {
    for (std::size_t leg_idx = 0; leg_idx < state_.legs.size(); ++leg_idx) {
      const auto & target_tip = state_.path_3d[leg_idx][point_idx];
      double j1 = 0.0, j2 = 0.0, j3 = 0.0;
      Kinematics::compute_ik(target_tip, j1, j2, j3, node_.get_logger(), *node_.get_clock());
      const std::array<double, 3> target_angles = {j1, j2, j3};

      bool any_joint_changed = false;
      for (std::size_t joint_idx = 0; joint_idx < 3; ++joint_idx) {
        const double current = candidate_joint_angles[leg_idx][joint_idx];
        const double delta = std::abs(target_angles[joint_idx] - current);
        if (!std::isfinite(current) || delta > min_angle_rad) {
          candidate_joint_angles[leg_idx][joint_idx] = target_angles[joint_idx];
          any_joint_changed = true;
        }
      }

      if (any_joint_changed) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.reserve(state_.legs.size() * 3);
        for (const auto & leg_joints : candidate_joint_angles) {
          point.positions.push_back(leg_joints[0]);
          point.positions.push_back(leg_joints[1]);
          point.positions.push_back(leg_joints[2]);
        }

        const double time_from_start_sec = static_cast<double>(state_.joints_trajectory.points.size() + 1) * config_.discrete_step;
        const int32_t sec = static_cast<int32_t>(time_from_start_sec);
        const double frac_sec = time_from_start_sec - static_cast<double>(sec);
        point.time_from_start.sec = sec;
        point.time_from_start.nanosec = static_cast<uint32_t>(frac_sec * 1e9);
        state_.joints_trajectory.points.push_back(point);
      }
    }
  }

  if (!state_.joints_trajectory.points.empty()) {
    RCLCPP_INFO(node_.get_logger(), "[executor] sending trajectory points=%zu", state_.joints_trajectory.points.size());
    if (!send_joints_trajectory()) {
      return false;
    }
  }

  for (std::size_t leg_idx = 0; leg_idx < state_.legs.size(); ++leg_idx) {
    state_.legs[leg_idx].current_joint_angles = candidate_joint_angles[leg_idx];
    state_.legs[leg_idx].current_tip_position = state_.path_3d[leg_idx].back();
  }

  return true;
}

// Sends the prebuilt joints trajectory and waits for completion.
bool TrajectoryExecutor::send_joints_trajectory()
{
  if (state_.joints_trajectory.points.empty()) {
    return true;
  }

  FollowJointTrajectory::Goal goal_msg;
  goal_msg.trajectory = state_.joints_trajectory;

  auto goal_handle_future = action_client_->async_send_goal(goal_msg);
  const auto send_status = rclcpp::spin_until_future_complete(node_.get_node_base_interface(), goal_handle_future);
  if (send_status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_.get_logger(), "[executor] failed to send goal to action server");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_.get_logger(), "[executor] action server rejected trajectory goal");
    return false;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  const auto result_status = rclcpp::spin_until_future_complete(node_.get_node_base_interface(), result_future);
  if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_.get_logger(), "[executor] failed while waiting for action result");
    return false;
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(
      node_.get_logger(),
      "[executor] trajectory action failed with result code %d",
      static_cast<int>(wrapped_result.code));
    return false;
  }
  return true;
}

}  // namespace hexapod_sim
