#include "hexapod_sim/gait_controller_node.hpp"

#include <algorithm>
#include <chrono>

namespace hexapod_sim
{

GaitControllerNode::GaitControllerNode()
: Node("gait_controller")
{
  // Reads runtime config and seeds immutable robot model/state containers.
  config_.action_name = this->declare_parameter<std::string>("action_name", config_.action_name);
  const int64_t configured_num_steps = this->declare_parameter<int64_t>("num_steps", static_cast<int64_t>(config_.num_steps));
  const int64_t configured_trajectory = this->declare_parameter<int64_t>(
    "trajectory_id",
    static_cast<int64_t>(trajectory_type_id(config_.trajectory_type)));
  config_.num_steps = static_cast<int>(std::max<int64_t>(0, configured_num_steps));
  config_.trajectory_type = trajectory_type_from_id(static_cast<int>(configured_trajectory));

  state_.legs = create_default_legs();
  initialize_path_tip_state(state_);

  master_path_ = std::make_unique<MasterPath>(config_);
  path_builder_ = std::make_unique<PathBuilder>(config_, state_, *master_path_, this->get_logger());
  executor_ = std::make_unique<TrajectoryExecutor>(*this, config_, state_);
  coordinator_ = std::make_unique<GaitCoordinator>(*this, config_, state_, *path_builder_, *executor_);

  RCLCPP_INFO(
    this->get_logger(),
    "[node] initialized legs=%zu | dt=%.3f | min_angle=%.1f deg | limit=%.3f | swing_h=%.3f | steps=%d | trajectory=%s (%d)",
    state_.legs.size(),
    config_.discrete_step,
    config_.min_angle_deg,
    config_.limit_radius,
    config_.swing_height,
    config_.num_steps,
    master_path_->name(config_.trajectory_type).c_str(),
    trajectory_type_id(config_.trajectory_type));
}

bool GaitControllerNode::run_startup_sequence()
{
  using namespace std::chrono_literals;
  // Waits for action server, then runs the complete gait sequence once.
  RCLCPP_INFO(this->get_logger(), "[node] waiting for action server: %s", config_.action_name.c_str());
  if (!executor_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "[node] action server unavailable after 5 seconds");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[node] action server connected");

  if (!coordinator_->run()) {
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[node] gait sequence complete at t=%.3f", state_.master_path_time);
  return true;
}

}  // namespace hexapod_sim
