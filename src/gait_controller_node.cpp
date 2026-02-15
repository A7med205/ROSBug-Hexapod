#include "hexapod_sim/gait_controller_node.hpp"

#include <algorithm>
#include <chrono>

namespace hexapod_sim
{

GaitControllerNode::GaitControllerNode()
: Node("gait_controller")
{
  config_.action_name = this->declare_parameter<std::string>("action_name", config_.action_name);
  const int64_t configured_num_steps = this->declare_parameter<int64_t>("num_steps", static_cast<int64_t>(config_.num_steps));
  const int64_t configured_trajectory = this->declare_parameter<int64_t>("trajectory_id", static_cast<int64_t>(config_.trajectory_id));
  config_.num_steps = static_cast<int>(std::max<int64_t>(0, configured_num_steps));
  config_.trajectory_id = static_cast<int>(configured_trajectory);

  state_.legs = create_default_legs();
  initialize_path_tip_state(state_);

  master_path_ = std::make_unique<MasterPath>(config_);
  path_builder_ = std::make_unique<PathBuilder>(config_, state_, *master_path_, this->get_logger());
  executor_ = std::make_unique<TrajectoryExecutor>(*this, config_, state_);
  coordinator_ = std::make_unique<GaitCoordinator>(*this, config_, state_, *master_path_, *path_builder_, *executor_);

  RCLCPP_INFO(
    this->get_logger(),
    "Initialized %zu legs | discrete_step=%.3f, min_angle=%.1f deg, limit_radius=%.3f, swing_height=%.3f, num_steps=%d, trajectory=%s (%d)",
    state_.legs.size(),
    config_.discrete_step,
    config_.min_angle_deg,
    config_.limit_radius,
    config_.swing_height,
    config_.num_steps,
    master_path_->name(config_.trajectory_id).c_str(),
    config_.trajectory_id);
}

bool GaitControllerNode::run_startup_sequence()
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_logger(), "Waiting for action server: %s", config_.action_name.c_str());
  if (!executor_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server unavailable after 5 seconds");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Action server connected");

  if (!coordinator_->run()) {
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Gait sequence complete at master time t=%.3f", state_.master_path_time);
  return true;
}

}  // namespace hexapod_sim
