#include "hexapod_sim/gait_controller_node.hpp"

#include <chrono>
#include <functional>

namespace hexapod_sim
{

GaitControllerNode::GaitControllerNode()
: Node("gait_controller")
{
  // Reads runtime config and seeds immutable robot model/state containers.
  config_.action_name = this->declare_parameter<std::string>("action_name", config_.action_name);
  config_.trajectory_cmd_topic = this->declare_parameter<std::string>("trajectory_cmd_topic", config_.trajectory_cmd_topic);
  const int64_t configured_trajectory = this->declare_parameter<int64_t>(
    "trajectory_id",
    static_cast<int64_t>(trajectory_type_id(config_.trajectory_type)));
  config_.trajectory_type = trajectory_type_from_id(static_cast<int>(configured_trajectory));

  state_.legs = create_default_legs();
  initialize_path_tip_state(state_);
  state_.current_trajectory_type = TrajectoryType::Stationary;
  state_.requested_trajectory_type = config_.trajectory_type;
  state_.stop_requested = false;

  master_path_ = std::make_unique<MasterPath>(config_);
  path_builder_ = std::make_unique<PathBuilder>(*this, config_, state_, *master_path_);
  executor_ = std::make_unique<TrajectoryExecutor>(*this, config_, state_);
  coordinator_ = std::make_unique<GaitCoordinator>(*this, config_, state_, *path_builder_, *executor_);

  trajectory_type_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    config_.trajectory_cmd_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&GaitControllerNode::on_trajectory_type_msg, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "[node] initialized | trajectory_cmd_topic=%s | initial_request=%s (%d)",
    config_.trajectory_cmd_topic.c_str(),
    master_path_->name(state_.requested_trajectory_type).c_str(),
    trajectory_type_id(state_.requested_trajectory_type));
}

void GaitControllerNode::on_trajectory_type_msg(const std_msgs::msg::Int32::SharedPtr msg)
{
  const auto requested = trajectory_type_from_id(msg->data);
  if (requested == state_.requested_trajectory_type) return;
  state_.requested_trajectory_type = requested;
  RCLCPP_INFO(
    this->get_logger(),
    "[node] trajectory request updated to %s (%d)",
    master_path_->name(state_.requested_trajectory_type).c_str(),
    trajectory_type_id(state_.requested_trajectory_type));
}

bool GaitControllerNode::run_startup_sequence()
{
  using namespace std::chrono_literals;
  // Waits for action server, then runs coordinator loop.
  RCLCPP_INFO(this->get_logger(), "[node] waiting for action server: %s", config_.action_name.c_str());
  if (!executor_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(this->get_logger(), "[node] action server unavailable after 5 seconds");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[node] action server connected");

  return coordinator_->run();
}

}  // namespace hexapod_sim
