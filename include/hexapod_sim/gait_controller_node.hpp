#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hexapod_sim/coordinator.hpp"
#include "hexapod_sim/executor.hpp"
#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/gait_state.hpp"
#include "hexapod_sim/master_path.hpp"
#include "hexapod_sim/path_builder.hpp"

namespace hexapod_sim
{

class GaitControllerNode : public rclcpp::Node
{
public:
  GaitControllerNode();

  bool run_startup_sequence();

private:
  void on_trajectory_type_msg(const std_msgs::msg::Int32::SharedPtr msg);

  GaitConfig config_;
  GaitState state_;
  std::unique_ptr<MasterPath> master_path_;
  std::unique_ptr<PathBuilder> path_builder_;
  std::unique_ptr<TrajectoryExecutor> executor_;
  std::unique_ptr<GaitCoordinator> coordinator_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trajectory_type_sub_;
};

}  // namespace hexapod_sim
