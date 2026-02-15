#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

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
  GaitConfig config_;
  GaitState state_;
  std::unique_ptr<MasterPath> master_path_;
  std::unique_ptr<PathBuilder> path_builder_;
  std::unique_ptr<TrajectoryExecutor> executor_;
  std::unique_ptr<GaitCoordinator> coordinator_;
};

}  // namespace hexapod_sim
