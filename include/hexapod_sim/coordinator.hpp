#pragma once

#include <rclcpp/rclcpp.hpp>

#include "hexapod_sim/executor.hpp"
#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/gait_state.hpp"
#include "hexapod_sim/master_path.hpp"
#include "hexapod_sim/path_builder.hpp"

namespace hexapod_sim
{

class GaitCoordinator
{
public:
  GaitCoordinator(
    rclcpp::Node & node,
    const GaitConfig & config,
    GaitState & state,
    const MasterPath & master_path,
    PathBuilder & path_builder,
    TrajectoryExecutor & executor);

  bool run();

private:
  bool initialize_neutral_joint_values();

  rclcpp::Node & node_;
  const GaitConfig & config_;
  GaitState & state_;
  const MasterPath & master_path_;
  PathBuilder & path_builder_;
  TrajectoryExecutor & executor_;
};

}  // namespace hexapod_sim
