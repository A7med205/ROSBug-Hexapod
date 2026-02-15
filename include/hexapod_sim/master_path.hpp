#pragma once

#include <string>

#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/gait_state.hpp"

namespace hexapod_sim
{

class MasterPath
{
public:
  explicit MasterPath(const GaitConfig & config);

  // Returns base pose in world frame at time t for a selected trajectory family.
  BasePose2D pose(double t, TrajectoryType trajectory_type) const;
  // Human-readable label for logging and status output.
  std::string name(TrajectoryType trajectory_type) const;

private:
  const GaitConfig & config_;
};

}  // namespace hexapod_sim
