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

  BasePose2D pose(double t, int trajectory_id) const;
  std::string name(int trajectory_id) const;

private:
  const GaitConfig & config_;
};

}  // namespace hexapod_sim
