#pragma once

#include <array>
#include <cstddef>

#include <rclcpp/rclcpp.hpp>

#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/gait_state.hpp"
#include "hexapod_sim/master_path.hpp"

namespace hexapod_sim
{

class PathBuilder
{
public:
  PathBuilder(
    const GaitConfig & config,
    GaitState & state,
    const MasterPath & master_path,
    const rclcpp::Logger & logger);

  bool build_pulls(
    bool tripod_a,
    double start_time,
    int trajectory_id,
    int limit_hits_required,
    double & end_time,
    std::size_t & point_count);

  bool build_swings(
    bool tripod_a,
    double start_time,
    double pull_duration,
    std::size_t point_count,
    bool end_to_neutral,
    int trajectory_id);

private:
  const std::array<std::size_t, 3> & tripod_indices(bool tripod_a) const;
  LocalDisplacement2D base_to_tip(const BasePose2D & base1, const BasePose2D & base2, int leg_id);
  WorldNeutralPose wbase_to_wneutral(double t, int leg_id, int trajectory_id);
  std::array<LocalPose2D, 3> future_tip_poses(bool tripod_a, double start_time, int trajectory_id);

  const GaitConfig & config_;
  GaitState & state_;
  const MasterPath & master_path_;
  rclcpp::Logger logger_;
};

}  // namespace hexapod_sim
