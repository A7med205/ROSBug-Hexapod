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
  // Builds stance/swing tip paths in local leg frames from base motion.
  PathBuilder(
    rclcpp::Node & node,
    const GaitConfig & config,
    GaitState & state,
    const MasterPath & master_path);

  // Builds pull trajectories for one tripod until the required hit count is met.
  bool build_pulls(
    Tripod pull_tripod,
    double start_time,
    TrajectoryType trajectory_type,
    PullPhaseSpan pull_phase_span,
    double & end_time,
    std::size_t & point_count);

  // Builds swing trajectories for one tripod with optional neutral landing.
  bool build_swings(
    Tripod swing_tripod,
    double start_time,
    double pull_duration,
    std::size_t point_count,
    bool end_to_neutral,
    TrajectoryType trajectory_type);

private:
  void log_sampled_paths(double phase_start_time);
  // Converts base motion into local tip displacement for a stance-locked foot.
  LocalDisplacement2D tip_delta(const BasePose2D & base1, const BasePose2D & base2, int leg_id);
  // Returns world-frame neutral tip pose (x,y,theta) for a leg at time t.
  WorldNeutralPose neutral_tip_delta(double t, int leg_id, TrajectoryType trajectory_type);
  // Predicts swing landing offsets in local frame for the selected tripod.
  std::array<LocalPose2D, 3> tip_landing_pose(Tripod tripod, double start_time, TrajectoryType trajectory_type);

  const GaitConfig & config_;
  GaitState & state_;
  const MasterPath & master_path_;
  rclcpp::Node & node_;
};

}  // namespace hexapod_sim
