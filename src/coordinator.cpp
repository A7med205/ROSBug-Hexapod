#include "hexapod_sim/coordinator.hpp"

#include <string>
#include <vector>

#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/kinematics.hpp"

namespace hexapod_sim
{

GaitCoordinator::GaitCoordinator(
  rclcpp::Node & node,
  const GaitConfig & config,
  GaitState & state,
  const MasterPath & master_path,
  PathBuilder & path_builder,
  TrajectoryExecutor & executor)
: node_(node),
  config_(config),
  state_(state),
  master_path_(master_path),
  path_builder_(path_builder),
  executor_(executor)
{}

bool GaitCoordinator::initialize_neutral_joint_values()
{
  for (auto & leg : state_.legs) {
    leg.current_tip_position = {kXHome, kYHome, kZHome};
    double j1 = 0.0, j2 = 0.0, j3 = 0.0;
    Kinematics::compute_ik(leg.current_tip_position, j1, j2, j3, node_.get_logger(), *node_.get_clock());
    leg.current_joint_angles = {j1, j2, j3};
  }
  return true;
}

bool GaitCoordinator::run()
{
  if (!initialize_neutral_joint_values()) return false;

  state_.master_path_time = 0.0;
  std::vector<double> phase_end_times;
  phase_end_times.reserve(static_cast<std::size_t>(config_.num_steps) + 2);
  bool last_pull_tripod_a = true;

  auto run_phase = [&](bool pull_tripod_a, int pull_limit_hits_required, const std::string & phase_label, bool end_swing_to_neutral) -> bool {
    const bool swing_tripod_a = !pull_tripod_a;
    const double pull_start_time = state_.master_path_time;
    double pull_end_time = pull_start_time;
    std::size_t pull_point_count = 0;

    RCLCPP_INFO(
      node_.get_logger(),
      "gait_coordinator: %s | pull tripod %c | limit hits=%d",
      phase_label.c_str(),
      pull_tripod_a ? 'A' : 'B',
      pull_limit_hits_required);

    if (!path_builder_.build_pulls(
          pull_tripod_a,
          pull_start_time,
          config_.trajectory_id,
          pull_limit_hits_required,
          pull_end_time,
          pull_point_count))
    {
      return false;
    }

    const double pull_duration = pull_end_time - pull_start_time;
    if (!path_builder_.build_swings(
          swing_tripod_a,
          pull_end_time,
          pull_duration,
          pull_point_count,
          end_swing_to_neutral,
          config_.trajectory_id))
    {
      return false;
    }
    if (!executor_.execute_current_paths()) {
      return false;
    }

    state_.master_path_time = pull_end_time;
    last_pull_tripod_a = pull_tripod_a;
    phase_end_times.push_back(state_.master_path_time);
    RCLCPP_INFO(node_.get_logger(), "gait_coordinator phase complete: %s at t=%.6f", phase_label.c_str(), pull_end_time);
    return true;
  };

  if (!run_phase(true, 1, "first half-step", false)) {
    return false;
  }

  for (int full_step = 0; full_step < config_.num_steps; ++full_step) {
    const bool pull_tripod_a = (full_step % 2 == 1);
    if (!run_phase(
          pull_tripod_a,
          2,
          "full step " + std::to_string(full_step + 1),
          false))
    {
      return false;
    }
  }

  const bool final_pull_tripod_a = !last_pull_tripod_a;
  if (!run_phase(final_pull_tripod_a, 1, "final half-step", true)) {
    return false;
  }

  for (std::size_t i = 0; i < phase_end_times.size(); ++i) {
    RCLCPP_INFO(node_.get_logger(), "Phase %zu end master time: %.3f", i + 1, phase_end_times[i]);
  }
  return true;
}

}  // namespace hexapod_sim
