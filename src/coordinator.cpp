#include "hexapod_sim/coordinator.hpp"

#include <string>

#include "hexapod_sim/gait_config.hpp"
#include "hexapod_sim/kinematics.hpp"

namespace hexapod_sim
{

GaitCoordinator::GaitCoordinator(
  rclcpp::Node & node,
  const GaitConfig & config,
  GaitState & state,
  PathBuilder & path_builder,
  TrajectoryExecutor & executor)
: node_(node),
  config_(config),
  state_(state),
  path_builder_(path_builder),
  executor_(executor)
{}

// Initializes current joint angles from neutral tip targets for all legs.
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

// Runs: first half-step, full-step sequence, and final half-step.
bool GaitCoordinator::run()
{
  if (!initialize_neutral_joint_values()) return false;

  state_.master_path_time = 0.0;
  Tripod last_pull_tripod = Tripod::A;

  auto run_phase = [&](Tripod pull_tripod, PullPhaseSpan pull_phase_span, const std::string & phase_label, bool end_swing_to_neutral) -> bool {
    const Tripod swing_tripod = opposite_tripod(pull_tripod);
    const int peak_tip_limit_hits_target = required_peak_tip_limit_hits(pull_phase_span);
    const double pull_start_time = state_.master_path_time;
    double pull_end_time = pull_start_time;
    std::size_t pull_point_count = 0;

    RCLCPP_INFO(
      node_.get_logger(),
      "[coordinator] %s | pull tripod=%c | peak_hits=%d",
      phase_label.c_str(),
      tripod_label(pull_tripod),
      peak_tip_limit_hits_target);

    if (!path_builder_.build_pulls(
          pull_tripod,
          pull_start_time,
          config_.trajectory_type,
          pull_phase_span,
          pull_end_time,
          pull_point_count))
    {
      return false;
    }

    if (!path_builder_.build_swings(
          swing_tripod,
          pull_end_time,
          pull_end_time - pull_start_time,
          pull_point_count,
          end_swing_to_neutral,
          config_.trajectory_type))
    {
      return false;
    }
    if (!executor_.execute_current_paths()) {
      return false;
    }

    state_.master_path_time = pull_end_time;
    last_pull_tripod = pull_tripod;
    RCLCPP_INFO(node_.get_logger(), "[coordinator] %s complete at t=%.6f", phase_label.c_str(), pull_end_time);
    return true;
  };

  if (!run_phase(Tripod::A, PullPhaseSpan::HalfStep, "first half-step", false)) return false;

  for (int full_step = 0; full_step < config_.num_steps; ++full_step) {
    const Tripod pull_tripod = (full_step % 2 == 1) ? Tripod::A : Tripod::B;
    if (!run_phase(
          pull_tripod,
          PullPhaseSpan::FullStep,
          "full step " + std::to_string(full_step + 1),
          false))
    {
      return false;
    }
  }

  return run_phase(opposite_tripod(last_pull_tripod), PullPhaseSpan::HalfStep, "final half-step", true);
}

}  // namespace hexapod_sim
