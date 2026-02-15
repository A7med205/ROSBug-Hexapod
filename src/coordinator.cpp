#include "hexapod_sim/coordinator.hpp"

#include <chrono>
#include <string>
#include <thread>

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

// Runs continuously: idle in stationary, then first half-step and continuous full-steps while moving.
bool GaitCoordinator::run()
{
  using namespace std::chrono_literals;
  if (!initialize_neutral_joint_values()) return false;

  state_.master_path_time = 0.0;

  auto run_phase = [&](Tripod pull_tripod, PullPhaseSpan pull_phase_span, const char * phase_label, bool end_swing_to_neutral) -> bool {
    double pull_end_time = state_.master_path_time;
    std::size_t pull_point_count = 0;

    if (!path_builder_.build_pulls(
          pull_tripod,
          state_.master_path_time,
          state_.current_trajectory_type,
          pull_phase_span,
          pull_end_time,
          pull_point_count))
    {
      RCLCPP_ERROR(node_.get_logger(), "[coordinator] build_pulls failed in %s", phase_label);
      return false;
    }

    if (!path_builder_.build_swings(
          opposite_tripod(pull_tripod),
          pull_end_time,
          pull_end_time - state_.master_path_time,
          pull_point_count,
          end_swing_to_neutral,
          state_.current_trajectory_type))
    {
      RCLCPP_ERROR(node_.get_logger(), "[coordinator] build_swings failed in %s", phase_label);
      return false;
    }

    if (!executor_.execute_current_paths()) {
      RCLCPP_ERROR(node_.get_logger(), "[coordinator] execute_current_paths failed in %s", phase_label);
      return false;
    }

    state_.master_path_time = pull_end_time;
    return true;
  };

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_.get_node_base_interface());

    if (state_.current_trajectory_type == TrajectoryType::Stationary) {
      if (state_.requested_trajectory_type == TrajectoryType::Stationary) {
        std::this_thread::sleep_for(50ms);
        continue;
      }

      state_.current_trajectory_type = state_.requested_trajectory_type;
      state_.stop_requested = false;
      RCLCPP_INFO(node_.get_logger(), "[coordinator] leaving stationary with trajectory=%d", trajectory_type_id(state_.current_trajectory_type));

      if (!run_phase(Tripod::A, PullPhaseSpan::HalfStep, "first half-step", false)) return false;

      Tripod next_full_pull_tripod = Tripod::B;
      while (rclcpp::ok() && state_.current_trajectory_type != TrajectoryType::Stationary) {
        // If stationary was requested at a pull-hit boundary, skip full-step and close with final half-step.
        if (state_.stop_requested) {
          if (!run_phase(next_full_pull_tripod, PullPhaseSpan::HalfStep, "final half-step", true)) return false;
          state_.current_trajectory_type = TrajectoryType::Stationary;
          state_.requested_trajectory_type = TrajectoryType::Stationary;
          state_.stop_requested = false;
          RCLCPP_INFO(node_.get_logger(), "[coordinator] entered stationary mode");
          break;
        }

        if (!run_phase(next_full_pull_tripod, PullPhaseSpan::FullStep, "full-step", false)) return false;
        next_full_pull_tripod = opposite_tripod(next_full_pull_tripod);
      }
    }
  }

  return true;
}

}  // namespace hexapod_sim
