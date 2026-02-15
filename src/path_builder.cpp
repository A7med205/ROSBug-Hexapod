#include "hexapod_sim/path_builder.hpp"

#include <cmath>

#include "hexapod_sim/kinematics.hpp"

namespace hexapod_sim
{

PathBuilder::PathBuilder(
  rclcpp::Node & node,
  const GaitConfig & config,
  GaitState & state,
  const MasterPath & master_path)
: config_(config), state_(state), master_path_(master_path), node_(node)
{}

// Converts base motion into local tip displacement for a stance-locked foot.
LocalDisplacement2D PathBuilder::tip_delta(const BasePose2D & base1, const BasePose2D & base2, int leg_id)
{
  if (leg_id < 1 || leg_id > static_cast<int>(state_.legs.size())) {
    RCLCPP_ERROR(node_.get_logger(), "[path_builder] invalid leg_id=%d in tip_delta", leg_id);
    return {0.0, 0.0};
  }

  const std::size_t idx = static_cast<std::size_t>(leg_id - 1);
  const auto & leg = state_.legs[idx];
  const double leg_theta = Kinematics::deg_to_rad(leg.frame_pose.theta);
  const double c1 = std::cos(base1.theta), s1 = std::sin(base1.theta), c2 = std::cos(base2.theta), s2 = std::sin(base2.theta);

  const double leg1_x = base1.x + c1 * leg.frame_pose.x - s1 * leg.frame_pose.y;
  const double leg1_y = base1.y + s1 * leg.frame_pose.x + c1 * leg.frame_pose.y;
  const double leg2_x = base2.x + c2 * leg.frame_pose.x - s2 * leg.frame_pose.y;
  const double leg2_y = base2.y + s2 * leg.frame_pose.x + c2 * leg.frame_pose.y;

  const double psi1 = base1.theta + leg_theta, psi2 = base2.theta + leg_theta;
  const auto & tip_local_1 = state_.path_tip_state[idx];

  const double cp1 = std::cos(psi1), sp1 = std::sin(psi1);
  const double tip_world_x = leg1_x + cp1 * tip_local_1.x - sp1 * tip_local_1.y;
  const double tip_world_y = leg1_y + sp1 * tip_local_1.x + cp1 * tip_local_1.y;

  const double dx_world = tip_world_x - leg2_x, dy_world = tip_world_y - leg2_y;
  const double cp2 = std::cos(psi2), sp2 = std::sin(psi2);
  const double tip_local_2_x = cp2 * dx_world + sp2 * dy_world;
  const double tip_local_2_y = -sp2 * dx_world + cp2 * dy_world;

  return {tip_local_2_x - tip_local_1.x, tip_local_2_y - tip_local_1.y};
}

// Returns world-frame neutral tip pose (x,y,theta) for a leg at time t.
WorldNeutralPose PathBuilder::neutral_tip_delta(double t, int leg_id, TrajectoryType trajectory_type)
{
  if (leg_id < 1 || leg_id > static_cast<int>(state_.legs.size())) {
    RCLCPP_ERROR(node_.get_logger(), "[path_builder] invalid leg_id=%d in neutral_tip_delta", leg_id);
    return {0.0, 0.0, 0.0};
  }

  const auto base = master_path_.pose(t, trajectory_type);
  const auto & leg = state_.legs[static_cast<std::size_t>(leg_id - 1)];

  const double c = std::cos(base.theta), s = std::sin(base.theta);
  const double leg_origin_x = base.x + c * leg.frame_pose.x - s * leg.frame_pose.y;
  const double leg_origin_y = base.y + s * leg.frame_pose.x + c * leg.frame_pose.y;

  const double theta_world = base.theta + Kinematics::deg_to_rad(leg.frame_pose.theta);
  const double cp = std::cos(theta_world), sp = std::sin(theta_world);
  const double neutral_world_x = leg_origin_x + cp * kXHome - sp * kYHome;
  const double neutral_world_y = leg_origin_y + sp * kXHome + cp * kYHome;

  return {neutral_world_x, neutral_world_y, theta_world};
}

// Predicts swing landing offsets in local frame for the selected tripod.
std::array<LocalPose2D, 3> PathBuilder::tip_landing_pose(Tripod tripod, double start_time, TrajectoryType trajectory_type)
{
  const auto & tripod_legs_idx = tripod_legs(tripod);
  std::array<WorldNeutralPose, 3> start_world{}, end_world{};
  std::array<LocalPose2D, 3> local_displacements{};

  for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
    start_world[i] = neutral_tip_delta(start_time, state_.legs[tripod_legs_idx[i]].leg_id, trajectory_type);
    end_world[i] = start_world[i];
  }

  bool reached = false;
  constexpr std::size_t max_steps = 10000;
  double t = start_time;
  for (std::size_t step = 1; step <= max_steps; ++step) {
    (void)step;
    t += config_.discrete_step;
    for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
      const auto current = neutral_tip_delta(t, state_.legs[tripod_legs_idx[i]].leg_id, trajectory_type);
      end_world[i] = current;
      if (std::hypot(current.x - start_world[i].x, current.y - start_world[i].y) >= config_.limit_radius) reached = true;
    }
    if (reached) break;
  }

  if (!reached) RCLCPP_WARN(node_.get_logger(), "[path_builder] tip_landing_pose reached max iterations before limit_radius");

  for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
    const double dx_world = end_world[i].x - start_world[i].x;
    const double dy_world = end_world[i].y - start_world[i].y;
    const double ct = std::cos(start_world[i].theta), st = std::sin(start_world[i].theta);
    local_displacements[i].x = ct * dx_world + st * dy_world;
    local_displacements[i].y = -st * dx_world + ct * dy_world;
  }

  return local_displacements;
}

// Builds pull trajectories for one tripod; full-step pulls need two radius hits, half-step pulls one.
bool PathBuilder::build_pulls(
  Tripod pull_tripod,
  double start_time,
  TrajectoryType trajectory_type,
  PullPhaseSpan pull_phase_span,
  double & end_time,
  std::size_t & point_count)
{
  const int peak_tip_limit_hits_target = required_peak_tip_limit_hits(pull_phase_span);
  const auto & tripod_legs_idx = tripod_legs(pull_tripod);

  for (auto & path : state_.path_3d) { path.clear(); path.reserve(2048); }
  for (std::size_t i = 0; i < state_.legs.size(); ++i) {
    state_.path_tip_state[i] = state_.legs[i].current_tip_position;
    state_.path_3d[i].push_back(state_.path_tip_state[i]);
  }

  std::array<LocalPose2D, 3> radius_origin_xy{};
  for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
    radius_origin_xy[i] = {state_.path_tip_state[tripod_legs_idx[i]].x, state_.path_tip_state[tripod_legs_idx[i]].y};
  }

  TrajectoryType active_trajectory_type = trajectory_type;
  BasePose2D base_prev = master_path_.pose(start_time, active_trajectory_type);
  constexpr std::size_t max_path_steps = 10000;
  int peak_tip_limit_hits = 0;
  double t = start_time;

  for (std::size_t step_idx = 1; step_idx <= max_path_steps; ++step_idx) {
    (void)step_idx;
    rclcpp::spin_some(node_.get_node_base_interface());
    t += config_.discrete_step;
    const BasePose2D base_curr = master_path_.pose(t, active_trajectory_type);
    bool any_tip_hit_limit = false;

    for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
      const auto leg_idx = tripod_legs_idx[i];
      const auto delta = tip_delta(base_prev, base_curr, state_.legs[leg_idx].leg_id);
      auto & tip = state_.path_tip_state[leg_idx];
      tip.x += delta.dx_local;
      tip.y += delta.dy_local;
      tip.z = kZHome;
      state_.path_3d[leg_idx].push_back(tip);

      if (!any_tip_hit_limit && std::hypot(tip.x - radius_origin_xy[i].x, tip.y - radius_origin_xy[i].y) >= config_.limit_radius) {
        any_tip_hit_limit = true;
      }
    }

    base_prev = base_curr;
    if (any_tip_hit_limit) {
      ++peak_tip_limit_hits;

      const auto requested = state_.requested_trajectory_type;
      if (requested != state_.current_trajectory_type) {
        if (requested == TrajectoryType::Stationary) {
          if (!state_.stop_requested) RCLCPP_INFO(node_.get_logger(), "[path_builder] stationary requested; scheduling final half-step");
          state_.stop_requested = true;
        } else {
          state_.current_trajectory_type = requested;
          active_trajectory_type = requested;
          state_.stop_requested = false;
          base_prev = master_path_.pose(t, active_trajectory_type);
          RCLCPP_INFO(node_.get_logger(), "[path_builder] trajectory switched at hit to %s (%d)", master_path_.name(requested).c_str(), trajectory_type_id(requested));
        }
      }

      if (peak_tip_limit_hits >= peak_tip_limit_hits_target) {
        end_time = t;
        point_count = state_.path_3d[tripod_legs_idx[0]].size();
        state_.master_path_time = end_time;
        for (const auto leg_idx : tripod_legs_idx) state_.legs[leg_idx].current_tip_position = state_.path_3d[leg_idx].back();
        return true;
      }
      for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
        const auto leg_idx = tripod_legs_idx[i];
        radius_origin_xy[i] = {state_.path_tip_state[leg_idx].x, state_.path_tip_state[leg_idx].y};
      }
    }
  }

  RCLCPP_WARN(
    node_.get_logger(),
    "[path_builder] build_pulls reached max steps before %d required radius hits",
    peak_tip_limit_hits_target);
  return false;
}

// Builds swing trajectories for one tripod with optional return-to-neutral landing.
bool PathBuilder::build_swings(
  Tripod swing_tripod,
  double start_time,
  double pull_duration,
  std::size_t point_count,
  bool end_to_neutral,
  TrajectoryType trajectory_type)
{
  const auto & tripod_legs_idx = tripod_legs(swing_tripod);
  std::array<LocalPose2D, 3> local_end_displacements{};
  if (!end_to_neutral) local_end_displacements = tip_landing_pose(swing_tripod, start_time, trajectory_type);

  if (point_count < 2) {
    point_count = static_cast<std::size_t>(std::round(pull_duration / config_.discrete_step)) + 1;
    if (point_count < 2) point_count = 2;
  }

  for (std::size_t i = 0; i < tripod_legs_idx.size(); ++i) {
    const auto leg_idx = tripod_legs_idx[i];
    const auto start_tip = state_.legs[leg_idx].current_tip_position;
    const double end_x = end_to_neutral ? kXHome : (kXHome + local_end_displacements[i].x);
    const double end_y = end_to_neutral ? kYHome : (kYHome + local_end_displacements[i].y);
    const double end_z = end_to_neutral ? kZHome : start_tip.z;

    state_.path_3d[leg_idx].clear();
    state_.path_3d[leg_idx].reserve(point_count);

    for (std::size_t p = 0; p < point_count; ++p) {
      const double s = (point_count <= 1) ? 1.0 : static_cast<double>(p) / static_cast<double>(point_count - 1);
      const double x = start_tip.x + s * (end_x - start_tip.x);
      const double y = start_tip.y + s * (end_y - start_tip.y);
      const double z = (1.0 - s) * start_tip.z + s * end_z + config_.swing_height * std::sin(kPi * s);
      state_.path_3d[leg_idx].push_back({x, y, z});
    }

    state_.legs[leg_idx].current_tip_position = {end_x, end_y, end_z};
  }

  return true;
}

}  // namespace hexapod_sim
