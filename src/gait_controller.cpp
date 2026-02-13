#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

struct Point3D { double x; double y; double z; };
struct BasePose2D { double x; double y; double theta; };
struct LocalDisplacement2D { double dx_local; double dy_local; };
struct WorldNeutralPose { double x; double y; double theta; };
struct LocalPose2D { double x; double y; };

double discrete_step = 0.01;
double min_angle = 1.0;
double limit_radius = 0.05;
double swing_height = 0.02;
int num_steps = 2;
double g_master_path_time = 0.0;
std::array<std::vector<Point3D>, 6> g_3d_path;

class GaitController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using TipPosition = Point3D;

private:
  struct LegFramePose { double x; double y; double theta; };
  struct Leg {
    int leg_id;
    std::array<std::string, 3> joint_names;
    LegFramePose frame_pose;
    TipPosition current_tip_position;
    std::array<double, 3> current_joint_angles;
  };

public:
  GaitController() : Node("gait_controller")
  {
    action_name_ = this->declare_parameter<std::string>("action_name", "/joint_trajectory_controller/follow_joint_trajectory");
    const int64_t configured_num_steps = this->declare_parameter<int64_t>("num_steps", static_cast<int64_t>(num_steps));
    num_steps = static_cast<int>(std::max<int64_t>(0, configured_num_steps));
    legs_ = create_legs();
    for (auto & tip : path_tip_state_) tip = {kXHome, kYHome, kZHome};
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);

    RCLCPP_INFO(
      this->get_logger(),
      "Initialized %zu legs | discrete_step=%.3f, min_angle=%.1f deg, limit_radius=%.3f, swing_height=%.3f, num_steps=%d",
      legs_.size(), discrete_step, min_angle, limit_radius, swing_height, num_steps);
  }

  bool run_startup_sequence()
  {
    using namespace std::chrono_literals;
    RCLCPP_INFO(this->get_logger(), "Waiting for action server: %s", action_name_.c_str());
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server unavailable after 5 seconds");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Action server connected");

    if (!gait_coordinator()) return false;
    RCLCPP_INFO(this->get_logger(), "First half-step (pull+swing) complete at master time t=%.3f", g_master_path_time);
    return true;
  }

private:
  static double clamp(double value, double min_val, double max_val) { return std::max(min_val, std::min(value, max_val)); }
  static double deg_to_rad(double deg) { return deg * (kPi / 180.0); }

  const std::array<std::size_t, 3> & tripod_indices(bool tripod_a) const { return tripod_a ? kTripodA : kTripodB; }

  std::vector<Leg> create_legs() const
  {
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    return {
      {1, {"jl11", "jl12", "jl13"}, {-0.0535, 0.0900, 135.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
      {2, {"jl21", "jl22", "jl23"}, {-0.0700, 0.0000, 180.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
      {3, {"jl31", "jl32", "jl33"}, {-0.0535, -0.0900, -135.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
      {4, {"jl41", "jl42", "jl43"}, {0.0535, 0.0900, 45.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
      {5, {"jl51", "jl52", "jl53"}, {0.0700, 0.0000, 0.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}},
      {6, {"jl61", "jl62", "jl63"}, {0.0535, -0.0900, -45.0}, {kXHome, kYHome, kZHome}, {nan, nan, nan}}
    };
  }

  BasePose2D master_path(double t, int trajectory_id) const
  {
    constexpr double straight_speed = 0.2;                  // m/s (+Y)
    constexpr double diag_speed_x = 0.15, diag_speed_y = 0.20; // m/s
    constexpr double angular_speed = 15.0 * kPi / 180.0;    // rad/s

    BasePose2D pose{0.0, 0.0, 0.0};
    switch (trajectory_id) {
      case 0: pose.y = straight_speed * t; break;
      case 1: pose.x = diag_speed_x * t; pose.y = diag_speed_y * t; break;
      case 2: pose.theta = angular_speed * t; break;
      default: pose.y = straight_speed * t; break;
    }
    return pose;
  }

  std::string trajectory_name(int trajectory_id) const
  {
    switch (trajectory_id) {
      case 0: return "straight +Y";
      case 1: return "diagonal +X/+Y";
      case 2: return "in-place rotation";
      default: return "unknown(default straight +Y)";
    }
  }

  LocalDisplacement2D base_to_tip(const BasePose2D & base1, const BasePose2D & base2, int leg_id)
  {
    if (leg_id < 1 || leg_id > static_cast<int>(legs_.size())) {
      RCLCPP_ERROR(this->get_logger(), "Invalid leg_id=%d in base_to_tip", leg_id);
      return {0.0, 0.0};
    }

    const std::size_t idx = static_cast<std::size_t>(leg_id - 1);
    const auto & leg = legs_[idx];
    const double leg_theta = deg_to_rad(leg.frame_pose.theta);
    const double c1 = std::cos(base1.theta), s1 = std::sin(base1.theta), c2 = std::cos(base2.theta), s2 = std::sin(base2.theta);

    const double leg1_x = base1.x + c1 * leg.frame_pose.x - s1 * leg.frame_pose.y;
    const double leg1_y = base1.y + s1 * leg.frame_pose.x + c1 * leg.frame_pose.y;
    const double leg2_x = base2.x + c2 * leg.frame_pose.x - s2 * leg.frame_pose.y;
    const double leg2_y = base2.y + s2 * leg.frame_pose.x + c2 * leg.frame_pose.y;

    const double psi1 = base1.theta + leg_theta, psi2 = base2.theta + leg_theta;
    const auto & tip_local_1 = path_tip_state_[idx];

    const double cp1 = std::cos(psi1), sp1 = std::sin(psi1);
    const double tip_world_x = leg1_x + cp1 * tip_local_1.x - sp1 * tip_local_1.y;
    const double tip_world_y = leg1_y + sp1 * tip_local_1.x + cp1 * tip_local_1.y;

    const double dx_world = tip_world_x - leg2_x, dy_world = tip_world_y - leg2_y;
    const double cp2 = std::cos(psi2), sp2 = std::sin(psi2);
    const double tip_local_2_x = cp2 * dx_world + sp2 * dy_world;
    const double tip_local_2_y = -sp2 * dx_world + cp2 * dy_world;

    return {tip_local_2_x - tip_local_1.x, tip_local_2_y - tip_local_1.y};
  }

  void compute_ik(const TipPosition & tip, double & j1, double & j2, double & j3)
  {
    const double y = tip.y, x = tip.x, z = tip.z;
    j1 = -std::atan2(y, x);

    const double x_prime = std::sqrt(x * x + y * y) - kL1;
    double d = std::sqrt(x_prime * x_prime + z * z);

    const double min_reach = std::abs(kL2 - kL3), max_reach = kL2 + kL3;
    if (d > max_reach || d < min_reach) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "IK Warning: Position (%.3f, %.3f) is unreachable. D=%.3f", x_prime, z, d);
      d = clamp(d, min_reach, max_reach);
    }

    const double alpha1 = std::atan2(-z, x_prime);
    const double cos_alpha2 = clamp((kL2 * kL2 + d * d - kL3 * kL3) / (2.0 * kL2 * d), -1.0, 1.0);
    const double alpha2 = std::acos(cos_alpha2);
    const double cos_knee = clamp((kL2 * kL2 + kL3 * kL3 - d * d) / (2.0 * kL2 * kL3), -1.0, 1.0);

    j2 = alpha1 - alpha2;
    j3 = kPi - std::acos(cos_knee);
  }

  bool initialize_neutral_joint_values()
  {
    for (auto & leg : legs_) {
      leg.current_tip_position = {kXHome, kYHome, kZHome};
      double j1 = 0.0, j2 = 0.0, j3 = 0.0;
      compute_ik(leg.current_tip_position, j1, j2, j3);
      leg.current_joint_angles = {j1, j2, j3};
    }
    return true;
  }

  WorldNeutralPose wbase_to_wnuetral(double t, int leg_id)
  {
    if (leg_id < 1 || leg_id > static_cast<int>(legs_.size())) {
      RCLCPP_ERROR(this->get_logger(), "Invalid leg_id=%d in wbase_to_wnuetral", leg_id);
      return {0.0, 0.0, 0.0};
    }

    const auto & base = master_path(t, kStraightTrajectoryId);
    const auto & leg = legs_[static_cast<std::size_t>(leg_id - 1)];

    const double c = std::cos(base.theta), s = std::sin(base.theta);
    const double leg_origin_x = base.x + c * leg.frame_pose.x - s * leg.frame_pose.y;
    const double leg_origin_y = base.y + s * leg.frame_pose.x + c * leg.frame_pose.y;

    const double theta_world = base.theta + deg_to_rad(leg.frame_pose.theta);
    const double cp = std::cos(theta_world), sp = std::sin(theta_world);
    const double neutral_world_x = leg_origin_x + cp * kXHome - sp * kYHome;
    const double neutral_world_y = leg_origin_y + sp * kXHome + cp * kYHome;

    return {neutral_world_x, neutral_world_y, theta_world};
  }

  std::array<LocalPose2D, 3> future_tip_poses(bool tripod_a, double start_time)
  {
    const auto & tripod = tripod_indices(tripod_a);
    std::array<WorldNeutralPose, 3> start_world{}, end_world{};
    std::array<LocalPose2D, 3> local_displacements{};

    for (std::size_t i = 0; i < tripod.size(); ++i) {
      start_world[i] = wbase_to_wnuetral(start_time, legs_[tripod[i]].leg_id);
      end_world[i] = start_world[i];
    }

    bool reached = false;
    constexpr std::size_t max_steps = 10000;
    double t = start_time;
    for (std::size_t step = 1; step <= max_steps; ++step) {
      t += discrete_step;
      for (std::size_t i = 0; i < tripod.size(); ++i) {
        const auto current = wbase_to_wnuetral(t, legs_[tripod[i]].leg_id);
        end_world[i] = current;

        const double dx = current.x - start_world[i].x;
        const double dy = current.y - start_world[i].y;
        if (std::hypot(dx, dy) >= limit_radius) reached = true;
      }
      if (reached) break;
    }

    if (!reached) RCLCPP_WARN(this->get_logger(), "future_tip_poses hit max iterations before limit_radius reached");

    for (std::size_t i = 0; i < tripod.size(); ++i) {
      const double dx_world = end_world[i].x - start_world[i].x;
      const double dy_world = end_world[i].y - start_world[i].y;
      const double ct = std::cos(start_world[i].theta), st = std::sin(start_world[i].theta);
      local_displacements[i].x = ct * dx_world + st * dy_world;
      local_displacements[i].y = -st * dx_world + ct * dy_world;
    }

    return local_displacements;
  }

  bool build_pulls(
    bool tripod_a,
    double start_time,
    int trajectory_id,
    int limit_hits_required,
    double & end_time,
    std::size_t & point_count)
  {
    if (limit_hits_required < 1) limit_hits_required = 1;

    const auto & tripod = tripod_indices(tripod_a);
    for (auto & path : g_3d_path) { path.clear(); path.reserve(2048); }
    for (std::size_t i = 0; i < legs_.size(); ++i) {
      path_tip_state_[i] = legs_[i].current_tip_position;
      g_3d_path[i].push_back(path_tip_state_[i]);
    }

    std::array<LocalPose2D, 3> start_tip_xy{};
    for (std::size_t i = 0; i < tripod.size(); ++i) {
      start_tip_xy[i] = {path_tip_state_[tripod[i]].x, path_tip_state_[tripod[i]].y};
    }
    auto radius_origin_xy = start_tip_xy;

    BasePose2D base_prev = master_path(start_time, trajectory_id);
    constexpr std::size_t max_path_steps = 10000;
    bool pull_complete = false;
    int limit_hits = 0;
    double t = start_time;

    for (std::size_t step_idx = 1; step_idx <= max_path_steps; ++step_idx) {
      t += discrete_step;
      const BasePose2D base_curr = master_path(t, trajectory_id);
      int first_leg_over_limit = -1;

      for (std::size_t i = 0; i < tripod.size(); ++i) {
        const auto leg_idx = tripod[i];
        const auto delta = base_to_tip(base_prev, base_curr, legs_[leg_idx].leg_id);
        auto & tip = path_tip_state_[leg_idx];
        tip.x += delta.dx_local;
        tip.y += delta.dy_local;
        tip.z = kZHome;
        g_3d_path[leg_idx].push_back(tip);

        const double radius = std::hypot(tip.x - radius_origin_xy[i].x, tip.y - radius_origin_xy[i].y);
        if (first_leg_over_limit < 0 && radius >= limit_radius) {
          first_leg_over_limit = static_cast<int>(legs_[leg_idx].leg_id);
        }
      }

      base_prev = base_curr;
      if (first_leg_over_limit >= 0) {
        ++limit_hits;
        RCLCPP_INFO(
          this->get_logger(),
          "Pull path limit hit %d/%d by leg %d",
          limit_hits,
          limit_hits_required,
          first_leg_over_limit);

        if (limit_hits >= limit_hits_required) {
          pull_complete = true;
          break;
        }

        for (std::size_t i = 0; i < tripod.size(); ++i) {
          const auto leg_idx = tripod[i];
          radius_origin_xy[i] = {path_tip_state_[leg_idx].x, path_tip_state_[leg_idx].y};
        }
      }
    }

    if (!pull_complete) {
      RCLCPP_WARN(
        this->get_logger(),
        "build_pulls hit max steps before reaching %d limit hits",
        limit_hits_required);
      return false;
    }

    for (const auto leg_idx : tripod) legs_[leg_idx].current_tip_position = g_3d_path[leg_idx].back();

    end_time = t;
    point_count = g_3d_path[tripod[0]].size();
    g_master_path_time = end_time;
    return true;
  }

  bool build_swings(
    bool tripod_a,
    double start_time,
    double pull_duration,
    std::size_t point_count,
    bool end_to_neutral)
  {
    const auto & tripod = tripod_indices(tripod_a);
    std::array<LocalPose2D, 3> local_end_displacements{};
    if (!end_to_neutral) local_end_displacements = future_tip_poses(tripod_a, start_time);

    if (point_count < 2) {
      point_count = static_cast<std::size_t>(std::round(pull_duration / discrete_step)) + 1;
      if (point_count < 2) point_count = 2;
    }

    for (std::size_t i = 0; i < tripod.size(); ++i) {
      const auto leg_idx = tripod[i];
      const auto start_tip = legs_[leg_idx].current_tip_position;
      const double end_x = end_to_neutral ? kXHome : (start_tip.x + local_end_displacements[i].x);
      const double end_y = end_to_neutral ? kYHome : (start_tip.y + local_end_displacements[i].y);
      const double end_z = end_to_neutral ? kZHome : start_tip.z;

      g_3d_path[leg_idx].clear();
      g_3d_path[leg_idx].reserve(point_count);

      for (std::size_t p = 0; p < point_count; ++p) {
        const double s = (point_count <= 1) ? 1.0 : static_cast<double>(p) / static_cast<double>(point_count - 1);
        const double x = start_tip.x + s * (end_x - start_tip.x);
        const double y = start_tip.y + s * (end_y - start_tip.y);
        const double z = (1.0 - s) * start_tip.z + s * end_z + swing_height * std::sin(kPi * s);
        g_3d_path[leg_idx].push_back({x, y, z});
      }

      legs_[leg_idx].current_tip_position = {end_x, end_y, end_z};
    }

    return true;
  }

  bool gait_coordinator()
  {
    if (!initialize_neutral_joint_values()) return false;

    const int trajectory_id = kStraightTrajectoryId;
    std::vector<double> phase_end_times;
    phase_end_times.reserve(static_cast<std::size_t>(num_steps) + 2);
    bool last_pull_tripod_a = true;

    auto run_phase = [&](
      bool pull_tripod_a,
      int pull_limit_hits_required,
      const std::string & phase_label,
      bool end_swing_to_neutral) -> bool {
      const bool swing_tripod_a = !pull_tripod_a;
      const double pull_start_time = g_master_path_time;
      double pull_end_time = pull_start_time;
      std::size_t pull_point_count = 0;

      RCLCPP_INFO(
        this->get_logger(),
        "gait_coordinator: %s | pull tripod %c | limit hits=%d",
        phase_label.c_str(),
        pull_tripod_a ? 'A' : 'B',
        pull_limit_hits_required);

      if (!build_pulls(
            pull_tripod_a,
            pull_start_time,
            trajectory_id,
            pull_limit_hits_required,
            pull_end_time,
            pull_point_count))
      {
        return false;
      }

      const double pull_duration = pull_end_time - pull_start_time;
      if (!build_swings(swing_tripod_a, pull_end_time, pull_duration, pull_point_count, end_swing_to_neutral)) return false;
      if (!execute_current_paths()) return false;

      g_master_path_time = pull_end_time;
      last_pull_tripod_a = pull_tripod_a;
      phase_end_times.push_back(g_master_path_time);
      return true;
    };

    if (!run_phase(true, 1, "first half-step", false)) return false;

    for (int full_step = 0; full_step < num_steps; ++full_step) {
      const bool pull_tripod_a = (full_step % 2 == 1);  // 1st full: B pulls, 2nd full: A pulls
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
    if (!run_phase(final_pull_tripod_a, 1, "final half-step", true)) return false;

    for (std::size_t i = 0; i < phase_end_times.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "Phase %zu end master time: %.3f", i + 1, phase_end_times[i]);
    }
    return true;
  }

  bool execute_current_paths()
  {
    if (g_3d_path[0].empty()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot execute empty path set");
      return false;
    }

    const std::size_t points_per_leg = g_3d_path[0].size();
    for (std::size_t i = 1; i < g_3d_path.size(); ++i) {
      if (g_3d_path[i].size() != points_per_leg) {
        RCLCPP_ERROR(this->get_logger(), "Path size mismatch across legs");
        return false;
      }
    }

    const double min_angle_rad = min_angle * (kPi / 180.0);
    std::array<std::array<double, 3>, 6> candidate_joint_angles{};
    for (std::size_t leg_idx = 0; leg_idx < legs_.size(); ++leg_idx) candidate_joint_angles[leg_idx] = legs_[leg_idx].current_joint_angles;

    joints_trajectory.joint_names.clear();
    joints_trajectory.points.clear();
    joints_trajectory.joint_names.reserve(legs_.size() * 3);
    joints_trajectory.points.reserve(points_per_leg * legs_.size());
    for (const auto & leg : legs_) joints_trajectory.joint_names.insert(joints_trajectory.joint_names.end(), leg.joint_names.begin(), leg.joint_names.end());

    for (std::size_t point_idx = 0; point_idx < points_per_leg; ++point_idx) {
      for (std::size_t leg_idx = 0; leg_idx < legs_.size(); ++leg_idx) {
        const auto & target_tip = g_3d_path[leg_idx][point_idx];
        double j1 = 0.0, j2 = 0.0, j3 = 0.0;
        compute_ik(target_tip, j1, j2, j3);
        const std::array<double, 3> target_angles = {j1, j2, j3};

        bool any_joint_changed = false;
        for (std::size_t joint_idx = 0; joint_idx < 3; ++joint_idx) {
          const double current = candidate_joint_angles[leg_idx][joint_idx];
          const double delta = std::abs(target_angles[joint_idx] - current);
          if (!std::isfinite(current) || delta > min_angle_rad) {
            candidate_joint_angles[leg_idx][joint_idx] = target_angles[joint_idx];
            any_joint_changed = true;
          }
        }

        if (any_joint_changed) {
          trajectory_msgs::msg::JointTrajectoryPoint point;
          point.positions.reserve(legs_.size() * 3);
          for (const auto & leg_joints : candidate_joint_angles) {
            point.positions.push_back(leg_joints[0]);
            point.positions.push_back(leg_joints[1]);
            point.positions.push_back(leg_joints[2]);
          }

          const double time_from_start_sec = static_cast<double>(joints_trajectory.points.size() + 1) * discrete_step;
          const int32_t sec = static_cast<int32_t>(time_from_start_sec);
          const double frac_sec = time_from_start_sec - static_cast<double>(sec);
          point.time_from_start.sec = sec;
          point.time_from_start.nanosec = static_cast<uint32_t>(frac_sec * 1e9);
          joints_trajectory.points.push_back(point);
        }
      }
    }

    if (!joints_trajectory.points.empty()) {
      RCLCPP_INFO(this->get_logger(), "Built joints_trajectory with %zu points, sending once to action server", joints_trajectory.points.size());
      if (!send_joints_trajectory()) return false;
    }

    for (std::size_t leg_idx = 0; leg_idx < legs_.size(); ++leg_idx) {
      legs_[leg_idx].current_joint_angles = candidate_joint_angles[leg_idx];
      legs_[leg_idx].current_tip_position = g_3d_path[leg_idx].back();
    }

    RCLCPP_INFO(this->get_logger(), "Path execution complete. joints_trajectory points=%zu", joints_trajectory.points.size());
    return true;
  }

  bool send_joints_trajectory()
  {
    if (joints_trajectory.points.empty()) return true;

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = joints_trajectory;

    auto goal_handle_future = action_client_->async_send_goal(goal_msg);
    const auto send_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future);
    if (send_status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal to action server");
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Action server rejected trajectory goal");
      return false;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    const auto result_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
    if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed while waiting for action result");
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory action failed with result code %d", static_cast<int>(wrapped_result.code));
      return false;
    }

    return true;
  }

  std::string action_name_;
  std::vector<Leg> legs_;
  std::array<TipPosition, 6> path_tip_state_;
  trajectory_msgs::msg::JointTrajectory joints_trajectory;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  static constexpr std::array<std::size_t, 3> kTripodA = {0, 2, 4};  // legs 1,3,5
  static constexpr std::array<std::size_t, 3> kTripodB = {1, 3, 5};  // legs 2,4,6
  static constexpr int kStraightTrajectoryId = 0;

  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kXHome = 0.110;
  static constexpr double kYHome = 0.0;
  static constexpr double kZHome = -0.050;
  static constexpr double kL1 = 0.0385;
  static constexpr double kL2 = 0.0700;
  static constexpr double kL3 = 0.1020;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitController>();
  const bool ok = node->run_startup_sequence();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
