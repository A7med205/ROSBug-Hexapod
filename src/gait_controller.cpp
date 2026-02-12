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

class GaitController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  struct TipPosition
  {
    double x;
    double y;
    double z;
  };

private:
  struct LegFramePose
  {
    double x;
    double y;
    double theta;
  };

  struct Leg
  {
    int leg_id;
    std::array<std::string, 3> joint_names;
    LegFramePose frame_pose;
    TipPosition current_tip_position;
    std::array<double, 3> current_joint_angles;
  };

public:
  GaitController()
  : Node("gait_controller")
  {
    action_name_ = this->declare_parameter<std::string>(
      "action_name", "/joint_trajectory_controller/follow_joint_trajectory");

    legs_ = create_legs();

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);

    RCLCPP_INFO(this->get_logger(), "Initialized %zu legs", legs_.size());
    for (const auto & leg : legs_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Leg %d frame: x=%.4f y=%.4f theta=%.1f deg | joints: [%s, %s, %s] | tip home: (%.3f, %.3f, %.3f)",
        leg.leg_id,
        leg.frame_pose.x,
        leg.frame_pose.y,
        leg.frame_pose.theta,
        leg.joint_names[0].c_str(),
        leg.joint_names[1].c_str(),
        leg.joint_names[2].c_str(),
        leg.current_tip_position.x,
        leg.current_tip_position.y,
        leg.current_tip_position.z);
    }
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

    const std::vector<TipPosition> test_targets = {
      {0.150, 0.0, -0.050},
      {0.110, 0.04, -0.050},
      {0.110, 0.0, -0.010}
    };

    for (std::size_t i = 0; i < test_targets.size(); ++i) {
      const auto & target = test_targets[i];
      RCLCPP_INFO(
        this->get_logger(),
        "Test step %zu/%zu -> target local tip (x=%.3f, y=%.3f, z=%.3f)",
        i + 1,
        test_targets.size(),
        target.x,
        target.y,
        target.z);

      if (!send_joint_values_for_all_legs(target, 1.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute test step %zu", i + 1);
        return false;
      }

      if (i + 1 < test_targets.size()) {
        RCLCPP_INFO(this->get_logger(), "Sleeping for 2 seconds before next command");
        std::this_thread::sleep_for(2s);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Startup gait-controller test sequence complete");
    return true;
  }

private:
  static double clamp(double value, double min_val, double max_val)
  {
    return std::max(min_val, std::min(value, max_val));
  }

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

  void compute_ik(
    const TipPosition & tip,
    double & j1,
    double & j2,
    double & j3)
  {
    // Local leg frame IK, right-handed coordinates (x right, y forward, z up).
    const double y = tip.y;
    const double x = tip.x;
    const double z = tip.z;

    j1 = -std::atan2(y, x);

    const double x_prime = std::sqrt(x * x + y * y) - kL1;
    double d = std::sqrt(x_prime * x_prime + z * z);

    const double min_reach = std::abs(kL2 - kL3);
    const double max_reach = kL2 + kL3;
    if (d > max_reach || d < min_reach) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "IK Warning: Position (%.3f, %.3f) is unreachable. D=%.3f",
        x_prime,
        z,
        d);
      d = clamp(d, min_reach, max_reach);
    }

    const double alpha1 = std::atan2(-z, x_prime);

    const double cos_alpha2 = clamp((kL2 * kL2 + d * d - kL3 * kL3) / (2.0 * kL2 * d), -1.0, 1.0);
    const double alpha2 = std::acos(cos_alpha2);

    const double cos_knee = clamp((kL2 * kL2 + kL3 * kL3 - d * d) / (2.0 * kL2 * kL3), -1.0, 1.0);

    j2 = alpha1 - alpha2;
    j3 = kPi - std::acos(cos_knee);
  }

  bool send_joint_values_for_all_legs(const TipPosition & target_tip, double time_from_start_sec)
  {
    std::vector<std::string> all_joint_names;
    std::vector<double> all_joint_positions;
    all_joint_names.reserve(legs_.size() * 3);
    all_joint_positions.reserve(legs_.size() * 3);

    for (auto & leg : legs_) {
      double j1 = 0.0;
      double j2 = 0.0;
      double j3 = 0.0;

      compute_ik(target_tip, j1, j2, j3);

      leg.current_tip_position = target_tip;
      leg.current_joint_angles = {j1, j2, j3};

      all_joint_names.insert(
        all_joint_names.end(),
        leg.joint_names.begin(),
        leg.joint_names.end());
      all_joint_positions.push_back(j1);
      all_joint_positions.push_back(j2);
      all_joint_positions.push_back(j3);

      RCLCPP_INFO(
        this->get_logger(),
        "Leg %d IK -> J1=%.5f J2=%.5f J3=%.5f",
        leg.leg_id,
        j1,
        j2,
        j3);
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = all_joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = all_joint_positions;
    const int32_t sec = static_cast<int32_t>(time_from_start_sec);
    const double frac_sec = time_from_start_sec - static_cast<double>(sec);
    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = static_cast<uint32_t>(frac_sec * 1e9);
    trajectory.points.push_back(point);

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = trajectory;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending trajectory goal with %zu joints and %.2fs duration",
      all_joint_positions.size(),
      time_from_start_sec);

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
    const auto result_status =
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
    if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed while waiting for action result");
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Trajectory action failed with result code %d",
        static_cast<int>(wrapped_result.code));
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory action succeeded");
    return true;
  }

  std::string action_name_;
  std::vector<Leg> legs_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

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
