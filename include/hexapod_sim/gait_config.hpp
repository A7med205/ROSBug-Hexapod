#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace hexapod_sim
{

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kXHome = 0.110;
inline constexpr double kYHome = 0.0;
inline constexpr double kZHome = -0.050;
inline constexpr double kL1 = 0.0385;
inline constexpr double kL2 = 0.0700;
inline constexpr double kL3 = 0.1020;
inline constexpr std::array<std::size_t, 3> kTripodA = {0, 2, 4};
inline constexpr std::array<std::size_t, 3> kTripodB = {1, 3, 5};

struct GaitConfig
{
  double discrete_step{0.01};
  double min_angle_deg{1.0};
  double limit_radius{0.05};
  double swing_height{0.02};
  int num_steps{30};
  int trajectory_id{4};
  std::string action_name{"/joint_trajectory_controller/follow_joint_trajectory"};
};

}  // namespace hexapod_sim
