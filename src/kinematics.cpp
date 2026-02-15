#include "hexapod_sim/kinematics.hpp"

#include <algorithm>
#include <cmath>

#include "hexapod_sim/gait_config.hpp"

namespace hexapod_sim
{

double Kinematics::clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

double Kinematics::deg_to_rad(double deg)
{
  return deg * (kPi / 180.0);
}

void Kinematics::compute_ik(
  const Point3D & tip,
  double & j1,
  double & j2,
  double & j3,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock)
{
  const double y = tip.y, x = tip.x, z = tip.z;
  j1 = -std::atan2(y, x);

  const double x_prime = std::sqrt(x * x + y * y) - kL1;
  double d = std::sqrt(x_prime * x_prime + z * z);

  const double min_reach = std::abs(kL2 - kL3), max_reach = kL2 + kL3;
  if (d > max_reach || d < min_reach) {
    RCLCPP_WARN_THROTTLE(
      logger,
      clock,
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

}  // namespace hexapod_sim
