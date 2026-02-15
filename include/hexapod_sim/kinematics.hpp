#pragma once

#include <rclcpp/rclcpp.hpp>

#include "hexapod_sim/gait_state.hpp"

namespace hexapod_sim
{

class Kinematics
{
public:
  static double clamp(double value, double min_val, double max_val);
  static double deg_to_rad(double deg);
  static void compute_ik(
    const Point3D & tip,
    double & j1,
    double & j2,
    double & j3,
    const rclcpp::Logger & logger,
    rclcpp::Clock & clock);
};

}  // namespace hexapod_sim
