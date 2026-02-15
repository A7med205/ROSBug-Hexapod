#include "hexapod_sim/master_path.hpp"

#include <cmath>

#include "hexapod_sim/gait_config.hpp"

namespace hexapod_sim
{

MasterPath::MasterPath(const GaitConfig & config)
: config_(config)
{}

BasePose2D MasterPath::pose(double t, int trajectory_id) const
{
  (void)config_;
  constexpr double straight_speed = 0.4;
  constexpr double diag_speed_x = 0.15, diag_speed_y = 0.20;
  constexpr double angular_speed = 60.0 * kPi / 180.0;
  constexpr double external_orbit_r = 0.30;
  constexpr double external_orbit_w = 60.0 * kPi / 180.0;

  BasePose2D pose{0.0, 0.0, 0.0};
  switch (trajectory_id) {
    case 0: pose.y = straight_speed * t; break;
    case 1: pose.x = diag_speed_x * t; pose.y = diag_speed_y * t; break;
    case 2: pose.theta = angular_speed * t; break;
    case 3:
      pose.x = external_orbit_r * (std::cos(external_orbit_w * t) - 1.0);
      pose.y = external_orbit_r * std::sin(external_orbit_w * t);
      pose.theta = external_orbit_w * t;
      break;
    case 4:
      pose.y = straight_speed * t;
      pose.theta = angular_speed * t;
      break;
    default: pose.y = straight_speed * t; break;
  }
  return pose;
}

std::string MasterPath::name(int trajectory_id) const
{
  switch (trajectory_id) {
    case 0: return "straight +Y";
    case 1: return "diagonal +X/+Y";
    case 2: return "in-place rotation";
    case 3: return "external-center orbit";
    case 4: return "rotate +Y";
    default: return "unknown(default straight +Y)";
  }
}

}  // namespace hexapod_sim
