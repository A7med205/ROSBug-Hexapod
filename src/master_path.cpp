#include "hexapod_sim/master_path.hpp"

#include <algorithm>
#include <cmath>

#include "hexapod_sim/gait_config.hpp"

namespace hexapod_sim
{

MasterPath::MasterPath(const GaitConfig & config)
: config_(config)
{}

// Returns base world pose for the selected trajectory at time t.
BasePose2D MasterPath::pose(double t, TrajectoryType trajectory_type) const
{
  (void)config_;
  constexpr double straight_speed = 1.2;
  constexpr double diag_speed_x = 0.15, diag_speed_y = 0.20;
  constexpr double angular_speed = 180.0 * kPi / 180.0;
  constexpr double external_orbit_r = 0.30;
  constexpr double external_orbit_w = 60.0 * kPi / 180.0;

  if (trajectory_type != active_trajectory_type_) {
    active_trajectory_type_ = trajectory_type;
    trajectory_start_global_time_ = t;
  }
  if (trajectory_type == TrajectoryType::Stationary) return {0.0, 0.0, 0.0};
  const double local_t = std::max(0.0, t - trajectory_start_global_time_);

  BasePose2D pose{0.0, 0.0, 0.0};
  switch (trajectory_type) {
    case TrajectoryType::StraightY: pose.y = straight_speed * local_t; break;
    case TrajectoryType::DiagonalXY: pose.x = diag_speed_x * local_t; pose.y = diag_speed_y * local_t; break;
    case TrajectoryType::InPlaceRotation: pose.theta = angular_speed * local_t; break;
    case TrajectoryType::ExternalCenterOrbit:
      pose.x = external_orbit_r * (std::cos(external_orbit_w * local_t) - 1.0);
      pose.y = external_orbit_r * std::sin(external_orbit_w * local_t);
      pose.theta = external_orbit_w * local_t;
      break;
    case TrajectoryType::RotateAndTranslateY:
      pose.y = straight_speed * local_t;
      pose.theta = angular_speed * local_t;
      break;
    case TrajectoryType::Stationary:
      break;
    default: pose.y = straight_speed * t; break;
  }
  return pose;
}

// Returns a compact name used in status logs.
std::string MasterPath::name(TrajectoryType trajectory_type) const
{
  switch (trajectory_type) {
    case TrajectoryType::StraightY: return "straight +Y";
    case TrajectoryType::DiagonalXY: return "diagonal +X/+Y";
    case TrajectoryType::InPlaceRotation: return "in-place rotation";
    case TrajectoryType::ExternalCenterOrbit: return "external-center orbit";
    case TrajectoryType::RotateAndTranslateY: return "rotate +Y";
    case TrajectoryType::Stationary: return "stationary";
    default: return "unknown(default straight +Y)";
  }
}

}  // namespace hexapod_sim
