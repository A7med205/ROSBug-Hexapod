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
  constexpr double linear_speed_x = 0.12;
  constexpr double linear_speed_y = 0.12;
  constexpr double diagonal_speed = 0.12;
  constexpr double self_angular_speed = 0.75;
  constexpr double external_orbit_r = 0.30;
  constexpr double external_orbit_w = 0.75;

  if (trajectory_type != active_trajectory_type_) {
    active_trajectory_type_ = trajectory_type;
    trajectory_start_global_time_ = t;
  }
  if (trajectory_type == TrajectoryType::Stationary) return {0.0, 0.0, 0.0};
  const double local_t = std::max(0.0, t - trajectory_start_global_time_);

  BasePose2D pose{0.0, 0.0, 0.0};
  switch (trajectory_type) {
    case TrajectoryType::Stationary:
      break;
    case TrajectoryType::StraightPositiveY:
      pose.y = linear_speed_y * local_t;
      break;
    case TrajectoryType::StraightPositiveX:
      pose.x = linear_speed_x * local_t;
      break;
    case TrajectoryType::DiagonalPositiveXPositiveY:
      pose.x = diagonal_speed * local_t;
      pose.y = diagonal_speed * local_t;
      break;
    case TrajectoryType::DiagonalNegativeXPositiveY:
      pose.x = -diagonal_speed * local_t;
      pose.y = diagonal_speed * local_t;
      break;
    case TrajectoryType::ExternalCenterOrbitPositiveX: {
      const double phi = kPi + external_orbit_w * local_t;
      pose.x = external_orbit_r + external_orbit_r * std::cos(phi);
      pose.y = external_orbit_r * std::sin(phi);
      pose.theta = external_orbit_w * local_t;
      break;
    }
    case TrajectoryType::ExternalCenterOrbitNegativeX: {
      const double phi = external_orbit_w * local_t;
      pose.x = -external_orbit_r + external_orbit_r * std::cos(phi);
      pose.y = external_orbit_r * std::sin(phi);
      pose.theta = external_orbit_w * local_t;
      break;
    }
    case TrajectoryType::InPlaceRotationClockwise:
      pose.theta = -self_angular_speed * local_t;
      break;
    case TrajectoryType::StraightNegativeY:
      pose.y = -linear_speed_y * local_t;
      break;
    case TrajectoryType::StraightNegativeX:
      pose.x = -linear_speed_x * local_t;
      break;
    case TrajectoryType::DiagonalNegativeXNegativeY:
      pose.x = -diagonal_speed * local_t;
      pose.y = -diagonal_speed * local_t;
      break;
    case TrajectoryType::DiagonalPositiveXNegativeY:
      pose.x = diagonal_speed * local_t;
      pose.y = -diagonal_speed * local_t;
      break;
    case TrajectoryType::ExternalCenterOrbitPositiveXReverse: {
      const double phi = kPi - external_orbit_w * local_t;
      pose.x = external_orbit_r + external_orbit_r * std::cos(phi);
      pose.y = external_orbit_r * std::sin(phi);
      pose.theta = -external_orbit_w * local_t;
      break;
    }
    case TrajectoryType::ExternalCenterOrbitNegativeXReverse: {
      const double phi = -external_orbit_w * local_t;
      pose.x = -external_orbit_r + external_orbit_r * std::cos(phi);
      pose.y = external_orbit_r * std::sin(phi);
      pose.theta = -external_orbit_w * local_t;
      break;
    }
    case TrajectoryType::InPlaceRotationCounterClockwise:
      pose.theta = self_angular_speed * local_t;
      break;
    case TrajectoryType::RotateAndTranslatePositiveY:
      pose.y = linear_speed_y * local_t;
      pose.theta = self_angular_speed * local_t;
      break;
    default:
      break;
  }
  return pose;
}

// Returns a compact name used in status logs.
std::string MasterPath::name(TrajectoryType trajectory_type) const
{
  switch (trajectory_type) {
    case TrajectoryType::Stationary: return "stationary";
    case TrajectoryType::StraightPositiveY: return "line +Y";
    case TrajectoryType::StraightPositiveX: return "line +X";
    case TrajectoryType::DiagonalPositiveXPositiveY: return "diagonal +X/+Y";
    case TrajectoryType::DiagonalNegativeXPositiveY: return "diagonal -X/+Y";
    case TrajectoryType::ExternalCenterOrbitPositiveX: return "orbit center +X";
    case TrajectoryType::ExternalCenterOrbitNegativeX: return "orbit center -X";
    case TrajectoryType::InPlaceRotationClockwise: return "self rotation CW";
    case TrajectoryType::StraightNegativeY: return "line -Y";
    case TrajectoryType::StraightNegativeX: return "line -X";
    case TrajectoryType::DiagonalNegativeXNegativeY: return "diagonal -X/-Y";
    case TrajectoryType::DiagonalPositiveXNegativeY: return "diagonal +X/-Y";
    case TrajectoryType::ExternalCenterOrbitPositiveXReverse: return "orbit reverse center +X";
    case TrajectoryType::ExternalCenterOrbitNegativeXReverse: return "orbit reverse center -X";
    case TrajectoryType::InPlaceRotationCounterClockwise: return "self rotation CCW";
    case TrajectoryType::RotateAndTranslatePositiveY: return "rotate +Y";
    default: return "unknown";
  }
}

}  // namespace hexapod_sim
