#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace hexapod_sim
{

enum class TrajectoryType : int
{
  Stationary = 0,
  StraightPositiveY = 1,
  StraightPositiveX = 2,
  DiagonalPositiveXPositiveY = 3,
  DiagonalNegativeXPositiveY = 4,
  ExternalCenterOrbitPositiveX = 5,
  ExternalCenterOrbitNegativeX = 6,
  InPlaceRotationClockwise = 7,
  StraightNegativeY = 8,
  StraightNegativeX = 9,
  DiagonalNegativeXNegativeY = 10,
  DiagonalPositiveXNegativeY = 11,
  ExternalCenterOrbitPositiveXReverse = 12,
  ExternalCenterOrbitNegativeXReverse = 13,
  InPlaceRotationCounterClockwise = 14
};

enum class Tripod
{
  A,
  B
};

enum class PullPhaseSpan
{
  HalfStep,
  FullStep
};

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kXHome = 0.110;
inline constexpr double kYHome = 0.0;
inline constexpr double kZHome = -0.050;
inline constexpr double kL1 = 0.0385;
inline constexpr double kL2 = 0.0700;
inline constexpr double kL3 = 0.1020;
inline constexpr std::array<std::size_t, 3> kTripodA = {0, 2, 4};
inline constexpr std::array<std::size_t, 3> kTripodB = {1, 3, 5};
inline constexpr Tripod opposite_tripod(Tripod tripod) { return tripod == Tripod::A ? Tripod::B : Tripod::A; }
inline constexpr char tripod_label(Tripod tripod) { return tripod == Tripod::A ? 'A' : 'B'; }
inline constexpr const std::array<std::size_t, 3> & tripod_legs(Tripod tripod) { return tripod == Tripod::A ? kTripodA : kTripodB; }
inline constexpr int required_peak_tip_limit_hits(PullPhaseSpan phase_span) { return phase_span == PullPhaseSpan::FullStep ? 2 : 1; }
inline constexpr int trajectory_type_id(TrajectoryType type) { return static_cast<int>(type); }
inline constexpr bool is_valid_trajectory_type_id(int id) { return id >= 0 && id <= 14; }

inline constexpr TrajectoryType trajectory_type_from_id(int id)
{
  switch (id) {
    case 0: return TrajectoryType::Stationary;
    case 1: return TrajectoryType::StraightPositiveY;
    case 2: return TrajectoryType::StraightPositiveX;
    case 3: return TrajectoryType::DiagonalPositiveXPositiveY;
    case 4: return TrajectoryType::DiagonalNegativeXPositiveY;
    case 5: return TrajectoryType::ExternalCenterOrbitPositiveX;
    case 6: return TrajectoryType::ExternalCenterOrbitNegativeX;
    case 7: return TrajectoryType::InPlaceRotationClockwise;
    case 8: return TrajectoryType::StraightNegativeY;
    case 9: return TrajectoryType::StraightNegativeX;
    case 10: return TrajectoryType::DiagonalNegativeXNegativeY;
    case 11: return TrajectoryType::DiagonalPositiveXNegativeY;
    case 12: return TrajectoryType::ExternalCenterOrbitPositiveXReverse;
    case 13: return TrajectoryType::ExternalCenterOrbitNegativeXReverse;
    case 14: return TrajectoryType::InPlaceRotationCounterClockwise;
    default: return TrajectoryType::Stationary;
  }
}

struct GaitConfig
{
  double discrete_step{0.02};
  double min_angle_deg{1.0};
  double limit_radius{0.05};
  double swing_height{0.02};
  TrajectoryType trajectory_type{TrajectoryType::Stationary};
  std::string action_name{"/joint_trajectory_controller/follow_joint_trajectory"};
  std::string trajectory_cmd_topic{"/trajectory_type"};
};

}  // namespace hexapod_sim
