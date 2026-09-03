#ifndef GZ_HUMANOID_WALKING_LEGIK_HH_
#define GZ_HUMANOID_WALKING_LEGIK_HH_

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace gz_humanoid_walking
{

enum class LegSide
{
  LEFT,
  RIGHT
};

struct LegJointAngles
{
  double hipPitch{0.0};
  double hipRoll{0.0};
  double hipYaw{0.0};
  double kneePitch{0.0};
  double ankleRoll{0.0};
  double anklePitch{0.0};

  std::array<double, 6> ToArray() const
  {
    return {hipPitch, hipRoll, hipYaw, kneePitch, ankleRoll, anklePitch};
  }
};

class LegIK
{
public:
  LegIK();

  /// \brief Compute 6-DoF inverse kinematics for one leg
  /// \param[in] _side Left or Right leg
  /// \param[in] _footPos Target foot position in hip base frame [m]
  /// \param[in] _footRot Target foot orientation in hip base frame
  /// \param[out] _joints Computed joint angles [rad]
  /// \return True if kinematic solution is found within reachable workspace
  bool SolveIK(
      LegSide _side,
      const Eigen::Vector3d &_footPos,
      const Eigen::Matrix3d &_footRot,
      LegJointAngles &_joints) const;

  /// \brief Compute forward kinematics for one leg
  /// \param[in] _side Left or Right leg
  /// \param[in] _joints Joint angles [rad]
  /// \param[out] _footPos Resulting foot position [m]
  /// \param[out] _footRot Resulting foot orientation matrix
  void ForwardKinematics(
      LegSide _side,
      const LegJointAngles &_joints,
      Eigen::Vector3d &_footPos,
      Eigen::Matrix3d &_footRot) const;

  double ThighLength() const { return thighLength_; }
  double ShinLength() const { return shinLength_; }
  double AnkleOffsetZ() const { return ankleOffsetZ_; }

private:
  // Kinematic parameters matching JVRC-1 humanoid model
  double thighOffsetX_{-0.02};
  double thighOffsetZ_{-0.389};
  double shinOffsetX_{0.04};
  double shinOffsetZ_{-0.357};
  double ankleOffsetZ_{-0.07};

  double thighLength_{0.38951}; // sqrt((-0.02)^2 + (-0.389)^2)
  double shinLength_{0.35923};  // sqrt((0.04)^2 + (-0.357)^2)
  double thighKneeAngleOffset_{0.05138}; // atan2(0.02, 0.389)
  double shinKneeAngleOffset_{0.11158};  // atan2(0.04, 0.357)
};

} // namespace gz_humanoid_walking

#endif // GZ_HUMANOID_WALKING_LEGIK_HH_
