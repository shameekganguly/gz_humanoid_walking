#include "LegIK.hh"
#include <algorithm>
#include <iostream>

namespace gz_humanoid_walking
{

LegIK::LegIK()
{
  thighLength_ = std::sqrt(thighOffsetX_ * thighOffsetX_ + thighOffsetZ_ * thighOffsetZ_);
  shinLength_ = std::sqrt(shinOffsetX_ * shinOffsetX_ + shinOffsetZ_ * shinOffsetZ_);
}

bool LegIK::SolveIK(
    LegSide /*_side*/,
    const Eigen::Vector3d &_footPos,
    const Eigen::Matrix3d &_footRot,
    LegJointAngles &_joints) const
{
  // 1. Vector from hip origin to ankle joint
  Eigen::Vector3d anklePos = _footPos - _footRot * Eigen::Vector3d(0, 0, ankleOffsetZ_);

  double D = anklePos.norm();
  double maxReach = thighLength_ + shinLength_ - 1e-4;
  double minReach = std::abs(thighLength_ - shinLength_) + 1e-4;

  if (D > maxReach || D < minReach)
  {
    return false;
  }

  // 2. Exact Knee Pitch via: A1*cos(q) + B1*sin(q) = D^2 - L1^2 - L2^2
  double A1 = 2.0 * (thighOffsetX_ * shinOffsetX_ + thighOffsetZ_ * shinOffsetZ_);
  double B1 = 2.0 * (thighOffsetX_ * shinOffsetZ_ - thighOffsetZ_ * shinOffsetX_);
  double C1 = D * D - (thighLength_ * thighLength_ + shinLength_ * shinLength_);

  double R1 = std::sqrt(A1 * A1 + B1 * B1);
  double delta1 = std::atan2(B1, A1);

  double cosTerm = std::clamp(C1 / R1, -1.0, 1.0);
  double acosTerm = std::acos(cosTerm);

  // Positive knee bending
  _joints.kneePitch = delta1 + acosTerm;

  // 3. Lower leg vector in zero-hip frame: v_leg = thighVec + R_knee * shinVec
  Eigen::Vector3d thighVec(thighOffsetX_, 0.0, thighOffsetZ_);
  Eigen::Vector3d shinVec(shinOffsetX_, 0.0, shinOffsetZ_);
  Eigen::Matrix3d R_knee_local = Eigen::AngleAxisd(_joints.kneePitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d v_leg = thighVec + R_knee_local * shinVec;

  // 4. Solve Hip Roll and Hip Pitch from R_y(q_hp) * R_x(q_hr) * v_leg = anklePos
  double vx = v_leg.x();
  double vz = v_leg.z();

  double sinRoll = std::clamp(-anklePos.y() / vz, -1.0, 1.0);
  _joints.hipRoll = std::asin(sinRoll);
  _joints.hipYaw = 0.0;

  double k1 = vx;
  double k2 = vz * std::cos(_joints.hipRoll);
  double xa = anklePos.x();
  double za = anklePos.z();

  _joints.hipPitch = std::atan2(k2 * xa - k1 * za, k1 * xa + k2 * za);

  // 5. Hip rotation matrix
  Eigen::Matrix3d R_hip = (Eigen::AngleAxisd(_joints.hipPitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(_joints.hipRoll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(_joints.hipYaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  Eigen::Matrix3d R_knee = R_hip * R_knee_local;

  // 6. Ankle orientation relative to lower leg:
  // R_foot = R_knee * R_x(ankleRoll) * R_y(anklePitch)
  // => R_ankle = R_knee^T * R_foot
  Eigen::Matrix3d R_ankle = R_knee.transpose() * _footRot;

  _joints.ankleRoll = std::atan2(R_ankle(2, 1), R_ankle(1, 1));
  _joints.anklePitch = std::atan2(R_ankle(0, 2), R_ankle(0, 0));

  return true;
}

void LegIK::ForwardKinematics(
    LegSide /*_side*/,
    const LegJointAngles &_joints,
    Eigen::Vector3d &_footPos,
    Eigen::Matrix3d &_footRot) const
{
  Eigen::Matrix3d R_hip = (Eigen::AngleAxisd(_joints.hipPitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(_joints.hipRoll, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(_joints.hipYaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();

  Eigen::Vector3d thighVec(thighOffsetX_, 0, thighOffsetZ_);
  Eigen::Vector3d kneePos = R_hip * thighVec;

  Eigen::Matrix3d R_knee = R_hip * Eigen::AngleAxisd(_joints.kneePitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d shinVec(shinOffsetX_, 0, shinOffsetZ_);
  Eigen::Vector3d anklePos = kneePos + R_knee * shinVec;

  Eigen::Matrix3d R_ankle = R_knee * (Eigen::AngleAxisd(_joints.ankleRoll, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(_joints.anklePitch, Eigen::Vector3d::UnitY())).toRotationMatrix();

  _footRot = R_ankle;
  _footPos = anklePos + _footRot * Eigen::Vector3d(0, 0, ankleOffsetZ_);
}

} // namespace gz_humanoid_walking
