#pragma once

#include <array>
#include <memory>
#include <string>
#include <string_view>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "types.hh"

namespace gz_humanoid_walking
{

using ContactSupportMode = SupportState;

template <std::size_t NumJoints = 12>
class PinocchioDynamicsWrapper
{
public:
  using VectorJoints = Eigen::Matrix<double, NumJoints, 1>;
  using MatrixJoints = Eigen::Matrix<double, NumJoints, NumJoints>;

  PinocchioDynamicsWrapper(
      const std::string &sdfPath,
      const std::string &rootLinkName,
      const std::array<std::string_view, NumJoints> &actuatedJointNames);
  ~PinocchioDynamicsWrapper();

  /// \brief Compute contact-nullspace filtered gravity compensation torques
  /// \param[in] basePos World position of the floating base (pelvis)
  /// \param[in] baseRot World orientation quaternion of the floating base
  /// \param[in] actuatedQ Vector of actuated joint angles in configured order
  /// \param[in] supportMode Contact state (DOUBLE_SUPPORT, LEFT_SUPPORT, RIGHT_SUPPORT)
  /// \return Vector of gravity compensation torques for the actuated joints
  VectorJoints ComputeNullspaceGravityTorques(
      const Eigen::Vector3d &basePos,
      const Eigen::Quaterniond &baseRot,
      const VectorJoints &actuatedQ,
      SupportState supportMode = SupportState::DOUBLE_SUPPORT);

  /// \brief Compute the actuated joint-space mass matrix M_a in R^(NumJoints x NumJoints)
  /// \param[in] basePos World position of the floating base (pelvis)
  /// \param[in] baseRot World orientation quaternion of the floating base
  /// \param[in] actuatedQ Vector of actuated joint angles in configured order
  /// \return Symmetric positive-definite mass matrix M_a
  MatrixJoints ComputeActuatedMassMatrix(
      const Eigen::Vector3d &basePos,
      const Eigen::Quaterniond &baseRot,
      const VectorJoints &actuatedQ);

  /// \brief Get total robot mass in kg
  double TotalMass() const;

  /// \brief Check if wrapper is initialized
  bool IsInitialized() const;

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};

} // namespace gz_humanoid_walking
