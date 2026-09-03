#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gz_humanoid_walking
{

enum class ContactSupportMode
{
  DOUBLE_SUPPORT,
  LEFT_SUPPORT,
  RIGHT_SUPPORT
};

class PinocchioDynamicsWrapper
{
public:
  PinocchioDynamicsWrapper();
  ~PinocchioDynamicsWrapper();

  /// \brief Initialize Pinocchio model from SDF file
  /// \param[in] sdfPath Path to the robot SDF file (e.g., model.sdf)
  /// \param[in] rootLinkName Root link name (e.g., "PELVIS_S")
  /// \param[in] actuatedJointNames List of actuated joint names in order
  /// \return True if model loaded successfully
  bool Initialize(
      const std::string &sdfPath,
      const std::string &rootLinkName,
      const std::vector<std::string> &actuatedJointNames);

  /// \brief Compute contact-nullspace filtered gravity compensation torques
  /// \param[in] basePos World position of the floating base (pelvis)
  /// \param[in] baseRot World orientation quaternion of the floating base
  /// \param[in] actuatedQ Vector of actuated joint angles in configured order
  /// \param[in] supportMode Contact state (DOUBLE_SUPPORT, LEFT_SUPPORT, RIGHT_SUPPORT)
  /// \return Vector of gravity compensation torques for the actuated joints
  Eigen::VectorXd ComputeNullspaceGravityTorques(
      const Eigen::Vector3d &basePos,
      const Eigen::Quaterniond &baseRot,
      const Eigen::VectorXd &actuatedQ,
      ContactSupportMode supportMode = ContactSupportMode::DOUBLE_SUPPORT);

  /// \brief Compute the actuated joint-space mass matrix M_a in R^(nActuated x nActuated)
  /// \param[in] basePos World position of the floating base (pelvis)
  /// \param[in] baseRot World orientation quaternion of the floating base
  /// \param[in] actuatedQ Vector of actuated joint angles in configured order
  /// \return Symmetric positive-definite mass matrix M_a
  Eigen::MatrixXd ComputeActuatedMassMatrix(
      const Eigen::Vector3d &basePos,
      const Eigen::Quaterniond &baseRot,
      const Eigen::VectorXd &actuatedQ);

  /// \brief Get total robot mass in kg
  double TotalMass() const;

  /// \brief Check if wrapper is initialized
  bool IsInitialized() const;

private:
  struct Impl;
  std::unique_ptr<Impl> pimpl_;
};

} // namespace gz_humanoid_walking
