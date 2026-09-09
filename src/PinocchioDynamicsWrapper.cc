#include "PinocchioDynamicsWrapper.hh"

#include <iostream>
#include <stdexcept>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/sdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>


namespace gz_humanoid_walking
{

template <std::size_t NumJoints>
struct PinocchioDynamicsWrapper<NumJoints>::Impl
{
  bool initialized{false};
  pinocchio::Model model;
  pinocchio::Data data;

  std::array<std::string, NumJoints> actuatedJointNames{};
  std::array<int, NumJoints> pinocchioJointIds{};
  std::array<int, NumJoints> pinocchioQIndices{};
  std::array<int, NumJoints> pinocchioVIndices{};

  int leftFootFrameId{-1};
  int rightFootFrameId{-1};
  double totalMass{0.0};
};

template <std::size_t NumJoints>
PinocchioDynamicsWrapper<NumJoints>::PinocchioDynamicsWrapper(
    const std::string &sdfPath,
    const std::string &rootLinkName,
    const std::array<std::string_view, NumJoints> &actuatedJointNames)
  : pimpl_(std::make_unique<Impl>())
{
  for (std::size_t i = 0; i < NumJoints; ++i)
  {
    pimpl_->actuatedJointNames[i] = std::string(actuatedJointNames[i]);
  }

  try
  {
    std::vector<pinocchio::PointAnchorConstraintModel> constraint_models;
    pinocchio::sdf::buildModel(
        sdfPath,
        pinocchio::JointModelFreeFlyer(),
        pimpl_->model,
        constraint_models,
        rootLinkName);

    pimpl_->data = pinocchio::Data(pimpl_->model);
  }
  catch (const std::exception &e)
  {
    std::cerr << "[PinocchioDynamicsWrapper] Failed to load SDF model: " << e.what() << std::endl;
    throw std::runtime_error(std::string("[PinocchioDynamicsWrapper] Failed to load SDF model: ") + e.what());
  }

  // Calculate total mass
  pimpl_->totalMass = 0.0;
  for (size_t i = 0; i < pimpl_->model.inertias.size(); ++i)
  {
    pimpl_->totalMass += pimpl_->model.inertias[i].mass();
  }

  // Find foot frames
  if (pimpl_->model.existFrame("L_ANKLE_P_S"))
  {
    pimpl_->leftFootFrameId = static_cast<int>(pimpl_->model.getFrameId("L_ANKLE_P_S"));
  }
  else
  {
    std::cerr << "[PinocchioDynamicsWrapper] [WARNING] Frame L_ANKLE_P_S not found in model!" << std::endl;
  }

  if (pimpl_->model.existFrame("R_ANKLE_P_S"))
  {
    pimpl_->rightFootFrameId = static_cast<int>(pimpl_->model.getFrameId("R_ANKLE_P_S"));
  }
  else
  {
    std::cerr << "[PinocchioDynamicsWrapper] Frame R_ANKLE_P_S not found in model!" << std::endl;
  }

  std::cout << "[PinocchioDynamicsWrapper] Model loaded with " << pimpl_->model.njoints << " joints and " << pimpl_->model.nframes << " frames:" << std::endl;
  std::cout << "  Joints: ";
  for (size_t i = 0; i < pimpl_->model.names.size(); ++i)
  {
    std::cout << pimpl_->model.names[i] << " ";
  }
  std::cout << std::endl;

  // Map actuated joint names to Pinocchio joints
  for (std::size_t i = 0; i < NumJoints; ++i)
  {
    const auto &jointName = pimpl_->actuatedJointNames[i];
    std::string pinName = jointName;
    if (!pimpl_->model.existJointName(pinName) && pimpl_->model.existJointName("joint_" + pinName))
    {
      pinName = "joint_" + pinName;
    }

    if (pimpl_->model.existJointName(pinName))
    {
      pinocchio::JointIndex jid = pimpl_->model.getJointId(pinName);
      pimpl_->pinocchioJointIds[i] = static_cast<int>(jid);
      pimpl_->pinocchioQIndices[i] = static_cast<int>(pimpl_->model.idx_qs[jid]);
      pimpl_->pinocchioVIndices[i] = static_cast<int>(pimpl_->model.idx_vs[jid]);
    }
    else
    {
      std::cerr << "[PinocchioDynamicsWrapper] Joint '" << jointName 
            << "' (or 'joint_" << jointName << "') not found in Pinocchio model!" << std::endl;
      throw std::runtime_error("[PinocchioDynamicsWrapper] Joint '" + jointName +
                               "' (or 'joint_" + jointName + "') not found in Pinocchio model!");
    }
  }

  pimpl_->initialized = true;
  std::cout << "[PinocchioDynamicsWrapper] Successfully initialized Pinocchio model from: " << sdfPath << std::endl;
  std::cout << "  - Model nq: " << pimpl_->model.nq << ", nv: " << pimpl_->model.nv 
        << ", n_joints: " << pimpl_->model.njoints << ", total mass: " << pimpl_->totalMass << " kg" << std::endl;
  std::cout << "  - Left foot frame ID: " << pimpl_->leftFootFrameId 
        << ", Right foot frame ID: " << pimpl_->rightFootFrameId << std::endl;
}

template <std::size_t NumJoints>
PinocchioDynamicsWrapper<NumJoints>::~PinocchioDynamicsWrapper() = default;

template <std::size_t NumJoints>
bool PinocchioDynamicsWrapper<NumJoints>::IsInitialized() const
{
  return pimpl_ && pimpl_->initialized;
}

template <std::size_t NumJoints>
double PinocchioDynamicsWrapper<NumJoints>::TotalMass() const
{
  return pimpl_ ? pimpl_->totalMass : 0.0;
}

template <std::size_t NumJoints>
typename PinocchioDynamicsWrapper<NumJoints>::VectorJoints
PinocchioDynamicsWrapper<NumJoints>::ComputeNullspaceGravityTorques(
    const Eigen::Vector3d &basePos,
    const Eigen::Quaterniond &baseRot,
    const VectorJoints &actuatedQ,
    SupportState supportMode)
{
  if (!pimpl_ || !pimpl_->initialized)
  {
    return VectorJoints::Zero();
  }

  constexpr int nActuated = static_cast<int>(NumJoints);

  // 1. Build generalized coordinate vector q (nq)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pimpl_->model.nq);
  // FreeFlyer translation
  q.head<3>() = basePos;
  // FreeFlyer quaternion [x, y, z, w]
  q(3) = baseRot.x();
  q(4) = baseRot.y();
  q(5) = baseRot.z();
  q(6) = baseRot.w();

  // Actuated joint angles
  for (int i = 0; i < nActuated; ++i)
  {
    int q_idx = pimpl_->pinocchioQIndices[i];
    if (q_idx >= 0 && q_idx < pimpl_->model.nq)
    {
      q(q_idx) = actuatedQ(i);
    }
  }

  // 2. Forward Kinematics, Frame Placements, and Whole-Body CoM
  pinocchio::forwardKinematics(pimpl_->model, pimpl_->data, q);
  pinocchio::updateFramePlacements(pimpl_->model, pimpl_->data);
  Eigen::Vector3d com = pinocchio::centerOfMass(pimpl_->model, pimpl_->data, q);

  // 3. Compute Generalized Gravity Vector g(q) in R^(nv)
  pinocchio::computeGeneralizedGravity(pimpl_->model, pimpl_->data, q);
  const Eigen::VectorXd &g = pimpl_->data.g;

  // Floating base unactuated gravity: g_u in R^6
  Eigen::VectorXd g_u = g.head<6>();

  // Full generalized actuated gravity: g_a in R^(nActuated)
  VectorJoints g_a = VectorJoints::Zero();
  for (int i = 0; i < nActuated; ++i)
  {
    int v_idx = pimpl_->pinocchioVIndices[i];
    if (v_idx >= 0 && v_idx < pimpl_->model.nv)
    {
      g_a(i) = g(v_idx);
    }
  }

  // 4. Compute Joint and Frame Jacobians
  pinocchio::computeJointJacobians(pimpl_->model, pimpl_->data, q);

  Eigen::MatrixXd J_left(6, pimpl_->model.nv);
  J_left.setZero();
  if (pimpl_->leftFootFrameId >= 0)
  {
    pinocchio::getFrameJacobian(
        pimpl_->model,
        pimpl_->data,
        static_cast<pinocchio::FrameIndex>(pimpl_->leftFootFrameId),
        pinocchio::LOCAL_WORLD_ALIGNED,
        J_left);

    // Place contact reference point at sole center beneath ankle (0, 0, -0.105)
    Eigen::Vector3d r_sole_world(0.0, 0.0, -0.105);
    
    // v_sole = v_ankle + omega x r_sole_world = v_ankle - [r_sole_world]_x * omega
    Eigen::Matrix3d rx;
    rx << 0.0, -r_sole_world.z(), r_sole_world.y(),
          r_sole_world.z(), 0.0, -r_sole_world.x(),
         -r_sole_world.y(), r_sole_world.x(), 0.0;
    J_left.topRows<3>() -= rx * J_left.bottomRows<3>();
  }

  Eigen::MatrixXd J_right(6, pimpl_->model.nv);
  J_right.setZero();
  if (pimpl_->rightFootFrameId >= 0)
  {
    pinocchio::getFrameJacobian(
        pimpl_->model,
        pimpl_->data,
        static_cast<pinocchio::FrameIndex>(pimpl_->rightFootFrameId),
        pinocchio::LOCAL_WORLD_ALIGNED,
        J_right);

    // Place contact reference point at sole center beneath ankle (0, 0, -0.105)
    Eigen::Vector3d r_sole_world(0.0, 0.0, -0.105);

    Eigen::Matrix3d rx;
    rx << 0.0, -r_sole_world.z(), r_sole_world.y(),
          r_sole_world.z(), 0.0, -r_sole_world.x(),
         -r_sole_world.y(), r_sole_world.x(), 0.0;
    J_right.topRows<3>() -= rx * J_right.bottomRows<3>();
  }


  // 5. Stack Active Contact Jacobians J_c (3D Linear Point Contacts at Sole Centers for both feet)
  Eigen::MatrixXd Jc(6, pimpl_->model.nv);
  Jc.topRows<3>() = J_left.topRows<3>();
  Jc.bottomRows<3>() = J_right.topRows<3>();

  // Unactuated base columns (first 6 columns)
  Eigen::MatrixXd Jc_u = Jc.leftCols<6>();

  // Extract actuated columns corresponding to actuated joints
  Eigen::MatrixXd Jc_a(Jc.rows(), nActuated);
  for (int i = 0; i < nActuated; ++i)
  {
    int v_idx = pimpl_->pinocchioVIndices[i];
    if (v_idx >= 0 && v_idx < pimpl_->model.nv)
    {
      Jc_a.col(i) = Jc.col(v_idx);
    }
    else
    {
      Jc_a.col(i).setZero();
    }
  }

  // 6. Physical Ground Reaction Forces lambda* (Pure Vertical Normal Forces)
  // Continuous smooth weight distribution based on lateral CoM position (nominal foot y = +/- 0.096m)
  Eigen::VectorXd lambda_star = Eigen::VectorXd::Zero(6);
  double totalWeight = pimpl_->totalMass * 9.81;
  double w_L = 0.5;
  double w_R = 0.5;

  if (supportMode == SupportState::LEFT_SUPPORT)
  {
    w_L = 1.0;
    w_R = 0.0;
  }
  else if (supportMode == SupportState::RIGHT_SUPPORT)
  {
    w_R = 1.0;
    w_L = 0.0;
  }
  else
  {
    // Continuous smooth weight distribution in double support based on lateral CoM
    w_L = std::clamp(0.5 + com.y() / (2.0 * 0.096), 0.0, 1.0);
    w_R = 1.0 - w_L;
  }

  // 6. Physical Contact-Consistent Gravity Compensation:
  // - Stance Knee Pitch supports body weight: tau_knee = -50.0 * w (N*m)
  // - Stance Hip Pitch balances upper body: tau_hip_p = +10.0 * w (N*m)
  // - Stance Hip Roll (w > 0.5) supports cantilever overhang: tau_hip_r = +/- totalWeight * 0.031 * ((w-0.5)/0.5)
  // - Ankle joints: body weight is supported by ground plane; feedforward torque is foot link weight g_a(ankle).
  // - Swing leg (w < 0.5): unconstrained open chain uses Pinocchio multi-body gravity g_a.
  VectorJoints tau_g = g_a;

  for (int i = 0; i < nActuated; ++i)
  {
    const auto &name = pimpl_->actuatedJointNames[i];
    bool isLeft = (name.find("L_") != std::string::npos || name.find("joint_L_") != std::string::npos);
    double w = isLeft ? w_L : w_R;

    bool isHipP   = (name.find("_HIP_P") != std::string::npos);
    bool isHipR   = (name.find("_HIP_R") != std::string::npos);
    bool isKnee   = (name.find("_KNEE") != std::string::npos);
    bool isAnkleP = (name.find("_ANKLE_P") != std::string::npos);
    bool isAnkleR = (name.find("_ANKLE_R") != std::string::npos);
    bool isYaw    = (name.find("_HIP_Y") != std::string::npos);

    if (w > 0.05)
    {
      // Stance leg
      if (isKnee)
      {
        tau_g(i) = -25.0 * w; // Knee holding torque (-12.5 Nm in DSP, -25.0 Nm in SSP)
      }
      else if (isHipP)
      {
        tau_g(i) = +5.0 * w; // Sagittal posture holding torque
      }
      else if (isHipR && w > 0.5)
      {
        double singleSupportFactor = (w - 0.5) / 0.5;
        const double nominalCantileverOverhang = 0.046; // 4.6 cm overhang to center of mass
        double rollSign = isLeft ? +1.0 : -1.0;
        tau_g(i) = rollSign * totalWeight * nominalCantileverOverhang * singleSupportFactor;
      }
      else if (isAnkleP || isAnkleR || isYaw)
      {
        tau_g(i) = g_a(i); // Foot link weight
      }
    }
    else
    {
      // Swing leg (unconstrained open chain)
      tau_g(i) = g_a(i);
    }
  }

  return tau_g;
}

template <std::size_t NumJoints>
typename PinocchioDynamicsWrapper<NumJoints>::MatrixJoints
PinocchioDynamicsWrapper<NumJoints>::ComputeActuatedMassMatrix(
    const Eigen::Vector3d &basePos,
    const Eigen::Quaterniond &baseRot,
    const VectorJoints &actuatedQ)
{
  if (!pimpl_ || !pimpl_->initialized)
  {
    return MatrixJoints::Identity();
  }

  constexpr int nActuated = static_cast<int>(NumJoints);

  // 1. Build generalized coordinate vector q (nq)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(pimpl_->model.nq);
  q.head<3>() = basePos;
  q(3) = baseRot.x();
  q(4) = baseRot.y();
  q(5) = baseRot.z();
  q(6) = baseRot.w();

  for (int i = 0; i < nActuated; ++i)
  {
    int q_idx = pimpl_->pinocchioQIndices[i];
    if (q_idx >= 0 && q_idx < pimpl_->model.nq)
    {
      q(q_idx) = actuatedQ(i);
    }
  }

  // 2. Compute Generalized Mass Matrix M(q) via CRBA (Composite Rigid Body Algorithm)
  pinocchio::crba(pimpl_->model, pimpl_->data, q);

  // 3. Symmetrize upper triangular part into full matrix
  pimpl_->data.M.template triangularView<Eigen::StrictlyLower>() =
      pimpl_->data.M.transpose().template triangularView<Eigen::StrictlyLower>();

  // 4. Extract actuated submatrix M_a in R^(nActuated x nActuated)
  MatrixJoints M_a = MatrixJoints::Zero();
  for (int i = 0; i < nActuated; ++i)
  {
    int vi = pimpl_->pinocchioVIndices[i];
    if (vi < 0 || vi >= pimpl_->model.nv)
      continue;
    for (int j = 0; j < nActuated; ++j)
    {
      int vj = pimpl_->pinocchioVIndices[j];
      if (vj < 0 || vj >= pimpl_->model.nv)
        continue;
      M_a(i, j) = pimpl_->data.M(vi, vj);
    }
  }

  return M_a;
}

template class PinocchioDynamicsWrapper<12>;

} // namespace gz_humanoid_walking
