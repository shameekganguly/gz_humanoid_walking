#include "WholeBodyQPController.hh"
#include <eiquadprog/eiquadprog.hpp>
#include <iostream>
#include <algorithm>

namespace gz_humanoid_walking
{

template <std::size_t NumJoints>
WholeBodyQPController<NumJoints>::WholeBodyQPController(
    const std::array<std::string_view, NumJoints> &_jointNames,
    int _verbosity)
    : jointNames_(_jointNames),
      verbosity_(_verbosity)
{
  for (std::size_t i = 0; i < NumJoints; ++i)
  {
    jointIndexMap_[jointNames_[i]] = static_cast<int>(i);
  }

  qNominal_.setZero();
  qMin_.setConstant(-2.5);
  qMax_.setConstant(2.5);
  qdMax_.setConstant(6.0); // 6 rad/s max velocity

  // Set specific leg joint limits
  for (std::size_t i = 0; i < NumJoints; ++i)
  {
    const auto &name = jointNames_[i];
    if (name == "joint_R_KNEE" || name == "joint_L_KNEE")
    {
      qMin_(i) = 0.0;
      qMax_(i) = 2.6;
    }
  }
}

template <std::size_t NumJoints>
bool WholeBodyQPController<NumJoints>::Solve(
    const Eigen::Vector3d &_comDes,
    const Eigen::Vector3d &/*_comVelDes*/,
    const Eigen::Vector3d &_leftFootDes,
    const Eigen::Matrix3d &_leftFootRot,
    const Eigen::Vector3d &_rightFootDes,
    const Eigen::Matrix3d &_rightFootRot,
    SupportState _support,
    const Eigen::VectorXd &_currentQ,
    double _dt,
    Eigen::Matrix<double, NumJoints, 1> &_targetQ,
    Eigen::Matrix<double, NumJoints, 1> &_targetQd)
{
  constexpr int numJoints = static_cast<int>(NumJoints);
  if (numJoints == 0 || _currentQ.size() != numJoints)
  {
    return false;
  }

  // 1. Compute Reference Leg Joint Targets via Kinematics in Body Frame
  const double torsoForwardOffset = 0.0;

  // Use foot orientation as base heading orientation
  Eigen::Matrix3d R_base = _leftFootRot;
  Eigen::Vector3d delta_left_body = R_base.transpose() * (_leftFootDes - _comDes);
  Eigen::Vector3d delta_right_body = R_base.transpose() * (_rightFootDes - _comDes);

  // Stance leg nominal pelvis frame offsets
  Eigen::Vector3d leftFootInHip(delta_left_body.x() - torsoForwardOffset,
                               delta_left_body.y() - 0.096,
                               delta_left_body.z());

  Eigen::Vector3d rightFootInHip(delta_right_body.x() - torsoForwardOffset,
                                delta_right_body.y() + 0.096,
                                delta_right_body.z());

  Eigen::Matrix3d R_left_rel = R_base.transpose() * _leftFootRot;
  Eigen::Matrix3d R_right_rel = R_base.transpose() * _rightFootRot;

  LegJointAngles leftLegJoints, rightLegJoints;
  legIK_.SolveIK(LegSide::LEFT, leftFootInHip, R_left_rel, leftLegJoints);
  legIK_.SolveIK(LegSide::RIGHT, rightFootInHip, R_right_rel, rightLegJoints);

  Eigen::Matrix<double, NumJoints, 1> qRef = qNominal_;

  // Map leg joint references
  auto setRef = [&](std::string_view name, double val) {
    auto it = jointIndexMap_.find(name);
    if (it != jointIndexMap_.end())
    {
      qRef(it->second) = val;
    }
  };

  setRef("joint_R_HIP_P", rightLegJoints.hipPitch);
  setRef("joint_R_HIP_R", rightLegJoints.hipRoll);
  setRef("joint_R_HIP_Y", rightLegJoints.hipYaw);
  setRef("joint_R_KNEE", rightLegJoints.kneePitch);
  setRef("joint_R_ANKLE_R", rightLegJoints.ankleRoll);
  setRef("joint_R_ANKLE_P", rightLegJoints.anklePitch);

  setRef("joint_L_HIP_P", leftLegJoints.hipPitch);
  setRef("joint_L_HIP_R", leftLegJoints.hipRoll);
  setRef("joint_L_HIP_Y", leftLegJoints.hipYaw);
  setRef("joint_L_KNEE", leftLegJoints.kneePitch);
  setRef("joint_L_ANKLE_R", leftLegJoints.ankleRoll);
  setRef("joint_L_ANKLE_P", leftLegJoints.anklePitch);

  // 2. Set up Whole-Body QP: min 0.5 * qd^T * G * qd + g0^T * qd
  double Kp = 20.0;
  Eigen::Matrix<double, NumJoints, 1> qdDes = Kp * (qRef - _currentQ);

  // Weight matrix
  Eigen::Matrix<double, NumJoints, 1> weights = Eigen::Matrix<double, NumJoints, 1>::Constant(100.0);
  Eigen::MatrixXd G = weights.asDiagonal();
  G.diagonal().array() += 0.1; // Regularization

  Eigen::VectorXd g0 = -G * qdDes;

  // 3. Inequality Constraints: CI^T * qd + ci0 >= 0
  int numConstraints = 2 * numJoints;
  Eigen::MatrixXd CI(numJoints, numConstraints);
  Eigen::VectorXd ci0(numConstraints);

  for (int i = 0; i < numJoints; ++i)
  {
    double lowerBound = std::max(-qdMax_(i), (qMin_(i) - _currentQ(i)) / _dt);
    double upperBound = std::min( qdMax_(i), (qMax_(i) - _currentQ(i)) / _dt);
    if (upperBound < lowerBound)
    {
      upperBound = lowerBound;
    }

    // qd(i) >= lowerBound  <=>  +1 * qd(i) - lowerBound >= 0
    CI.col(i) = Eigen::VectorXd::Unit(numJoints, i);
    ci0(i) = -lowerBound;

    // qd(i) <= upperBound  <=>  -1 * qd(i) + upperBound >= 0
    CI.col(numJoints + i) = -Eigen::VectorXd::Unit(numJoints, i);
    ci0(numJoints + i) = upperBound;
  }

  // 4. Solve via eiquadprog
  Eigen::MatrixXd CE(numJoints, 0);
  Eigen::VectorXd ce0(0);

  Eigen::VectorXd qdSol = Eigen::VectorXd::Zero(numJoints);
  Eigen::VectorXi activeSet;
  size_t activeSetSize = 0;

  double cost = eiquadprog::solvers::solve_quadprog(
      G, g0, CE, ce0, CI, ci0, qdSol, activeSet, activeSetSize);

  bool qpSuccess = !std::isinf(cost) && !std::isnan(cost);
  if (!qpSuccess)
  {
    qpFailedSolvesInWindow_++;
    // Fallback if infeasible
    for (int i = 0; i < numJoints; ++i)
    {
      double lb = std::max(-qdMax_(i), (qMin_(i) - _currentQ(i)) / _dt);
      double ub = std::min( qdMax_(i), (qMax_(i) - _currentQ(i)) / _dt);
      if (ub < lb)
      {
        ub = lb;
      }
      _targetQd(i) = std::clamp(qdDes(i), lb, ub);
    }
  }
  else
  {
    qpConvergedSolvesInWindow_++;
    qpAccumulatedCostInWindow_ += cost;
    _targetQd = qdSol;
  }

  qpTotalSolvesInWindow_++;
  qpLogTimer_ += _dt;

  if (qpLogTimer_ >= 1.0)
  {
    if (verbosity_ >= 3)
    {
      double rate = (100.0 * qpConvergedSolvesInWindow_) / std::max(1, qpTotalSolvesInWindow_);
      double avgCost = (qpConvergedSolvesInWindow_ > 0)
                           ? (qpAccumulatedCostInWindow_ / qpConvergedSolvesInWindow_)
                           : 0.0;

      std::cout << "[WholeBodyQP] [1s Window Stats] Solves: " << qpTotalSolvesInWindow_
                << " | Converged: " << qpConvergedSolvesInWindow_ << "/" << qpTotalSolvesInWindow_
                << " (" << rate << "%)"
                << " | Failed: " << qpFailedSolvesInWindow_
                << " | Avg Cost: " << avgCost << std::endl;
    }

    qpLogTimer_ = 0.0;
    qpTotalSolvesInWindow_ = 0;
    qpConvergedSolvesInWindow_ = 0;
    qpFailedSolvesInWindow_ = 0;
    qpAccumulatedCostInWindow_ = 0.0;
  }

  // 5. Target positions from kinematic reference
  _targetQ = qRef;
  for (int i = 0; i < numJoints; ++i)
  {
    _targetQ(i) = std::clamp(_targetQ(i), qMin_(i), qMax_(i));
  }

  return true;
}

template class WholeBodyQPController<12>;

} // namespace gz_humanoid_walking
