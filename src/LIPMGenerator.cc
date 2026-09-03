#include "LIPMGenerator.hh"
#include <cmath>
#include <iostream>

namespace gz_humanoid_walking
{

LIPMGenerator::LIPMGenerator(const LIPMConfig &_config)
  : config_(_config),
    numSteps_(_config.numSteps)
{
  ComputePreviewGains();
}

void LIPMGenerator::ComputePreviewGains()
{
  double dt = config_.dt;
  double z_c = config_.z_c;
  double g = config_.gravity;

  // State transition matrices
  Eigen::Matrix3d A;
  A << 1.0, dt, dt * dt / 2.0,
       0.0, 1.0, dt,
       0.0, 0.0, 1.0;

  Eigen::Vector3d B;
  B << dt * dt * dt / 6.0,
       dt * dt / 2.0,
       dt;

  Eigen::RowVector3d C;
  C << 1.0, 0.0, -z_c / g;

  // Augmented system for tracking error integration
  Eigen::Matrix4d Atilde;
  Atilde.setZero();
  Atilde(0, 0) = 1.0;
  Atilde.block<1, 3>(0, 1) = C * A;
  Atilde.block<3, 3>(1, 1) = A;

  Eigen::Vector4d Btilde;
  Btilde(0) = (C * B)(0, 0);
  Btilde.segment<3>(1) = B;

  Eigen::Vector4d Ftilde;
  Ftilde << -1.0, 0.0, 0.0, 0.0;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0, 0) = 1.0; // Error weight Qe

  double R = 1.0e-6; // Control weight (Kajita et al. 2003)

  // Solve DARE via Structure-preserving Doubling Algorithm (SDA)
  Eigen::Matrix4d Ak = Atilde;
  Eigen::Matrix4d Gk = Btilde * (1.0 / R) * Btilde.transpose();
  Eigen::Matrix4d Hk = Q;
  Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();

  dareConverged_ = false;
  dareIterations_ = 0;
  dareResidual_ = 0.0;
  const int maxIter = 50;
  const double tol = 1e-11;

  for (int iter = 0; iter < maxIter; ++iter)
  {
    dareIterations_ = iter + 1;
    Eigen::Matrix4d W = I4 + Gk * Hk;
    Eigen::Matrix4d W_inv = W.inverse();
    Eigen::Matrix4d Ak_next = Ak * W_inv * Ak;
    Eigen::Matrix4d Gk_next = Gk + Ak * W_inv * Gk * Ak.transpose();
    Eigen::Matrix4d Hk_next = Hk + Ak.transpose() * Hk * W_inv * Ak;

    dareResidual_ = (Hk_next - Hk).norm();
    Hk = Hk_next;
    Ak = Ak_next;
    Gk = Gk_next;

    if (dareResidual_ < tol)
    {
      dareConverged_ = true;
      break;
    }
  }

  Eigen::Matrix4d P = Hk;

  if (!dareConverged_)
  {
    if (config_.verbosity >= 3)
    {
      std::cerr << "[LIPMGenerator] [WARNING] DARE solver failed to converge within "
                << maxIter << " iterations! Final residual norm: " << dareResidual_
                << " (tolerance=" << tol << ", g=" << config_.gravity
                << ", z_c=" << config_.z_c << ", dt=" << config_.dt << ")." << std::endl;
    }
  }
  else if (config_.verbosity >= 4)
  {
    std::cout << "[LIPMGenerator] [INFO] DARE solver successfully converged in "
              << dareIterations_ << " iterations (residual=" << dareResidual_ << ")." << std::endl;
  }

  double temp = R + (Btilde.transpose() * P * Btilde)(0, 0);
  Eigen::RowVector4d K = (1.0 / temp) * (Btilde.transpose() * P * Atilde);

  Gi_ = K(0);
  Gx_ = K.segment<3>(1);

  // Closed loop matrix
  Eigen::Matrix4d Ac = Atilde - Btilde * K;

  // Preview gains
  Gp_.resize(config_.previewSteps);
  Eigen::Matrix4d Ac_pow = Eigen::Matrix4d::Identity();
  for (int l = 0; l < config_.previewSteps; ++l)
  {
    if (l == 0)
    {
      Gp_[l] = -Gi_;
    }
    else
    {
      Gp_[l] = ((1.0 / temp) * Btilde.transpose() * Ac_pow.transpose() * P * Ftilde)(0, 0);
      Ac_pow = Ac_pow * Ac;
    }
  }
}

Eigen::Vector2d LIPMGenerator::GetZMPAtTime(double _t) const
{
  double initDsp = config_.initDspDuration;
  double Tstep = config_.stepDuration;
  double Tdsp = config_.dspDuration;
  Eigen::Vector2d p0 = (initFootL_.head<2>() + initFootR_.head<2>()) * 0.5;
  double y_zmp = config_.zmpMargin;

  if (_t < initDsp)
  {
    // Smooth initial sway: ZMP moves smoothly from 0 to first stance foot (Right foot at -y_zmp in body frame)
    double s = std::clamp(_t / initDsp, 0.0, 1.0);
    double s_poly = s * s * (3.0 - 2.0 * s);
    double zmp_yb = -y_zmp * s_poly;
    return p0 + zmp_yb * u_y_;
  }

  double tWalk = _t - initDsp;
  int stepIdx = static_cast<int>(std::floor(tWalk / Tstep));
  double s_step = (tWalk - stepIdx * Tstep) / Tstep;
  double stepLen = cmdVx_ * Tstep;

  // If numSteps is specified and stepIdx >= numSteps_, the robot is stopped with feet side-by-side at finalXb
  if (numSteps_ > 0 && stepIdx >= numSteps_)
  {
    double finalXb = (numSteps_ - 1) * stepLen;
    return p0 + finalXb * u_x_;
  }

  // Stance foot forward position for step stepIdx
  auto getStanceXb = [&](int idx) -> double {
    if (idx == 0) return 0.0;
    if (numSteps_ > 0 && idx >= numSteps_) return (numSteps_ - 1) * stepLen;
    return idx * stepLen;
  };

  // Stance foot lateral ZMP for step stepIdx
  auto getStanceYb = [&](int idx) -> double {
    if (numSteps_ > 0 && idx >= numSteps_) return 0.0;
    return (idx % 2 == 0) ? -y_zmp : +y_zmp;
  };

  double curZmpXb = getStanceXb(stepIdx);
  double curZmpYb = getStanceYb(stepIdx);

  double nextZmpXb = getStanceXb(stepIdx + 1);
  double nextZmpYb = getStanceYb(stepIdx + 1);

  double s_dsp_start = 1.0 - (Tdsp / Tstep);
  if (s_step > s_dsp_start && Tdsp > 1e-4)
  {
    double alpha = (s_step - s_dsp_start) / (Tdsp / Tstep);
    alpha = std::clamp(alpha, 0.0, 1.0);
    double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
    double zmpXb = curZmpXb + (nextZmpXb - curZmpXb) * smooth_alpha;
    double zmpYb = curZmpYb + (nextZmpYb - curZmpYb) * smooth_alpha;
    return p0 + zmpXb * u_x_ + zmpYb * u_y_;
  }

  return p0 + curZmpXb * u_x_ + curZmpYb * u_y_;
}

void LIPMGenerator::Initialize(
    const Eigen::Vector3d &_initCoM,
    const Eigen::Vector3d &_leftFoot,
    const Eigen::Vector3d &_rightFoot,
    double _initYaw)
{
  initYaw_ = _initYaw;
  u_x_ << std::cos(initYaw_), std::sin(initYaw_);
  u_y_ << -std::sin(initYaw_), std::cos(initYaw_);

  Eigen::Vector2d p0 = (_leftFoot.head<2>() + _rightFoot.head<2>()) * 0.5;
  comPos_ = _initCoM;
  comPos_.x() = p0.x();
  comPos_.y() = p0.y();
  comPos_.z() = config_.z_c;
  comVel_.setZero();
  comAcc_.setZero();

  stateX_ << comPos_.x(), 0.0, 0.0;
  stateY_ << comPos_.y(), 0.0, 0.0;

  sumErrorX_ = 0.0;
  sumErrorY_ = 0.0;

  leftFootPos_ = _leftFoot;
  rightFootPos_ = _rightFoot;

  initFootL_ = _leftFoot;
  initFootR_ = _rightFoot;

  Eigen::Matrix3d R_init = Eigen::AngleAxisd(initYaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  leftFootRot_ = R_init;
  rightFootRot_ = R_init;

  currentSupport_ = SupportState::DOUBLE_SUPPORT;
  stepTime_ = 0.0;
  totalTime_ = 0.0;
  stepCount_ = 0;
  isStopped_ = false;

  double measuredSeparation = (_leftFoot.head<2>() - _rightFoot.head<2>()).norm();
  if (measuredSeparation > 0.05)
  {
    config_.footSeparation = measuredSeparation;
  }

  // Pre-fill preview queue with predictive footsteps
  zmpRefQueue_.clear();
  for (int i = 0; i < config_.previewSteps + 50; ++i)
  {
    double tFuture = i * config_.dt;
    zmpRefQueue_.push_back(GetZMPAtTime(tFuture));
  }
}

void LIPMGenerator::SetVelocity(double _vx)
{
  cmdVx_ = _vx;
}

void LIPMGenerator::Step()
{
  double dt = config_.dt;
  totalTime_ += dt;
  stepTime_ += dt;

  // 1. Maintain preview queue of future footsteps
  while (zmpRefQueue_.size() < static_cast<size_t>(config_.previewSteps + 50))
  {
    double tFuture = totalTime_ + zmpRefQueue_.size() * dt;
    zmpRefQueue_.push_back(GetZMPAtTime(tFuture));
  }

  // 2. Footstep State Machine & Foot Trajectory Generation
  double initDsp = config_.initDspDuration;
  double Tstep = config_.stepDuration;
  double Tdsp = config_.dspDuration;
  double Tssp = std::max(0.01, Tstep - Tdsp);
  double stepLen = cmdVx_ * Tstep;
  double halfSep = config_.footSeparation * 0.5;
  Eigen::Vector2d p0 = (initFootL_.head<2>() + initFootR_.head<2>()) * 0.5;

  Eigen::Matrix3d R_foot = Eigen::AngleAxisd(initYaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  leftFootRot_ = R_foot;
  rightFootRot_ = R_foot;

  if (totalTime_ < initDsp)
  {
    currentSupport_ = SupportState::DOUBLE_SUPPORT;
    leftFootPos_ = initFootL_;
    rightFootPos_ = initFootR_;
  }
  else
  {
    double tWalk = totalTime_ - initDsp;
    int stepIdx = static_cast<int>(std::floor(tWalk / Tstep));
    double tInStep = tWalk - stepIdx * Tstep;

    if (numSteps_ > 0 && stepIdx >= numSteps_)
    {
      // Finished all requested steps: robot has brought feet side-by-side at finalXb
      currentSupport_ = SupportState::DOUBLE_SUPPORT;
      isStopped_ = true;
      double finalXb = (numSteps_ - 1) * stepLen;
      Eigen::Vector2d lStop2d = p0 + finalXb * u_x_ + halfSep * u_y_;
      Eigen::Vector2d rStop2d = p0 + finalXb * u_x_ - halfSep * u_y_;
      leftFootPos_ = Eigen::Vector3d(lStop2d.x(), lStop2d.y(), 0.0);
      rightFootPos_ = Eigen::Vector3d(rStop2d.x(), rStop2d.y(), 0.0);
    }
    else if (tInStep < Tssp)
    {
      // Single Support Phase: Quintic polynomial for X along heading, C2-smooth zero-impact profile for Z
      double s = std::clamp(tInStep / Tssp, 0.0, 1.0);
      double s_poly = s * s * s * (6.0 * s * s - 15.0 * s + 10.0);
      double z_clear = 16.0 * config_.footClearance * s * s * (1.0 - s) * (1.0 - s);

      double stanceXb = (stepIdx == 0 ? 0.0 : stepIdx * stepLen);
      double swingStartXb = (stepIdx == 0 ? 0.0 : (stepIdx - 1) * stepLen);
      // For the final step (stepIdx == numSteps_ - 1), swing foot lands at (numSteps_ - 1) * stepLen
      // bringing both feet to the exact same stance position side-by-side!
      double swingEndXb = (numSteps_ > 0 && stepIdx == numSteps_ - 1)
                              ? (numSteps_ - 1) * stepLen
                              : (stepIdx + 1) * stepLen;
      double swingCurrentXb = swingStartXb + (swingEndXb - swingStartXb) * s_poly;

      if (stepIdx % 2 == 0)
      {
        // Step 0, 2, 4... : Right Leg is Stance, Left Leg Swings
        currentSupport_ = SupportState::RIGHT_SUPPORT;
        Eigen::Vector2d rStance2d = p0 + stanceXb * u_x_ - halfSep * u_y_;
        Eigen::Vector2d lSwing2d = p0 + swingCurrentXb * u_x_ + halfSep * u_y_;
        rightFootPos_ = Eigen::Vector3d(rStance2d.x(), rStance2d.y(), 0.0);
        leftFootPos_  = Eigen::Vector3d(lSwing2d.x(),  lSwing2d.y(),  z_clear);
      }
      else
      {
        // Step 1, 3, 5... : Left Leg is Stance, Right Leg Swings
        currentSupport_ = SupportState::LEFT_SUPPORT;
        Eigen::Vector2d lStance2d = p0 + stanceXb * u_x_ + halfSep * u_y_;
        Eigen::Vector2d rSwing2d = p0 + swingCurrentXb * u_x_ - halfSep * u_y_;
        leftFootPos_  = Eigen::Vector3d(lStance2d.x(), lStance2d.y(), 0.0);
        rightFootPos_ = Eigen::Vector3d(rSwing2d.x(),  rSwing2d.y(),  z_clear);
      }
    }
    else
    {
      // Double Support Phase at end of step
      currentSupport_ = SupportState::DOUBLE_SUPPORT;

      double stanceXb = (stepIdx == 0 ? 0.0 : stepIdx * stepLen);
      double landedXb = (numSteps_ > 0 && stepIdx == numSteps_ - 1)
                            ? (numSteps_ - 1) * stepLen
                            : (stepIdx + 1) * stepLen;

      if (stepIdx % 2 == 0)
      {
        Eigen::Vector2d rStance2d = p0 + stanceXb * u_x_ - halfSep * u_y_;
        Eigen::Vector2d lLanded2d = p0 + landedXb * u_x_ + halfSep * u_y_;
        rightFootPos_ = Eigen::Vector3d(rStance2d.x(), rStance2d.y(), 0.0);
        leftFootPos_  = Eigen::Vector3d(lLanded2d.x(), lLanded2d.y(), 0.0);
      }
      else
      {
        Eigen::Vector2d lStance2d = p0 + stanceXb * u_x_ + halfSep * u_y_;
        Eigen::Vector2d rLanded2d = p0 + landedXb * u_x_ - halfSep * u_y_;
        leftFootPos_  = Eigen::Vector3d(lStance2d.x(), lStance2d.y(), 0.0);
        rightFootPos_ = Eigen::Vector3d(rLanded2d.x(), rLanded2d.y(), 0.0);
      }
    }
  }

  // 3. Preview control calculation
  Eigen::RowVector3d C(1.0, 0.0, -config_.z_c / config_.gravity);
  double currentZmpX = (C * stateX_)(0, 0);
  double currentZmpY = (C * stateY_)(0, 0);

  Eigen::Vector2d refZmp = zmpRefQueue_.front();

  sumErrorX_ += (currentZmpX - refZmp.x());
  sumErrorY_ += (currentZmpY - refZmp.y());

  double duX = -Gi_ * sumErrorX_ - (Gx_ * stateX_)(0, 0);
  double duY = -Gi_ * sumErrorY_ - (Gx_ * stateY_)(0, 0);

  for (size_t l = 0; l < Gp_.size() && l < zmpRefQueue_.size(); ++l)
  {
    duX -= Gp_[l] * zmpRefQueue_[l].x();
    duY -= Gp_[l] * zmpRefQueue_[l].y();
  }

  // State update
  Eigen::Matrix3d A;
  A << 1.0, dt, dt * dt / 2.0,
       0.0, 1.0, dt,
       0.0, 0.0, 1.0;

  Eigen::Vector3d B;
  B << dt * dt * dt / 6.0,
       dt * dt / 2.0,
       dt;

  stateX_ = A * stateX_ + B * duX;
  stateY_ = A * stateY_ + B * duY;

  comPos_.x() = stateX_(0);
  comPos_.y() = stateY_(0);
  comPos_.z() = config_.z_c;

  comVel_.x() = stateX_(1);
  comVel_.y() = stateY_(1);
  comVel_.z() = 0.0;

  comAcc_.x() = stateX_(2);
  comAcc_.y() = stateY_(2);
  comAcc_.z() = 0.0;

  zmpRefQueue_.pop_front();

  // 1-Second Periodic Logging & Convergence Accumulation
  logWindowTimer_ += dt;
  totalStepsInWindow_++;
  if (dareConverged_)
    convergedCountInWindow_++;
  accumulatedResidualInWindow_ += dareResidual_;
  accumulatedItersInWindow_ += dareIterations_;

  if (logWindowTimer_ >= 1.0)
  {
    if (config_.verbosity >= 3)
    {
      double convRate = (100.0 * convergedCountInWindow_) / std::max(1, totalStepsInWindow_);
      double avgResidual = accumulatedResidualInWindow_ / std::max(1, totalStepsInWindow_);
      double avgIters = (double)accumulatedItersInWindow_ / std::max(1, totalStepsInWindow_);

      std::cout << "[LIPMGenerator] [1s Window Stats] Steps: " << totalStepsInWindow_
                << " | DARE Converged: " << convergedCountInWindow_ << "/" << totalStepsInWindow_
                << " (" << convRate << "%)"
                << " | Avg Iterations: " << avgIters
                << " | Avg Residual Norm: " << avgResidual << std::endl;
    }

    logWindowTimer_ = 0.0;
    totalStepsInWindow_ = 0;
    accumulatedResidualInWindow_ = 0.0;
    accumulatedItersInWindow_ = 0;
    convergedCountInWindow_ = 0;
  }
}

} // namespace gz_humanoid_walking
