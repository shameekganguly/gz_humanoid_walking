#include "HumanoidWalkingSystem.hh"

#include <gz/common/Console.hh>
#include <gz/math/eigen3/Conversions.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

#include <algorithm>
#include <array>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace
{
// Controlled lower-body joint names in canonical order.
// The system assumes the upper body to be fixed (i.e. revolute joints
// have been replaced with fixed joints).
constexpr std::array<std::string_view, gz_humanoid_walking::kNumControlledDofs> kJointNames = {
    "joint_R_HIP_P", "joint_R_HIP_R", "joint_R_HIP_Y", "joint_R_KNEE", "joint_R_ANKLE_R", "joint_R_ANKLE_P",
    "joint_L_HIP_P", "joint_L_HIP_R", "joint_L_HIP_Y", "joint_L_KNEE", "joint_L_ANKLE_R", "joint_L_ANKLE_P"
};

std::optional<double> GetGravityMagFromECM(const gz::sim::EntityComponentManager &_ecm)
{
  std::optional<double> gravityMag;
  _ecm.Each<gz::sim::components::Gravity>(
      [&](const gz::sim::Entity &, const gz::sim::components::Gravity *_grav) -> bool
      {
        if (_grav)
        {
          double gNorm = _grav->Data().Length();
          if (gNorm > 1e-4)
          {
            gravityMag = gNorm;
            gzmsg << "HumanoidWalkingSystem read gravity from ECM: " << _grav->Data()
                  << " (magnitude=" << gNorm << " m/s^2)" << std::endl;
          }
        }
        return false;
      });
  return gravityMag;
}
} // namespace

namespace gz_humanoid_walking
{

std::optional<LIPMConfig> HumanoidWalkingSystem::LoadXmlConfig(
    const std::shared_ptr<const sdf::Element> &_sdf, double _gravityMag)
{
  if (!_sdf)
    return std::nullopt;

  if (_sdf->HasElement("transition_time"))
    transitionTime_ = _sdf->Get<double>("transition_time");
  if (_sdf->HasElement("leg_kp"))
    legKp_ = _sdf->Get<double>("leg_kp");
  if (_sdf->HasElement("leg_ki"))
    legKi_ = _sdf->Get<double>("leg_ki");
  if (_sdf->HasElement("leg_kd"))
    legKd_ = _sdf->Get<double>("leg_kd");
  if (_sdf->HasElement("leg_max_torque"))
    legMaxTorque_ = _sdf->Get<double>("leg_max_torque");
  if (_sdf->HasElement("gyro_kp"))
    gyroKp_ = _sdf->Get<double>("gyro_kp");
  if (_sdf->HasElement("gyro_kd"))
    gyroKd_ = _sdf->Get<double>("gyro_kd");
  if (_sdf->HasElement("initial_knee_bend"))
    initialKneeBend_ = _sdf->Get<double>("initial_knee_bend");
  if (std::abs(initialKneeBend_) <= 1e-4)
  {
    gzerr << "HumanoidWalkingSystem: initial_knee_bend must be non-zero (greater than 1e-4 rad)." << std::endl;
    return std::nullopt;
  }
  if (_sdf->HasElement("sdf_model_dirname"))
    sdfModelDirname_ = _sdf->Get<std::string>("sdf_model_dirname");
  if (_sdf->HasElement("enable_sway_test"))
    enableSwayTest_ = _sdf->Get<bool>("enable_sway_test");
  if (_sdf->HasElement("sway_amplitude"))
    swayAmplitude_ = _sdf->Get<double>("sway_amplitude");
  if (_sdf->HasElement("sway_period"))
    swayPeriod_ = _sdf->Get<double>("sway_period");
  if (_sdf->HasElement("trajectory_step_period"))
    trajectoryStepPeriod_ = _sdf->Get<double>("trajectory_step_period");

  if (trajectoryStepPeriod_ <= 0.0)
  {
    gzerr << "HumanoidWalkingSystem: trajectory_step_period must be positive." << std::endl;
    return std::nullopt;
  }

  // Read verbosity from Gazebo Console verbosity set via command line
  verbosity_ = gz::common::Console::Verbosity();

  LIPMConfig lipmCfg;
  lipmCfg.dt = trajectoryStepPeriod_;
  lipmCfg.gravity = _gravityMag;
  lipmCfg.verbosity = verbosity_;

  if (_sdf->HasElement("cmd_vel_x"))
    lipmCfg.initialCmdVx = _sdf->Get<double>("cmd_vel_x");

  if (enableSwayTest_ && std::abs(lipmCfg.initialCmdVx) > 1e-6)
  {
    gzerr << "HumanoidWalkingSystem: both enable_sway_test and non-zero cmd_vel_x ("
          << lipmCfg.initialCmdVx << ") cannot be set simultaneously." << std::endl;
    return std::nullopt;
  }

  if (_sdf->HasElement("com_height"))
    lipmCfg.z_c = _sdf->Get<double>("com_height");
  if (_sdf->HasElement("step_duration"))
    lipmCfg.stepDuration = _sdf->Get<double>("step_duration");
  if (_sdf->HasElement("dsp_duration"))
    lipmCfg.dspDuration = _sdf->Get<double>("dsp_duration");
  if (_sdf->HasElement("init_dsp_duration"))
    lipmCfg.initDspDuration = _sdf->Get<double>("init_dsp_duration");
  if (_sdf->HasElement("foot_clearance"))
    lipmCfg.footClearance = _sdf->Get<double>("foot_clearance");
  if (_sdf->HasElement("zmp_margin"))
    lipmCfg.zmpMargin = _sdf->Get<double>("zmp_margin");
  if (_sdf->HasElement("num_steps"))
    lipmCfg.numSteps = _sdf->Get<int>("num_steps");

  if (lipmCfg.numSteps != 0 && std::abs(lipmCfg.initialCmdVx) < 1e-6)
  {
    gzerr << "HumanoidWalkingSystem: cmd_vel_x must not be zero when num_steps ("
          << lipmCfg.numSteps << ") is non-zero." << std::endl;
    return std::nullopt;
  }

  return lipmCfg;
}

void HumanoidWalkingSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  model_ = gz::sim::Model(_entity);
  if (!model_.Valid(_ecm))
  {
    throw std::runtime_error(
        "HumanoidWalkingSystem plugin should be attached to a model entity. Failed to initialize.");
  }

  modelName_ = model_.Name(_ecm);
  gzmsg << "Initializing HumanoidWalkingSystem for model [" << modelName_ << "]" << std::endl;

  // Read gravity from ECM
  double gravityMag = GetGravityMagFromECM(_ecm).value_or(9.81);

  // Read configuration parameters
  auto lipmCfgOpt = LoadXmlConfig(_sdf, gravityMag);
  if (!lipmCfgOpt)
  {
    throw std::runtime_error(
        "HumanoidWalkingSystem failed to load valid XML configuration. Initialization aborted.");
  }

  // Cache joint entities and ensure ECM components exist for reading/controlling joints
  for (size_t i = 0; i < kNumControlledDofs; ++i)
  {
    const auto &jName = kJointNames[i];
    auto jEntity = model_.JointByName(_ecm, std::string(jName));
    if (jEntity == gz::sim::kNullEntity)
    {
      throw std::runtime_error(
          "HumanoidWalkingSystem: Joint [" + std::string(jName) + "] not found in model [" + modelName_ + "]");
    }

    if (!_ecm.Component<gz::sim::components::JointForceCmd>(jEntity))
      _ecm.CreateComponent(jEntity, gz::sim::components::JointForceCmd({0.0}));

    if (!_ecm.Component<gz::sim::components::JointPosition>(jEntity))
      _ecm.CreateComponent(jEntity, gz::sim::components::JointPosition({0.0}));

    if (!_ecm.Component<gz::sim::components::JointVelocity>(jEntity))
      _ecm.CreateComponent(jEntity, gz::sim::components::JointVelocity({0.0}));

    joints_[i] = {std::string(jName), jEntity};

    if (jName == "joint_L_ANKLE_P")
      lAnklePJoint_ = static_cast<int>(i);
    else if (jName == "joint_R_ANKLE_P")
      rAnklePJoint_ = static_cast<int>(i);
    else if (jName == "joint_L_ANKLE_R")
      lAnkleRJoint_ = static_cast<int>(i);
    else if (jName == "joint_R_ANKLE_R")
      rAnkleRJoint_ = static_cast<int>(i);
  }

  // Cache link entities
  linkEntities_ = model_.Links(_ecm);

  pelvisEntity_ = model_.LinkByName(_ecm, "PELVIS_S");
  if (pelvisEntity_ == gz::sim::kNullEntity)
  {
    throw std::runtime_error("HumanoidWalkingSystem: Critical error! Pelvis link 'PELVIS_S' not found in model.");
  }
  gz::sim::Link(pelvisEntity_).EnableVelocityChecks(_ecm, true);
  waistEntity_ = model_.LinkByName(_ecm, "WAIST_P_S");
  lFootEntity_ = model_.LinkByName(_ecm, "L_ANKLE_P_S");
  if (lFootEntity_ == gz::sim::kNullEntity)
  {
    throw std::runtime_error("HumanoidWalkingSystem: Critical error! Left foot link 'L_ANKLE_P_S' not found in model.");
  }
  rFootEntity_ = model_.LinkByName(_ecm, "R_ANKLE_P_S");
  if (rFootEntity_ == gz::sim::kNullEntity)
  {
    throw std::runtime_error("HumanoidWalkingSystem: Critical error! Right foot link 'R_ANKLE_P_S' not found in model.");
  }

  // Initialize QP solver/ controller.
  qpController_ = std::make_unique<WholeBodyQPController<kNumControlledDofs>>(kJointNames, verbosity_);

  // Initialize LIPM trajectory generator.
  lipmGenerator_ = std::make_unique<LIPMGenerator>(*lipmCfgOpt);

  // Initialize Pinocchio Dynamics Wrapper
  std::string relativePath = sdfModelDirname_ + "/model.sdf";
  std::string sdfPath = gz::common::SystemPaths::LocateLocalFile(
      relativePath, gz::sim::resourcePaths());

  if (sdfPath.empty())
  {
    throw std::runtime_error(
        "HumanoidWalkingSystem: Could not locate '" + relativePath + "' in GZ_SIM_RESOURCE_PATH!");
  }

  gzmsg << "HumanoidWalkingSystem: Resolved robot SDF for Pinocchio: " << sdfPath << std::endl;

  pinocchioWrapper_ = std::make_unique<PinocchioDynamicsWrapper<kNumControlledDofs>>(sdfPath, "PELVIS_S", kJointNames);

  // Subscribe to velocity commands
  std::string cmdVelTopic = "/cmd_vel";
  if (_sdf->HasElement("cmd_vel_topic"))
    cmdVelTopic = _sdf->Get<std::string>("cmd_vel_topic");

  node_.Subscribe(cmdVelTopic, &HumanoidWalkingSystem::OnCmdVel, this);
}

void HumanoidWalkingSystem::OnCmdVel(const gz::msgs::Twist &_msg)
{
  if (lipmGenerator_)
  {
    lipmGenerator_->SetVelocity(_msg.linear().x());
  }
}

void HumanoidWalkingSystem::InitializeJointControl(gz::sim::EntityComponentManager &_ecm)
{
  // Kinematic link offset vectors from JVRC-1 leg specifications (matching LegIK.hh):
  // - thighVec: Offset from Hip Pitch joint (L/R_HIP_P_S) to Knee Pitch joint (L/R_KNEE_P_S) [x=-0.02m, z=-0.389m, length ~0.3895m]
  // - shinVec:  Offset from Knee Pitch joint (L/R_KNEE_P_S) to Ankle Pitch joint (L/R_ANKLE_P_S) [x=+0.04m, z=-0.357m, length ~0.3592m]
  Eigen::Vector3d thighVec(-0.02, 0.0, -0.389);
  Eigen::Vector3d shinVec(0.04, 0.0, -0.357);
  Eigen::Matrix3d R_knee = Eigen::AngleAxisd(initialKneeBend_, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d v_leg = thighVec + R_knee * shinVec;

  double hipPitch = std::atan2(v_leg.x(), -v_leg.z());
  double kneePitch = initialKneeBend_;
  double anklePitch = -(hipPitch + kneePitch);

  // Distance foot was raised vertically relative to hip:
  // Straight leg vertical reach: 0.389 + 0.357 = 0.746m
  // Bent leg vertical reach: ||v_leg||
  double straightLegZ = 0.389 + 0.357;
  double bentLegZ = v_leg.norm();
  double deltaZ = straightLegZ - bentLegZ;

  if (verbosity_ >= 3)
  {
    gzmsg << "HumanoidWalkingSystem applying initial posture: KneeBend=" << initialKneeBend_
          << " rad (" << (initialKneeBend_ * 180.0 / M_PI) << " deg). Lowering model Z by "
          << deltaZ << " m via WorldPoseCmd." << std::endl;
  }

  auto setLegAngle = [&](const std::string &name, double angle) {
    for (const auto &joint : joints_)
    {
      if (joint.name == name)
      {
        auto resetComp = _ecm.Component<gz::sim::components::JointPositionReset>(joint.entity);
        if (resetComp)
          resetComp->Data() = {angle};
        else
          _ecm.CreateComponent(joint.entity, gz::sim::components::JointPositionReset({angle}));

        _ecm.Component<gz::sim::components::JointPosition>(joint.entity)->Data() = {angle};
        break;
      }
    }
  };

  // Pitch joints
  setLegAngle("joint_R_HIP_P", hipPitch);
  setLegAngle("joint_R_KNEE", kneePitch);
  setLegAngle("joint_R_ANKLE_P", anklePitch);

  setLegAngle("joint_L_HIP_P", hipPitch);
  setLegAngle("joint_L_KNEE", kneePitch);
  setLegAngle("joint_L_ANKLE_P", anklePitch);

  // Lower the overall model Z height so the feet stay on the ground
  gz::math::Pose3d currentModelPose = gz::sim::worldPose(model_.Entity(), _ecm);
  gz::math::Pose3d newModelPose = currentModelPose;
  newModelPose.Pos().Z() -= deltaZ;
  model_.SetWorldPoseCmd(_ecm, newModelPose);

  // Populate initialQ_ and integralQError_
  UpdateState(_ecm);
  initialQ_ = currentQ_;
  integralQError_.setZero();

  gz::math::Pose3d lFootPose = gz::sim::worldPose(lFootEntity_, _ecm);
  gz::math::Pose3d rFootPose = gz::sim::worldPose(rFootEntity_, _ecm);

  double initYaw = lFootPose.Rot().Yaw();

  Eigen::Vector3d initLeft(lFootPose.Pos().X(), lFootPose.Pos().Y(), 0.0);
  Eigen::Vector3d initRight(rFootPose.Pos().X(), rFootPose.Pos().Y(), 0.0);

  initialCoM_ = actualCoM_;

  // Initialize LIPM state with initial heading yaw
  lipmGenerator_->Initialize(actualCoM_, initLeft, initRight, initYaw);

  initialized_ = true;
  gzmsg << "HumanoidWalkingSystem successfully initialized [" << joints_.size()
        << "] leg joints and [" << linkEntities_.size() << "] links with eiquadprog Whole-Body QP." << std::endl;
}

Eigen::Vector3d HumanoidWalkingSystem::ComputeActualCoM(const gz::sim::EntityComponentManager &_ecm)
{
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  double totalMass = 0.0;

  for (const auto &linkEntity : linkEntities_)
  {
    auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(linkEntity);
    if (!inertialComp)
      continue;

    double m = inertialComp->Data().MassMatrix().Mass();
    if (m <= 0.0)
      continue;

    gz::math::Pose3d X_WL = gz::sim::worldPose(linkEntity, _ecm);
    gz::math::Pose3d X_LCoM = inertialComp->Data().Pose();
    gz::math::Vector3d p_WCoM = (X_WL * X_LCoM).Pos();

    com += m * gz::math::eigen3::convert(p_WCoM);
    totalMass += m;
  }

  if (totalMass > 0.0)
  {
    com /= totalMass;
  }
  return com;
}

void HumanoidWalkingSystem::UpdateState(const gz::sim::EntityComponentManager &_ecm, double dtSec)
{
  for (std::size_t i = 0; i < kNumControlledDofs; ++i)
  {
    currentQ_(i) = _ecm.Component<gz::sim::components::JointPosition>(joints_[i].entity)->Data()[0];
    currentQd_(i) = _ecm.Component<gz::sim::components::JointVelocity>(joints_[i].entity)->Data()[0];
  }

  actualCoM_ = ComputeActualCoM(_ecm);
  if (prevActualCoM_.isZero())
  {
    prevActualCoM_ = actualCoM_;
  }
  Eigen::Vector3d rawCoMVel = Eigen::Vector3d::Zero();
  if (dtSec > 1e-4)
  {
    rawCoMVel = (actualCoM_ - prevActualCoM_) / dtSec;
  }
  prevActualCoM_ = actualCoM_;

  // Exponential moving average filter on CoM velocity (cutoff ~6Hz)
  double filterAlpha = (dtSec > 1e-4) ? std::clamp(dtSec / (dtSec + 0.025), 0.0, 1.0) : 0.2;
  filteredCoMVel_ = (1.0 - filterAlpha) * filteredCoMVel_ + filterAlpha * rawCoMVel;
}

TrajectoryTargets HumanoidWalkingSystem::ComputeTrajectory(double simTimeSec, bool shouldStep)
{
  if (shouldStep && simTimeSec >= transitionTime_ && !enableSwayTest_)
  {
    lipmGenerator_->Step();
  }

  TrajectoryTargets traj;
  traj.comDes = lipmGenerator_->CoMPosition();
  traj.comVelDes = lipmGenerator_->CoMVelocity();
  traj.leftFootDes = lipmGenerator_->LeftFootPosition();
  traj.leftFootRot = lipmGenerator_->LeftFootOrientation();
  traj.rightFootDes = lipmGenerator_->RightFootPosition();
  traj.rightFootRot = lipmGenerator_->RightFootOrientation();
  traj.support = lipmGenerator_->CurrentSupportState();

  if (enableSwayTest_)
  {
    traj.leftFootDes = Eigen::Vector3d(0.0, 0.096, 0.0);
    traj.rightFootDes = Eigen::Vector3d(0.0, -0.096, 0.0);
    traj.leftFootRot = Eigen::Matrix3d::Identity();
    traj.rightFootRot = Eigen::Matrix3d::Identity();
    traj.comDes.x() = (initialCoM_.norm() > 0.0) ? initialCoM_.x() : 0.00655;
    traj.comVelDes.x() = 0.0;
    traj.comDes.z() = lipmGenerator_->CoMPosition().z();
    traj.comVelDes.z() = 0.0;

    if (simTimeSec >= transitionTime_)
    {
      double t_sway = simTimeSec - transitionTime_;
      double omega_sway = 2.0 * M_PI / swayPeriod_;
      double y_sway = swayAmplitude_ * std::sin(omega_sway * t_sway);
      double vy_sway = swayAmplitude_ * omega_sway * std::cos(omega_sway * t_sway);

      traj.comDes.y() = y_sway;
      traj.comVelDes.y() = vy_sway;

      if (y_sway > 0.02)
      {
        traj.support = SupportState::LEFT_SUPPORT;
      }
      else if (y_sway < -0.02)
      {
        traj.support = SupportState::RIGHT_SUPPORT;
      }
      else
      {
        traj.support = SupportState::DOUBLE_SUPPORT;
      }
    }
    else
    {
      traj.comDes.y() = 0.0;
      traj.comVelDes.y() = 0.0;
      traj.support = SupportState::DOUBLE_SUPPORT;
    }
  }

  return traj;
}

void HumanoidWalkingSystem::LogCOMStateAndError(
    const Eigen::Vector3d &comDes,
    const Eigen::Vector3d &comVelDes,
    SupportState support,
    double dtSec)
{
  Eigen::Vector3d comError = actualCoM_ - comDes;
  Eigen::Vector3d comVelError = filteredCoMVel_ - comVelDes;
  (void)comVelError;

  // Periodic 1-Second CoM Monitoring Log
  accumulatedComError_ += comError;
  comSamplesInWindow_++;
  comLogTimer_ += (dtSec > 0.0 ? dtSec : trajectoryStepPeriod_);

  if (comLogTimer_ >= 1.0)
  {
    if (verbosity_ >= 3)
    {
      Eigen::Vector3d avgErr = (comSamplesInWindow_ > 0)
                                   ? Eigen::Vector3d(accumulatedComError_ / static_cast<double>(comSamplesInWindow_))
                                   : Eigen::Vector3d::Zero();
      std::cout << "[CoM Monitor] [1s Stats] Actual CoM: ["
                << actualCoM_.x() << ", " << actualCoM_.y() << ", " << actualCoM_.z()
                << "] | Desired CoM: [" << comDes.x() << ", " << comDes.y() << ", " << comDes.z()
                << "] | Avg Error: [" << avgErr.x() << ", " << avgErr.y() << ", " << avgErr.z()
                << "] m (norm=" << avgErr.norm() << " m) | Support: "
                << (support == SupportState::DOUBLE_SUPPORT ? "DOUBLE" : (support == SupportState::LEFT_SUPPORT ? "LEFT" : "RIGHT"))
                << std::endl;
    }
    comLogTimer_ = 0.0;
    accumulatedComError_.setZero();
    comSamplesInWindow_ = 0;
  }
}

void HumanoidWalkingSystem::LogTargetState(
    const VectorDOFsd &targetQ,
    double simTimeSec,
    double dtSec)
{
  // Periodic TargetQ logging (every 0.5s)
  targetQLogTimer_ += (dtSec > 0.0 ? dtSec : trajectoryStepPeriod_);
  if (targetQLogTimer_ >= 0.5)
  {
    targetQLogTimer_ = 0.0;
    if (verbosity_ >= 4)
    {
      std::cout << "[QP TargetQ] t=" << simTimeSec
                << "s | R_Leg: [HP=" << targetQ(0) << ", HR=" << targetQ(1)
                << ", HY=" << targetQ(2) << ", KN=" << targetQ(3)
                << ", AR=" << targetQ(4) << ", AP=" << targetQ(5)
                << "] | L_Leg: [HP=" << targetQ(6) << ", HR=" << targetQ(7)
                << ", HY=" << targetQ(8) << ", KN=" << targetQ(9)
                << ", AR=" << targetQ(10) << ", AP=" << targetQ(11) << "]" << std::endl;
      std::cout << "[Actual  Q] t=" << simTimeSec
                << "s | R_Leg: [HP=" << currentQ_(0) << ", HR=" << currentQ_(1)
                << ", HY=" << currentQ_(2) << ", KN=" << currentQ_(3)
                << ", AR=" << currentQ_(4) << ", AP=" << currentQ_(5)
                << "] | L_Leg: [HP=" << currentQ_(6) << ", HR=" << currentQ_(7)
                << ", HY=" << currentQ_(8) << ", KN=" << currentQ_(9)
                << ", AR=" << currentQ_(10) << ", AP=" << currentQ_(11) << "]" << std::endl;
    }
  }
}

VectorDOFsd HumanoidWalkingSystem::ComputeJointTorques(
    const gz::sim::EntityComponentManager &_ecm,
    const VectorDOFsd &targetQ,
    const VectorDOFsd &targetQd,
    SupportState support,
    double dtSec)
{
  constexpr int numJoints = static_cast<int>(kNumControlledDofs);

  // Compute Contact-Nullspace Filtered Gravity Compensation
  gz::math::Pose3d pelvisPose = gz::sim::worldPose(pelvisEntity_, _ecm);
  Eigen::Vector3d basePos = gz::math::eigen3::convert(pelvisPose.Pos());
  Eigen::Quaterniond baseRot = gz::math::eigen3::convert(pelvisPose.Rot());

  VectorDOFsd tau_grav = pinocchioWrapper_->ComputeNullspaceGravityTorques(basePos, baseRot, currentQ_, support);
  MatrixDOFsd M_a = pinocchioWrapper_->ComputeActuatedMassMatrix(basePos, baseRot, currentQ_);

  // Unit-Mass Normalized Computed Torque Control (CTC) with Contact Consistency:
  //    - Swing leg joints: Use floating-base generalized mass matrix M_a for decoupled kinematic tracking.
  //    - Stance leg joints: The ground constraint supports the foot, so the effective joint inertia reflects
  //      the upper body. We ensure adequate stance joint stiffness to maintain sagittal/lateral posture.
  // Joint command torques are then computed with feedback linearization:
  // tau = M_eff * a_cmd + tau_grav
  MatrixDOFsd M_eff = M_a;
  for (int i = 0; i < numJoints; ++i)
  {
    const auto &name = kJointNames[i];
    bool isLeftJoint = (name.rfind("joint_L_", 0) == 0);
    bool isRightJoint = (name.rfind("joint_R_", 0) == 0);
    bool isStance = (support == SupportState::DOUBLE_SUPPORT) || enableSwayTest_ ||
                    (support == SupportState::LEFT_SUPPORT && isLeftJoint) ||
                    (support == SupportState::RIGHT_SUPPORT && isRightJoint);

    if (isStance)
    {
      if (name.find("_ANKLE_P") != std::string::npos)
      {
        M_eff(i, i) = std::max(M_eff(i, i), 0.75); // Stance ankle pitch: Kp = 0.75 * 400 = 300 N*m/rad
      }
      else if (name.find("_ANKLE_R") != std::string::npos)
      {
        M_eff(i, i) = std::max(M_eff(i, i), 0.75); // Stance ankle roll: Kp = 0.75 * 400 = 300 N*m/rad
      }
      else if (name.find("_KNEE") != std::string::npos)
      {
        M_eff(i, i) = std::max(M_eff(i, i), 0.80); // Stance knee: Kp = 0.80 * 400 = 320 N*m/rad
      }
      else if (name.find("_HIP_R") != std::string::npos)
      {
        M_eff(i, i) = std::max(M_eff(i, i), 1.20); // Stance hip roll: Kp = 1.20 * 400 = 480 N*m/rad
      }
      else if (name.find("_HIP_P") != std::string::npos)
      {
        M_eff(i, i) = std::max(M_eff(i, i), 1.00); // Stance hip pitch: Kp = 1.00 * 400 = 400 N*m/rad
      }
      else if (name.find("_HIP_Y") != std::string::npos)
      {
        M_eff(i, i) = std::max(M_eff(i, i), 0.40); // Stance hip yaw: Kp = 0.40 * 400 = 160 N*m/rad
      }
    }
  }

  VectorDOFsd a_cmd = VectorDOFsd::Zero();

  for (int i = 0; i < numJoints; ++i)
  {
    const auto &name = kJointNames[i];
    double q = currentQ_(i);
    double qd = currentQd_(i);
    double desQ = targetQ(i);
    double desQd = targetQd(i);

    bool isLeftJoint = (name.rfind("joint_L_", 0) == 0);
    bool isRightJoint = (name.rfind("joint_R_", 0) == 0);
    bool isStanceFoot = (support == SupportState::DOUBLE_SUPPORT) || enableSwayTest_ ||
                        (support == SupportState::LEFT_SUPPORT && isLeftJoint) ||
                        (support == SupportState::RIGHT_SUPPORT && isRightJoint);

    // Natural frequency and damping parameterized by SDF leg_kp (omega_n^2), leg_kd (2*zeta*omega_n), leg_ki
    double kp_acc = legKp_;
    double kd_acc = legKd_;
    double ki_acc = legKi_;

    if (isStanceFoot)
    {
      if (name.find("_ANKLE_P") != std::string::npos)
      {
        // For stance ankle pitch, cap Kd to 8.0 N*m*s/rad to ensure discrete stability (Kd*dt/I_foot = 0.56 < 1.0)
        kd_acc = 11.4; // Kd = 0.70 * 11.4 = 8.0 N*m*s/rad
      }
      else if (name.find("_ANKLE_R") != std::string::npos)
      {
        // For stance ankle roll, cap Kd to 8.0 N*m*s/rad
        kd_acc = 12.3; // Kd = 0.65 * 12.3 = 8.0 N*m*s/rad
      }
    }

    if (dtSec > 0.0 && dtSec < 0.1)
    {
      if (isStanceFoot)
      {
        // Leaky integrator: applies continuous gentle decay to prevent phase-lag accumulation
        // and limit-cycle oscillation at rest, while still providing robust steady-state rejection during walking.
        double leakRate = (lipmGenerator_ && lipmGenerator_->IsStopped()) ? 2.5 : 0.5; // [1/s]
        integralQError_(i) = integralQError_(i) * std::max(0.0, 1.0 - leakRate * dtSec) + (desQ - q) * dtSec;
        double maxIntAcc = 15.0; // rad/s^2 anti-windup clamp
        if (ki_acc > 1e-4)
        {
          integralQError_(i) = std::clamp(integralQError_(i), -maxIntAcc / ki_acc, maxIntAcc / ki_acc);
        }
      }
      else
      {
        integralQError_(i) = 0.0; // Prevent windup on swing leg
      }
    }

    a_cmd(i) = kp_acc * (desQ - q) + kd_acc * (desQd - qd) + (isStanceFoot ? ki_acc * integralQError_(i) : 0.0);
  }

  VectorDOFsd tau_cmd = M_eff * a_cmd + tau_grav;

  // Active Gyro Damping on Stance Ankles
  if (gyroKd_ > 0.0 || gyroKp_ > 0.0)
  {
    gz::sim::Link pelvisLink(pelvisEntity_);
    gz::math::Vector3d omegaWorld = pelvisLink.WorldAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
    gz::math::Vector3d omegaBody = pelvisPose.Rot().Inverse().RotateVector(omegaWorld);
    double omegaPitch = omegaBody.Y();
    double omegaRoll = omegaBody.X();
    double pitchAngle = pelvisPose.Rot().Pitch();
    double rollAngle = pelvisPose.Rot().Roll();

    double tauGyroPitch = std::clamp(gyroKd_ * omegaPitch + gyroKp_ * pitchAngle, -legMaxTorque_, legMaxTorque_);
    double tauGyroRoll = std::clamp(gyroKd_ * omegaRoll + gyroKp_ * rollAngle, -legMaxTorque_, legMaxTorque_);

    bool isLeftStance = (support == SupportState::DOUBLE_SUPPORT) || enableSwayTest_ ||
                        (support == SupportState::LEFT_SUPPORT);
    bool isRightStance = (support == SupportState::DOUBLE_SUPPORT) || enableSwayTest_ ||
                         (support == SupportState::RIGHT_SUPPORT);

    if (isLeftStance)
    {
      tau_cmd(lAnklePJoint_) += tauGyroPitch;
      tau_cmd(lAnkleRJoint_) += tauGyroRoll;
    }
    if (isRightStance)
    {
      tau_cmd(rAnklePJoint_) += tauGyroPitch;
      tau_cmd(rAnkleRJoint_) += tauGyroRoll;
    }
  }

  return tau_cmd;
}

void HumanoidWalkingSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (!initialized_)
  {
    InitializeJointControl(_ecm);
  }

  if (_info.paused)
  {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - lastPausedLogTime_).count() >= 1.0)
    {
      lastPausedLogTime_ = now;
      Eigen::Vector3d actualCoM = ComputeActualCoM(_ecm);
      if (verbosity_ >= 3)
      {
        std::cout << "[CoM Monitor (Paused)] [1s Stats] Actual CoM: ["
                  << actualCoM.x() << ", " << actualCoM.y() << ", " << actualCoM.z()
                  << "] m" << std::endl;
      }
    }
    return;
  }

  double simTimeSec = std::chrono::duration<double>(_info.simTime).count();
  double dtSec = std::chrono::duration<double>(_info.dt).count();

  // Read Current Joint and CoM States
  constexpr int numJoints = static_cast<int>(kNumControlledDofs);
  UpdateState(_ecm, dtSec);

  // Trajectory Targets from LIPM Generator
  auto trajDtDur = _info.simTime - lastTrajectoryStepTime_;
  if (trajDtDur < std::chrono::steady_clock::duration::zero())
  {
    lastTrajectoryStepTime_ = _info.simTime;
    trajDtDur = std::chrono::steady_clock::duration::zero();
  }

  double trajDtSec = std::chrono::duration<double>(trajDtDur).count();
  bool shouldStepTrajectory = (trajDtSec >= trajectoryStepPeriod_);
  if (shouldStepTrajectory)
  {
    lastTrajectoryStepTime_ = _info.simTime;
  }
  auto [comDes, comVelDes, leftFootDes, leftFootRot, rightFootDes, rightFootRot, support] =
      ComputeTrajectory(simTimeSec, shouldStepTrajectory);

  LogCOMStateAndError(comDes, comVelDes, support, dtSec);

  // Solve for joint positions and velocities, with limits.
  VectorDOFsd targetQ = initialQ_;
  VectorDOFsd targetQd = VectorDOFsd::Zero();
  qpController_->Solve(comDes, comVelDes,
                       leftFootDes, leftFootRot,
                       rightFootDes, rightFootRot,
                       support, currentQ_, trajectoryStepPeriod_,
                       targetQ, targetQd);

  LogTargetState(targetQ, simTimeSec, dtSec);

  VectorDOFsd tau_cmd = ComputeJointTorques(_ecm, targetQ, targetQd, support, dtSec);

  for (int i = 0; i < numJoints; ++i)
  {
    gz::sim::Entity joint = joints_[i].entity;
    double torque = std::clamp(tau_cmd(i), -legMaxTorque_, legMaxTorque_);

    auto forceCmdComp = _ecm.Component<gz::sim::components::JointForceCmd>(joint);
    forceCmdComp->Data()[0] = torque;
  }
}

void HumanoidWalkingSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Additional diagnostic logs are enabled in PostUpdate when verbosity_ is >= 3.

  static int unpausedStepCount = 0;
  if (verbosity_ >= 4 && unpausedStepCount < 3)
  {
    unpausedStepCount++;

    gz::math::Pose3d pelvisPose = gz::sim::worldPose(pelvisEntity_, _ecm);
    gz::math::Pose3d lFootPose = gz::sim::worldPose(lFootEntity_, _ecm);
    gz::math::Pose3d rFootPose = gz::sim::worldPose(rFootEntity_, _ecm);

    std::cout << "\n================ [PostUpdate Step " << unpausedStepCount << " Diagnostics] ================" << std::endl;
    std::cout << "Whole-Body CoM World Pos: [" << actualCoM_.x() << ", " << actualCoM_.y() << ", " << actualCoM_.z() << "] m" << std::endl;
    std::cout << "PELVIS_S World Pose: Pos=[" << pelvisPose.Pos() << "] RPY(deg)=[" 
              << pelvisPose.Rot().Roll() * 180.0 / M_PI << ", " 
              << pelvisPose.Rot().Pitch() * 180.0 / M_PI << ", " 
              << pelvisPose.Rot().Yaw() * 180.0 / M_PI << "]" << std::endl;

    if (waistEntity_ != gz::sim::kNullEntity)
    {
      gz::math::Pose3d waistPPose = gz::sim::worldPose(waistEntity_, _ecm);
      std::cout << "WAIST_P_S World Pose: Pos=[" << waistPPose.Pos() << "] RPY(deg)=[" 
                << waistPPose.Rot().Roll() * 180.0 / M_PI << ", " 
                << waistPPose.Rot().Pitch() * 180.0 / M_PI << ", " 
                << waistPPose.Rot().Yaw() * 180.0 / M_PI << "]" << std::endl;
    }

    std::cout << "L_ANKLE_P_S (Left Foot): Pos=[" << lFootPose.Pos() << "] RPY(deg)=[" 
              << lFootPose.Rot().Roll() * 180.0 / M_PI << ", " 
              << lFootPose.Rot().Pitch() * 180.0 / M_PI << ", " 
              << lFootPose.Rot().Yaw() * 180.0 / M_PI << "]" << std::endl;

    std::cout << "R_ANKLE_P_S (Right Foot): Pos=[" << rFootPose.Pos() << "] RPY(deg)=[" 
              << rFootPose.Rot().Roll() * 180.0 / M_PI << ", " 
              << rFootPose.Rot().Pitch() * 180.0 / M_PI << ", " 
              << rFootPose.Rot().Yaw() * 180.0 / M_PI << "]" << std::endl;
    std::cout << "==============================================================================\n" << std::endl;
  }

  // Periodic 1-second full pose diagnostics
  static double lastPeriodicLogSimTime = 0.0;
  double simTimeSec = std::chrono::duration<double>(_info.simTime).count();
  if (verbosity_ >= 3 && simTimeSec - lastPeriodicLogSimTime >= 1.0)
  {
    lastPeriodicLogSimTime = simTimeSec;

    gz::math::Pose3d pelvisPose = gz::sim::worldPose(pelvisEntity_, _ecm);
    gz::math::Pose3d lFootPose = gz::sim::worldPose(lFootEntity_, _ecm);
    gz::math::Pose3d rFootPose = gz::sim::worldPose(rFootEntity_, _ecm);

    std::cout << "[State Monitor] t=" << simTimeSec << "s"
              << " | Pelvis Z: " << pelvisPose.Pos().Z() << " m"
              << " | L_Foot Pos: [" << lFootPose.Pos() << "]"
              << " | R_Foot Pos: [" << rFootPose.Pos() << "]"
              << std::endl;
  }
}

} // namespace gz_humanoid_walking

GZ_ADD_PLUGIN(
    gz_humanoid_walking::HumanoidWalkingSystem,
    gz::sim::System,
    gz_humanoid_walking::HumanoidWalkingSystem::ISystemConfigure,
    gz_humanoid_walking::HumanoidWalkingSystem::ISystemPreUpdate,
    gz_humanoid_walking::HumanoidWalkingSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz_humanoid_walking::HumanoidWalkingSystem,
    "gz_humanoid_walking::HumanoidWalkingSystem")
