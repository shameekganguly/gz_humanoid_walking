#include "HumanoidWalkingSystem.hh"

#include <gz/common/Console.hh>
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
#include <gz/sim/Util.hh>

#include <algorithm>
#include <iostream>

namespace gz_humanoid_walking
{

HumanoidWalkingSystem::HumanoidWalkingSystem()
{
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
    gzerr << "HumanoidWalkingSystem plugin should be attached to a model entity. Failed to initialize." << std::endl;
    return;
  }

  modelName_ = model_.Name(_ecm);
  gzmsg << "Initializing HumanoidWalkingSystem (eiquadprog Whole-Body QP + CoM Monitoring) for model [" << modelName_ << "]" << std::endl;

  // Read gravity from ECM
  double gravityMag = 9.81;
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
                  << " (magnitude=" << gravityMag << " m/s^2)" << std::endl;
          }
        }
        return false;
      });

  // Read configuration parameters
  if (_sdf->HasElement("cmd_vel_x"))
    cmdVx_ = _sdf->Get<double>("cmd_vel_x");
  if (_sdf->HasElement("transition_time"))
    transitionTime_ = _sdf->Get<double>("transition_time");
  else if (_sdf->HasElement("startup_time"))
    transitionTime_ = _sdf->Get<double>("startup_time");
  if (_sdf->HasElement("leg_kp"))
    legKp_ = _sdf->Get<double>("leg_kp");
  if (_sdf->HasElement("leg_ki"))
    legKi_ = _sdf->Get<double>("leg_ki");
  if (_sdf->HasElement("leg_kd"))
    legKd_ = _sdf->Get<double>("leg_kd");
  if (_sdf->HasElement("leg_max_torque"))
    legMaxTorque_ = _sdf->Get<double>("leg_max_torque");
  if (_sdf->HasElement("initial_knee_bend"))
    initialKneeBend_ = _sdf->Get<double>("initial_knee_bend");
  if (_sdf->HasElement("sdf_model_dirname"))
    sdfModelDirname_ = _sdf->Get<std::string>("sdf_model_dirname");
  if (_sdf->HasElement("enable_sway_test"))
    enableSwayTest_ = _sdf->Get<bool>("enable_sway_test");
  if (_sdf->HasElement("sway_amplitude"))
    swayAmplitude_ = _sdf->Get<double>("sway_amplitude");
  if (_sdf->HasElement("sway_period"))
    swayPeriod_ = _sdf->Get<double>("sway_period");

  // Read verbosity from Gazebo Console verbosity by default, with optional SDF override
  verbosity_ = gz::common::Console::Verbosity();
  if (_sdf->HasElement("verbosity"))
    verbosity_ = _sdf->Get<int>("verbosity");

  qpController_.SetVerbosity(verbosity_);

  LIPMConfig lipmCfg;
  lipmCfg.dt = controlPeriod_;
  lipmCfg.gravity = gravityMag;
  lipmCfg.verbosity = verbosity_;
  if (_sdf->HasElement("gravity"))
    lipmCfg.gravity = _sdf->Get<double>("gravity");
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

  lipmGenerator_ = std::make_unique<LIPMGenerator>(lipmCfg);
  lipmGenerator_->SetVelocity(cmdVx_);

  // Subscribe to velocity commands
  std::string cmdVelTopic = "/cmd_vel";
  if (_sdf->HasElement("cmd_vel_topic"))
    cmdVelTopic = _sdf->Get<std::string>("cmd_vel_topic");

  node_.Subscribe(cmdVelTopic, &HumanoidWalkingSystem::OnCmdVel, this);

  // Advertize CoM monitoring topics
  actualComPub_ = node_.Advertise<gz::msgs::Vector3d>("/humanoid_walking/actual_com");
  desiredComPub_ = node_.Advertise<gz::msgs::Vector3d>("/humanoid_walking/desired_com");
  comErrorPub_ = node_.Advertise<gz::msgs::Vector3d>("/humanoid_walking/com_error");

  // Joint list for the 12-DoF humanoid legs
  jointNamesList_ = {
      "joint_R_HIP_P", "joint_R_HIP_R", "joint_R_HIP_Y", "joint_R_KNEE", "joint_R_ANKLE_R", "joint_R_ANKLE_P",
      "joint_L_HIP_P", "joint_L_HIP_R", "joint_L_HIP_Y", "joint_L_KNEE", "joint_L_ANKLE_R", "joint_L_ANKLE_P"};
}

void HumanoidWalkingSystem::OnCmdVel(const gz::msgs::Twist &_msg)
{
  cmdVx_ = _msg.linear().x();

  if (lipmGenerator_)
  {
    lipmGenerator_->SetVelocity(cmdVx_);
  }
}

void HumanoidWalkingSystem::InitializeJointEntities(gz::sim::EntityComponentManager &_ecm)
{
  // Cache joint entities
  jointEntities_.clear();
  for (const auto &jName : jointNamesList_)
  {
    auto jEntity = model_.JointByName(_ecm, jName);
    if (jEntity != gz::sim::kNullEntity)
    {
      jointEntities_[jName] = jEntity;
    }
    else
    {
      gzerr << "HumanoidWalkingSystem: Joint [" << jName << "] not found in model [" << modelName_ << "]" << std::endl;
    }
  }

  // Ensure ECM components exist for reading/controlling joints
  for (const auto &[name, entity] : jointEntities_)
  {
    if (!_ecm.Component<gz::sim::components::JointForceCmd>(entity))
      _ecm.CreateComponent(entity, gz::sim::components::JointForceCmd({0.0}));

    if (!_ecm.Component<gz::sim::components::JointPosition>(entity))
      _ecm.CreateComponent(entity, gz::sim::components::JointPosition());

    if (!_ecm.Component<gz::sim::components::JointVelocity>(entity))
      _ecm.CreateComponent(entity, gz::sim::components::JointVelocity());
  }

  // Populate initialQ_ and integralQError_
  initialQ_ = Eigen::VectorXd::Zero(jointNamesList_.size());
  integralQError_ = Eigen::VectorXd::Zero(jointNamesList_.size());

  for (size_t i = 0; i < jointNamesList_.size(); ++i)
  {
    auto it = jointEntities_.find(jointNamesList_[i]);
    if (it != jointEntities_.end())
    {
      auto posComp = _ecm.Component<gz::sim::components::JointPosition>(it->second);
      if (posComp && !posComp->Data().empty())
        initialQ_(i) = posComp->Data()[0];
    }
  }

  // Cache link entities and compute total mass
  linkEntities_ = model_.Links(_ecm);
  totalRobotMass_ = 0.0;
  for (const auto &lEntity : linkEntities_)
  {
    auto inertialComp = _ecm.Component<gz::sim::components::Inertial>(lEntity);
    if (inertialComp)
    {
      totalRobotMass_ += inertialComp->Data().MassMatrix().Mass();
    }
  }

  // Initialize Whole-Body QP Controller
  qpController_.Initialize(jointNamesList_);

  // Initialize Pinocchio Dynamics Wrapper
  pinocchioWrapper_ = std::make_unique<PinocchioDynamicsWrapper>();
  std::string relativePath = sdfModelDirname_ + "/model.sdf";
  std::string sdfPath = gz::common::SystemPaths::LocateLocalFile(
      relativePath, gz::sim::resourcePaths());

  if (sdfPath.empty())
  {
    gzerr << "HumanoidWalkingSystem: Could not locate '" << relativePath
          << "' in GZ_SIM_RESOURCE_PATH!" << std::endl;
  }
  else
  {
    gzmsg << "HumanoidWalkingSystem: Resolved robot SDF for Pinocchio: " << sdfPath << std::endl;
  }

  if (sdfPath.empty() || !pinocchioWrapper_->Initialize(sdfPath, "PELVIS_S", jointNamesList_))
  {
    gzerr << "HumanoidWalkingSystem: Failed to initialize PinocchioDynamicsWrapper!" << std::endl;
  }

  // Measure initial poses from ECM
  Eigen::Vector3d actualCoM = ComputeActualCoM(_ecm);

  auto lFootEntity = model_.LinkByName(_ecm, "L_ANKLE_P_S");
  auto rFootEntity = model_.LinkByName(_ecm, "R_ANKLE_P_S");

  if (lFootEntity == gz::sim::kNullEntity)
  {
    gzerr << "HumanoidWalkingSystem: Critical error! Left foot link 'L_ANKLE_P_S' not found in model." << std::endl;
    throw std::runtime_error("HumanoidWalkingSystem: Left foot link 'L_ANKLE_P_S' not found in model.");
  }
  if (rFootEntity == gz::sim::kNullEntity)
  {
    gzerr << "HumanoidWalkingSystem: Critical error! Right foot link 'R_ANKLE_P_S' not found in model." << std::endl;
    throw std::runtime_error("HumanoidWalkingSystem: Right foot link 'R_ANKLE_P_S' not found in model.");
  }

  gz::math::Pose3d lFootPose = gz::sim::worldPose(lFootEntity, _ecm);
  gz::math::Pose3d rFootPose = gz::sim::worldPose(rFootEntity, _ecm);

  double initYaw = lFootPose.Rot().Yaw();

  Eigen::Vector3d initLeft(lFootPose.Pos().X(), lFootPose.Pos().Y(), 0.0);
  Eigen::Vector3d initRight(rFootPose.Pos().X(), rFootPose.Pos().Y(), 0.0);

  // If actual measured height is valid (>0.2m), adapt LIPM nominal height to match physics
  if (actualCoM.z() > 0.2)
  {
    gzmsg << "HumanoidWalkingSystem measured initial CoM: [" << actualCoM.x() << ", "
          << actualCoM.y() << ", " << actualCoM.z() << "] m. Left Foot: " << lFootPose.Pos()
          << ", Right Foot: " << rFootPose.Pos() << ", Yaw: " << (initYaw * 180.0 / M_PI) << " deg" << std::endl;
  }

  initialCoM_ = actualCoM;

  // Initialize LIPM state with initial heading yaw
  lipmGenerator_->Initialize(actualCoM, initLeft, initRight, initYaw);

  initialized_ = true;
  gzmsg << "HumanoidWalkingSystem successfully initialized [" << jointEntities_.size()
        << "] leg joints and [" << linkEntities_.size() << "] links (total mass="
        << totalRobotMass_ << " kg) with eiquadprog Whole-Body QP." << std::endl;
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

    com += m * Eigen::Vector3d(p_WCoM.X(), p_WCoM.Y(), p_WCoM.Z());
    totalMass += m;
  }

  if (totalMass > 0.0)
  {
    com /= totalMass;
  }
  return com;
}

void HumanoidWalkingSystem::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    if (!initialized_)
    {
      InitializeJointEntities(_ecm);
      lastControlTime_ = _info.simTime;
    }

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
      if (actualComPub_.HasConnections())
      {
        gz::msgs::Vector3d msg;
        msg.set_x(actualCoM.x()); msg.set_y(actualCoM.y()); msg.set_z(actualCoM.z());
        actualComPub_.Publish(msg);
      }
    }
    return;
  }

  if (!initialized_)
  {
    InitializeJointEntities(_ecm);
    lastControlTime_ = _info.simTime;
    return;
  }

  // Apply initial knee bend on the first unpaused PreUpdate
  if (!appliedInitialKneeBend_)
  {
    appliedInitialKneeBend_ = true;
    if (std::abs(initialKneeBend_) > 1e-4)
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
        auto it = jointEntities_.find(name);
        if (it != jointEntities_.end())
        {
          auto resetComp = _ecm.Component<gz::sim::components::JointPositionReset>(it->second);
          if (resetComp)
            resetComp->Data() = {angle};
          else
            _ecm.CreateComponent(it->second, gz::sim::components::JointPositionReset({angle}));

          auto posComp = _ecm.Component<gz::sim::components::JointPosition>(it->second);
          if (posComp)
            posComp->Data() = {angle};
          else
            _ecm.CreateComponent(it->second, gz::sim::components::JointPosition({angle}));
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

      // Re-populate initialQ_
      for (size_t i = 0; i < jointNamesList_.size(); ++i)
      {
        auto it = jointEntities_.find(jointNamesList_[i]);
        if (it != jointEntities_.end())
        {
          auto posComp = _ecm.Component<gz::sim::components::JointPosition>(it->second);
          if (posComp && !posComp->Data().empty())
            initialQ_(i) = posComp->Data()[0];
        }
      }
    }
  }

  double simTimeSec = std::chrono::duration<double>(_info.simTime).count();

  // Control cycle update
  auto dtDur = _info.simTime - lastControlTime_;
  if (dtDur < std::chrono::steady_clock::duration::zero())
  {
    lastControlTime_ = _info.simTime;
    dtDur = std::chrono::steady_clock::duration::zero();
  }

  double dtSec = std::chrono::duration<double>(dtDur).count();

  if (dtSec >= controlPeriod_)
  {
    lastControlTime_ = _info.simTime;
    if (simTimeSec >= transitionTime_ && !enableSwayTest_)
    {
      lipmGenerator_->Step();
    }
  }

  // 1. Read Current Joint States
  int numJoints = static_cast<int>(jointNamesList_.size());
  Eigen::VectorXd currentQ = Eigen::VectorXd::Zero(numJoints);
  Eigen::VectorXd currentQd = Eigen::VectorXd::Zero(numJoints);

  for (int i = 0; i < numJoints; ++i)
  {
    const auto &name = jointNamesList_[i];
    auto it = jointEntities_.find(name);
    if (it != jointEntities_.end())
    {
      auto posComp = _ecm.Component<gz::sim::components::JointPosition>(it->second);
      if (posComp && !posComp->Data().empty())
        currentQ(i) = posComp->Data()[0];

      auto velComp = _ecm.Component<gz::sim::components::JointVelocity>(it->second);
      if (velComp && !velComp->Data().empty())
        currentQd(i) = velComp->Data()[0];
    }
  }

  // 2. Trajectory Targets from LIPM Generator or Sway Balance Test
  Eigen::Vector3d comDes = lipmGenerator_->CoMPosition();
  Eigen::Vector3d comVelDes = lipmGenerator_->CoMVelocity();
  Eigen::Vector3d leftFootDes = lipmGenerator_->LeftFootPosition();
  Eigen::Matrix3d leftFootRot = lipmGenerator_->LeftFootOrientation();
  Eigen::Vector3d rightFootDes = lipmGenerator_->RightFootPosition();
  Eigen::Matrix3d rightFootRot = lipmGenerator_->RightFootOrientation();
  SupportState support = lipmGenerator_->CurrentSupportState();

  if (enableSwayTest_)
  {
    leftFootDes = Eigen::Vector3d(0.0, 0.096, 0.0);
    rightFootDes = Eigen::Vector3d(0.0, -0.096, 0.0);
    leftFootRot = Eigen::Matrix3d::Identity();
    rightFootRot = Eigen::Matrix3d::Identity();
    comDes.x() = (initialCoM_.norm() > 0.0) ? initialCoM_.x() : 0.00655;
    comVelDes.x() = 0.0;
    comDes.z() = lipmGenerator_->CoMPosition().z();
    comVelDes.z() = 0.0;

    if (simTimeSec >= transitionTime_)
    {
      double t_sway = simTimeSec - transitionTime_;
      double omega_sway = 2.0 * M_PI / swayPeriod_;
      double y_sway = swayAmplitude_ * std::sin(omega_sway * t_sway);
      double vy_sway = swayAmplitude_ * omega_sway * std::cos(omega_sway * t_sway);

      comDes.y() = y_sway;
      comVelDes.y() = vy_sway;

      if (y_sway > 0.02)
      {
        support = SupportState::LEFT_SUPPORT;
      }
      else if (y_sway < -0.02)
      {
        support = SupportState::RIGHT_SUPPORT;
      }
      else
      {
        support = SupportState::DOUBLE_SUPPORT;
      }
    }
    else
    {
      comDes.y() = 0.0;
      comVelDes.y() = 0.0;
      support = SupportState::DOUBLE_SUPPORT;
    }
  }

  // 3. Compute Actual CoM from ECM & Calculate Tracking Error
  Eigen::Vector3d actualCoM = ComputeActualCoM(_ecm);
  if (prevActualCoM_.isZero())
  {
    prevActualCoM_ = actualCoM;
  }
  Eigen::Vector3d rawCoMVel = Eigen::Vector3d::Zero();
  if (dtSec > 1e-4)
  {
    rawCoMVel = (actualCoM - prevActualCoM_) / dtSec;
  }
  prevActualCoM_ = actualCoM;

  // Exponential moving average filter on CoM velocity (cutoff ~6Hz)
  double filterAlpha = (dtSec > 1e-4) ? std::clamp(dtSec / (dtSec + 0.025), 0.0, 1.0) : 0.2;
  filteredCoMVel_ = (1.0 - filterAlpha) * filteredCoMVel_ + filterAlpha * rawCoMVel;

  auto lFootEntity = model_.LinkByName(_ecm, "L_ANKLE_P_S");
  auto rFootEntity = model_.LinkByName(_ecm, "R_ANKLE_P_S");
  gz::math::Pose3d lFootPose = (lFootEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(lFootEntity, _ecm) : gz::math::Pose3d();
  gz::math::Pose3d rFootPose = (rFootEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(rFootEntity, _ecm) : gz::math::Pose3d();

  Eigen::Vector3d lFootAct(lFootPose.Pos().X(), lFootPose.Pos().Y(), lFootPose.Pos().Z());
  Eigen::Vector3d rFootAct(rFootPose.Pos().X(), rFootPose.Pos().Y(), rFootPose.Pos().Z());

  // Smooth C0-continuous CoM tracking error in world frame
  Eigen::Vector3d comError = actualCoM - comDes;
  Eigen::Vector3d comVelError = filteredCoMVel_ - comVelDes;

  // Publish CoM feedback to Gazebo Transport
  if (actualComPub_.HasConnections())
  {
    gz::msgs::Vector3d msg;
    msg.set_x(actualCoM.x()); msg.set_y(actualCoM.y()); msg.set_z(actualCoM.z());
    actualComPub_.Publish(msg);
  }
  if (desiredComPub_.HasConnections())
  {
    gz::msgs::Vector3d msg;
    msg.set_x(comDes.x()); msg.set_y(comDes.y()); msg.set_z(comDes.z());
    desiredComPub_.Publish(msg);
  }
  if (comErrorPub_.HasConnections())
  {
    gz::msgs::Vector3d msg;
    msg.set_x(comError.x()); msg.set_y(comError.y()); msg.set_z(comError.z());
    comErrorPub_.Publish(msg);
  }

  // Periodic 1-Second CoM Monitoring Log
  accumulatedComError_ += comError;
  comSamplesInWindow_++;
  comLogTimer_ += (dtSec > 0.0 ? dtSec : controlPeriod_);

  if (comLogTimer_ >= 1.0)
  {
    if (verbosity_ >= 3)
    {
      Eigen::Vector3d avgErr = (comSamplesInWindow_ > 0)
                                   ? Eigen::Vector3d(accumulatedComError_ / static_cast<double>(comSamplesInWindow_))
                                   : Eigen::Vector3d::Zero();
      std::cout << "[CoM Monitor] [1s Stats] Actual CoM: ["
                << actualCoM.x() << ", " << actualCoM.y() << ", " << actualCoM.z()
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

  // 4. Solve Whole-Body QP via eiquadprog
  Eigen::VectorXd targetQ, targetQd;
  qpController_.Solve(comDes, comVelDes,
                      leftFootDes, leftFootRot,
                      rightFootDes, rightFootRot,
                      support, currentQ, controlPeriod_,
                      targetQ, targetQd);

  // Periodic TargetQ logging (every 0.5s)
  targetQLogTimer_ += (dtSec > 0.0 ? dtSec : controlPeriod_);
  if (targetQLogTimer_ >= 0.5)
  {
    targetQLogTimer_ = 0.0;
    if (verbosity_ >= 4 && targetQ.size() >= 12 && currentQ.size() >= 12)
    {
      std::cout << "[QP TargetQ] t=" << simTimeSec
                << "s | R_Leg: [HP=" << targetQ(0) << ", HR=" << targetQ(1)
                << ", HY=" << targetQ(2) << ", KN=" << targetQ(3)
                << ", AR=" << targetQ(4) << ", AP=" << targetQ(5)
                << "] | L_Leg: [HP=" << targetQ(6) << ", HR=" << targetQ(7)
                << ", HY=" << targetQ(8) << ", KN=" << targetQ(9)
                << ", AR=" << targetQ(10) << ", AP=" << targetQ(11) << "]" << std::endl;
      std::cout << "[Actual  Q] t=" << simTimeSec
                << "s | R_Leg: [HP=" << currentQ(0) << ", HR=" << currentQ(1)
                << ", HY=" << currentQ(2) << ", KN=" << currentQ(3)
                << ", AR=" << currentQ(4) << ", AP=" << currentQ(5)
                << "] | L_Leg: [HP=" << currentQ(6) << ", HR=" << currentQ(7)
                << ", HY=" << currentQ(8) << ", KN=" << currentQ(9)
                << ", AR=" << currentQ(10) << ", AP=" << currentQ(11) << "]" << std::endl;
    }
  }

  // 4. Compute Contact-Nullspace Filtered Gravity Compensation
  auto pelvisEntity = model_.LinkByName(_ecm, "PELVIS_S");
  gz::math::Pose3d pelvisPose = (pelvisEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(pelvisEntity, _ecm) : gz::math::Pose3d();
  Eigen::Vector3d basePos(pelvisPose.Pos().X(), pelvisPose.Pos().Y(), pelvisPose.Pos().Z());
  Eigen::Quaterniond baseRot(pelvisPose.Rot().W(), pelvisPose.Rot().X(), pelvisPose.Rot().Y(), pelvisPose.Rot().Z());

  ContactSupportMode supportMode = ContactSupportMode::DOUBLE_SUPPORT;
  if (support == SupportState::LEFT_SUPPORT)
    supportMode = ContactSupportMode::LEFT_SUPPORT;
  else if (support == SupportState::RIGHT_SUPPORT)
    supportMode = ContactSupportMode::RIGHT_SUPPORT;

  Eigen::VectorXd tau_grav = Eigen::VectorXd::Zero(numJoints);
  Eigen::MatrixXd M_a = Eigen::MatrixXd::Identity(numJoints, numJoints);
  if (pinocchioWrapper_ && pinocchioWrapper_->IsInitialized())
  {
    tau_grav = pinocchioWrapper_->ComputeNullspaceGravityTorques(basePos, baseRot, currentQ, supportMode);
    M_a = pinocchioWrapper_->ComputeActuatedMassMatrix(basePos, baseRot, currentQ);
  }

  // 5. Unit-Mass Normalized Computed Torque Control (CTC) with Contact Consistency:
  //    - Swing leg joints: Use floating-base generalized mass matrix M_a for decoupled kinematic tracking.
  //    - Stance leg joints: The ground constraint supports the foot, so the effective joint inertia reflects
  //      the upper body. We ensure adequate stance joint stiffness to maintain sagittal/lateral posture.
  Eigen::MatrixXd M_eff = M_a;
  for (int i = 0; i < numJoints; ++i)
  {
    const auto &name = jointNamesList_[i];
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

  Eigen::VectorXd a_cmd = Eigen::VectorXd::Zero(numJoints);

  for (int i = 0; i < numJoints; ++i)
  {
    const auto &name = jointNamesList_[i];
    double q = currentQ(i);
    double qd = currentQd(i);
    double desQ = (targetQ.size() == numJoints) ? targetQ(i) : initialQ_(i);
    double desQd = (targetQd.size() == numJoints) ? targetQd(i) : 0.0;

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

  // Full multi-body feedback linearization: tau = M_eff * a_cmd + tau_grav
  Eigen::VectorXd tau_cmd = M_eff * a_cmd + tau_grav;

  for (int i = 0; i < numJoints; ++i)
  {
    const auto &name = jointNamesList_[i];
    auto it = jointEntities_.find(name);
    if (it == jointEntities_.end())
      continue;

    bool isLeftJoint = (name.rfind("joint_L_", 0) == 0);
    bool isRightJoint = (name.rfind("joint_R_", 0) == 0);
    bool isStanceFoot = (support == SupportState::DOUBLE_SUPPORT) || enableSwayTest_ ||
                        (support == SupportState::LEFT_SUPPORT && isLeftJoint) ||
                        (support == SupportState::RIGHT_SUPPORT && isRightJoint);

    gz::sim::Entity joint = it->second;
    double torque = std::clamp(tau_cmd(i), -legMaxTorque_, legMaxTorque_);

    if (verbosity_ >= 4 && simTimeSec < 0.015 && i == 0)
    {
      std::cout << "[PreUpdate CTC Debug t=" << simTimeSec << "s]:" << std::endl;
      for (int j = 0; j < numJoints; ++j)
      {
        std::cout << "  " << jointNamesList_[j] << ": M_ii=" << M_a(j, j) << ", a_cmd=" << a_cmd(j) 
                  << ", tau_grav=" << tau_grav(j) << " -> tau=" << tau_cmd(j) << " N*m" << std::endl;
      }
    }

    auto forceCmdComp = _ecm.Component<gz::sim::components::JointForceCmd>(joint);
    if (forceCmdComp)
    {
      forceCmdComp->Data()[0] = torque;
    }
    else
    {
      _ecm.CreateComponent(joint, gz::sim::components::JointForceCmd({torque}));
    }
  }
}

void HumanoidWalkingSystem::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  static int unpausedStepCount = 0;
  if (verbosity_ >= 4 && unpausedStepCount < 3)
  {
    unpausedStepCount++;

    auto pelvisEntity = model_.LinkByName(_ecm, "PELVIS_S");
    auto waistPEntity = model_.LinkByName(_ecm, "WAIST_P_S");
    auto chestPEntity = model_.LinkByName(_ecm, "CHEST_P_S");
    auto lFootEntity = model_.LinkByName(_ecm, "L_ANKLE_P_S");
    auto rFootEntity = model_.LinkByName(_ecm, "R_ANKLE_P_S");

    gz::math::Pose3d pelvisPose = (pelvisEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(pelvisEntity, _ecm) : gz::math::Pose3d();
    gz::math::Pose3d waistPPose = (waistPEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(waistPEntity, _ecm) : gz::math::Pose3d();
    gz::math::Pose3d chestPPose = (chestPEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(chestPEntity, _ecm) : gz::math::Pose3d();
    gz::math::Pose3d lFootPose = (lFootEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(lFootEntity, _ecm) : gz::math::Pose3d();
    gz::math::Pose3d rFootPose = (rFootEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(rFootEntity, _ecm) : gz::math::Pose3d();
    Eigen::Vector3d actualCoM = ComputeActualCoM(_ecm);

    std::cout << "\n================ [PostUpdate Step " << unpausedStepCount << " Diagnostics] ================" << std::endl;
    std::cout << "Whole-Body CoM World Pos: [" << actualCoM.x() << ", " << actualCoM.y() << ", " << actualCoM.z() << "] m" << std::endl;
    std::cout << "PELVIS_S World Pose: Pos=[" << pelvisPose.Pos() << "] RPY(deg)=[" 
              << pelvisPose.Rot().Roll() * 180.0 / M_PI << ", " 
              << pelvisPose.Rot().Pitch() * 180.0 / M_PI << ", " 
              << pelvisPose.Rot().Yaw() * 180.0 / M_PI << "]" << std::endl;

    if (waistPEntity != gz::sim::kNullEntity)
    {
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

    auto pelvisEntity = model_.LinkByName(_ecm, "PELVIS_S");
    auto lFootEntity = model_.LinkByName(_ecm, "L_ANKLE_P_S");
    auto rFootEntity = model_.LinkByName(_ecm, "R_ANKLE_P_S");

    gz::math::Pose3d pelvisPose = (pelvisEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(pelvisEntity, _ecm) : gz::math::Pose3d();
    gz::math::Pose3d lFootPose = (lFootEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(lFootEntity, _ecm) : gz::math::Pose3d();
    gz::math::Pose3d rFootPose = (rFootEntity != gz::sim::kNullEntity) ? gz::sim::worldPose(rFootEntity, _ecm) : gz::math::Pose3d();
    Eigen::Vector3d actualCoM = ComputeActualCoM(_ecm);

    std::cout << "[State Monitor] t=" << simTimeSec << "s | CoM: ["
              << actualCoM.x() << ", " << actualCoM.y() << ", " << actualCoM.z() << "] m"
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
