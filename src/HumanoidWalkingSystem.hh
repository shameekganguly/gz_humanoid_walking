#ifndef GZ_HUMANOID_WALKING_HUMANOIDWALKINGSYSTEM_HH_
#define GZ_HUMANOID_WALKING_HUMANOIDWALKINGSYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include "LIPMGenerator.hh"
#include "WholeBodyQPController.hh"
#include "PinocchioDynamicsWrapper.hh"

namespace gz_humanoid_walking
{

class HumanoidWalkingSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
public:
  HumanoidWalkingSystem();
  ~HumanoidWalkingSystem() override = default;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Computes true actual Center of Mass from all link poses and inertias
  Eigen::Vector3d ComputeActualCoM(const gz::sim::EntityComponentManager &_ecm);

private:
  void OnCmdVel(const gz::msgs::Twist &_msg);
  void InitializeJointEntities(gz::sim::EntityComponentManager &_ecm);

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string modelName_;

  std::unique_ptr<LIPMGenerator> lipmGenerator_;
  WholeBodyQPController qpController_;
  std::unique_ptr<PinocchioDynamicsWrapper> pinocchioWrapper_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher actualComPub_;
  gz::transport::Node::Publisher desiredComPub_;
  gz::transport::Node::Publisher comErrorPub_;

  // Commanded forward velocity
  double cmdVx_{0.0};

  // Transition / Settling time before step generation
  /// \brief Settling duration before commencing walking / sway motion [s] (default 0.8s).
  double transitionTime_{0.8};

  /// \brief Model directory name to locate model.sdf for Pinocchio dynamics (default "jvrc1").
  std::string sdfModelDirname_{"jvrc1"};

  /// \brief Initial knee flexion angle applied at spawn time [rad] (default 0.64 rad / 36.7 deg).
  double initialKneeBend_{0.64};

  /// \brief Verbosity logging level (< 3: quiet, >= 3: periodic stats, >= 4: verbose diagnostics).
  int verbosity_{3};

  // Control loop timing
  std::chrono::steady_clock::duration lastControlTime_{0};
  double controlPeriod_{0.005}; // 5ms (200Hz) control cycle

  // Joint and Link entities
  std::vector<std::string> jointNamesList_;
  std::unordered_map<std::string, gz::sim::Entity> jointEntities_;
  std::vector<gz::sim::Entity> linkEntities_;
  double totalRobotMass_{0.0};

  // CoM Periodic 1s Logging
  double comLogTimer_{0.0};
  double targetQLogTimer_{0.0};
  Eigen::Vector3d accumulatedComError_{Eigen::Vector3d::Zero()};
  int comSamplesInWindow_{0};
  std::chrono::steady_clock::time_point lastPausedLogTime_{std::chrono::steady_clock::now()};

  // Joint Gains & Limits (Unit-Mass Normalized Computed Torque Control)
  /// \brief Joint proportional acceleration gain (omega_n^2, default 400.0 rad/s^2).
  double legKp_{400.0};

  /// \brief Joint integral acceleration gain (default 8.0 rad/s^3).
  double legKi_{8.0};

  /// \brief Joint derivative acceleration gain (2 * zeta * omega_n, default 40.0 rad/s).
  double legKd_{40.0};

  /// \brief Maximum torque limit applied to any leg joint [N*m] (default 600.0 N*m).
  double legMaxTorque_{600.0};

  // Sway Balance Shift Test
  /// \brief Enable lateral sway test instead of forward walking (default false).
  bool enableSwayTest_{false};

  /// \brief Lateral CoM sway test amplitude [m] (default 0.045m).
  double swayAmplitude_{0.045};

  /// \brief Lateral CoM sway test period [s] (default 4.0s).
  double swayPeriod_{4.0};

  Eigen::VectorXd initialQ_;
  Eigen::VectorXd integralQError_;
  Eigen::Vector3d initialCoM_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prevActualCoM_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filteredCoMVel_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
  bool appliedInitialKneeBend_{false};
  bool loggedFirstPostUpdate_{false};
};



} // namespace gz_humanoid_walking

#endif // GZ_HUMANOID_WALKING_HUMANOIDWALKINGSYSTEM_HH_
