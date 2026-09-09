#ifndef GZ_HUMANOID_WALKING_HUMANOIDWALKINGSYSTEM_HH_
#define GZ_HUMANOID_WALKING_HUMANOIDWALKINGSYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "LIPMGenerator.hh"
#include "WholeBodyQPController.hh"
#include "PinocchioDynamicsWrapper.hh"

namespace gz_humanoid_walking
{

constexpr std::size_t kNumControlledDofs = 12;

typedef Eigen::Matrix<double, kNumControlledDofs, 1> VectorDOFsd;
typedef Eigen::Matrix<double, kNumControlledDofs, kNumControlledDofs> MatrixDOFsd;

struct JointInfo
{
  std::string name;
  gz::sim::Entity entity{gz::sim::kNullEntity};
};

struct TrajectoryTargets
{
  Eigen::Vector3d comDes{Eigen::Vector3d::Zero()};
  Eigen::Vector3d comVelDes{Eigen::Vector3d::Zero()};
  Eigen::Vector3d leftFootDes{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d leftFootRot{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d rightFootDes{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rightFootRot{Eigen::Matrix3d::Identity()};
  SupportState support{SupportState::DOUBLE_SUPPORT};
};

/// \brief Humanoid walking controller system plugin that generates 3D bipedal
/// walking trajectories using Linear Inverted Pendulum Model (LIPM) preview
/// control, resolves kinematic targets via whole-body QP optimization, and
/// computes joint torques using Pinocchio rigid body dynamics.
///
/// A Gazebo Transport subscriber is created for dynamic velocity commands.
/// The default topic is "/model/<model_name>/cmd_vel".
///
/// ## System Parameters
///
/// - `<cmd_vel_x>` Commanded forward walking velocity in m/s. Optional
/// parameter. The default value is 0.0. Constraint: Must be non-zero if
/// `<num_steps>` is non-zero. Cannot be set to non-zero when `<enable_sway_test>`
/// is true.
///
/// - `<num_steps>` Total number of walking steps to take before bringing feet
/// together and stopping. Optional parameter. The default value is -1 (infinite /
/// continuous walking when `<cmd_vel_x>` is non-zero). Constraint: If
/// `<num_steps>` is non-zero, `<cmd_vel_x>` must be non-zero.
///
/// - `<cmd_vel_topic>` Gazebo Transport topic for incoming velocity commands
/// (gz::msgs::Twist). Optional parameter. The default value is
/// "/model/<model_name>/cmd_vel".
///
/// - `<trajectory_step_period>` Trajectory generator update period in seconds.
/// Optional parameter. The default value is 0.005s (200 Hz). Constraint: Must
/// be strictly positive (> 0.0).
///
/// - `<transition_time>` Settling duration before beginning walking or sway
/// motion in seconds. Optional parameter. The default value is 0.8s.
///
/// - `<com_height>` Target Center of Mass (CoM) height above the ground in
/// meters. Optional parameter. The default value is 0.795m.
///
/// - `<step_duration>` Total duration of a single step (single support phase +
/// double support phase) in seconds. Optional parameter. The default value is
/// 1.20s.
///
/// - `<dsp_duration>` Duration of the double support phase (DSP) within each
/// step in seconds. Optional parameter. The default value is 0.80s.
///
/// - `<init_dsp_duration>` Initial double support preparation duration before
/// the first step in seconds. Optional parameter. The default value is 0.80s.
///
/// - `<foot_clearance>` Peak swing foot clearance above the ground during single
/// support in meters. Optional parameter. The default value is 0.045m.
///
/// - `<zmp_margin>` Lateral Zero Moment Point (ZMP) reference offset from the
/// centerline in meters. Optional parameter. The default value is 0.065m.
///
/// - `<leg_kp>` Joint proportional gain (omega_n^2) for computed torque control.
/// Optional parameter. The default value is 400.0 rad/s^2.
///
/// - `<leg_ki>` Joint integral gain for computed torque control. Optional
/// parameter. The default value is 8.0 rad/s^3.
///
/// - `<leg_kd>` Joint derivative gain (2 * zeta * omega_n) for computed torque
/// control. Optional parameter. The default value is 40.0 rad/s.
///
/// - `<leg_max_torque>` Maximum torque limit applied to any leg joint in N*m.
/// Optional parameter. The default value is 600.0 N*m.
///
/// - `<initial_knee_bend>` Initial knee flexion angle applied at configuration
/// time in radians. Optional parameter. The default value is 0.64 rad (~36.7 deg).
/// Constraint: Absolute value must be greater than 1e-4 rad.
///
/// - `<sdf_model_dirname>` Directory name under model search paths used to locate
/// model.sdf for Pinocchio dynamics loading. Optional parameter. The default
/// value is "jvrc1".
///
/// - `<enable_sway_test>` When true, enables lateral balance sway testing
/// instead of forward walking. Optional parameter. The default value is false.
/// Constraint: Cannot be true when `<cmd_vel_x>` is non-zero.
///
/// - `<sway_amplitude>` Lateral CoM oscillation amplitude during sway test in
/// meters. Optional parameter. The default value is 0.045m.
///
/// - `<sway_period>` Lateral CoM oscillation period during sway test in seconds.
/// Optional parameter. The default value is 4.0s.
///
/// - `<gyro_kp>` Proportional attitude restoration gain using pelvis orientation
/// in N*m/rad. Optional parameter. The default value is 0.0.
///
/// - `<gyro_kd>` Derivative rate gyro damping gain using pelvis angular velocity
/// in N*m*s/rad. Optional parameter. The default value is 10.0.
class HumanoidWalkingSystem final
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate,
      public gz::sim::ISystemPostUpdate
{
public:
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

private:
  std::optional<LIPMConfig> LoadXmlConfig(
      const std::shared_ptr<const sdf::Element> &_sdf, double _gravityMag = 9.81);

  /// \brief Computes true actual Center of Mass from all link poses and inertias
  Eigen::Vector3d ComputeActualCoM(const gz::sim::EntityComponentManager &_ecm);

  void OnCmdVel(const gz::msgs::Twist &_msg);
  void InitializeJointControl(gz::sim::EntityComponentManager &_ecm);
  void UpdateState(const gz::sim::EntityComponentManager &_ecm, double dtSec = 0.0);
  TrajectoryTargets ComputeTrajectory(double simTimeSec, bool shouldStep = false);
  void LogCOMStateAndError(
      const Eigen::Vector3d &comDes,
      const Eigen::Vector3d &comVelDes,
      SupportState support,
      double dtSec);
  void LogTargetState(
      const VectorDOFsd &targetQ,
      double simTimeSec,
      double dtSec);
  VectorDOFsd ComputeJointTorques(
      const gz::sim::EntityComponentManager &_ecm,
      const VectorDOFsd &targetQ,
      const VectorDOFsd &targetQd,
      SupportState support,
      double dtSec);

  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string modelName_;

  std::unique_ptr<LIPMGenerator> lipmGenerator_;
  std::unique_ptr<WholeBodyQPController<kNumControlledDofs>> qpController_;
  std::unique_ptr<PinocchioDynamicsWrapper<kNumControlledDofs>> pinocchioWrapper_;

  gz::transport::Node node_;

  // Transition / Settling time before step generation
  /// \brief Settling duration before commencing walking / sway motion [s] (default 0.8s).
  double transitionTime_{0.8};

  /// \brief Model directory name to locate model.sdf for Pinocchio dynamics (default "jvrc1").
  std::string sdfModelDirname_{"jvrc1"};

  /// \brief Initial knee flexion angle applied at spawn time [rad] (default 0.64 rad / 36.7 deg).
  double initialKneeBend_{0.64};

  /// \brief Verbosity logging level (< 3: quiet, >= 3: periodic stats, >= 4: verbose diagnostics).
  int verbosity_{3};

  // Trajectory generation timing
  std::chrono::steady_clock::duration lastTrajectoryStepTime_{0};
  double trajectoryStepPeriod_{0.005}; // 5ms (200Hz) trajectory update cycle

  // Joint and Link entities
  std::array<JointInfo, kNumControlledDofs> joints_{};
  std::vector<gz::sim::Entity> linkEntities_;
  gz::sim::Entity pelvisEntity_{gz::sim::kNullEntity};
  gz::sim::Entity waistEntity_{gz::sim::kNullEntity};
  gz::sim::Entity lFootEntity_{gz::sim::kNullEntity};
  gz::sim::Entity rFootEntity_{gz::sim::kNullEntity};

  // Ankle joint indices in joints_ / VectorDOFsd
  int lAnklePJoint_{-1};
  int rAnklePJoint_{-1};
  int lAnkleRJoint_{-1};
  int rAnkleRJoint_{-1};

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

  // Active Gyro Damping
  /// \brief Proportional gain for active attitude restoration using pelvis orientation (default 0.0 N*m/rad).
  double gyroKp_{0.0};

  /// \brief Derivative gain for active gyro damping using pelvis angular velocity (default 10.0 N*m*s/rad).
  double gyroKd_{10.0};

  // Sway Balance Shift Test
  /// \brief Enable lateral sway test instead of forward walking (default false).
  bool enableSwayTest_{false};

  /// \brief Lateral CoM sway test amplitude [m] (default 0.045m).
  double swayAmplitude_{0.045};

  /// \brief Lateral CoM sway test period [s] (default 4.0s).
  double swayPeriod_{4.0};

  VectorDOFsd initialQ_{VectorDOFsd::Zero()};
  VectorDOFsd integralQError_{VectorDOFsd::Zero()};
  VectorDOFsd currentQ_{VectorDOFsd::Zero()};
  VectorDOFsd currentQd_{VectorDOFsd::Zero()};
  Eigen::Vector3d actualCoM_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d initialCoM_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d prevActualCoM_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filteredCoMVel_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
  bool loggedFirstPostUpdate_{false};
};



} // namespace gz_humanoid_walking

#endif // GZ_HUMANOID_WALKING_HUMANOIDWALKINGSYSTEM_HH_
