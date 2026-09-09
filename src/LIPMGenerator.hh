#ifndef GZ_HUMANOID_WALKING_LIPMGENERATOR_HH_
#define GZ_HUMANOID_WALKING_LIPMGENERATOR_HH_

#include <Eigen/Dense>
#include <atomic>
#include <deque>
#include <vector>

#include "types.hh"

namespace gz_humanoid_walking
{

struct Footstep
{
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  SupportState support{SupportState::DOUBLE_SUPPORT};
};

struct LIPMConfig
{
  /// \brief Control timestep [s] (default 5ms / 200Hz).
  double dt{0.005};

  /// \brief Nominal kinematic CoM / leg height target [m].
  double z_c{0.795};

  /// \brief Gravitational acceleration [m/s^2].
  double gravity{9.81};

  /// \brief Total step duration [s] (SSP + DSP, default 1.20s).
  double stepDuration{1.20};

  /// \brief Double support phase (DSP) duration within each step [s] (default 0.80s).
  double dspDuration{0.80};

  /// \brief Initial double support preparation / settling duration [s] (default 0.80s).
  double initDspDuration{0.80};

  /// \brief Swing foot maximum ground clearance [m] (default 0.045m / 4.5cm).
  double footClearance{0.045};

  /// \brief Nominal lateral distance between left and right foot centers [m] (default 0.192m).
  double footSeparation{0.192};

  /// \brief Lateral ZMP reference offset from centerline [m] (default 0.065m).
  double zmpMargin{0.065};

  /// \brief Preview horizon length in discrete control timesteps (default 160 * 0.005s = 0.8s lookahead).
  /// Because the LIPM inverted pendulum dynamics have an unstable pole
  /// omega_0 = sqrt(g / z_c) ~= 3.51 rad/s (time constant tau = 1 / omega_0 ~= 0.28 s),
  /// the preview controller requires at least 2 to 3 * tau (~0.6 to 0.8 s) of future ZMP
  /// lookahead to anticipate footstep transitions and avoid dynamic divergence.
  int previewSteps{160};

  /// \brief Diagnostic verbosity level (< 3: quiet, >= 3: periodic stats, >= 4: verbose diagnostics).
  int verbosity{3};

  /// \brief Total number of steps to walk before bringing feet together and stopping (-1 for infinite).
  int numSteps{-1};

  /// \brief Initial commanded forward velocity [m/s] (default 0.0).
  double initialCmdVx{0.0};
};

/// \brief Linear Inverted Pendulum Model (LIPM) walking pattern generator
/// utilizing a discrete-time preview controller on Zero-Moment Point (ZMP).
///
/// This generator plans dynamic Center of Mass (CoM) trajectories and swing
/// foot profiles from commanded walking velocity and planned footstep locations.
/// The preview control law resolves future ZMP preview errors by solving the
/// Discrete Algebraic Riccati Equation (DARE) for the cart-table preview system.
///
/// Reference:
///   S. Kajita et al., "Biped walking pattern generation by using preview control
///   of zero-moment point," 2003 IEEE International Conference on Robotics and
///   Automation (Cat. No.03CH37422), Taipei, Taiwan, 2003, pp. 1620-1626 vol.2,
///   doi: 10.1109/ROBOT.2003.1241826.
class LIPMGenerator
{
public:
  explicit LIPMGenerator(const LIPMConfig &_config = LIPMConfig());

  /// \brief Initialize generator with starting poses and initial heading
  /// \param[in] _initCoM Initial CoM world position [m]
  /// \param[in] _leftFoot Initial left foot world position [m]
  /// \param[in] _rightFoot Initial right foot world position [m]
  /// \param[in] _initYaw Initial robot heading yaw angle [rad] (default 0.0 rad)
  void Initialize(
      const Eigen::Vector3d &_initCoM,
      const Eigen::Vector3d &_leftFoot,
      const Eigen::Vector3d &_rightFoot,
      double _initYaw = 0.0);

  /// \brief Set commanded forward walking velocity in Body frame
  /// \param[in] _vx Forward velocity in Body X frame [m/s]
  void SetVelocity(double _vx);

  bool IsStopped() const { return isStopped_; }

  /// \brief Step the preview controller and trajectory generation by dt
  void Step();

  // DARE Solver Diagnostics
  bool DareConverged() const { return dareConverged_; }
  int DareIterations() const { return dareIterations_; }
  double DareResidual() const { return dareResidual_; }

  // Accessors
  Eigen::Vector3d CoMPosition() const { return comPos_; }
  Eigen::Vector3d CoMVelocity() const { return comVel_; }

  Eigen::Vector3d LeftFootPosition() const { return leftFootPos_; }
  Eigen::Matrix3d LeftFootOrientation() const { return leftFootRot_; }

  Eigen::Vector3d RightFootPosition() const { return rightFootPos_; }
  Eigen::Matrix3d RightFootOrientation() const { return rightFootRot_; }

  SupportState CurrentSupportState() const { return currentSupport_; }

  /// \brief Commanded forward velocity in Body frame [m/s] (thread-safe)
  double CommandedVelocity() const { return pendingCmdVx_.load(std::memory_order_relaxed); }

  /// \brief Active forward velocity currently latched for the step [m/s]
  double ActiveVelocity() const { return cmdVx_; }

private:
  void ComputePreviewGains();
  Eigen::Vector2d GetZMPAtTime(double _t) const;

  LIPMConfig config_;

  // Commanded forward velocity (Body frame)
  // pendingCmdVx_ is written asynchronously from the transport thread (thread-safe).
  // cmdVx_ is latched at step boundaries on the simulation thread to prevent mid-swing shifts.
  std::atomic<double> pendingCmdVx_{0.0};
  double cmdVx_{0.0};
  int lastStepIdx_{-1};
  int numSteps_{-1};
  bool isStopped_{false};

  // Heading & Frame orientation
  double initYaw_{0.0};
  Eigen::Vector2d u_x_{1.0, 0.0}; // Body forward unit vector in world XY
  Eigen::Vector2d u_y_{0.0, 1.0}; // Body lateral unit vector in world XY

  // State
  Eigen::Vector3d comPos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d comVel_{Eigen::Vector3d::Zero()};

  Eigen::Vector3d leftFootPos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d leftFootRot_{Eigen::Matrix3d::Identity()};

  Eigen::Vector3d rightFootPos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rightFootRot_{Eigen::Matrix3d::Identity()};

  Eigen::Vector3d initFootL_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d initFootR_{Eigen::Vector3d::Zero()};

  SupportState currentSupport_{SupportState::DOUBLE_SUPPORT};
  double stepTime_{0.0};
  double totalTime_{0.0};
  int stepCount_{0};

  // Preview Controller Gains and Buffers
  double Gi_{0.0};
  Eigen::RowVector3d Gx_{Eigen::RowVector3d::Zero()};
  std::vector<double> Gp_;

  Eigen::Vector3d stateX_{Eigen::Vector3d::Zero()}; // [x, dot_x, ddot_x]
  Eigen::Vector3d stateY_{Eigen::Vector3d::Zero()}; // [y, dot_y, ddot_y]

  double sumErrorX_{0.0};
  double sumErrorY_{0.0};

  std::deque<Eigen::Vector2d> zmpRefQueue_; // [zmp_x, zmp_y]
  std::deque<Footstep> footstepPlan_;

  // DARE Solver Diagnostics
  bool dareConverged_{false};
  int dareIterations_{0};
  double dareResidual_{0.0};

  // 1-Second Periodic Logging Accumulators
  double logWindowTimer_{0.0};
  int totalStepsInWindow_{0};
  double accumulatedResidualInWindow_{0.0};
  int accumulatedItersInWindow_{0};
  int convergedCountInWindow_{0};
};

} // namespace gz_humanoid_walking

#endif // GZ_HUMANOID_WALKING_LIPMGENERATOR_HH_
