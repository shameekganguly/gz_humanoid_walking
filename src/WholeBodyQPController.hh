#ifndef GZ_HUMANOID_WALKING_WHOLEBODYQPCONTROLLER_HH_
#define GZ_HUMANOID_WALKING_WHOLEBODYQPCONTROLLER_HH_

#include <Eigen/Dense>
#include <array>
#include <string_view>
#include <unordered_map>

#include "LegIK.hh"
#include "LIPMGenerator.hh"

namespace gz_humanoid_walking
{

template <std::size_t NumJoints = 12>
class WholeBodyQPController
{
public:
  explicit WholeBodyQPController(
      const std::array<std::string_view, NumJoints> &_jointNames,
      int _verbosity = 3);
  ~WholeBodyQPController() = default;

  /**
   * @brief Solves the Whole-Body QP using eiquadprog
   */
  bool Solve(
      const Eigen::Vector3d &_comDes,
      const Eigen::Vector3d &_comVelDes,
      const Eigen::Vector3d &_leftFootDes,
      const Eigen::Matrix3d &_leftFootRot,
      const Eigen::Vector3d &_rightFootDes,
      const Eigen::Matrix3d &_rightFootRot,
      SupportState _support,
      const Eigen::VectorXd &_currentQ,
      double _dt,
      Eigen::Matrix<double, NumJoints, 1> &_targetQ,
      Eigen::Matrix<double, NumJoints, 1> &_targetQd);

  const std::array<std::string_view, NumJoints> &JointNames() const { return jointNames_; }

private:
  std::array<std::string_view, NumJoints> jointNames_;
  std::unordered_map<std::string_view, int> jointIndexMap_;

  LegIK legIK_;
  Eigen::Matrix<double, NumJoints, 1> qNominal_;
  Eigen::Matrix<double, NumJoints, 1> qMin_;
  Eigen::Matrix<double, NumJoints, 1> qMax_;
  Eigen::Matrix<double, NumJoints, 1> qdMax_;

  // 1-Second Periodic Logging Accumulators
  double qpLogTimer_{0.0};
  int qpTotalSolvesInWindow_{0};
  int qpConvergedSolvesInWindow_{0};
  int qpFailedSolvesInWindow_{0};
  double qpAccumulatedCostInWindow_{0.0};
  /// \brief Diagnostic verbosity level (< 3: quiet, >= 3: periodic stats, >= 4: verbose diagnostics).
  int verbosity_{3};
};

} // namespace gz_humanoid_walking

#endif // GZ_HUMANOID_WALKING_WHOLEBODYQPCONTROLLER_HH_
