#ifndef GZ_HUMANOID_WALKING_WHOLEBODYQPCONTROLLER_HH_
#define GZ_HUMANOID_WALKING_WHOLEBODYQPCONTROLLER_HH_

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>

#include <eiquadprog/eiquadprog.hpp>
#include "LegIK.hh"
#include "LIPMGenerator.hh"

namespace gz_humanoid_walking
{

class WholeBodyQPController
{
public:
  WholeBodyQPController();
  ~WholeBodyQPController() = default;

  void Initialize(const std::vector<std::string> &_jointNames);

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
      Eigen::VectorXd &_targetQ,
      Eigen::VectorXd &_targetQd);

  void SetVerbosity(int _v) { verbosity_ = _v; }
  const std::vector<std::string> &JointNames() const { return jointNames_; }

private:
  std::vector<std::string> jointNames_;
  std::unordered_map<std::string, int> jointIndexMap_;

  LegIK legIK_;
  Eigen::VectorXd qNominal_;
  Eigen::VectorXd qMin_;
  Eigen::VectorXd qMax_;
  Eigen::VectorXd qdMax_;

  int numJoints_{12};

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
