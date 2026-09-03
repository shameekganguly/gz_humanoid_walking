#include "LegIK.hh"
#include <cmath>
#include <iostream>
#include <vector>

using namespace gz_humanoid_walking;

int main()
{
  std::cout << "Running LegIK tests..." << std::endl;
  LegIK ik;

  // 1. Check lengths
  if (std::abs(ik.ThighLength() - 0.3895) > 0.01 ||
      std::abs(ik.ShinLength() - 0.3592) > 0.01)
  {
    std::cerr << "FAIL: Thigh/Shin lengths mismatch" << std::endl;
    return 1;
  }
  std::cout << "  [PASS] Thigh/Shin kinematic lengths verified." << std::endl;

  // 2. Standing Pose IK/FK Consistency
  Eigen::Vector3d footPos(0.0, 0.0, -0.74);
  Eigen::Matrix3d footRot = Eigen::Matrix3d::Identity();

  LegJointAngles joints;
  if (!ik.SolveIK(LegSide::LEFT, footPos, footRot, joints))
  {
    std::cerr << "FAIL: SolveIK failed for standing pose" << std::endl;
    return 1;
  }

  Eigen::Vector3d fkFootPos;
  Eigen::Matrix3d fkFootRot;
  ik.ForwardKinematics(LegSide::LEFT, joints, fkFootPos, fkFootRot);

  if ((fkFootPos - footPos).norm() > 1e-4)
  {
    std::cerr << "FAIL: ForwardKinematics output mismatch: "
              << fkFootPos.transpose() << " vs " << footPos.transpose() << std::endl;
    return 1;
  }
  std::cout << "  [PASS] Standing pose IK/FK consistency verified (error < 1e-4 m)." << std::endl;

  // 3. Walking Workspace Reachability & Round-trip Precision
  std::vector<Eigen::Vector3d> testPoses = {
      {0.10, 0.0, -0.72},
      {-0.10, 0.0, -0.72},
      {0.05, 0.01, -0.68},
      {0.0, 0.0, -0.70},
      {0.08, -0.01, -0.69}
  };

  for (const auto &p : testPoses)
  {
    LegJointAngles j;
    if (!ik.SolveIK(LegSide::RIGHT, p, Eigen::Matrix3d::Identity(), j))
    {
      std::cerr << "FAIL: Failed to solve IK for pose: " << p.transpose() << std::endl;
      return 1;
    }

    Eigen::Vector3d fkP;
    Eigen::Matrix3d fkR;
    ik.ForwardKinematics(LegSide::RIGHT, j, fkP, fkR);
    if ((fkP - p).norm() > 2e-3)
    {
      std::cerr << "FAIL: FK mismatch for pose: " << p.transpose()
                << " (Got: " << fkP.transpose() << ")" << std::endl;
      return 1;
    }
  }
  std::cout << "  [PASS] Walking workspace poses verified across 5 trajectory points." << std::endl;

  std::cout << "All LegIK tests PASSED successfully." << std::endl;
  return 0;
}
