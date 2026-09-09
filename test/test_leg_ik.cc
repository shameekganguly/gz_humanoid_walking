#include <gtest/gtest.h>
#include "LegIK.hh"

#include <cmath>
#include <vector>

using namespace gz_humanoid_walking;

/// \brief Test fixture for LegIK unit tests
class LegIKTest : public ::testing::Test
{
protected:
  LegIK ik_;
};

/// \brief Test kinematic dimension consistency
TEST_F(LegIKTest, KinematicDimensions)
{
  EXPECT_NEAR(ik_.ThighLength(), 0.3895, 0.01);
  EXPECT_NEAR(ik_.ShinLength(), 0.3592, 0.01);
  EXPECT_NEAR(ik_.AnkleOffsetZ(), -0.07, 0.01);
}

/// \brief Test standing pose IK/FK round-trip consistency for both left and right legs
TEST_F(LegIKTest, StandingPoseConsistency)
{
  const Eigen::Vector3d footPos(0.0, 0.0, -0.74);
  const Eigen::Matrix3d footRot = Eigen::Matrix3d::Identity();

  for (auto side : {LegSide::LEFT, LegSide::RIGHT})
  {
    LegJointAngles joints;
    ASSERT_TRUE(ik_.SolveIK(side, footPos, footRot, joints));

    Eigen::Vector3d fkFootPos;
    Eigen::Matrix3d fkFootRot;
    ik_.ForwardKinematics(side, joints, fkFootPos, fkFootRot);

    EXPECT_LT((fkFootPos - footPos).norm(), 1e-4);
    EXPECT_LT((fkFootRot - footRot).norm(), 1e-4);
  }
}

/// \brief Test walking workspace reachability and precision across representative poses
TEST_F(LegIKTest, WalkingWorkspaceReachability)
{
  const std::vector<Eigen::Vector3d> testPoses = {
      {0.10, 0.0, -0.72},
      {-0.10, 0.0, -0.72},
      {0.05, 0.01, -0.68},
      {0.0, 0.0, -0.70},
      {0.08, -0.01, -0.69}
  };

  for (auto side : {LegSide::LEFT, LegSide::RIGHT})
  {
    for (const auto &p : testPoses)
    {
      LegJointAngles joints;
      ASSERT_TRUE(ik_.SolveIK(side, p, Eigen::Matrix3d::Identity(), joints))
          << "Failed to solve IK for side=" << (side == LegSide::LEFT ? "LEFT" : "RIGHT")
          << " pose: " << p.transpose();

      Eigen::Vector3d fkP;
      Eigen::Matrix3d fkR;
      ik_.ForwardKinematics(side, joints, fkP, fkR);

      EXPECT_LT((fkP - p).norm(), 2e-3)
          << "FK mismatch for side=" << (side == LegSide::LEFT ? "LEFT" : "RIGHT")
          << " pose: " << p.transpose() << " (Got: " << fkP.transpose() << ")";
    }
  }
}

/// \brief Test graceful handling and rejection of unreachable target poses
TEST_F(LegIKTest, UnreachableTargetRejection)
{
  // Target position well beyond maximum leg extension (> 0.3895 + 0.3592 + 0.07 = 0.8187 m)
  const Eigen::Vector3d unreachablePos(0.0, 0.0, -1.20);
  LegJointAngles joints;

  EXPECT_FALSE(ik_.SolveIK(LegSide::LEFT, unreachablePos, Eigen::Matrix3d::Identity(), joints));
  EXPECT_FALSE(ik_.SolveIK(LegSide::RIGHT, unreachablePos, Eigen::Matrix3d::Identity(), joints));
}

