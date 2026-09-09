#include <gtest/gtest.h>
#include "PinocchioDynamicsWrapper.hh"
#include "WholeBodyQPController.hh"

#include <cmath>
#include <fstream>
#include <vector>

using namespace gz_humanoid_walking;

namespace
{
constexpr std::array<std::string_view, 12> kLegJoints = {
    "joint_R_HIP_P", "joint_R_HIP_R", "joint_R_HIP_Y",
    "joint_R_KNEE",  "joint_R_ANKLE_R", "joint_R_ANKLE_P",
    "joint_L_HIP_P", "joint_L_HIP_R", "joint_L_HIP_Y",
    "joint_L_KNEE",  "joint_L_ANKLE_R", "joint_L_ANKLE_P"
};

std::string FindRobotSdf()
{
  const std::vector<std::string> candidates = {
      "src/gz_humanoid_walking/models/jvrc1/model.sdf",
      "../src/gz_humanoid_walking/models/jvrc1/model.sdf",
      "../../src/gz_humanoid_walking/models/jvrc1/model.sdf",
      "install/share/gz_humanoid_walking/models/jvrc1/model.sdf",
      "../install/share/gz_humanoid_walking/models/jvrc1/model.sdf",
      "../../install/share/gz_humanoid_walking/models/jvrc1/model.sdf"
  };
  for (const auto &path : candidates)
  {
    std::ifstream f(path);
    if (f.good())
      return path;
  }
  return "";
}
} // namespace

/// \brief Test fixture for WholeBodyQPController kinematics and QP solving
class WholeBodyQPControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    initAngles_.hipPitch = -0.141101;
    initAngles_.kneePitch = 0.350;
    initAngles_.anklePitch = -0.208899;

    legIK_.ForwardKinematics(LegSide::LEFT, initAngles_, initFootPos_, initFootRot_);

    comDes_ = Eigen::Vector3d(0.0, 0.0, -initFootPos_.z());
    comVelDes_.setZero();
    leftFootDes_ << 0.0, 0.096, 0.0;
    rightFootDes_ << 0.0, -0.096, 0.0;
    leftFootRot_ = Eigen::Matrix3d::Identity();
    rightFootRot_ = Eigen::Matrix3d::Identity();

    currentQ_.setZero();
    currentQ_(0) = -0.141; currentQ_(3) = 0.350; currentQ_(5) = -0.209;
    currentQ_(6) = -0.141; currentQ_(9) = 0.350; currentQ_(11) = -0.209;
  }

  WholeBodyQPController<12> qp_{kLegJoints};
  LegIK legIK_;
  LegJointAngles initAngles_;
  Eigen::Vector3d initFootPos_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d initFootRot_{Eigen::Matrix3d::Identity()};

  Eigen::Vector3d comDes_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d comVelDes_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d leftFootDes_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rightFootDes_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d leftFootRot_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d rightFootRot_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 12, 1> currentQ_{Eigen::Matrix<double, 12, 1>::Zero()};
};

/// \brief Test nominal standing double-support posture QP solve
TEST_F(WholeBodyQPControllerTest, StandingDoubleSupportIK)
{
  Eigen::Matrix<double, 12, 1> targetQ, targetQd;
  bool ok = qp_.Solve(
      comDes_, comVelDes_,
      leftFootDes_, leftFootRot_,
      rightFootDes_, rightFootRot_,
      SupportState::DOUBLE_SUPPORT,
      currentQ_, 0.005, targetQ, targetQd);

  ASSERT_TRUE(ok);

  for (int i = 0; i < 12; ++i)
  {
    EXPECT_FALSE(std::isnan(targetQ(i))) << "NaN in targetQ(" << i << ")";
    EXPECT_FALSE(std::isnan(targetQd(i))) << "NaN in targetQd(" << i << ")";
  }

  // Sagittal symmetry between right and left legs
  EXPECT_NEAR(targetQ(0), targetQ(6), 1e-2);  // Hip pitch
  EXPECT_NEAR(targetQ(3), targetQ(9), 1e-2);  // Knee pitch
  EXPECT_NEAR(targetQ(5), targetQ(11), 1e-2); // Ankle pitch

  // Lateral antisymmetry
  EXPECT_NEAR(targetQ(1), -targetQ(7), 1e-2);  // Hip roll
  EXPECT_NEAR(targetQ(4), -targetQ(10), 1e-2); // Ankle roll
}

/// \brief Test lateral sway toward left support (Y = +0.06m)
TEST_F(WholeBodyQPControllerTest, LateralSwayLeftSupportIK)
{
  Eigen::Vector3d comDesLeft(0.0, 0.06, 0.815481);
  Eigen::Matrix<double, 12, 1> targetQ, targetQd;

  bool ok = qp_.Solve(
      comDesLeft, comVelDes_,
      leftFootDes_, leftFootRot_,
      rightFootDes_, rightFootRot_,
      SupportState::LEFT_SUPPORT,
      currentQ_, 0.005, targetQ, targetQd);

  ASSERT_TRUE(ok);

  for (int i = 0; i < 12; ++i)
  {
    EXPECT_FALSE(std::isnan(targetQ(i)));
    EXPECT_FALSE(std::isnan(targetQd(i)));
  }

  // Left hip roll should shift to adjust for lateral displacement
  EXPECT_NE(targetQ(7), 0.0);
}

/// \brief Test dynamic walking step with right stance and left foot in swing
TEST_F(WholeBodyQPControllerTest, DynamicWalkingStepRightSupportIK)
{
  Eigen::Vector3d comDesStep(0.02, -0.045, 0.795);
  Eigen::Vector3d leftFootSwing(0.05, 0.096, 0.03);
  Eigen::Vector3d rightFootStance(0.00, -0.096, 0.00);

  Eigen::Matrix<double, 12, 1> targetQ, targetQd;
  bool ok = qp_.Solve(
      comDesStep, comVelDes_,
      leftFootSwing, leftFootRot_,
      rightFootStance, rightFootRot_,
      SupportState::RIGHT_SUPPORT,
      currentQ_, 0.005, targetQ, targetQd);

  ASSERT_TRUE(ok);

  for (int i = 0; i < 12; ++i)
  {
    EXPECT_FALSE(std::isnan(targetQ(i)));
    EXPECT_FALSE(std::isnan(targetQd(i)));
  }

  // Stance knee and swing knee should exhibit distinct configurations
  EXPECT_GT(targetQ(3), 0.1); // Stance knee flexed
  EXPECT_GT(targetQ(9), 0.1); // Swing knee flexed for ground clearance
}

/// \brief Test PinocchioDynamicsWrapper rigid body dynamics properties
TEST(PinocchioDynamicsTest, ModelLoadingAndMassProperties)
{
  std::string sdfPath = FindRobotSdf();
  if (sdfPath.empty())
  {
    GTEST_SKIP() << "JVRC1 model.sdf not found in standard paths; skipping dynamics test.";
  }

  PinocchioDynamicsWrapper<12> wrapper(sdfPath, "PELVIS_S", kLegJoints);
  ASSERT_TRUE(wrapper.IsInitialized());

  // JVRC-1 humanoid total mass is approximately 44 kg
  EXPECT_GT(wrapper.TotalMass(), 35.0);
  EXPECT_LT(wrapper.TotalMass(), 65.0);
}

/// \brief Test actuated mass matrix symmetry and positive-definiteness
TEST(PinocchioDynamicsTest, ActuatedMassMatrixSymmetryAndPositivity)
{
  std::string sdfPath = FindRobotSdf();
  if (sdfPath.empty())
  {
    GTEST_SKIP() << "JVRC1 model.sdf not found; skipping dynamics test.";
  }

  PinocchioDynamicsWrapper<12> wrapper(sdfPath, "PELVIS_S", kLegJoints);
  ASSERT_TRUE(wrapper.IsInitialized());

  const Eigen::Vector3d basePos(0.0, 0.0, 0.83);
  const Eigen::Quaterniond baseRot = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 12, 1> actuatedQ;
  actuatedQ << -0.141, 0.0, 0.0, 0.350, 0.0, -0.209,
               -0.141, 0.0, 0.0, 0.350, 0.0, -0.209;

  auto M_a = wrapper.ComputeActuatedMassMatrix(basePos, baseRot, actuatedQ);

  // Check symmetry: M_a(i, j) == M_a(j, i)
  for (int i = 0; i < 12; ++i)
  {
    EXPECT_GT(M_a(i, i), 0.0) << "Diagonal entry M_a(" << i << ", " << i << ") not positive";
    for (int j = 0; j < 12; ++j)
    {
      EXPECT_NEAR(M_a(i, j), M_a(j, i), 1e-6) << "Asymmetry at (" << i << ", " << j << ")";
    }
  }

  // Eigenvalues must all be strictly positive
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> es(M_a);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

/// \brief Test nullspace gravity compensation torque computation
TEST(PinocchioDynamicsTest, NullspaceGravityTorques)
{
  std::string sdfPath = FindRobotSdf();
  if (sdfPath.empty())
  {
    GTEST_SKIP() << "JVRC1 model.sdf not found; skipping dynamics test.";
  }

  PinocchioDynamicsWrapper<12> wrapper(sdfPath, "PELVIS_S", kLegJoints);
  ASSERT_TRUE(wrapper.IsInitialized());

  const Eigen::Vector3d basePos(0.0, 0.0, 0.83);
  const Eigen::Quaterniond baseRot = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 12, 1> actuatedQ;
  actuatedQ << -0.141, 0.0, 0.0, 0.350, 0.0, -0.209,
               -0.141, 0.0, 0.0, 0.350, 0.0, -0.209;

  auto tau_grav = wrapper.ComputeNullspaceGravityTorques(
      basePos, baseRot, actuatedQ, SupportState::DOUBLE_SUPPORT);

  for (int i = 0; i < 12; ++i)
  {
    EXPECT_FALSE(std::isnan(tau_grav(i))) << "NaN in tau_grav(" << i << ")";
  }

  // Knee pitch joints (indices 3 and 9) should provide negative holding torques to counter flexion
  EXPECT_LT(tau_grav(3), 0.0);
  EXPECT_LT(tau_grav(9), 0.0);
}


