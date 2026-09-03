#include "PinocchioDynamicsWrapper.hh"
#include "WholeBodyQPController.hh"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace gz_humanoid_walking;

int main()
{
  std::cout << "=== Running WholeBodyQPController IK Test ===" << std::endl;

  WholeBodyQPController qp;
  const std::vector<std::string> legJoints = {
      "joint_R_HIP_P", "joint_R_HIP_R", "joint_R_HIP_Y",
      "joint_R_KNEE", "joint_R_ANKLE_R", "joint_R_ANKLE_P",
      "joint_L_HIP_P", "joint_L_HIP_R", "joint_L_HIP_Y",
      "joint_L_KNEE", "joint_L_ANKLE_R", "joint_L_ANKLE_P"
  };

  qp.Initialize(legJoints);

  LegIK legIK;
  LegJointAngles initAngles;
  initAngles.hipPitch = -0.141101;
  initAngles.kneePitch = 0.350;
  initAngles.anklePitch = -0.208899;
  Eigen::Vector3d initFootPos;
  Eigen::Matrix3d initFootRot;
  legIK.ForwardKinematics(LegSide::LEFT, initAngles, initFootPos, initFootRot);
  std::cout << "FK of initialQ: footPos in hip = [" << initFootPos.transpose() << "]" << std::endl;

  Eigen::Vector3d comDes(0.0, 0.0, -initFootPos.z());
  std::cout << "Corresponding nominal com_height = " << -initFootPos.z() << std::endl;
  Eigen::Vector3d comVelDes(0.0, 0.0, 0.0);
  Eigen::Vector3d leftFootDes(0.0, 0.096, 0.0);
  Eigen::Vector3d rightFootDes(0.0, -0.096, 0.0);
  Eigen::Matrix3d leftFootRot = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rightFootRot = Eigen::Matrix3d::Identity();
  SupportState support = SupportState::DOUBLE_SUPPORT;

  Eigen::VectorXd currentQ = Eigen::VectorXd::Zero(12);
  currentQ(0) = -0.141; currentQ(3) = 0.350; currentQ(5) = -0.209;
  currentQ(6) = -0.141; currentQ(9) = 0.350; currentQ(11) = -0.209;

  Eigen::VectorXd targetQ, targetQd;
  bool ok = qp.Solve(comDes, comVelDes, leftFootDes, leftFootRot, rightFootDes, rightFootRot, support, currentQ, 0.005, targetQ, targetQd);

  std::cout << "QP Solve result: " << (ok ? "SUCCESS" : "FAILED") << std::endl;
  std::cout << "R_Leg TargetQ: HP=" << targetQ(0) << ", HR=" << targetQ(1) << ", HY=" << targetQ(2)
            << ", KN=" << targetQ(3) << ", AR=" << targetQ(4) << ", AP=" << targetQ(5) << std::endl;
  std::cout << "L_Leg TargetQ: HP=" << targetQ(6) << ", HR=" << targetQ(7) << ", HY=" << targetQ(8)
            << ", KN=" << targetQ(9) << ", AR=" << targetQ(10) << ", AP=" << targetQ(11) << std::endl;

  // Test swaying left to y = +0.06m
  Eigen::Vector3d comDesLeft(0.0, 0.06, 0.815481);
  bool okLeft = qp.Solve(comDesLeft, comVelDes, leftFootDes, leftFootRot, rightFootDes, rightFootRot, SupportState::LEFT_SUPPORT, currentQ, 0.005, targetQ, targetQd);
  std::cout << "\nSway Left (y=+0.06m) QP Solve result: " << (okLeft ? "SUCCESS" : "FAILED") << std::endl;
  std::cout << "R_Leg TargetQ: HP=" << targetQ(0) << ", HR=" << targetQ(1) << ", HY=" << targetQ(2)
            << ", KN=" << targetQ(3) << ", AR=" << targetQ(4) << ", AP=" << targetQ(5) << std::endl;
  std::cout << "L_Leg TargetQ: HP=" << targetQ(6) << ", HR=" << targetQ(7) << ", HY=" << targetQ(8)
            << ", KN=" << targetQ(9) << ", AR=" << targetQ(10) << ", AP=" << targetQ(11) << std::endl;

  // Test dynamic walking step: CoM over right foot (y=-0.045m), Left foot swinging (x=+0.05m, z=+0.03m)
  Eigen::Vector3d comDesStep(0.02, -0.045, 0.795);
  Eigen::Vector3d leftFootSwing(0.05, 0.096, 0.03);
  Eigen::Vector3d rightFootStance(0.00, -0.096, 0.00);
  bool okStep = qp.Solve(comDesStep, comVelDes, leftFootSwing, leftFootRot, rightFootStance, rightFootRot, SupportState::RIGHT_SUPPORT, currentQ, 0.005, targetQ, targetQd);
  std::cout << "\nWalking Step (CoM y=-0.045m, L_Swing z=+0.03m) QP Solve result: " << (okStep ? "SUCCESS" : "FAILED") << std::endl;
  std::cout << "R_Leg (Stance) TargetQ: HP=" << targetQ(0) << ", HR=" << targetQ(1) << ", HY=" << targetQ(2)
            << ", KN=" << targetQ(3) << ", AR=" << targetQ(4) << ", AP=" << targetQ(5) << std::endl;
  std::cout << "L_Leg (Swing)  TargetQ: HP=" << targetQ(6) << ", HR=" << targetQ(7) << ", HY=" << targetQ(8)
            << ", KN=" << targetQ(9) << ", AR=" << targetQ(10) << ", AP=" << targetQ(11) << std::endl;

  return 0;
}


