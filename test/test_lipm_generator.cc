#include "LIPMGenerator.hh"
#include <cmath>
#include <iostream>
#include <vector>

using namespace gz_humanoid_walking;

int main()
{
  std::cout << "Running LIPMGenerator tests..." << std::endl;

  LIPMConfig config;
  config.dt = 0.005;
  config.z_c = 0.795;
  config.stepDuration = 1.00;
  config.dspDuration = 0.50;
  config.initDspDuration = 0.50;
  config.zmpMargin = 0.065;
  config.footSeparation = 0.192;

  LIPMGenerator gen(config);

  Eigen::Vector3d initCoM(0.0, 0.0, 0.78);
  Eigen::Vector3d initLeft(0.0, 0.096, 0.0);
  Eigen::Vector3d initRight(0.0, -0.096, 0.0);

  gen.Initialize(initCoM, initLeft, initRight);
  gen.SetVelocity(0.15);

  if (gen.CurrentSupportState() != SupportState::DOUBLE_SUPPORT)
  {
    std::cerr << "FAIL: Initial state is not double support" << std::endl;
    return 1;
  }
  std::cout << "  [PASS] Initial state verified." << std::endl;

  // Run 600 steps (3.0s -> multiple full step cycles)
  for (int i = 0; i < 600; ++i)
  {
    double t = i * 0.005;
    if (i % 40 == 0)
    {
      std::string sup = (gen.CurrentSupportState() == SupportState::DOUBLE_SUPPORT) ? "DOUBLE" :
                        (gen.CurrentSupportState() == SupportState::LEFT_SUPPORT) ? "LEFT  " : "RIGHT ";
      std::cout << "  [LIPM Test] t=" << t << "s | " << sup
                << " | CoM: [" << gen.CoMPosition().x() << ", " << gen.CoMPosition().y() << "]"
                << " | L_Foot: [" << gen.LeftFootPosition().x() << ", " << gen.LeftFootPosition().y() << "]"
                << " | R_Foot: [" << gen.RightFootPosition().x() << ", " << gen.RightFootPosition().y() << "]"
                << std::endl;
    }
    gen.Step();
    if (std::isnan(gen.CoMPosition().x()) || std::isnan(gen.CoMPosition().y()))
    {
      std::cerr << "FAIL: NaN detected in CoM position at step " << i << std::endl;
      return 1;
    }
  }

  if (gen.CoMPosition().x() <= 0.10)
  {
    std::cerr << "FAIL: CoM did not advance forward: x = " << gen.CoMPosition().x() << std::endl;
    return 1;
  }
  if (std::abs(gen.CoMPosition().z() - config.z_c) > 1e-4)
  {
    std::cerr << "FAIL: CoM height varied: z = " << gen.CoMPosition().z() << std::endl;
    return 1;
  }
  std::cout << "  [PASS] CoM forward progression verified (x = " << gen.CoMPosition().x() << " m)." << std::endl;

  // Test Swing Foot Clearance
  LIPMConfig config2;
  config2.dt = 0.005;
  config2.footClearance = 0.05;
  config2.dspDuration = 0.10;
  config2.initDspDuration = 0.20;

  LIPMGenerator gen2(config2);
  gen2.Initialize({0, 0, 0.78}, {0, 0.096, 0}, {0, -0.096, 0});
  gen2.SetVelocity(0.2);

  double maxSwingHeight = 0.0;
  for (int i = 0; i < 300; ++i)
  {
    gen2.Step();
    if (gen2.CurrentSupportState() == SupportState::RIGHT_SUPPORT)
    {
      maxSwingHeight = std::max(maxSwingHeight, gen2.LeftFootPosition().z());
    }
  }

  if (maxSwingHeight < 0.04 || maxSwingHeight > 0.06)
  {
    std::cerr << "FAIL: Swing foot clearance out of bounds: " << maxSwingHeight << std::endl;
    return 1;
  }
  std::cout << "  [PASS] Swing foot clearance verified (max height = " << maxSwingHeight << " m)." << std::endl;

  // Test 3: Rotated Heading (90 degrees, facing world +Y)
  LIPMConfig config3;
  config3.dt = 0.005;
  config3.stepDuration = 0.8;
  config3.dspDuration = 0.10;
  config3.initDspDuration = 0.20;

  LIPMGenerator gen3(config3);
  double yaw90 = M_PI / 2.0;
  // In 90-deg yaw: left foot is at (-0.096, 0), right foot is at (+0.096, 0)
  gen3.Initialize({0, 0, 0.78}, {-0.096, 0.0, 0.0}, {0.096, 0.0, 0.0}, yaw90);
  gen3.SetVelocity(0.15); // 0.15 m/s forward in Body frame (should move along world +Y)

  for (int i = 0; i < 600; ++i)
  {
    gen3.Step();
  }

  if (gen3.CoMPosition().y() <= 0.10)
  {
    std::cerr << "FAIL: Rotated CoM did not advance along world +Y: y = " << gen3.CoMPosition().y() << std::endl;
    return 1;
  }
  std::cout << "  [PASS] Rotated 90-deg heading progression verified (world y = " << gen3.CoMPosition().y() << " m)." << std::endl;

  // Test 4: Num steps stopping and side-by-side feet alignment
  LIPMConfig config4;
  config4.dt = 0.005;
  config4.stepDuration = 1.0;
  config4.dspDuration = 0.50;
  config4.initDspDuration = 0.50;
  config4.numSteps = 4;

  LIPMGenerator gen4(config4);
  gen4.Initialize({0, 0, 0.78}, {0, 0.096, 0}, {0, -0.096, 0});
  gen4.SetVelocity(0.10); // 0.1 m/s -> stepLen = 0.1 m, finalX = 4 * 0.1 = 0.4 m

  // Step for 8 seconds (1600 iterations)
  for (int i = 0; i < 1600; ++i)
  {
    gen4.Step();
  }

  if (!gen4.IsStopped())
  {
    std::cerr << "FAIL: Generator did not enter stopped state after numSteps=4" << std::endl;
    return 1;
  }
  if (gen4.CurrentSupportState() != SupportState::DOUBLE_SUPPORT)
  {
    std::cerr << "FAIL: Final state is not DOUBLE_SUPPORT" << std::endl;
    return 1;
  }

  double expectedX = (4 - 1) * 0.10; // 0.3 m (after 4 steps: steps 0, 1, 2, and closing step 3)
  double lFootX = gen4.LeftFootPosition().x();
  double rFootX = gen4.RightFootPosition().x();
  double footDiff = std::abs(lFootX - rFootX);

  if (footDiff > 1e-3)
  {
    std::cerr << "FAIL: Feet are not aligned side-by-side! L_x=" << lFootX << ", R_x=" << rFootX << ", diff=" << footDiff << std::endl;
    return 1;
  }
  if (std::abs(lFootX - expectedX) > 1e-3)
  {
    std::cerr << "FAIL: Final foot position (" << lFootX << ") does not match expected (" << expectedX << ")" << std::endl;
    return 1;
  }
  if (std::abs(gen4.CoMVelocity().x()) > 0.02 || std::abs(gen4.CoMVelocity().y()) > 0.02)
  {
    std::cerr << "FAIL: CoM velocity did not settle to ~0: vx=" << gen4.CoMVelocity().x() << ", vy=" << gen4.CoMVelocity().y() << std::endl;
    return 1;
  }

  std::cout << "  [PASS] Num steps stopping verified (Feet side-by-side at X = " << lFootX << " m, diff = " << footDiff << " m, CoM settled)." << std::endl;

  std::cout << "All LIPMGenerator tests PASSED successfully." << std::endl;
  return 0;
}
