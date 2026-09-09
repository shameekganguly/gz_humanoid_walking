#include <gtest/gtest.h>
#include "LIPMGenerator.hh"

#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

using namespace gz_humanoid_walking;

/// \brief Test fixture for LIPMGenerator unit tests
class LIPMGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_.dt = 0.005;
    config_.z_c = 0.795;
    config_.stepDuration = 1.00;
    config_.dspDuration = 0.50;
    config_.initDspDuration = 0.50;
    config_.zmpMargin = 0.065;
    config_.footSeparation = 0.192;
  }

  LIPMConfig config_;
};

/// \brief Test DARE preview gain convergence and residual metrics
TEST_F(LIPMGeneratorTest, DAREConvergence)
{
  LIPMGenerator gen(config_);

  EXPECT_TRUE(gen.DareConverged());
  EXPECT_GT(gen.DareIterations(), 0);
  EXPECT_LT(gen.DareIterations(), 100);
  EXPECT_LT(gen.DareResidual(), 1e-6);
}

/// \brief Test initialization and initial double support state
TEST_F(LIPMGeneratorTest, InitialStateAndDoubleSupport)
{
  LIPMGenerator gen(config_);

  const Eigen::Vector3d initCoM(0.0, 0.0, 0.78);
  const Eigen::Vector3d initLeft(0.0, 0.096, 0.0);
  const Eigen::Vector3d initRight(0.0, -0.096, 0.0);

  gen.Initialize(initCoM, initLeft, initRight);
  gen.SetVelocity(0.15);

  EXPECT_EQ(gen.CurrentSupportState(), SupportState::DOUBLE_SUPPORT);
  EXPECT_NEAR(gen.CoMPosition().z(), config_.z_c, 1e-4);
  EXPECT_NEAR(gen.LeftFootPosition().y(), 0.096, 1e-4);
  EXPECT_NEAR(gen.RightFootPosition().y(), -0.096, 1e-4);
  EXPECT_FALSE(gen.IsStopped());
}

/// \brief Test multi-cycle walking, no NaNs, support state transitions, and forward progression
TEST_F(LIPMGeneratorTest, WalkCycleAndForwardProgression)
{
  LIPMGenerator gen(config_);

  const Eigen::Vector3d initCoM(0.0, 0.0, 0.78);
  const Eigen::Vector3d initLeft(0.0, 0.096, 0.0);
  const Eigen::Vector3d initRight(0.0, -0.096, 0.0);

  gen.Initialize(initCoM, initLeft, initRight);
  gen.SetVelocity(0.15);

  bool visitedLeftSupport = false;
  bool visitedRightSupport = false;

  // Run 600 steps (3.0s -> multiple full step cycles)
  for (int i = 0; i < 600; ++i)
  {
    gen.Step();

    EXPECT_FALSE(std::isnan(gen.CoMPosition().x())) << "NaN in CoM x at step " << i;
    EXPECT_FALSE(std::isnan(gen.CoMPosition().y())) << "NaN in CoM y at step " << i;
    EXPECT_FALSE(std::isnan(gen.CoMPosition().z())) << "NaN in CoM z at step " << i;

    if (gen.CurrentSupportState() == SupportState::LEFT_SUPPORT)
      visitedLeftSupport = true;
    if (gen.CurrentSupportState() == SupportState::RIGHT_SUPPORT)
      visitedRightSupport = true;
  }

  // Verify support state transitions occurred
  EXPECT_TRUE(visitedLeftSupport);
  EXPECT_TRUE(visitedRightSupport);

  // CoM should advance forward significantly (> 0.10 m)
  EXPECT_GT(gen.CoMPosition().x(), 0.10);

  // CoM height should stay clamped to configured z_c
  EXPECT_NEAR(gen.CoMPosition().z(), config_.z_c, 1e-4);
}

/// \brief Test swing foot parabolic clearance trajectory
TEST_F(LIPMGeneratorTest, SwingFootClearance)
{
  LIPMConfig swingConfig = config_;
  swingConfig.footClearance = 0.05;
  swingConfig.dspDuration = 0.10;
  swingConfig.initDspDuration = 0.20;

  LIPMGenerator gen(swingConfig);
  gen.Initialize({0, 0, 0.78}, {0, 0.096, 0}, {0, -0.096, 0});
  gen.SetVelocity(0.2);

  double maxSwingHeight = 0.0;
  for (int i = 0; i < 300; ++i)
  {
    gen.Step();
    if (gen.CurrentSupportState() == SupportState::RIGHT_SUPPORT)
    {
      maxSwingHeight = std::max(maxSwingHeight, gen.LeftFootPosition().z());
    }
  }

  // Apex clearance should be within reasonable bounds around configured footClearance (0.05 m)
  EXPECT_GE(maxSwingHeight, 0.04);
  EXPECT_LE(maxSwingHeight, 0.06);
}

/// \brief Test walking progression along rotated heading (90 degrees, facing world +Y)
TEST_F(LIPMGeneratorTest, RotatedHeading90Degrees)
{
  LIPMConfig rotConfig = config_;
  rotConfig.stepDuration = 0.8;
  rotConfig.dspDuration = 0.10;
  rotConfig.initDspDuration = 0.20;

  LIPMGenerator gen(rotConfig);
  const double yaw90 = M_PI / 2.0;

  // In 90-deg yaw: left foot is at (-0.096, 0), right foot is at (+0.096, 0)
  gen.Initialize({0, 0, 0.78}, {-0.096, 0.0, 0.0}, {0.096, 0.0, 0.0}, yaw90);
  gen.SetVelocity(0.15); // Forward in Body frame -> +Y in World frame

  for (int i = 0; i < 600; ++i)
  {
    gen.Step();
  }

  // World Y should advance forward (> 0.10 m)
  EXPECT_GT(gen.CoMPosition().y(), 0.10);

  // World X should remain close to 0 with minimal lateral drift
  EXPECT_NEAR(gen.CoMPosition().x(), 0.0, 0.05);
}

/// \brief Test walking stop after finite numSteps and side-by-side feet alignment
TEST_F(LIPMGeneratorTest, StoppingAndFeetAlignment)
{
  LIPMConfig stepConfig = config_;
  stepConfig.stepDuration = 1.0;
  stepConfig.dspDuration = 0.50;
  stepConfig.initDspDuration = 0.50;
  stepConfig.numSteps = 4;

  LIPMGenerator gen(stepConfig);
  gen.Initialize({0, 0, 0.78}, {0, 0.096, 0}, {0, -0.096, 0});
  gen.SetVelocity(0.10); // 0.1 m/s -> stepLen = 0.1 m, final expected X = (4 - 1) * 0.1 = 0.3 m

  // Step for 8 seconds (1600 iterations)
  for (int i = 0; i < 1600; ++i)
  {
    gen.Step();
  }

  EXPECT_TRUE(gen.IsStopped());
  EXPECT_EQ(gen.CurrentSupportState(), SupportState::DOUBLE_SUPPORT);

  const double expectedX = (4 - 1) * 0.10; // 0.3 m
  const double lFootX = gen.LeftFootPosition().x();
  const double rFootX = gen.RightFootPosition().x();

  // Feet should be aligned side-by-side within 1mm
  EXPECT_NEAR(lFootX, rFootX, 1e-3);
  EXPECT_NEAR(lFootX, expectedX, 1e-3);

  // CoM velocity should settle to near zero (< 0.02 m/s)
  EXPECT_NEAR(gen.CoMVelocity().x(), 0.0, 0.02);
  EXPECT_NEAR(gen.CoMVelocity().y(), 0.0, 0.02);
}

/// \brief Test that commanded velocity updates are latched only at step boundaries
TEST_F(LIPMGeneratorTest, StepBoundaryVelocityLatching)
{
  LIPMConfig latchConfig = config_;
  latchConfig.stepDuration = 1.0;
  latchConfig.initDspDuration = 0.5;
  latchConfig.dspDuration = 0.2;

  LIPMGenerator gen(latchConfig);
  gen.Initialize({0, 0, 0.78}, {0, 0.096, 0}, {0, -0.096, 0});
  gen.SetVelocity(0.10);

  EXPECT_DOUBLE_EQ(gen.CommandedVelocity(), 0.10);
  EXPECT_DOUBLE_EQ(gen.ActiveVelocity(), 0.10);

  // Step into middle of step 0 (totalTime = 0.5s initDsp + 0.3s into step 0 = 0.8s)
  // 0.8s / 0.005s = 160 steps
  for (int i = 0; i < 160; ++i)
  {
    gen.Step();
  }

  // Active velocity during step 0 should still be 0.10
  EXPECT_DOUBLE_EQ(gen.ActiveVelocity(), 0.10);

  // Command a new velocity mid-swing
  gen.SetVelocity(0.25);
  EXPECT_DOUBLE_EQ(gen.CommandedVelocity(), 0.25);
  // Active velocity must NOT change mid-swing
  gen.Step();
  EXPECT_DOUBLE_EQ(gen.ActiveVelocity(), 0.10);

  // Step until step 1 begins (totalTime reaches 0.5s + 1.0s = 1.5s -> 300 steps)
  // Currently at step 161; step to 305 steps
  for (int i = 161; i < 305; ++i)
  {
    gen.Step();
  }

  // At step 1, the new velocity should now be latched
  EXPECT_DOUBLE_EQ(gen.ActiveVelocity(), 0.25);
}

/// \brief Test concurrent thread-safe SetVelocity calls while simulation thread executes Step()
TEST_F(LIPMGeneratorTest, ConcurrentVelocityUpdates)
{
  LIPMGenerator gen(config_);
  gen.Initialize({0, 0, 0.78}, {0, 0.096, 0}, {0, -0.096, 0});
  gen.SetVelocity(0.10);

  std::atomic<bool> running{true};

  // Background thread simulating asynchronous Gazebo Transport callbacks
  std::thread transportThread([&]() {
    double v = 0.05;
    while (running.load(std::memory_order_relaxed))
    {
      gen.SetVelocity(v);
      v += 0.01;
      if (v > 0.30) v = 0.05;
      std::this_thread::yield();
    }
  });

  // Simulation thread executing physics / trajectory steps
  for (int i = 0; i < 400; ++i)
  {
    gen.Step();
    EXPECT_FALSE(std::isnan(gen.CoMPosition().x()));
    EXPECT_FALSE(std::isnan(gen.CoMPosition().y()));
    EXPECT_FALSE(std::isnan(gen.CoMPosition().z()));
  }

  running.store(false, std::memory_order_relaxed);
  transportThread.join();

  EXPECT_GT(gen.CoMPosition().x(), 0.05);
}
