// Copyright 2026 aatb-ch
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hw_test_fixture.hpp"

using namespace abb_test;
using namespace std::chrono_literals;

namespace
{
const Joints START{ 10.0, 20.0, -30.0, 40.0, 30.0, -60.0 };  // deg
}

// Activation seeds command = actual state; the robot is not sent to zero.
TEST_F(HwTest, ActivationSeedsFromActualNotZero)
{
  startRobot(START);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(state("joint_" + std::to_string(i + 1)), START[i] * DEG, 1e-6);
  }
  robot_->clearReplies();
  run(100);
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 20u);
  EXPECT_LT(maxDeviation(r, START), 1e-3);
}

// The robot stops sending: after egm_loss_grace_ms the hardware deactivates.
TEST_F(HwTest, EgmSilenceDeactivatesAfterGrace)
{
  startRobot(START);
  robot_->start();
  init({ { "egm_loss_grace_ms", "50" } });
  ASSERT_TRUE(activate());
  run(50);
  ASSERT_TRUE(active_);
  robot_->setSending(false);
  const auto t0 = std::chrono::steady_clock::now();
  run(500);
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_FALSE(active_);
  EXPECT_GE(dt, 45ms);  // counted from the last message, up to one period earlier
  EXPECT_LT(dt, 200ms);
}

// RAPID stops (reported by the robot): the hardware deactivates.
TEST_F(HwTest, RapidStopDeactivates)
{
  startRobot(START);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  run(50);
  ASSERT_TRUE(active_);
  robot_->setStates(true, true, false);
  run(500);
  EXPECT_FALSE(active_);
}

// A command beyond the URDF limit is clamped inside it (soft_limit_margin),
// and the position step per cycle is bounded by max_velocity x period.
TEST_F(HwTest, SoftLimitsClampPositionAndStep)
{
  startRobot(START);
  robot_->start();
  init({}, 1.0);  // 1 rad/s
  ASSERT_TRUE(activate());
  claim(true);
  robot_->clearReplies();
  run(400, [&] { command("joint_1", 10.0); });  // far beyond upper = 3.0 rad
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 50u);
  // at most 2 cycles (2 ms each) per 4 ms packet at 1 rad/s, plus jitter
  EXPECT_LE(maxStep(r), 1.0 * 0.006 / DEG + 1e-6);
  for (const auto& x : r)
  {
    EXPECT_LE(x.position[0], (3.0 - 0.00872665) / DEG + 1e-6);
  }
}

// A non-finite command holds the last position; nothing throws.
TEST_F(HwTest, NonFiniteCommandHolds)
{
  startRobot(START);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  claim(true);
  robot_->clearReplies();
  EXPECT_EQ(run(100, [&] { command("joint_2", std::nan("")); }), 100);
  EXPECT_TRUE(active_);
  EXPECT_LT(maxDeviation(robot_->replies(), START), 1e-3);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
