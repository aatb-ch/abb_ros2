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

// Idle hold: with no controller claiming the command interfaces the robot is
// sent one target, latched once from its own reference; never a stale
// command, never a re-sampled (drifting) one, never a queued old state.

#include "hw_test_fixture.hpp"

using namespace abb_test;
using namespace std::chrono_literals;

namespace
{
const Joints START{ 10.0, 20.0, -30.0, 40.0, 30.0, -60.0 };  // deg
const Joints SAG{ 0.0, -0.4, 0.3, 0.0, -0.2, 0.0 };          // measured - reference
}

// Idle: the target is the robot's reference (not the sagging, noisy
// measurement) and stays bit-identical - no creep, however long.
TEST_F(HwTest, IdleHoldIsTheReferenceAndBitIdentical)
{
  startRobot(START);
  robot_->setSag(SAG);
  robot_->setNoise(0.05);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  robot_->clearReplies();
  run(2500);  // 5 s idle
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 1000u);
  for (const auto& x : r)
  {
    for (int i = 0; i < 6; ++i)
    {
      ASSERT_EQ(x.position[i], r.front().position[i]) << "hold target changed while idle";
      ASSERT_EQ(x.speed[i], 0.0);
    }
  }
  EXPECT_LT(maxDeviation(r, START), 1e-6) << "hold is not the robot's reference";
}

// A controller moves the arm, then releases the interfaces mid-motion: the
// robot keeps the last position command (no re-sampling, no step) and gets
// no speed reference any more; a stray write while unclaimed is ignored.
TEST_F(HwTest, ControllerReleaseKeepsPositionDropsVelocity)
{
  startRobot(START);
  robot_->setFollow(true);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  claim(true);
  double q = START[0] * DEG;
  run(500, [&] {
    q += 0.5 * 0.002;  // 0.5 rad/s on joint_1
    command("joint_1", q);
    command("joint_1", 0.5, "velocity");
  });
  ASSERT_GT(robot_->replies().back().speed[0], 1.0);
  claim(false);
  run(2);  // the reply already queued in abb_libegm goes out first
  robot_->clearReplies();
  run(500, [&] { command("joint_1", q + 1.0); });  // a stray write, unclaimed
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 100u);
  EXPECT_EQ(maxStep(r), 0.0);
  EXPECT_NEAR(r.front().position[0], q / DEG, 0.005 / DEG);  // the last one sent (a few cycles back)
  for (const auto& x : r)
  {
    EXPECT_EQ(x.speed[0], 0.0);
  }
}

// A link gap, then the hardware deactivates on silence while the controller
// had kept advancing: when the link is back, the robot must get the last
// command it actually got, never the catch-up that was written but not sent.
TEST_F(HwTest, DeactivationAfterGapHoldsWhatWasSent)
{
  startRobot(START);
  robot_->setFollow(true);
  robot_->start();
  init({ { "egm_loss_grace_ms", "100" } });
  ASSERT_TRUE(activate());
  claim(true);
  double q = START[0] * DEG;
  auto controller = [&] {
    q += 0.5 * 0.002;
    command("joint_1", q);
  };
  run(250, controller);
  const double before_gap = robot_->replies().back().position[0];
  robot_->setSending(false);
  run(250, controller);  // 0.5 s gap: the controller runs ahead by ~14 deg
  ASSERT_FALSE(active_);
  robot_->clearReplies();
  robot_->setSending(true);
  run(250);
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 50u);
  // The first reply may still carry a write from the gap's first 10 ms:
  // abb_egm_rws_managers forwards writes until 5 reads saw no new message
  // (10 ms at 500 Hz), then drops them, so the hold cannot replace it in
  // abb_libegm before the next message. Bounded by 5 cycles of motion (the
  // per-packet step limit bounds it to one step). Everything after: the hold.
  EXPECT_NEAR(r.front().position[0], before_gap, 0.5);
  std::vector<EgmReply> rest(r.begin() + 1, r.end());
  EXPECT_EQ(maxStep(rest), 0.0);
  EXPECT_NEAR(rest.front().position[0], before_gap, 0.06) << "the robot got a command that was never sent before";
}

// The resume snap: a controller drove the arm, the session stopped (RAPID
// stop -> the hardware deactivates), the arm was moved by RAPID, RAPID
// resumes the same session. The robot must get its new position as the
// target, never the old command.
TEST_F(HwTest, NoStaleTargetAfterResumeWhileInactive)
{
  startRobot(START);
  robot_->setFollow(true);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  claim(true);
  double q = START[0] * DEG;
  run(500, [&] {
    q = std::min(q + 0.5 * 0.002, (START[0] + 20.0) * DEG);
    command("joint_1", q);
  });
  robot_->setStates(true, true, false);  // RAPID stops
  run(500);
  ASSERT_FALSE(active_);
  const Joints moved{ -5.0, 10.0, -20.0, 30.0, 45.0, -50.0 };
  robot_->setPosition(moved);  // RAPID moved the arm
  run(250);
  robot_->setStates(true, true, true);  // RAPID resumes the session
  robot_->clearReplies();
  run(500);  // the hardware stays inactive; only read() runs
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 100u);
  EXPECT_LT(maxDeviation(r, moved), 1e-6) << "the robot was sent an old target";
  const auto p = robot_->position();
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(p[i], moved[i], 1e-6);
  }
}

// Nothing answered EGM for a while (e.g. between start-up and activation):
// activation seeds from the newest message, not from a queue of old ones.
TEST_F(HwTest, ActivationAfterUnansweredPeriodUsesNewestState)
{
  startRobot(START);
  robot_->start();
  init();
  Joints p = START;
  for (int k = 0; k < 100; ++k)  // 1 s, joint_1 moves at 10 deg/s, unanswered
  {
    p[0] += 0.1;
    robot_->setPosition(p);
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(activate());
  EXPECT_NEAR(state("joint_1") / DEG, robot_->position()[0], 0.2);
  robot_->clearReplies();
  run(100);
  EXPECT_EQ(maxStep(robot_->replies()), 0.0);
  EXPECT_LT(maxDeviation(robot_->replies(), robot_->reference()), 1e-6);
}
