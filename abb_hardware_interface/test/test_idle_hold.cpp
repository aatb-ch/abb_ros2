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
// sent one target, latched once from the measured position; never a stale
// command, never a re-sampled (drifting) one, never a queued old state.

#include "hw_test_fixture.hpp"

using namespace abb_test;
using namespace std::chrono_literals;

namespace
{
const Joints START{ 10.0, 20.0, -30.0, 40.0, 30.0, -60.0 };  // deg
const Joints SAG{ 0.0, -0.4, 0.3, 0.0, -0.2, 0.0 };          // measured - reference

Joints plus(const Joints& a, const Joints& b)
{
  Joints r;
  for (int i = 0; i < 6; ++i)
  {
    r[i] = a[i] + b[i];
  }
  return r;
}

// Every reply equals the first one, bit for bit, with no speed reference.
void expectConstantHold(const std::vector<EgmReply>& r)
{
  ASSERT_FALSE(r.empty());
  for (const auto& x : r)
  {
    for (int i = 0; i < 6; ++i)
    {
      ASSERT_EQ(x.position[i], r.front().position[i]) << "hold target changed while idle";
      ASSERT_EQ(x.speed[i], 0.0);
    }
  }
}
}  // namespace

// Idle: the target is the measured position, sampled once, and stays
// bit-identical however noisy the measurement - no creep, no sag follow.
TEST_F(HwTest, IdleHoldIsLatchedOnceAndBitIdentical)
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
  expectConstantHold(r);
  EXPECT_LT(maxDeviation(r, plus(START, SAG)), 0.05 + 1e-6) << "hold is not the measured position";
}

// A controller drives joint_1 faster than the arm follows, then is stopped
// mid-move (the JTC keeps its last position command, velocity 0): the robot
// holds where it is when released - once, velocity 0, bit-identical for 2 s
// of idle - never the controller's last command, which is far ahead. A stray
// write while unclaimed is ignored.
TEST_F(HwTest, MidMoveReleaseHoldsWhereTheRobotIs)
{
  startRobot(START);
  robot_->setFollow(true, 20.0);  // the arm follows at most 20 deg/s
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  claim(true);
  double q = START[0] * DEG;
  run(500, [&] {
    q += 0.5 * 0.002;  // 0.5 rad/s (28.6 deg/s) on joint_1
    command("joint_1", q);
    command("joint_1", 0.5, "velocity");
  });
  ASSERT_GT(robot_->replies().back().speed[0], 1.0);
  const double at_release = robot_->position()[0];
  ASSERT_GT(q / DEG - at_release, 3.0) << "the command should run ahead of the arm";
  claim(false);  // the JTC's on_deactivate keeps q, sets velocity 0
  // The target goes back from the last command to the latched position at
  // no more than max_velocity (the soft limits' step bound), so the arm runs
  // on a little and returns.
  double furthest = at_release;
  for (int k = 0; k < 250; ++k)
  {
    run(1);
    furthest = std::max(furthest, robot_->position()[0]);
  }
  RecordProperty("run_on_deg", std::to_string(furthest - at_release));
  // Here the command leads the arm by ~8.6 deg; holding the last command
  // (the previous behaviour) ran on all of it.
  EXPECT_LT(furthest - at_release, 3.0) << "the arm ran on towards the old command";
  robot_->clearReplies();
  run(1000, [&] { command("joint_1", q + 1.0); });  // 2 s idle; a stray write, unclaimed
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 400u);
  expectConstantHold(r);
  // Within the few cycles between the sample and the latch (20 deg/s).
  EXPECT_NEAR(r.front().position[0], at_release, 0.2);
  EXPECT_NEAR(robot_->position()[0], at_release, 0.2) << "the arm did not come to rest where it was released";
}

// A link gap, then the hardware deactivates on silence while the controller
// had kept advancing: when the link is back, the robot must hold where it
// was, never the catch-up that was written but not sent.
TEST_F(HwTest, DeactivationAfterGapHoldsWhereTheRobotIs)
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
  const double before_gap = robot_->position()[0];
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
  expectConstantHold(rest);
  EXPECT_NEAR(rest.front().position[0], before_gap, 0.15) << "the robot got a command that was never sent before";
}

// A new EGM session (sequence number restart) while a controller claims the
// interfaces, the arm moved in between: the hardware deactivates and holds
// the new session's position, never the old session's last command.
TEST_F(HwTest, RestartWhileClaimedHoldsTheNewSessionPosition)
{
  startRobot(START);
  robot_->setFollow(true);
  robot_->start();
  init();
  ASSERT_TRUE(activate());
  claim(true);
  double q = START[0] * DEG;
  auto controller = [&] {
    q += 0.5 * 0.002;
    command("joint_1", q);
  };
  run(250, controller);
  robot_->setFollow(false);
  const Joints moved{ -5.0, 10.0, -20.0, 30.0, 45.0, -50.0 };
  robot_->setPosition(moved);  // e.g. RAPID moved the arm between sessions
  robot_->newSession();
  run(100, controller);
  ASSERT_FALSE(active_);
  run(10);  // inactive: read() answers EGM
  robot_->clearReplies();
  run(500);
  const auto r = robot_->replies();
  ASSERT_GT(r.size(), 100u);
  expectConstantHold(r);
  EXPECT_LT(maxDeviation(r, moved), 1e-6) << "held the old session's command";
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
