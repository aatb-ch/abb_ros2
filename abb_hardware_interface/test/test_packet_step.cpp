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

// Per-sent-packet step limit: after an inbound gap the first reply must not
// carry the catch-up of an open-loop controller.

#include "hw_test_fixture.hpp"

using namespace abb_test;
using namespace std::chrono_literals;

namespace
{
const Joints START{ 10.0, 20.0, -30.0, 40.0, 30.0, -60.0 };  // deg
constexpr double VMAX = 2.0;                                  // rad/s (URDF limit in the fixture)
}

class PacketStep : public HwTest
{
protected:
  // An open-loop controller moves joint_1 at 1.5 rad/s; the link drops for
  // 60 ms in the middle (shorter than the 300 ms grace). Returns the largest
  // step between consecutive replies [deg].
  double gapStep(const std::map<std::string, std::string>& params)
  {
    startRobot(START);
    robot_->setFollow(true);
    robot_->start();
    auto p = params;
    p["egm_loss_grace_ms"] = "300";
    init(p, VMAX);
    EXPECT_TRUE(activate());
    claim(true);
    double q = START[0] * DEG;
    auto controller = [&] {
      q += 1.5 * 0.002;
      command("joint_1", q);
    };
    run(100, controller);
    robot_->clearReplies();
    robot_->setSending(false);
    run(30, controller);  // 60 ms gap
    robot_->setSending(true);
    run(100, controller);
    EXPECT_TRUE(active_);
    return maxStep(robot_->replies());
  }
};

TEST_F(PacketStep, OffByDefaultTheGapCatchUpGoesOutInOnePacket)
{
  const double step = gapStep({});
  RecordProperty("max_step_deg", std::to_string(step));
  EXPECT_GT(step, 3.0);  // ~1.5 rad/s x 64 ms, documents the default (off)
}

TEST_F(PacketStep, EnabledNoReplyStepsMoreThanTheLimit)
{
  const double step = gapStep({ { "egm_max_step_scale", "1.0" } });
  RecordProperty("max_step_deg", std::to_string(step));
  EXPECT_LE(step, 1.0 * VMAX * 0.004 / DEG + 1e-6);
}

// Below the limit, the limit changes nothing.
TEST_F(PacketStep, EnabledSteadyMotionIsUnaffected)
{
  startRobot(START);
  robot_->setFollow(true);
  robot_->start();
  init({ { "egm_max_step_scale", "1.0" } }, VMAX);
  ASSERT_TRUE(activate());
  claim(true);
  double q = START[0] * DEG;
  run(300, [&] {
    q += 1.5 * 0.002;
    command("joint_1", q);
  });
  const auto r = robot_->replies();
  ASSERT_FALSE(r.empty());
  EXPECT_NEAR(r.back().position[0], q / DEG, 1.5 * 0.006 / DEG);
}
