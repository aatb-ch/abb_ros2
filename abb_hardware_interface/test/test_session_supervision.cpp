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

// Session supervision: what the robot reports ends the session at once;
// only silence gets the grace; a laggy link (jitter, duplicates, reordering)
// within the grace never ends it.

#include "hw_test_fixture.hpp"

using namespace abb_test;
using namespace std::chrono_literals;

namespace
{
const Joints START{ 10.0, 20.0, -30.0, 40.0, 30.0, -60.0 };  // deg

// Cycles until the hardware deactivated itself, as a duration.
std::chrono::milliseconds timeToDeactivate(HwTest* t, const std::function<int()>& run_until)
{
  const auto t0 = std::chrono::steady_clock::now();
  run_until();
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
}
}  // namespace

class Supervision : public HwTest
{
protected:
  void up(const std::string& grace_ms = "500")
  {
    startRobot(START);
    robot_->start();
    init({ { "egm_loss_grace_ms", grace_ms } });
    ASSERT_TRUE(activate());
    claim(true);
    run(50);
    ASSERT_TRUE(active_);
  }
};

TEST_F(Supervision, RapidStopEndsTheSessionAtOnce)
{
  up();
  robot_->setStates(true, true, false);
  const auto dt = timeToDeactivate(this, [&] { return run(1000); });
  EXPECT_FALSE(active_);
  EXPECT_LT(dt, 60ms) << "waited for the loss grace";
}

TEST_F(Supervision, EgmStopEndsTheSessionAtOnce)
{
  up();
  robot_->setStates(true, false, true);
  const auto dt = timeToDeactivate(this, [&] { return run(1000); });
  EXPECT_FALSE(active_);
  EXPECT_LT(dt, 60ms);
}

TEST_F(Supervision, MotorsOffEndsTheSessionAtOnce)
{
  up();
  robot_->setStates(false, true, true);
  const auto dt = timeToDeactivate(this, [&] { return run(1000); });
  EXPECT_FALSE(active_);
  EXPECT_LT(dt, 60ms);
}

TEST_F(Supervision, NewSessionEndsTheOldOne)
{
  up();
  robot_->newSession();
  const auto dt = timeToDeactivate(this, [&] { return run(1000); });
  EXPECT_FALSE(active_);
  EXPECT_LT(dt, 60ms);
}

TEST_F(Supervision, SilenceGetsTheGrace)
{
  up("200");
  robot_->setSending(false);
  const auto dt = timeToDeactivate(this, [&] { return run(1000); });
  EXPECT_FALSE(active_);
  EXPECT_GE(dt, 200ms);
  EXPECT_LT(dt, 300ms);
}

// Bunching / gaps shorter than the grace, duplicated and reordered datagrams:
// tracking may degrade, the session does not end.
TEST_F(Supervision, LaggyLinkWithinGraceKeepsTheSession)
{
  up("300");
  run(250);  // past the first 200 ms: an old datagram from then would read as a new session (fails safe)
  robot_->setDuplicate(true);
  for (int k = 0; k < 10; ++k)
  {
    robot_->setSending(false);
    run(100);  // a 200 ms gap
    robot_->setSending(true);
    robot_->sendStaleOnce(100);  // an old datagram arrives late
    run(100);
    ASSERT_TRUE(active_) << "gap " << k;
  }
}

TEST_F(Supervision, ControllerStartRefusedUnlessRunning)
{
  up();
  claim(false);
  robot_->setStates(true, false, true);
  run(100);
  ASSERT_FALSE(active_);
  std::vector<std::string> names{ "joint_1/position" };
  EXPECT_EQ(hw_->prepare_command_mode_switch(names, {}), return_type::ERROR);
}

TEST_F(Supervision, ActivationRefusedUnlessRunning)
{
  startRobot(START);
  robot_->setStates(true, false, true);  // streaming, EGM not running
  robot_->start();
  init();
  EXPECT_FALSE(activate());
  robot_->setStates(true, true, true);
  EXPECT_TRUE(activate());
}
