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

// Command shaping: the speedRef feed-forward mode, and joint bounds never
// wider than what the robot controller reports.

#include "hw_test_fixture.hpp"

using namespace abb_test;
using namespace std::chrono_literals;

namespace
{
const Joints START{ 10.0, 20.0, -30.0, 40.0, 30.0, -60.0 };  // deg
}

class Shaping : public HwTest
{
protected:
  // speedRef [deg/s] sent for a 0.5 rad/s velocity command on joint_1.
  double speedRefFor(const std::map<std::string, std::string>& params)
  {
    startRobot(START);
    robot_->start();
    init(params);
    EXPECT_TRUE(activate());
    claim(true);
    const double q = START[0] * DEG;
    run(100, [&] {
      command("joint_1", q);
      command("joint_1", 0.5, "velocity");
    });
    return robot_->replies().back().speed[0];
  }
};

TEST_F(Shaping, FeedforwardIsTheDefault)
{
  EXPECT_NEAR(speedRefFor({}), 0.5 / DEG, 1e-6);
}

TEST_F(Shaping, PositionOnlySendsNoSpeedRef)
{
  EXPECT_EQ(speedRefFor({ { "velocity_command_mode", "position_only" } }), 0.0);
}

TEST_F(Shaping, ScaledSendsAFraction)
{
  EXPECT_NEAR(speedRefFor({ { "velocity_command_mode", "scaled" }, { "velocity_feedforward_scale", "0.5" } }),
              0.25 / DEG, 1e-6);
}

TEST_F(Shaping, UnknownModeRefusesToLoad)
{
  startRobot(START);
  hw_ = std::make_unique<abb_hardware_interface::ABBSystemHardware>();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(hw_->on_init(makeInfo(port_, { { "velocity_command_mode", "fast" } })), CallbackReturn::ERROR);
#pragma GCC diagnostic pop
}

// URDF limits wider than the robot controller's bounds: commands are clamped
// to the controller's bounds instead of tripping abb_egm_rws_managers'
// out-of-bounds exception (hardware ERROR).
TEST_F(Shaping, ControllerBoundsNarrowWiderUrdfLimits)
{
  startRobot(START);
  robot_->start();
  auto info = makeInfo(port_, {}, 2.0);
  info.limits["joint_1"].min_position = -4.0;  // controller bounds stay [-3, 3]
  info.limits["joint_1"].max_position = 4.0;
  hw_ = std::make_unique<abb_hardware_interface::ABBSystemHardware>();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  ASSERT_EQ(hw_->on_init(info), CallbackReturn::SUCCESS);
  state_ifs_ = hw_->export_state_interfaces();
  cmd_ifs_ = hw_->export_command_interfaces();
#pragma GCC diagnostic pop
  ASSERT_TRUE(activate());
  claim(true);
  return_type last = return_type::OK;
  EXPECT_NO_THROW(for (int i = 0; i < 1500 && last == return_type::OK; ++i) {
    last = cycle([&] { command("joint_1", 3.5); });
    std::this_thread::sleep_for(cm_period_);
  });
  EXPECT_EQ(last, return_type::OK);
  ASSERT_FALSE(robot_->replies().empty());
  EXPECT_LE(robot_->replies().back().position[0], (3.0 - 0.00872665) / DEG + 1e-6);
  EXPECT_GT(robot_->replies().back().position[0], 2.9 / DEG);
}
