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

// Test fixture: an ABBSystemHardware driven the way ros2_control drives it
// (read() in ACTIVE and INACTIVE, write() only in ACTIVE, DEACTIVATE from
// write() -> on_deactivate() -> INACTIVE), at a fixed controller rate, against
// a FakeEgmRobot on localhost.

#pragma once

#include <gtest/gtest.h>

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fake_egm_robot.hpp"

namespace abb_test
{
constexpr double DEG = M_PI / 180.0;

inline hardware_interface::HardwareInfo makeInfo(uint16_t egm_port,
                                                 const std::map<std::string, std::string>& params = {},
                                                 double max_velocity = 2.0)
{
  hardware_interface::HardwareInfo info;
  info.name = "abb_test";
  info.type = "system";
  info.hardware_plugin_name = "abb_hardware_interface/ABBSystemHardware";
  info.hardware_parameters["configure_via_rws"] = "false";
  info.hardware_parameters["egm_port"] = std::to_string(egm_port);
  for (const auto& kv : params)
  {
    info.hardware_parameters[kv.first] = kv.second;
  }
  const double lower[6] = { -3.0, -1.5, -3.0, -6.9, -2.1, -6.9 };
  const double upper[6] = { 3.0, 2.6, 1.3, 6.9, 2.0, 6.9 };
  for (int i = 0; i < 6; ++i)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i + 1);
    joint.type = "joint";
    hardware_interface::InterfaceInfo pos;
    pos.name = "position";
    pos.min = std::to_string(lower[i]);
    pos.max = std::to_string(upper[i]);
    hardware_interface::InterfaceInfo vel;
    vel.name = "velocity";
    joint.command_interfaces = { pos, vel };
    hardware_interface::InterfaceInfo spos;
    spos.name = "position";
    hardware_interface::InterfaceInfo svel;
    svel.name = "velocity";
    joint.state_interfaces = { spos, svel };
    info.joints.push_back(joint);

    joint_limits::JointLimits lim;
    lim.has_position_limits = true;
    lim.min_position = lower[i];
    lim.max_position = upper[i];
    lim.has_velocity_limits = true;
    lim.max_velocity = max_velocity;
    info.limits[joint.name] = lim;
  }
  return info;
}

class HwTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  // Each test gets its own port, so a previous test's sockets never interfere.
  static uint16_t nextPort()
  {
    static uint16_t port = 26511;
    return port++;
  }

  void startRobot(const Joints& start_deg)
  {
    port_ = nextPort();
    robot_ = std::make_unique<FakeEgmRobot>(port_, start_deg);
  }

  // on_init + export; the robot is started (if wanted) before activation.
  void init(const std::map<std::string, std::string>& params = {}, double max_velocity = 2.0)
  {
    hw_ = std::make_unique<abb_hardware_interface::ABBSystemHardware>();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    ASSERT_EQ(hw_->on_init(makeInfo(port_, params, max_velocity)), CallbackReturn::SUCCESS);
    state_ifs_ = hw_->export_state_interfaces();
    cmd_ifs_ = hw_->export_command_interfaces();
#pragma GCC diagnostic pop
  }

  bool activate()
  {
    active_ = hw_->on_activate(rclcpp_lifecycle::State()) == CallbackReturn::SUCCESS;
    return active_;
  }

  double state(const std::string& joint, const std::string& iface = "position")
  {
    for (auto& s : state_ifs_)
    {
      if (s.get_prefix_name() == joint && s.get_interface_name() == iface)
      {
        return s.get_optional<double>().value();
      }
    }
    ADD_FAILURE() << "no state interface " << joint << "/" << iface;
    return NAN;
  }

  void command(const std::string& joint, double value, const std::string& iface = "position")
  {
    for (auto& c : cmd_ifs_)
    {
      if (c.get_prefix_name() == joint && c.get_interface_name() == iface)
      {
        (void)c.set_value(value);
        return;
      }
    }
    ADD_FAILURE() << "no command interface " << joint << "/" << iface;
  }

  double commanded(const std::string& joint, const std::string& iface = "position")
  {
    for (auto& c : cmd_ifs_)
    {
      if (c.get_prefix_name() == joint && c.get_interface_name() == iface)
      {
        return c.get_optional<double>().value();
      }
    }
    return NAN;
  }

  // One controller-manager cycle. `controller` runs between read() and write()
  // only while the hardware is active (it stands for an active controller).
  // Returns write()'s result (OK while inactive).
  return_type cycle(const std::function<void()>& controller = nullptr)
  {
    const rclcpp::Time now = clock_.now();
    const rclcpp::Duration period = rclcpp::Duration(cm_period_);
    EXPECT_EQ(hw_->read(now, period), return_type::OK);
    if (!active_)
    {
      return return_type::OK;
    }
    if (controller)
    {
      controller();
    }
    const auto ret = hw_->write(now, period);
    if (ret == return_type::DEACTIVATE)
    {
      EXPECT_EQ(hw_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
      active_ = false;
    }
    return ret;
  }

  // Runs n cycles at the controller rate; stops early (returning the count)
  // when the hardware deactivated itself.
  int run(int n, const std::function<void()>& controller = nullptr, bool stop_on_deactivate = true)
  {
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i)
    {
      const bool was_active = active_;
      cycle(controller);
      if (stop_on_deactivate && was_active && !active_)
      {
        return i + 1;
      }
      next += cm_period_;
      std::this_thread::sleep_until(next);
    }
    return n;
  }

  // Largest per-reply change of any joint [deg] over the given replies.
  static double maxStep(const std::vector<EgmReply>& r)
  {
    double m = 0.0;
    for (size_t k = 1; k < r.size(); ++k)
    {
      for (int i = 0; i < 6; ++i)
      {
        m = std::max(m, std::abs(r[k].position[i] - r[k - 1].position[i]));
      }
    }
    return m;
  }

  // Largest deviation [deg] of any reply from a reference position.
  static double maxDeviation(const std::vector<EgmReply>& r, const Joints& ref)
  {
    double m = 0.0;
    for (const auto& x : r)
    {
      for (int i = 0; i < 6; ++i)
      {
        m = std::max(m, std::abs(x.position[i] - ref[i]));
      }
    }
    return m;
  }

  void TearDown() override
  {
    hw_.reset();  // EGM servers first, then the robot
    if (robot_)
    {
      robot_->stop();
    }
  }

  uint16_t port_{ 0 };
  std::unique_ptr<FakeEgmRobot> robot_;
  std::unique_ptr<abb_hardware_interface::ABBSystemHardware> hw_;
  std::vector<hardware_interface::StateInterface> state_ifs_;
  std::vector<hardware_interface::CommandInterface> cmd_ifs_;
  bool active_{ false };
  std::chrono::milliseconds cm_period_{ 2 };  // 500 Hz controller manager
  rclcpp::Clock clock_{ RCL_STEADY_TIME };
};

}  // namespace abb_test
