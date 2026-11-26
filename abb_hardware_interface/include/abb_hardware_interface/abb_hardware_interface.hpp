// Copyright 2020 ROS2-Control Development Team
// Modifications Copyright 2022 PickNik Inc
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

#pragma once

#include <abb_egm_rws_managers/egm_manager.h>
#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_hardware_interface/visibility_control.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace abb_hardware_interface
{
class ABBSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ABBSystemHardware)

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // EGM
  abb::robot::RobotControllerDescription robot_controller_description_;
  std::unique_ptr<abb::robot::EGMManager> egm_manager_;

  // Store the state and commands for the robot(s)
  abb::robot::MotionData motion_data_;

  // EGM session supervision (see read()/write()): when EGM stops streaming
  // or stops running, the hardware deactivates itself so ros2_control
  // deactivates the controllers commanding it.
  // Robot-reported states (EGM running, RAPID running, motors on) are
  // checked on every new message and fail at once; only silence (no new
  // sequence number) gets egm_loss_grace_.
  bool robotReportsRunning(std::string& why) const;
  void superviseSession(std::chrono::steady_clock::time_point now);
  struct ChannelSupervision
  {
    bool seen{ false };
    uint32_t seqno{ 0 };
    uint32_t stamp_ms{ 0 };
    std::chrono::steady_clock::time_point last_new{};
    bool silent{ false };
  };
  std::vector<ChannelSupervision> channels_;
  bool restarted_{ false };   // a sequence number restart was seen in this read()
  std::chrono::steady_clock::time_point last_utilization_log_{};
  bool egm_lost_{ false };
  std::string egm_lost_reason_;
  std::chrono::milliseconds egm_loss_grace_{ 50 };

  // Soft joint limits on the commands, from the URDF <limit>:
  // one entry per joint, in motion_data_ iteration order.
  struct CommandLimit
  {
    std::string name;
    double min_position;
    double max_position;
    bool has_velocity;
    double max_velocity;
    double last_position;   // last position sent (seeded at activation)
  };
  bool initCommandLimits();
  void enforceCommandLimits(double period_s);
  std::vector<CommandLimit> command_limits_;
  double soft_limit_margin_{ 0.00872665 };   // rad (0.5 deg) inside the URDF limits
  std::chrono::steady_clock::time_point last_limit_log_{};

  // Idle hold: while no controller claims a joint's position command
  // interface, the robot gets a constant target with velocity 0 - the last
  // command (after a controller release / deactivation), re-latched ONCE
  // from the robot's own reference at activation and at every EGM session
  // (re)start. Never re-sampled while the robot servos on it. While the
  // hardware is inactive, read() answers EGM with it.
  void latchHold(const char* why);
  void holdLastCommand();
  void applyHold();
  void answerEgmWithHold(double period_s);
  void trackSession();
  void drainEgm();
  bool anyPositionClaimed() const;
  std::vector<std::string> joint_names_;   // exported names, motion_data_ order
  std::vector<double> hold_position_;      // latched target [rad or m]
  bool hold_valid_{ false };
  std::vector<bool> claimed_position_;
  std::vector<bool> claimed_velocity_;
  bool active_{ false };
  bool have_states_{ false };
  bool session_running_{ false };

  // What the robot actually got: the robot sends its next message only after
  // our reply to the previous one, so when a new sequence number arrives,
  // the last position written (pending_) went out (last_sent_). During an
  // inbound gap a controller keeps writing, but none of it is sent.
  void notePending();
  std::vector<double> last_sent_;
  std::vector<double> pending_;

  // Per-sent-packet step limit (egm_max_step_scale > 0; off by default): the
  // position sent may differ from the position SENT with the previous EGM
  // reply by at most scale x max_velocity x the robot's sample time.
  void enforcePacketStep();
  double max_step_scale_{ 0.0 };
  bool new_message_{ false };       // a new sequence number in this read()
  double sample_time_s_{ 0.004 };   // from the robot's header time stamps (4 ms until known)
  bool sample_time_known_{ false };
  std::chrono::steady_clock::time_point last_step_log_{};
};

}  // namespace abb_hardware_interface
