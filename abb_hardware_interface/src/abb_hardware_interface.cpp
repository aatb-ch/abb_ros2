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

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <abb_hardware_interface/utilities.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
static constexpr size_t NUM_CONNECTION_TRIES = 100;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemHardware");
static const std::string NOT_ACTIVE{ "the hardware is not active" };

// The joint names from the robot controller description carry a mechanical
// unit prefix (e.g. "ROB_1_joint_1"); the interfaces are exported under the
// name from "joint" on. A robot description may prefix its joint names as
// well (e.g. "irb4600_joint_1", several robots in one ROS graph): export the
// interfaces under the ros2_control joint whose name ends in that same
// "joint..." suffix, else under the stripped name (upstream behaviour).
static std::string urdfJointName(const std::string& rws_name,
                                 const std::vector<hardware_interface::ComponentInfo>& joints)
{
  const auto pos = rws_name.find("joint");
  const std::string stripped = pos == std::string::npos ? rws_name : rws_name.substr(pos);
  for (const auto& joint : joints)
  {
    const std::string& name = joint.name;
    if (name.size() > stripped.size() &&
        name.compare(name.size() - stripped.size(), stripped.size(), stripped) == 0 &&
        name[name.size() - stripped.size() - 1] == '_')
    {
      return name;
    }
  }
  return stripped;
}

CallbackReturn ABBSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // How long the EGM session may be not streaming / not running before the
  // hardware deactivates itself. Default 50 ms (~12 EGM cycles).
  const auto grace_it = info_.hardware_parameters.find("egm_loss_grace_ms");
  if (grace_it != info_.hardware_parameters.end())
  {
    egm_loss_grace_ = std::chrono::milliseconds(std::stoi(grace_it->second));
  }

  // Per-sent-packet step limit (off unless > 0), see enforcePacketStep().
  const auto step_it = info_.hardware_parameters.find("egm_max_step_scale");
  if (step_it != info_.hardware_parameters.end())
  {
    max_step_scale_ = std::stod(step_it->second);
  }
  if (max_step_scale_ > 0.0)
  {
    RCLCPP_INFO(LOGGER, "Per-packet step limit: %.2f x max_velocity per EGM sample", max_step_scale_);
  }

  // EGM speedRef feed-forward, see shapeVelocity().
  const auto vmode_it = info_.hardware_parameters.find("velocity_command_mode");
  if (vmode_it != info_.hardware_parameters.end())
  {
    if (vmode_it->second == "feedforward")
    {
      velocity_mode_ = VelocityMode::FEEDFORWARD;
    }
    else if (vmode_it->second == "position_only")
    {
      velocity_mode_ = VelocityMode::POSITION_ONLY;
    }
    else if (vmode_it->second == "scaled")
    {
      velocity_mode_ = VelocityMode::SCALED;
    }
    else
    {
      RCLCPP_FATAL(LOGGER, "velocity_command_mode '%s': expected feedforward, position_only or scaled",
                   vmode_it->second.c_str());
      return CallbackReturn::ERROR;
    }
  }
  const auto vscale_it = info_.hardware_parameters.find("velocity_feedforward_scale");
  if (vscale_it != info_.hardware_parameters.end())
  {
    velocity_scale_ = std::stod(vscale_it->second);
    if (!(velocity_scale_ >= 0.0 && velocity_scale_ <= 1.0))
    {
      RCLCPP_FATAL(LOGGER, "velocity_feedforward_scale must be in [0, 1]");
      return CallbackReturn::ERROR;
    }
  }

  // Validate interfaces configured in ros2_control xacro.
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // By default, construct the robot_controller_description_ by connecting to RWS.
  // If configure_via_rws is set to false, configure the robot_controller_description_
  // relying on joint information in the ros2_control xacro.
  const auto configure_it = info_.hardware_parameters.find("configure_via_rws");
  const bool configure_via_rws = configure_it == info_.hardware_parameters.end()                    ? true :
                                 configure_it->second == "false" || configure_it->second == "False" ? false :
                                                                                                      true;

  if (configure_via_rws)
  {
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from RWS.");
    const auto rws_port = stoi(info_.hardware_parameters["rws_port"]);
    const auto rws_ip = info_.hardware_parameters["rws_ip"];

    if (rws_ip == "None")
    {
      RCLCPP_FATAL(LOGGER, "RWS IP not specified");
      return CallbackReturn::ERROR;
    }

    // Get robot controller description from RWS. Bounded (5 attempts, as the
    // ROS 1 driver's RWS node), not waiting indefinitely: an endless retry kept
    // a start-up hostage when the controller's session table was full. The
    // manager (and its RWS session, logged out by librws when it is destroyed)
    // lives only for this block.
    try
    {
      abb::robot::RWSManager rws_manager(rws_ip, rws_port, "Default User", "robotics");
      robot_controller_description_ = abb::robot::utilities::establishRWSConnection(rws_manager, "IRB1200", false);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(LOGGER, "No robot controller description via RWS: %s", e.what());
      return CallbackReturn::ERROR;
    }
  }
  else
  {
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from HardwareInfo.");

    // Add header.
    auto header{ robot_controller_description_.mutable_header() };
    // Omnicore controllers have RobotWare version >=7.0.0.
    header->mutable_robot_ware_version()->set_major_number(7);
    header->mutable_robot_ware_version()->set_minor_number(3);
    header->mutable_robot_ware_version()->set_patch_number(2);

    // Add system indicators.
    auto system_indicators{ robot_controller_description_.mutable_system_indicators() };
    system_indicators->mutable_options()->set_egm(true);

    // Add single mechanical units group.
    auto mug{ robot_controller_description_.add_mechanical_units_groups() };
    mug->set_name("");

    // Add single robot to mechanical units group.
    auto robot{ mug->mutable_robot() };
    robot->set_type(abb::robot::MechanicalUnit_Type_TCP_ROBOT);
    robot->set_axes_total(info_.joints.size());
    robot->set_mode(abb::robot::MechanicalUnit_Mode_ACTIVATED);

    // Add joints to robot.
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const hardware_interface::ComponentInfo& joint = info_.joints[i];
      // We assume it's a revolute joint unless explicitly specified.
      // Check if a "type" key is present in joint.parameters with value other than "revolute"
      // as per sdformat conventions http://sdformat.org/spec?elem=joint.
      const auto type_it = joint.parameters.find("type");
      const bool is_revolute = type_it != joint.parameters.end() && type_it->second != "revolute" ? false : true;

      // Get the range of the joint from its command interfaces.
      for (const hardware_interface::InterfaceInfo& joint_info : joint.command_interfaces)
      {
        if (joint_info.name == hardware_interface::HW_IF_POSITION)
        {
          const double min = std::stod(joint_info.min);
          const double max = std::stod(joint_info.max);

          abb::robot::StandardizedJoint* p_joint = robot->add_standardized_joints();
          p_joint->set_standardized_name(joint.name);
          p_joint->set_rotating_move(is_revolute);
          p_joint->set_lower_joint_bound(min);
          p_joint->set_upper_joint_bound(max);

          RCLCPP_INFO(LOGGER, "Configured component %s of type %s with range [%.3f, %.3f]", joint.name.c_str(),
                      joint.type.c_str(), min, max);
          break;
        }
      }
    }
  }

  RCLCPP_INFO_STREAM(LOGGER, "Robot controller description:\n"
                                 << abb::robot::summaryText(robot_controller_description_));

  // Configure EGM
  RCLCPP_INFO(LOGGER, "Configuring EGM interface...");

  // Initialize motion data from robot controller description
  try
  {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description_);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  // Create channel configuration for each mechanical unit group
  std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
  for (const auto& group : robot_controller_description_.mechanical_units_groups())
  {
    try
    {
      const auto egm_port = stoi(info_.hardware_parameters[group.name() + "egm_port"]);
      const auto channel_configuration =
          abb::robot::EGMManager::ChannelConfiguration{ static_cast<uint16_t>(egm_port), group };
      channel_configurations.emplace_back(channel_configuration);
      RCLCPP_INFO_STREAM(LOGGER,
                         "Configuring EGM for mechanical unit group " << group.name() << " on port " << egm_port);
    }
    catch (std::invalid_argument& e)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "EGM port for mechanical unit group \"" << group.name()
                                                                          << "\" not specified in hardware parameters");
      return CallbackReturn::ERROR;
    }
  }
  try
  {
    egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
    return CallbackReturn::ERROR;
  }

  if (!initCommandLimits())
  {
    return CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto& group : motion_data_.groups)
    for (const auto& unit : group.units)
      for (const auto& joint : unit.joints)
        joint_names_.push_back(urdfJointName(joint.name, info_.joints));
  if (joint_names_.size() > 64)
  {
    RCLCPP_FATAL(LOGGER, "%zu joints: at most 64 supported", joint_names_.size());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

bool ABBSystemHardware::initCommandLimits()
{
  // Soft joint limits, as the ROS 1
  // abb_robot_driver enforced them: every command is checked against the
  // URDF <limit> of its joint, `soft_limit_margin` (hardware parameter, rad,
  // default 0.00872665 = 0.5 deg) inside it. The margin matters: a command AT
  // the hard limit trips abb_egm_rws_managers' own bounds check (it throws,
  // the hardware errors out) after the deg conversion, and the controller's
  // range supervision faults when the arm reaches the limit still moving.
  // Fail closed: a joint without position limits refuses to load.
  const auto margin_it = info_.hardware_parameters.find("soft_limit_margin");
  if (margin_it != info_.hardware_parameters.end())
  {
    soft_limit_margin_ = std::stod(margin_it->second);
  }
  command_limits_.clear();
  for (const auto& group : motion_data_.groups)
  {
    for (const auto& unit : group.units)
    {
      for (const auto& joint : unit.joints)
      {
        const auto name = urdfJointName(joint.name, info_.joints);
        const auto it = info_.limits.find(name);
        if (it == info_.limits.end() || !it->second.has_position_limits)
        {
          RCLCPP_FATAL(LOGGER, "Joint '%s' has no URDF position limits - refusing to command it",
                       name.c_str());
          return false;
        }
        const auto& lim = it->second;
        // As the ROS 1 driver (egm_hardware_interface.cpp:631-652): never
        // wider than the bounds the robot controller reports (via RWS, or the
        // ros2_control min/max without RWS). abb_egm_rws_managers throws on a
        // command outside those bounds, which puts the hardware into ERROR.
        double lo_bound = lim.min_position;
        double hi_bound = lim.max_position;
        if (joint.lower_limit < joint.upper_limit)
        {
          if (joint.lower_limit > lo_bound + 1e-9 || joint.upper_limit < hi_bound - 1e-9)
          {
            RCLCPP_WARN(LOGGER, "Joint '%s': URDF limits [%.4f, %.4f] narrowed to the robot controller's [%.4f, %.4f]",
                        name.c_str(), lo_bound, hi_bound, joint.lower_limit, joint.upper_limit);
          }
          lo_bound = std::max(lo_bound, joint.lower_limit);
          hi_bound = std::min(hi_bound, joint.upper_limit);
        }
        const double lo = lo_bound + soft_limit_margin_;
        const double hi = hi_bound - soft_limit_margin_;
        if (!(lo < hi))
        {
          RCLCPP_FATAL(LOGGER, "Joint '%s': soft_limit_margin leaves no range", name.c_str());
          return false;
        }
        command_limits_.push_back(CommandLimit{ name, lo, hi, lim.has_velocity_limits, lim.max_velocity, 0.0 });
        RCLCPP_INFO(LOGGER, "Soft limits %s: [%.4f, %.4f] rad, %.4f rad/s%s", name.c_str(), lo, hi,
                    lim.max_velocity, lim.has_velocity_limits ? "" : " (none)");
        if (!lim.has_velocity_limits)
        {
          RCLCPP_WARN(LOGGER, "Joint '%s' has no URDF velocity limit - position limits only", name.c_str());
        }
      }
    }
  }
  return true;
}

void ABBSystemHardware::enforceCommandLimits(double period_s)
{
  // Position: clamp into [min, max]. Velocity: the position may move at most
  // max_velocity x period from the last position sent, and the velocity
  // command is clamped to +-max_velocity. A clamp means something upstream
  // (a trajectory validation layer, the JTC) let a bad command through: it is logged.
  size_t k = 0;
  bool clamped = false;
  std::string what;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        if (k >= command_limits_.size())
        {
          return;
        }
        auto& lim = command_limits_[k++];
        const double wanted = joint.command.position;
        double pos = std::isfinite(wanted) ? wanted : lim.last_position;
        pos = std::clamp(pos, lim.min_position, lim.max_position);
        if (lim.has_velocity && period_s > 0.0)
        {
          const double step = lim.max_velocity * period_s;
          pos = std::clamp(pos, lim.last_position - step, lim.last_position + step);
        }
        double vel = std::isfinite(joint.command.velocity) ? joint.command.velocity : 0.0;
        if (lim.has_velocity)
        {
          vel = std::clamp(vel, -lim.max_velocity, lim.max_velocity);
        }
        if (pos <= lim.min_position || pos >= lim.max_position)
        {
          // at a position limit: never push further out
          if ((pos <= lim.min_position && vel < 0.0) || (pos >= lim.max_position && vel > 0.0))
          {
            vel = 0.0;
          }
        }
        if (!(std::abs(pos - wanted) <= 1e-9) || !(std::abs(vel - joint.command.velocity) <= 1e-9))
        {
          clamped = true;
          what = lim.name;
        }
        joint.command.position = pos;
        joint.command.velocity = vel;
        lim.last_position = pos;
      }
    }
  }
  if (clamped)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_limit_log_ >= std::chrono::seconds(1))
    {
      last_limit_log_ = now;
      RCLCPP_ERROR(LOGGER, "soft limits: command for %s clamped to its URDF limits", what.c_str());
    }
  }
}

std::vector<hardware_interface::StateInterface> ABBSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        const auto joint_name = urdfJointName(joint.name, info_.joints);
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        // TODO(seng): Consider changing joint names in robot description to match what comes
        // from the ABB robot description to avoid needing to strip the prefix here
        const auto joint_name = urdfJointName(joint.name, info_.joints);
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.command.position));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity));
      }
    }
  }

  return command_interfaces;
}

CallbackReturn ABBSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  size_t counter = 0;
  RCLCPP_INFO(LOGGER, "Connecting to robot...");
  while (rclcpp::ok() && counter++ < NUM_CONNECTION_TRIES)
  {
    // Wait for a message on any of the configured EGM channels.
    if (egm_manager_->waitForMessage(500))
    {
      RCLCPP_INFO(LOGGER, "Connected to robot");
      break;
    }

    RCLCPP_INFO(LOGGER, "Not connected to robot...");
    if (counter == NUM_CONNECTION_TRIES)
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
      return CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(500ms);
  }

  // EGMManager::Channel::read() only takes over the joint states once the
  // channel counts as active, i.e. once a message with a sequence number
  // different from the previous one was read. The first message of an EGM
  // session has seqno 0, equal to the default-constructed "previous" header,
  // so a single read() here leaves every joint state at 0.0 - and the command
  // below would then send the robot to all-zero joint angles. Keep reading
  // until real states arrived.
  bool have_states = false;
  for (size_t i = 0; i < NUM_CONNECTION_TRIES && rclcpp::ok() && !have_states; ++i)
  {
    have_states = egm_manager_->read(motion_data_);
    if (!have_states)
    {
      egm_manager_->waitForMessage(500);
    }
  }
  if (!have_states)
  {
    RCLCPP_ERROR(LOGGER, "No joint states received over EGM");
    return CallbackReturn::ERROR;
  }
  have_states_ = true;

  // Messages may have queued up while nothing answered them (abb_libegm
  // serves one datagram at a time and waits up to 24 ms for an answer to
  // each): answer them all, so the target below is latched from the newest
  // message and not from one that is hundreds of ms old.
  drainEgm();

  // No controller claims anything yet: hold where the robot is, latched
  // once. A controller activated afterwards starts from it.
  claimed_position_ = 0;
  claimed_velocity_ = 0;
  rt_position_claims_ = 0;
  rt_velocity_claims_ = 0;
  latchHold("activation");
  applyHold();

  // Soft limits: the step bound starts from the target that is sent.
  for (size_t k = 0; k < command_limits_.size() && k < hold_position_.size(); ++k)
  {
    command_limits_[k].last_position = hold_position_[k];
  }

  // Only a running session can be commanded (the ROS 1 driver refused to
  // start controllers otherwise, egm_hardware_interface.cpp:281-337).
  std::string why;
  if (!robotReportsRunning(why))
  {
    RCLCPP_ERROR(LOGGER, "Not activating: %s", why.c_str());
    publishSession(false, why);
    return CallbackReturn::FAILURE;
  }

  // A fresh session: supervision starts over.
  const auto now = std::chrono::steady_clock::now();
  channels_.assign(motion_data_.groups.size(), ChannelSupervision{});
  superviseSession(now);
  egm_lost_ = false;
  egm_lost_reason_.clear();

  active_ = true;
  publishSession(true, "");
  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // ros2_control deactivates the controllers claiming this hardware's
  // command interfaces along with it: hold where the robot is (latched once,
  // velocity 0). read() keeps answering EGM with it while inactive.
  active_ = false;
  claimed_position_ = 0;
  claimed_velocity_ = 0;
  rt_position_claims_ = 0;
  rt_velocity_claims_ = 0;
  if (have_states_)
  {
    latchHold("deactivation");
    applyHold();
  }
  publishSession(false, NOT_ACTIVE);
  return CallbackReturn::SUCCESS;
}

// The two switch callbacks run on the controller manager's non-realtime
// thread, possibly while read()/write() run. They share nothing with the
// realtime thread but atomics (and the refusal reason, under a mutex), as the
// ROS 1 driver's prepareSwitch() only read the EGM status flags the last read()
// left (egm_hardware_interface.cpp:287-288).
return_type ABBSystemHardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                          const std::vector<std::string>& /*stop_interfaces*/)
{
  // Refuse to let a controller claim a command interface unless the EGM
  // session is running (ROS 1: prepareSwitch(), egm_hardware_interface.cpp
  // :281-337). Releasing is always allowed.
  bool claims = false;
  for (const auto& full_name : start_interfaces)
  {
    for (const auto& joint : joint_names_)
    {
      claims = claims || full_name.rfind(joint + "/", 0) == 0;
    }
  }
  if (!claims || session_ok_)
  {
    return return_type::OK;
  }
  std::string why;
  {
    std::lock_guard<std::mutex> lock(session_why_mutex_);
    why = session_why_;
  }
  RCLCPP_ERROR(LOGGER, "Refusing to start a controller: %s", why.c_str());
  return return_type::ERROR;
}

return_type ABBSystemHardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                          const std::vector<std::string>& stop_interfaces)
{
  // Only track which joints' interfaces a controller claims; the realtime
  // thread acts on a release (see syncClaims()).
  auto mask = [&](const std::vector<std::string>& interfaces, const char* type) {
    uint64_t m = 0;
    for (const auto& full_name : interfaces)
    {
      for (size_t k = 0; k < joint_names_.size(); ++k)
      {
        if (full_name == joint_names_[k] + "/" + type)
        {
          m |= uint64_t{ 1 } << k;
        }
      }
    }
    return m;
  };
  claimed_position_ &= ~mask(stop_interfaces, hardware_interface::HW_IF_POSITION);
  claimed_velocity_ &= ~mask(stop_interfaces, hardware_interface::HW_IF_VELOCITY);
  claimed_position_ |= mask(start_interfaces, hardware_interface::HW_IF_POSITION);
  claimed_velocity_ |= mask(start_interfaces, hardware_interface::HW_IF_VELOCITY);
  return return_type::OK;
}

void ABBSystemHardware::syncClaims()
{
  // Realtime thread: take over the claims. When a controller released a
  // joint's position (e.g. stopped mid-move), hold where the robot is, once,
  // rather than the controller's last command.
  const uint64_t position = claimed_position_;
  const uint64_t velocity = claimed_velocity_;
  if ((rt_position_claims_ & ~position) != 0 && have_states_)
  {
    latchHold("controller released");
  }
  rt_position_claims_ = position;
  rt_velocity_claims_ = velocity;
}

void ABBSystemHardware::publishSession(bool ok, const std::string& why)
{
  // For prepare_command_mode_switch(); the reason only changes on events.
  if (ok != session_ok_ || why != published_why_)
  {
    published_why_ = why;
    std::lock_guard<std::mutex> lock(session_why_mutex_);
    session_why_ = why;
  }
  session_ok_ = ok;
}

void ABBSystemHardware::latchHold(const char* why)
{
  // The hold target is the measured position (as in the stock and ROS 1
  // drivers), taken ONCE per event: activation, a session (re)start, a
  // controller release, deactivation. Unlike the ROS 1 driver (command =
  // measured position on every read) it is never re-sampled while the robot
  // servos on it: a reference that follows the measurement each cycle follows
  // encoder noise and servo error, and the arm can creep or sag.
  hold_position_.clear();
  for (const auto& group : motion_data_.groups)
    for (const auto& unit : group.units)
      for (const auto& joint : unit.joints)
        hold_position_.push_back(joint.state.position);
  hold_valid_ = true;
  if (max_step_scale_ > 0.0)
  {
    last_sent_ = hold_position_;  // the step limit starts from the hold
    pending_ = hold_position_;
  }
  if (why)
  {
    RCLCPP_INFO(LOGGER, "Idle hold latched (%s) at the measured position", why);
  }
}

void ABBSystemHardware::applyHold()
{
  // Joints whose position no controller claims get the latched target;
  // velocities no controller claims are 0.
  if (!hold_valid_)
  {
    return;
  }
  size_t k = 0;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        const uint64_t bit = uint64_t{ 1 } << k;
        if (k < hold_position_.size() && !(rt_position_claims_ & bit))
        {
          joint.command.position = hold_position_[k];
        }
        if (!(rt_velocity_claims_ & bit))
        {
          joint.command.velocity = 0.0;
        }
        ++k;
      }
    }
  }
}

void ABBSystemHardware::answerEgmWithHold(double period_s)
{
  if (!hold_valid_)
  {
    return;
  }
  applyHold();
  // The target is constant (or the robot's own reference): no step bound
  // from an older command.
  for (size_t k = 0; k < command_limits_.size() && k < hold_position_.size(); ++k)
  {
    command_limits_[k].last_position = hold_position_[k];
  }
  enforceCommandLimits(period_s);
  enforcePacketStep();
  egm_manager_->write(motion_data_);
}

void ABBSystemHardware::shapeVelocity()
{
  // Feed-forward of the controller's velocity into EGM speedRef. With
  // pos_corr_gain < 1 on the robot, a full feed-forward made fast moves
  // overshoot their target (10 % at 60 deg/s, 43 % at 120 deg/s with a
  // reversal on a virtual controller); position_only removed it (0.3-0.8 %)
  // at the cost of ~140 ms more lag. Which one to use depends on the robot's
  // EGM settings: tune on the robot.
  if (velocity_mode_ == VelocityMode::FEEDFORWARD)
  {
    return;
  }
  const double scale = velocity_mode_ == VelocityMode::SCALED ? velocity_scale_ : 0.0;
  for (auto& group : motion_data_.groups)
    for (auto& unit : group.units)
      for (auto& joint : unit.joints)
        joint.command.velocity *= scale;
}

void ABBSystemHardware::enforcePacketStep()
{
  // Limit the step per EGM reply relative to the reply actually SENT before
  // (not to the last position written: during an inbound gap the controller
  // keeps writing, nothing is sent, and the first reply after the gap would
  // carry the whole catch-up - up to 11.7 deg in one 4 ms packet was seen on
  // a virtual controller). With this limit, a gap shorter than the loss grace
  // is caught up at no more than scale x max_velocity; a longer gap ends the
  // session (see superviseSession()). The soft limits' per-cycle step bound
  // stays as it is.
  if (max_step_scale_ <= 0.0 || last_sent_.size() != command_limits_.size() ||
      pending_.size() != command_limits_.size())
  {
    return;
  }
  size_t k = 0;
  bool clamped = false;
  std::string what;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        const auto& lim = command_limits_[k];
        if (lim.has_velocity)
        {
          const double step = max_step_scale_ * lim.max_velocity * sample_time_s_;
          const double pos = std::clamp(joint.command.position, last_sent_[k] - step, last_sent_[k] + step);
          if (pos != joint.command.position)
          {
            clamped = true;
            what = lim.name;
            joint.command.position = pos;
          }
        }
        pending_[k] = joint.command.position;
        ++k;
      }
    }
  }
  if (clamped)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_step_log_ >= std::chrono::seconds(1))
    {
      last_step_log_ = now;
      RCLCPP_ERROR(LOGGER, "per-packet step limit: command for %s clamped (link gap or too fast a command)",
                   what.c_str());
    }
  }
}

void ABBSystemHardware::trackSession()
{
  // An EGM session (re)starts when the robot reports EGM running + RAPID
  // running + motors on again, or when the sequence number restarts. A
  // session that starts while nothing claims the interfaces gets a freshly
  // latched target: the arm may have been moved (RAPID, a jog) in between,
  // and an old target would drive it back.
  // Robot-reported states only: channel activity (a read-count heuristic
  // in the managers) flaps with link jitter and is not a new session.
  std::string why;
  const bool running = !motion_data_.groups.empty() && robotReportsRunning(why);
  const bool restarted = restarted_;
  if (rt_position_claims_ == 0 && have_states_)
  {
    if (!running)
    {
      // The robot is not servoing on EGM references now (EGM, RAPID or the
      // motors stopped), so its reference may move (RAPID, a jog). Track it:
      // abb_libegm keeps the last output for the first message of a resumed
      // session, and that must be where the robot is, not an old target.
      latchHold(nullptr);
    }
    else if (!session_running_ || restarted)
    {
      latchHold("EGM session (re)started");
    }
  }
  session_running_ = running;
}

void ABBSystemHardware::drainEgm()
{
  // Each queued message is served by abb_libegm as soon as the previous one
  // was answered, so a wake-up within 1 ms means "queued"; at the robot's
  // own pace (>= 4 ms) the queue is empty.
  for (int i = 0; i < 2000 && rclcpp::ok(); ++i)
  {
    latchHold(nullptr);
    applyHold();
    egm_manager_->write(motion_data_);
    const auto t0 = std::chrono::steady_clock::now();
    const bool got = egm_manager_->waitForMessage(20);
    const auto waited = std::chrono::steady_clock::now() - t0;
    egm_manager_->read(motion_data_);
    if (!got || waited >= 1ms)
    {
      if (i > 0)
      {
        RCLCPP_INFO(LOGGER, "Drained %d queued EGM message(s)", i);
      }
      return;
    }
  }
}

bool ABBSystemHardware::robotReportsRunning(std::string& why) const
{
  for (const auto& group : motion_data_.groups)
  {
    const auto& channel = group.egm_channel_data;
    const auto& status = channel.input.status();
    if (status.egm_state() != abb::egm::wrapper::Status::EGM_RUNNING)
    {
      why = "EGM on channel '" + channel.name + "' is not running (" +
            abb::egm::wrapper::Status::EGMState_Name(status.egm_state()) + ")";
      return false;
    }
    // As abb_libegm's statesOk() (egm_base_interface.cpp): RAPID UNDEFINED
    // counts as running (an EGM bug reports it after a controller restart
    // although RAPID runs).
    if (status.rapid_execution_state() != abb::egm::wrapper::Status::RAPID_RUNNING &&
        status.rapid_execution_state() != abb::egm::wrapper::Status::RAPID_UNDEFINED)
    {
      why = "RAPID is not running (channel '" + channel.name + "')";
      return false;
    }
    if (status.motor_state() != abb::egm::wrapper::Status::MOTORS_ON)
    {
      why = "the motors are off (channel '" + channel.name + "')";
      return false;
    }
  }
  return true;
}

void ABBSystemHardware::superviseSession(std::chrono::steady_clock::time_point now)
{
  // Sequence numbers, per channel. The managers only count reads with an
  // unchanged number (5 reads = 10 ms at 500 Hz, not ~250 ms as in the
  // synchronous ROS 1 loop), so supervision here is by time:
  //  * a higher number is a new message;
  //  * the same number is a duplicate read (the controller manager runs
  //    faster than EGM) or a duplicated datagram: not fresh;
  //  * a lower number below SESSION_START_SEQNO is a new session (EGM
  //    restarted); any other lower number is a stale, reordered datagram:
  //    ignored.
  static constexpr uint32_t SESSION_START_SEQNO = 50;  // first 200 ms of a session
  restarted_ = false;
  new_message_ = false;
  if (channels_.size() != motion_data_.groups.size())
  {
    channels_.assign(motion_data_.groups.size(), ChannelSupervision{});
  }
  for (size_t g = 0; g < motion_data_.groups.size(); ++g)
  {
    const auto& channel = motion_data_.groups[g].egm_channel_data;
    auto& sup = channels_[g];
    const uint32_t seqno = channel.input.header().sequence_number();
    if (!have_states_)
    {
      continue;
    }
    const uint32_t stamp_ms = channel.input.header().time_stamp();
    if (!sup.seen || seqno > sup.seqno)
    {
      if (sup.seen && seqno == sup.seqno + 1 && stamp_ms > sup.stamp_ms && stamp_ms - sup.stamp_ms <= 100)
      {
        // The robot's own sample time: the smallest time-stamp step between
        // consecutive messages (a pause on the robot side shows up larger).
        // (EGM samples at 4 ms or slower.)
        const double dt = std::max(0.004, (stamp_ms - sup.stamp_ms) / 1000.0);
        if (!sample_time_known_ || dt < sample_time_s_)
        {
          sample_time_s_ = dt;
          sample_time_known_ = true;
        }
      }
      sup.seen = true;
      sup.seqno = seqno;
      sup.stamp_ms = stamp_ms;
      sup.last_new = now;
      new_message_ = true;
    }
    else if (seqno < sup.seqno && seqno < SESSION_START_SEQNO)
    {
      sup.seqno = seqno;
      sup.stamp_ms = stamp_ms;
      sup.last_new = now;
      restarted_ = true;
      new_message_ = true;
    }
    const bool silent = now - sup.last_new > egm_loss_grace_;
    if (silent != sup.silent)
    {
      sup.silent = silent;
      if (silent)
      {
        RCLCPP_WARN(LOGGER, "No EGM message on channel '%s' for more than %ld ms (did the EGM session end?)",
                    channel.name.c_str(), static_cast<long>(egm_loss_grace_.count()));
      }
      else
      {
        RCLCPP_INFO(LOGGER, "EGM messages on channel '%s' resumed", channel.name.c_str());
      }
    }
    // As the ROS 1 driver (egm_hardware_interface.cpp:221-229).
    if (channel.input.status().utilization_rate() > 100.0 && now - last_utilization_log_ > std::chrono::seconds(10))
    {
      last_utilization_log_ = now;
      RCLCPP_WARN(LOGGER,
                  "EGM utilization rate on channel '%s' is %.0f %% (the commanded motion is too aggressive and "
                  "should be reduced)",
                  channel.name.c_str(), channel.input.status().utilization_rate());
    }
  }

  // While active: a robot-reported stop or a new session ends the session at
  // once (the ROS 1 driver reset its controllers on every such change,
  // egm_hardware_interface.cpp:239-246); silence beyond the grace too.
  if (!active_ || egm_lost_ || !have_states_)
  {
    return;
  }
  std::string why;
  if (!robotReportsRunning(why))
  {
    egm_lost_ = true;
  }
  else if (restarted_)
  {
    egm_lost_ = true;
    why = "a new EGM session started";
  }
  else
  {
    for (size_t g = 0; g < channels_.size(); ++g)
    {
      if (channels_[g].silent)
      {
        egm_lost_ = true;
        why = "no EGM message on channel '" + motion_data_.groups[g].egm_channel_data.name + "' for more than " +
              std::to_string(egm_loss_grace_.count()) + " ms";
        break;
      }
    }
  }
  if (egm_lost_)
  {
    egm_lost_reason_ = why;
  }
}

return_type ABBSystemHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (egm_manager_->read(motion_data_))
  {
    have_states_ = true;
  }
  superviseSession(std::chrono::steady_clock::now());
  if (max_step_scale_ > 0.0 && new_message_ && pending_.size() == last_sent_.size())
  {
    // The robot only sends its next message after it got our reply to the
    // previous one: whatever was written last went out with that reply.
    last_sent_ = pending_;
  }
  syncClaims();
  trackSession();
  {
    std::string why;
    const bool running = robotReportsRunning(why);
    publishSession(active_ && !egm_lost_ && running, !active_ ? NOT_ACTIVE : egm_lost_ ? egm_lost_reason_ : why);
  }

  // ros2_control calls read() but not write() while the hardware is
  // inactive. Answer EGM here then, with the latched target: otherwise
  // abb_libegm replies to every message with the last command it got (the
  // robot drives to it when a session resumes) and blocks 24 ms per message
  // (old messages queue up).
  if (!active_)
  {
    answerEgmWithHold(period.seconds());
  }

  // EGM session supervision: write() returns DEACTIVATE once the session
  // is lost, and ros2_control deactivates this hardware and every
  // controller claiming its command interfaces in that cycle (the ROS 1
  // driver's controller reset + egm_controller_stopper). Activating it
  // again starts from the robot's reference (see on_activate()).
  return return_type::OK;
}

return_type ABBSystemHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (egm_lost_)
  {
    RCLCPP_ERROR(LOGGER, "%s - deactivating the hardware and its controllers (activate it again once EGM runs)",
                 egm_lost_reason_.c_str());
    return return_type::DEACTIVATE;
  }
  syncClaims();
  applyHold();
  shapeVelocity();
  enforceCommandLimits(period.seconds());
  enforcePacketStep();
  egm_manager_->write(motion_data_);
  return return_type::OK;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemHardware, hardware_interface::SystemInterface)
