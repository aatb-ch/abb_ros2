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

    // Get robot controller description from RWS
    abb::robot::RWSManager rws_manager(rws_ip, rws_port, "Default User", "robotics");
    robot_controller_description_ = abb::robot::utilities::establishRWSConnection(rws_manager, "IRB1200", true);
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
  claimed_position_.assign(joint_names_.size(), false);
  claimed_velocity_.assign(joint_names_.size(), false);

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
        const double lo = lim.min_position + soft_limit_margin_;
        const double hi = lim.max_position - soft_limit_margin_;
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

  // No controller claims anything yet: hold the robot's own reference,
  // latched once. A controller activated afterwards starts from it.
  std::fill(claimed_position_.begin(), claimed_position_.end(), false);
  std::fill(claimed_velocity_.begin(), claimed_velocity_.end(), false);
  latchHold("activation");
  applyHold();

  // Soft limits: the step bound starts from the target that is sent.
  for (size_t k = 0; k < command_limits_.size() && k < hold_position_.size(); ++k)
  {
    command_limits_[k].last_position = hold_position_[k];
  }

  // A fresh session: supervision starts over (an activation only happens
  // once EGM streams again - see the wait above).
  egm_not_ok_ = false;
  egm_lost_ = false;
  egm_lost_reason_.clear();

  active_ = true;
  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ABBSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // ros2_control deactivates the controllers claiming this hardware's
  // command interfaces along with it: keep the last position the robot got
  // (not one written but never sent, e.g. during a link gap), drop the
  // velocity. read() keeps answering EGM with it while inactive.
  active_ = false;
  std::fill(claimed_position_.begin(), claimed_position_.end(), false);
  std::fill(claimed_velocity_.begin(), claimed_velocity_.end(), false);
  holdLastCommand();
  return CallbackReturn::SUCCESS;
}

return_type ABBSystemHardware::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                          const std::vector<std::string>& stop_interfaces)
{
  // Track which joints' interfaces a controller claims. When the last
  // position claim goes away, the robot keeps the last position it got (no
  // re-sampling) but no longer a velocity: a controller stopped mid-motion
  // would otherwise leave its last speed reference behind.
  const bool claimed_before = anyPositionClaimed();
  auto update = [&](const std::vector<std::string>& interfaces, bool claim) {
    for (const auto& full_name : interfaces)
    {
      for (size_t k = 0; k < joint_names_.size(); ++k)
      {
        if (full_name == joint_names_[k] + "/" + hardware_interface::HW_IF_POSITION)
        {
          claimed_position_[k] = claim;
        }
        else if (full_name == joint_names_[k] + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          claimed_velocity_[k] = claim;
        }
      }
    }
  };
  update(stop_interfaces, false);
  update(start_interfaces, true);
  if (claimed_before && !anyPositionClaimed())
  {
    holdLastCommand();
  }
  return return_type::OK;
}

bool ABBSystemHardware::anyPositionClaimed() const
{
  return std::find(claimed_position_.begin(), claimed_position_.end(), true) != claimed_position_.end();
}

void ABBSystemHardware::latchHold(const char* why)
{
  // The hold target is the robot's OWN reference (EGM "planned" joints) when
  // the robot reports it, else the measured position - taken once. Unlike the
  // ROS 1 driver (command = measured position on every read), it is never
  // re-sampled while idle: a reference that follows the measurement each
  // cycle follows encoder noise and servo error, and the arm can creep or sag.
  // The planned position also has no servo lag, so latching it during motion
  // does not step the reference backwards.
  hold_position_.clear();
  bool all_planned = true;
  for (const auto& group : motion_data_.groups)
  {
    const auto& planned = group.egm_channel_data.input.planned();
    const auto& robot = planned.robot().joints().position();
    const auto& external = planned.external().joints().position();
    int ri = 0;
    int ei = 0;
    for (const auto& unit : group.units)
    {
      const bool is_robot = unit.active && unit.type == abb::robot::MechanicalUnit_Type_TCP_ROBOT;
      const bool is_external = unit.active && (unit.type == abb::robot::MechanicalUnit_Type_ROBOT ||
                                               unit.type == abb::robot::MechanicalUnit_Type_SINGLE);
      for (const auto& joint : unit.joints)
      {
        const double f = joint.rotational ? abb::robot::Constants::DEG_TO_RAD : abb::robot::Constants::MM_TO_M;
        double p = std::numeric_limits<double>::quiet_NaN();
        if (is_robot && ri < robot.values_size())
        {
          p = robot.values(ri) * f;
        }
        else if (is_external && ei < external.values_size())
        {
          p = external.values(ei) * f;
        }
        ri += is_robot ? 1 : 0;
        ei += is_external ? 1 : 0;
        if (!std::isfinite(p))
        {
          p = joint.state.position;
          all_planned = false;
        }
        hold_position_.push_back(p);
      }
    }
  }
  hold_valid_ = true;
  last_sent_ = hold_position_;  // the robot's own reference
  pending_ = hold_position_;
  if (why)
  {
    RCLCPP_INFO(LOGGER, "Idle hold latched (%s) from the robot's %s position", why,
                all_planned ? "planned (reference)" : "measured");
  }
}

void ABBSystemHardware::holdLastCommand()
{
  if (!have_states_ || last_sent_.empty())
  {
    return;
  }
  hold_position_ = last_sent_;
  hold_valid_ = true;
  applyHold();
}

void ABBSystemHardware::notePending()
{
  pending_.clear();
  for (const auto& group : motion_data_.groups)
    for (const auto& unit : group.units)
      for (const auto& joint : unit.joints)
        pending_.push_back(joint.command.position);
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
        if (k < hold_position_.size() && !claimed_position_[k])
        {
          joint.command.position = hold_position_[k];
        }
        if (k < claimed_velocity_.size() && !claimed_velocity_[k])
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
  notePending();
  egm_manager_->write(motion_data_);
}

void ABBSystemHardware::trackSession()
{
  // An EGM session (re)starts when the robot reports EGM running + RAPID
  // running + motors on again, or when the sequence number restarts. A
  // session that starts while nothing claims the interfaces gets a freshly
  // latched target: the arm may have been moved (RAPID, a jog) in between,
  // and an old target would drive it back.
  bool running = !motion_data_.groups.empty();
  bool restarted = false;
  for (const auto& group : motion_data_.groups)
  {
    const auto& channel = group.egm_channel_data;
    const auto& status = channel.input.status();
    // Robot-reported states only: channel activity (a read-count heuristic
    // in the managers) flaps with link jitter and is not a new session.
    running = running && status.egm_state() == abb::egm::wrapper::Status::EGM_RUNNING &&
              status.rapid_execution_state() == abb::egm::wrapper::Status::RAPID_RUNNING &&
              status.motor_state() == abb::egm::wrapper::Status::MOTORS_ON;
    const uint32_t seqno = channel.input.header().sequence_number();
    restarted = restarted || seqno < last_seqno_;
    last_seqno_ = seqno;
  }
  if (!anyPositionClaimed() && have_states_)
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
    notePending();
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

bool ABBSystemHardware::egmSessionOk(std::string& why) const
{
  for (const auto& group : motion_data_.groups)
  {
    const auto& channel = group.egm_channel_data;
    if (!channel.is_active)
    {
      why = "EGM channel '" + channel.name + "' stopped streaming";
      return false;
    }
    if (channel.input.status().egm_state() != abb::egm::wrapper::Status::EGM_RUNNING)
    {
      why = "EGM on channel '" + channel.name + "' is not running (" +
            abb::egm::wrapper::Status::EGMState_Name(channel.input.status().egm_state()) + ")";
      return false;
    }
    if (channel.input.status().rapid_execution_state() != abb::egm::wrapper::Status::RAPID_RUNNING)
    {
      why = "RAPID is not running (channel '" + channel.name + "')";
      return false;
    }
  }
  return true;
}

return_type ABBSystemHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  if (egm_manager_->read(motion_data_))
  {
    have_states_ = true;
  }
  if (!motion_data_.groups.empty())
  {
    const uint32_t seqno = motion_data_.groups.front().egm_channel_data.input.header().sequence_number();
    if (seqno != last_seen_seqno_ && pending_.size() == last_sent_.size())
    {
      last_sent_ = pending_;
    }
    last_seen_seqno_ = seqno;
  }
  trackSession();

  // ros2_control calls read() but not write() while the hardware is
  // inactive. Answer EGM here then, with the latched target: otherwise
  // abb_libegm replies to every message with the last command it got (the
  // robot drives to it when a session resumes) and blocks 24 ms per message
  // (old messages queue up).
  if (!active_)
  {
    answerEgmWithHold(period.seconds());
  }

  // EGM session supervision. Upstream keeps the
  // hardware and its controllers active when EGM goes away: the channel
  // just stops updating (the joint states freeze with fresh stamps), the
  // JTC keeps interpolating into nothing and reports success, and when EGM
  // comes back the robot jumps to wherever the JTC's command is by then.
  // Here a session that is not streaming/running for longer than the grace
  // period latches egm_lost_, and write() returns DEACTIVATE: the
  // controller_manager deactivates this hardware and every controller
  // claiming its command interfaces. Activating it again (on_activate)
  // re-seeds command = state, and a re-activated JTC holds the actual
  // position - never a jump. The ROS 1 abb_robot_driver did the same with
  // a controller reset + egm_controller_stopper.
  std::string why;
  if (egmSessionOk(why))
  {
    egm_not_ok_ = false;
  }
  else
  {
    const auto now = std::chrono::steady_clock::now();
    if (!egm_not_ok_)
    {
      egm_not_ok_ = true;
      egm_not_ok_since_ = now;
    }
    if (!egm_lost_ && now - egm_not_ok_since_ >= egm_loss_grace_)
    {
      egm_lost_ = true;
      egm_lost_reason_ = why;
    }
  }
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
  applyHold();
  enforceCommandLimits(period.seconds());
  notePending();
  egm_manager_->write(motion_data_);
  return return_type::OK;
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemHardware, hardware_interface::SystemInterface)
