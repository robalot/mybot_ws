#include "robot_description/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_description
{
DiffBotSystemHardware::~DiffBotSystemHardware() { stop_executor(); }

void DiffBotSystemHardware::stop_executor()
{
  executor_.cancel();
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (info_.hardware_parameters.count("wheel_states_topic") > 0)
  {
    wheel_states_topic_ = info_.hardware_parameters.at("wheel_states_topic");
  }
  if (info_.hardware_parameters.count("wheel_commands_topic") > 0)
  {
    wheel_commands_topic_ = info_.hardware_parameters.at("wheel_commands_topic");
  }
  if (info_.hardware_parameters.count("state_timeout_sec") > 0)
  {
    state_timeout_sec_ = hardware_interface::stod(info_.hardware_parameters.at("state_timeout_sec"));
  }
  if (info_.hardware_parameters.count("startup_timeout_sec") > 0)
  {
    startup_timeout_sec_ = hardware_interface::stod(info_.hardware_parameters.at("startup_timeout_sec"));
  }
  if (info_.hardware_parameters.count("command_stop_timeout_sec") > 0)
  {
    command_stop_timeout_sec_ =
      hardware_interface::stod(info_.hardware_parameters.at("command_stop_timeout_sec"));
  }

  node_ = std::make_shared<rclcpp::Node>("diffbot_hardware_interface");

  // The ESP32 firmware publishes with default (reliable) QoS, but a
  // best-effort subscriber is DDS-compatible with a reliable publisher, so
  // SensorDataQoS works here and matches the common micro-ROS convention.
  wheel_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    wheel_states_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(wheel_states_mutex_);

      for (std::size_t i = 0; i < msg->name.size(); ++i)
      {
        const auto & joint_name = msg->name[i];

        if (i < msg->position.size())
        {
          wheel_positions_[joint_name] = msg->position[i];
        }

        if (i < msg->velocity.size())
        {
          wheel_velocities_[joint_name] = msg->velocity[i];
        }
      }

      last_wheel_state_time_ = node_->now();
      wheel_states_received_ = true;
    });

  // Best-effort: a reliable publisher can block write() for hundreds of ms
  // trying to guarantee delivery to a subscriber that's gone unreachable
  // (e.g. the ESP32 mid-WiFi-reconnect), stalling the whole control loop.
  // A dropped velocity setpoint is harmless since the next cycle sends a
  // fresh one ~20ms later. The firmware's subscriber QoS must match
  // (best-effort), since a best-effort publisher is DDS-incompatible with a
  // reliable subscriber.
  wheel_commands_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    wheel_commands_topic_, rclcpp::SensorDataQoS());

  executor_.add_node(node_);
  executor_thread_ = std::thread([this]() { executor_.spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}  // DiffBotSystemHardware::on_init

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  {
    std::lock_guard<std::mutex> lock(wheel_states_mutex_);
    activation_time_ = node_->now();
    wheel_states_received_ = false;
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware...");

  stop_executor();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(wheel_states_mutex_);

  if (!wheel_states_received_)
  {
    // Longer grace period: discovery through a micro-ROS agent bridge can take
    // much longer than the steady-state message period.
    const double seconds_since_activation = (node_->now() - activation_time_).seconds();
    if (seconds_since_activation <= startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *node_->get_clock(), 2000, "Waiting for first message on '%s'.",
        wheel_states_topic_.c_str());
      return hardware_interface::return_type::OK;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *node_->get_clock(), 2000, "No message received yet on '%s'.",
      wheel_states_topic_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  const double seconds_since_last_msg = (node_->now() - last_wheel_state_time_).seconds();
  if (seconds_since_last_msg > state_timeout_sec_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *node_->get_clock(), 2000,
      "Wheel state data on '%s' is stale (%.3f s old, timeout is %.3f s).",
      wheel_states_topic_.c_str(), seconds_since_last_msg, state_timeout_sec_);
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & [interface_name, interface_description] : joint_state_interfaces_)
  {
    const std::string & joint_name = interface_description.get_prefix_name();
    const std::string & interface_type = interface_description.get_interface_name();

    if (interface_type == hardware_interface::HW_IF_POSITION)
    {
      const auto position_it = wheel_positions_.find(joint_name);

      if (position_it != wheel_positions_.end())
      {
        set_state(interface_name, position_it->second);
      }
    }
    else if (interface_type == hardware_interface::HW_IF_VELOCITY)
    {
      const auto velocity_it = wheel_velocities_.find(joint_name);

      if (velocity_it != wheel_velocities_.end())
      {
        set_state(interface_name, velocity_it->second);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool stale_command_stop = false;
  {
    std::lock_guard<std::mutex> lock(wheel_states_mutex_);
    if (wheel_states_received_)
    {
      const double seconds_since_last_msg = (node_->now() - last_wheel_state_time_).seconds();
      stale_command_stop = seconds_since_last_msg > command_stop_timeout_sec_;
    }
  }

  // Publish velocity commands (rad/s) as a Float64MultiArray, ordered by
  // joint declaration order in the ros2_control xacro (left, then right).
  // If wheel_states has gone stale (but not stale enough yet for read() to
  // declare a hard fault via state_timeout_sec_), zero the commands instead
  // of forwarding the controller's request — don't drive blind without
  // fresh position/velocity feedback.
  std_msgs::msg::Float64MultiArray command_msg;
  command_msg.data.reserve(info_.joints.size());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    command_msg.data.push_back(
      stale_command_stop
        ? 0.0
        : get_command<double>(joint.name + "/" + hardware_interface::HW_IF_VELOCITY));
  }

  wheel_commands_pub_->publish(command_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace robot_description

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_description::DiffBotSystemHardware, hardware_interface::SystemInterface)
