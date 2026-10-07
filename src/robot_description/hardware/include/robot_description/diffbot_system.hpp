#ifndef ROBOT_DESCRIPTION__DIFFBOT_SYSTEM_HPP_
#define ROBOT_DESCRIPTION__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robot_description
{
// Hardware interface that bridges ros2_control to an ESP32 running micro-ROS
// over WiFi/UDP.
//
// Subscribes to a sensor_msgs/msg/JointState topic (default: /wheel_states)
// published by the microcontroller to update the position/velocity state
// interfaces (position and velocity are computed and integrated on-board by
// the firmware from encoder ticks), and publishes a
// std_msgs/msg/Float64MultiArray (default: /wheel_commands) with the
// commanded wheel velocities (rad/s), ordered the same as <joint> entries in
// the ros2_control xacro (index 0 = left wheel, index 1 = right wheel), for
// the microcontroller to consume.
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)

  ~DiffBotSystemHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Stops and joins the executor thread, safe to call multiple times.
  void stop_executor();

  // Configurable via <param> tags in the ros2_control xacro.
  std::string wheel_states_topic_{"/wheel_states"};
  std::string wheel_commands_topic_{"/wheel_commands"};
  double state_timeout_sec_{1.0};
  double startup_timeout_sec_{10.0};
  // Shorter than state_timeout_sec_: once wheel_states goes stale past this,
  // write() zeros the commanded velocity instead of forwarding the
  // controller's request, so the robot never drives blind (without fresh
  // position/velocity feedback) while waiting out the longer, harder
  // state_timeout_sec_ deactivation threshold.
  double command_stop_timeout_sec_{1.0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_states_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_commands_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;

  std::mutex wheel_states_mutex_;
  std::unordered_map<std::string, double> wheel_positions_;
  std::unordered_map<std::string, double> wheel_velocities_;
  rclcpp::Time last_wheel_state_time_;
  rclcpp::Time activation_time_;
  bool wheel_states_received_{false};
};

}  // namespace robot_description

#endif  // ROBOT_DESCRIPTION__DIFFBOT_SYSTEM_HPP_
