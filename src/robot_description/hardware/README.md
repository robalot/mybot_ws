# DiffBotSystemHardware — ESP32 micro-ROS interface

`DiffBotSystemHardware` (`robot_description::DiffBotSystemHardware`) bridges ros2_control
to the ESP32 firmware in `robot_firmware/` over two topics, carried over micro-ROS/WiFi/UDP.

## Subscribed: wheel states

- Topic: `/wheel_states` (configurable via the `wheel_states_topic` hardware param)
- Type: `sensor_msgs/msg/JointState`
- QoS: `rclcpp::SensorDataQoS()` (best-effort, volatile). The firmware actually publishes
  with default (reliable) QoS; a best-effort subscriber is DDS-compatible with a reliable
  publisher, so this still works.
- Expected fields: `name` (`left_wheel_joint`, `right_wheel_joint`), `position` (rad,
  absolute, integrated on-board by the firmware from encoder ticks), `velocity` (rad/s,
  also computed on-board). `effort` is ignored.

The latest position/velocity per joint name are cached and copied into the `position`
and `velocity` state interfaces in `read()`.

## Published: wheel commands

- Topic: `/wheel_commands` (configurable via the `wheel_commands_topic` hardware param)
- Type: `std_msgs/msg/Float64MultiArray`
- QoS: `rclcpp::SystemDefaultsQoS()` (reliable, volatile), matching the firmware's
  reliable subscription.
- Payload: `data` is `[left_wheel_joint_velocity, right_wheel_joint_velocity]`
  (rad/s), ordered by `<joint>` declaration order in the ros2_control xacro
  (`left_wheel_joint` then `right_wheel_joint`). The firmware closes a PID
  velocity loop on-board using these setpoints.

## Stale-data protection

Two separate thresholds, intentionally decoupled:

- `command_stop_timeout_sec` (default `1.0` s) — the **soft** threshold.
  Once `/wheel_states` has gone this long without a fresh message, `write()`
  publishes `[0.0, 0.0]` instead of forwarding the controller's requested
  velocity, so the robot never drives blind (without fresh position/velocity
  feedback) during a brief WiFi/agent blip. `read()` is unaffected — state
  interfaces just hold their last known values — and once a fresh message
  arrives, normal command forwarding resumes immediately.
- `state_timeout_sec` (default `8.0` s) — the **hard** fault threshold. Past
  this, `read()` returns `hardware_interface::return_type::ERROR` and
  `controller_manager` deactivates `MyBot` *and* both controllers as a safety
  measure — reconnecting the ESP32 alone does **not** bring them back; that
  needs a manual reactivation (spawner/`ros2 control` commands). Set well
  above `command_stop_timeout_sec` so a blip triggers only the soft stop, not
  a full stack reset — loosened from an initial 1.0s, then 3.0s, after
  observing Docker-bridged micro-ROS agent + WiFi gaps up to ~0.6-1.0s
  between real `/wheel_states` messages even when otherwise healthy.
- `startup_timeout_sec` (default `10.0` s) — grace period for the *first*
  message after activation only; discovery through a micro-ROS agent bridge
  can take noticeably longer than the steady-state message period.

## Example `<hardware>` block

```xml
<hardware>
  <plugin>robot_description/DiffBotSystemHardware</plugin>
  <param name="wheel_states_topic">/wheel_states</param>
  <param name="wheel_commands_topic">/wheel_commands</param>
  <param name="state_timeout_sec">8.0</param>
  <param name="startup_timeout_sec">10.0</param>
  <param name="command_stop_timeout_sec">1.0</param>
</hardware>
```

## Fixed issue: controller spawning vs. a known upstream ros2_control bug

Verified end-to-end against the real ESP32 (Docker `micro_ros_agent`):
`DiffBotSystemHardware` loads/activates, `joint_state_broadcaster` and
`diff_drive_base_controller` both come up `active`, `/cmd_vel` reaches the
robot, and `/diff_drive_base_controller/odom` reflects real physical motion
(a 0.15 m/s x3s command moved `odom.pose.position.x` from 0.0 to ~0.375m).

**The bug, for reference if this regresses**: `ros2 launch robot_description
hardware.launch.py` used to load `MyBot` successfully but then fail spawning
both controllers with e.g.
```
[rclcpp]: Failed to set parameter: parameter 'diff_drive_base_controller.type' cannot be set because it was not declared
[spawner]: Could not set controller param "type" to "..." for diff_drive_base_controller
```
This reproduced identically against both `/opt/ros/jazzy`'s stock
`controller_manager` and `~/ws_ros2`'s from-source build, and only on real
hardware (never with `use_mock_hardware:=true`, and a 5s startup delay before
the spawners made no difference — not a timing/race issue). It's a known,
currently-unresolved family of upstream bugs, not caused by this package:
`allow_undeclared_parameters(true)` on `controller_manager`'s `NodeOptions`
doesn't reliably permit `spawner`'s dynamic `set_parameters` calls for
`<controller>.type`/`.params_file`/`.node_options_args` before
`load_controller` runs (see
[ros2/rclcpp#851](https://github.com/ros2/rclcpp/issues/851),
[ros-controls/ros2_control#3639](https://github.com/ros-controls/ros2_control/issues/3639),
[#3497](https://github.com/ros-controls/ros2_control/issues/3497)) — `spawner`
treats that failure as fatal and exits without ever calling `load_controller`.

**The fix** (already applied in `launch/hardware.launch.py`): pre-declare
those three parameters as flat dot-key **startup overrides** directly in
`ros2_control_node`'s own `parameters=[...]` dict
(`joint_state_broadcaster.type`, `diff_drive_base_controller.type`,
`<name>.params_file: [<path to this yaml>]`,
`diff_drive_base_controller.node_options_args: ["-r", "~/cmd_vel:=/cmd_vel"]`),
and give both spawners bare controller names with no `--param-file`/
`--controller-ros-args`. This sidesteps `spawner`'s broken dynamic-set step
entirely — `load_controller` reads the pre-declared parameters directly and
the controller's own detailed settings (`wheel_separation`, `wheel_radius`,
etc.) still come from the per-controller `ros__parameters` blocks lower down
in `my_bot_controllers.yaml`, loaded normally once the controller node itself
starts with the pre-declared `params_file` path.

If this resurfaces (e.g. after a `ros2_control` upgrade), check whether the
upstream issues above have a merged fix first — the pre-declare-as-overrides
workaround here may no longer be necessary, or may need adjusting if
`controller_manager`'s YAML-loading behavior changes.
