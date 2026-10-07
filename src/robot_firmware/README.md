# robot_firmware — ESP32 micro-ROS diff-drive firmware

PlatformIO/Arduino firmware for an ESP32 running micro-ROS over WiFi/UDP, driving a
2-wheel differential-drive base with encoder feedback and on-board PID velocity control.
This is a copy of the working project originally at
`~/Documents/PlatformIO/Projects/microros_ethernet` (the transport is WiFi, not Ethernet,
despite the original project name).

This package is **not** a ROS2/ament package — it's intentionally excluded from `colcon
build` via the `COLCON_IGNORE` marker file. Build and flash it with PlatformIO directly.

## Wire contract (matches `robot_description`'s `DiffBotSystemHardware` plugin)

- Publishes `/wheel_states` — `sensor_msgs/msg/JointState`, ~20 Hz nominal (firmware
  publishes every 50 ms), default (reliable)
  QoS. `name: [left_wheel_joint, right_wheel_joint]`, `position` (rad, absolute,
  integrated on-board from encoder ticks), `velocity` (rad/s, computed on-board).
- Subscribes `/wheel_commands` — `std_msgs/msg/Float64MultiArray`, default (reliable)
  QoS. `data: [left_wheel_velocity_rad_s, right_wheel_velocity_rad_s]`. Closes a PID
  velocity loop on-board using these setpoints — no watchdog on this topic yet, so the
  motors keep driving at the last received command if the agent connection drops.

## Hardware / network configuration (`src/main.cpp`)

- Board: `esp32-s3-devkitm-1` (see `platformio.ini`)
- Transport: WiFi, `set_microros_wifi_transports(ssid, password, agent_ip, agent_port)`
- Agent IP/port: `10.51.51.195:8888` — must match wherever `micro_ros_agent` runs;
  update `agent_ip`/`ssid`/`password` in `main.cpp` before flashing if these change.
- Encoders: `TICKS_PER_REVOLUTION = 240`, quadrature on `LEFT_ENC_A/B` (GPIO 4/5) and
  `RIGHT_ENC_A/B` (GPIO 7/6), interrupt-driven.
- Motor driver: H-bridge, `LEFT_PWM`/`LEFT_DIR_1`/`LEFT_DIR_2` (GPIO 9/11/10),
  `RIGHT_PWM`/`RIGHT_DIR_1`/`RIGHT_DIR_2` (GPIO 15/16/17), 20 kHz PWM.
- PID gains: `Kp = 1.5`, `Ki = 5.0`, `Kd = 0.05`.

## Build / flash

**Do not run `pio` directly in a normal terminal on this machine** — `.bashrc`
auto-sources `/home/rob/microros_ws` and `/home/rob/ws_ros2` on top of
`/opt/ros/jazzy`. PlatformIO's `micro_ros_platformio` library runs its own
internal `colcon build` to cross-compile the micro-ROS library for the ESP32
("mcu" target), and it picks up those extra overlays' already-built ROS2
interface packages (`rclc`, `sensor_msgs`, `unique_identifier_msgs`, etc.) as
conflicting underlays, which breaks the build with CMake export-set mismatch
errors. Use the provided wrapper instead, which runs `pio` with a clean
environment (only `/opt/ros/jazzy` sourced):

```bash
cd /home/rob/mybot_ws/src/robot_firmware
./pio_build.sh run -t upload
./pio_build.sh device monitor   # optional, 115200 baud
```

If you ever hit the underlay-conflict error anyway (e.g. after switching
ROS_DISTRO or editing platformio.ini), clean the partial mcu build first:
`rm -rf .pio/libdeps/esp32dev/micro_ros_platformio/build` before retrying.

## Bring-up

1. Start the micro-ROS agent on the machine at the configured agent IP:
   `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
2. Power on the ESP32 and confirm the agent log shows a session being established.
3. `ros2 topic echo /wheel_states` should show `JointState` messages arriving
   (nominally ~20 Hz; actual rate depends on WiFi/agent latency).
4. Bring up `robot_description`'s `hardware.launch.py` to drive it through
   `ros2_control`/`diff_drive_controller`.
