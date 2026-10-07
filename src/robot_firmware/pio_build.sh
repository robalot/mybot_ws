#!/usr/bin/env bash
# Runs PlatformIO with a clean environment so its internal micro-ROS colcon
# build doesn't see ~/microros_ws or ~/ws_ros2 (auto-sourced by .bashrc).
# Those overlays' already-built ROS2 interface packages conflict with the
# ESP32 cross-compiled "mcu" package set and break the build (CMake export
# set mismatches). Only /opt/ros/jazzy is needed for the host-side rosidl
# codegen tooling this build uses.
#
# Usage: ./pio_build.sh run -t upload    (or any other pio subcommand/args)
set -e
cd "$(dirname "$0")"
exec env -i HOME="$HOME" PATH="$PATH" USER="$USER" bash --noprofile --norc -c \
  "source /opt/ros/jazzy/setup.bash && cd '$(pwd)' && pio $*"
