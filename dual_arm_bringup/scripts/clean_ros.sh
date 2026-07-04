#!/usr/bin/env bash
set -euo pipefail

# Stop local ROS 2 daemon to avoid stale graph/cache state.
ros2 daemon stop >/dev/null 2>&1 || true

# Kill only processes related to this workspace.
pkill -f "ros2 launch dual_arm_bringup sim.launch.py" || true
pkill -f "keyboard_teleop.py" || true
pkill -f "trajectory_bridge.py" || true
pkill -f "servo_node_main" || true
pkill -f "move_group" || true
pkill -f "rviz2" || true
pkill -f "controller_manager" || true
pkill -f "ros2_control_node" || true

sleep 1
ros2 daemon stop >/dev/null 2>&1 || true

echo "ROS cleanup done."
