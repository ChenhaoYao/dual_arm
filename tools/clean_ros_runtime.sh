#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/dell/dual_arm"
START_ENDPOINT=0

usage() {
  cat <<'USAGE'
Usage: tools/clean_ros_runtime.sh [--start-endpoint]

Cleans stale ROS 2 runtime processes for this workspace before starting a new run.

Options:
  --start-endpoint   Start ros_tcp_endpoint on 0.0.0.0:10000 after cleanup.
  -h, --help         Show this help.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start-endpoint)
      START_ENDPOINT=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

kill_pattern() {
  local pattern="$1"
  local label="$2"
  mapfile -t pids < <(pgrep -f "$pattern" || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    echo "No stale $label processes."
    return
  fi

  echo "Stopping $label: ${pids[*]}"
  kill "${pids[@]}" 2>/dev/null || true
  sleep 1

  mapfile -t remaining < <(pgrep -f "$pattern" || true)
  if [[ ${#remaining[@]} -gt 0 ]]; then
    echo "Force stopping $label: ${remaining[*]}"
    kill -9 "${remaining[@]}" 2>/dev/null || true
  fi
}

echo "Cleaning ROS runtime processes for ${WORKSPACE}"

kill_pattern "${WORKSPACE}/install/ros_tcp_endpoint/lib/ros_tcp_endpoint/default_server_endpoint" "ros_tcp_endpoint executable"
kill_pattern "ros2 run ros_tcp_endpoint default_server_endpoint" "ros_tcp_endpoint ros2 wrapper"
kill_pattern "${WORKSPACE}/install/.*/lib/.*/.*" "workspace-installed ROS executables"
kill_pattern "ros2 launch .*dual_arm|ros2 run .*dual_arm|ros2 run .*vr_teleop" "workspace ros2 launch/run commands"
# Launch children can become orphaned after an abnormal shutdown, so their
# command lines no longer contain the workspace path or the ros2 launch parent.
kill_pattern "/opt/ros/jazzy/lib/moveit_servo/servo_node" "MoveIt Servo nodes"
kill_pattern "/opt/ros/jazzy/lib/controller_manager/ros2_control_node" "ros2_control nodes"
kill_pattern "/opt/ros/jazzy/lib/controller_manager/spawner" "controller spawners"
kill_pattern "/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher" "robot_state_publisher nodes"
kill_pattern "/opt/ros/jazzy/lib/rviz2/rviz2" "RViz nodes"

# One-off rclpy/ros2 diagnostics are not workspace executables. In particular,
# a malformed joint_state_probe previously survived cleanup at 100% CPU.
kill_pattern "python3 -c .*rclpy" "temporary rclpy diagnostics"
kill_pattern "ros2 (topic|node|service|param|action|doctor) " "ROS 2 diagnostic commands"

if command -v ros2 >/dev/null 2>&1; then
  echo "Stopping ros2 daemon."
  set +e
  bash -lc "source /opt/ros/jazzy/setup.bash; ros2 daemon stop" >/dev/null 2>&1
  set -e
  kill_pattern "_ros2_daemon" "ROS 2 daemon processes"
fi

if [[ "$START_ENDPOINT" -eq 1 ]]; then
  echo "Starting ros_tcp_endpoint on 0.0.0.0:10000"
  bash -lc "source /opt/ros/jazzy/setup.bash; source ${WORKSPACE}/install/setup.bash; setsid -f ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p tcp_port:=10000 >/tmp/ros_tcp_endpoint.log 2>&1"
  sleep 1
  if command -v ss >/dev/null 2>&1; then
    ss -ltnp | grep ':10000' || {
      echo "ros_tcp_endpoint did not start on :10000. Log:" >&2
      tail -80 /tmp/ros_tcp_endpoint.log >&2 || true
      exit 1
    }
  fi
fi

echo "Done."
