#!/usr/bin/env python3
import subprocess
import time


def run(cmd):
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def main():
    run(["ros2", "daemon", "stop"])

    patterns = [
        r"ros2 launch dual_arm_bringup sim.launch.py",
        r"keyboard_teleop.py",
        r"trajectory_bridge.py",
        r"servo_node_main",
        r"move_group",
        r"rviz2",
        r"controller_manager",
        r"ros2_control_node",
    ]

    for pattern in patterns:
        run(["pkill", "-f", pattern])

    time.sleep(1.0)
    run(["ros2", "daemon", "stop"])
    print("ROS cleanup done.")


if __name__ == "__main__":
    main()
