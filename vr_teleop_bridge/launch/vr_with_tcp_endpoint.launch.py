from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    vr_teleop_bridge_pkg = get_package_share_directory("vr_teleop_bridge")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            vr_teleop_bridge_pkg,
            "config",
            "vr_teleop_bridge.yaml",
        ]),
        description="VR teleop bridge parameter file",
    )
    ros_ip_arg = DeclareLaunchArgument(
        "ros_ip",
        default_value="0.0.0.0",
        description="IP address for ros_tcp_endpoint to bind",
    )
    ros_tcp_port_arg = DeclareLaunchArgument(
        "ros_tcp_port",
        default_value="10000",
        description="TCP port for Unity ROS TCP Connector",
    )

    endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="UnityEndpoint",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"ROS_IP": LaunchConfiguration("ros_ip")},
            {"ROS_TCP_PORT": LaunchConfiguration("ros_tcp_port")},
        ],
    )

    bridge = Node(
        package="vr_teleop_bridge",
        executable="vr_pose_to_servo_node.py",
        name="vr_pose_to_servo_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_arg,
        ros_ip_arg,
        ros_tcp_port_arg,
        endpoint,
        bridge,
    ])
