from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 真实 EtherCAT 网卡名，例如 enp0s31f6。
    ifname_arg = DeclareLaunchArgument(
        'ifname',
        default_value='',
        description='EtherCAT network interface name'
    )
    # 默认 dry-run，先验证 ROS 链路，不打开真实硬件。
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='true',
        description='Run without opening EtherCAT hardware'
    )

    # 启动 SOEM 桥接节点。
    soem_bridge_node = Node(
        package='dual_arm_soem_bridge',
        executable='soem_bridge_node',
        name='soem_bridge_node',
        output='screen',
        parameters=[{
            'ifname': LaunchConfiguration('ifname'),
            'dry_run': LaunchConfiguration('dry_run'),
        }]
    )

    return LaunchDescription([
        ifname_arg,
        dry_run_arg,
        soem_bridge_node,
    ])
