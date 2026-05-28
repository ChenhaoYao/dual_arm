from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ifname_arg = DeclareLaunchArgument(
        'ifname',
        default_value='',
        description='EtherCAT network interface name'
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='true',
        description='Run without opening EtherCAT hardware'
    )

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
