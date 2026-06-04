import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 所有参数从 config/soem_bridge.yaml 加载，修改参数直接改 yaml 即可。
    config_file = os.path.join(
        get_package_share_directory('dual_arm_soem_bridge'),
        'config', 'soem_bridge.yaml'
    )

    soem_bridge_node = Node(
        package='dual_arm_soem_bridge',
        executable='soem_bridge_node',
        name='soem_bridge_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        soem_bridge_node,
    ])
