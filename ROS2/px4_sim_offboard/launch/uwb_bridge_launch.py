import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


BRIDGE_CONFIG_ENV = "PX4_SIM_OFFBOARD_BRIDGE_CONFIG"


def generate_launch_description():
    uwb_bridge_config = os.environ.get(BRIDGE_CONFIG_ENV)
    if not uwb_bridge_config:
        uwb_bridge_config = str(
            Path(get_package_share_directory('px4_sim_offboard'))
            / 'config'
            / 'uwb_bridge.yaml'
        )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_parameter_bridge',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={uwb_bridge_config}']
        )
    ])
