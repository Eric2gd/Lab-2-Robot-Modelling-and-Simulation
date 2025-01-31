import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    package_description = "cybertruck_description"

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), "rviz", "cybertruck.rviz")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        rviz_node
    ])