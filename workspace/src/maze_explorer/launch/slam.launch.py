"""startet slam_toolbox im async mode mit unserer config."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('maze_explorer')
    cfg = os.path.join(pkg, 'config', 'slam_async.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[cfg, {'use_sim_time': True}],
    )

    return LaunchDescription([slam])
