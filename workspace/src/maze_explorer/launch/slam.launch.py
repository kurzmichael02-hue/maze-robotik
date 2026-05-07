"""startet slam_toolbox im async mode mit auto-activation.

slam_toolbox ist ein lifecycle node — wir nutzen das standard launch von
slam_toolbox das configure+activate automatisch macht.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('maze_explorer')
    cfg = os.path.join(pkg, 'config', 'slam_async.yaml')

    slam_pkg = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': cfg,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([slam_launch])
