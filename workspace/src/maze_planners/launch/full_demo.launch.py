"""master launch: maze + bot + slam + frontier + planner_node + rviz.

usage:
    ros2 launch maze_planners full_demo.launch.py size:=10 seed:=7

ablauf:
1. gazebo mit maze + bot
2. nach 5s: slam_toolbox + rviz
3. nach 8s: frontier_explorer + planner_node
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_worlds = get_package_share_directory('maze_worlds')
    pkg_explorer = get_package_share_directory('maze_explorer')
    pkg_planners = get_package_share_directory('maze_planners')

    rviz_cfg = os.path.join(pkg_planners, 'launch', 'demo.rviz')

    size = LaunchConfiguration('size')
    seed = LaunchConfiguration('seed')

    args = [
        DeclareLaunchArgument('size', default_value='10'),
        DeclareLaunchArgument('seed', default_value='7'),
    ]

    # 1. maze world + bot
    maze_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'maze_world.launch.py')),
        launch_arguments={'size': size, 'seed': seed}.items(),
    )

    # 2. slam (delay 5s damit gazebo stabil läuft)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_explorer, 'launch', 'slam.launch.py')),
    )

    # 3. rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 4. frontier explorer
    frontier = Node(
        package='maze_explorer',
        executable='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 5. planner node
    planner = Node(
        package='maze_planners',
        executable='planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cell_size': 1.0,
            'maze_size': 10,  # TODO: aus launch arg
        }],
    )

    return LaunchDescription([
        *args,
        maze_world,
        TimerAction(period=5.0, actions=[slam, rviz]),
        TimerAction(period=10.0, actions=[frontier, planner]),
    ])
