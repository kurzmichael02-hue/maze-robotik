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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
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
    headless = LaunchConfiguration('headless')

    args = [
        DeclareLaunchArgument('size', default_value='10'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='no rviz when true (für test runs)'),
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

    # 3. rviz - nur wenn nicht headless
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(headless),
    )

    # 4. path executor (deterministischer demo-fahrer)
    # alternative: frontier_explorer wenn man slam-exploration sehen will
    frontier = Node(
        package='maze_explorer',
        executable='path_executor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'maze_size': 8,
            'cell_size': 1.2,
            'seed': 7,
            'difficulty': 'easy',
        }],
    )

    # 5. planner node
    planner = Node(
        package='maze_planners',
        executable='planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'cell_size': 1.2,
            'maze_size': 8,
            'start_world_x': 0.6,
            'start_world_y': 0.6,
        }],
    )

    return LaunchDescription([
        *args,
        maze_world,
        TimerAction(period=5.0, actions=[slam, rviz]),
        TimerAction(period=10.0, actions=[frontier, planner]),
    ])
