"""startet gazebo + spawnt den bot + ros-gz bridge.

usage:
    ros2 launch maze_worlds maze_world.launch.py size:=10 seed:=7
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('maze_worlds')
    bridge_yaml = os.path.join(pkg, 'config', 'bridge.yaml')
    urdf_xacro = os.path.join(pkg, 'urdf', 'maze_bot.urdf.xacro')

    size_arg = DeclareLaunchArgument('size', default_value='10')
    seed_arg = DeclareLaunchArgument('seed', default_value='7')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'maze.sdf'),
        description='path to .sdf world file')

    # gazebo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', LaunchConfiguration('world')],
        output='screen'
    )

    # robot_state_publisher: konvertiert urdf -> /robot_description + tf
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open_xacro(urdf_xacro),
        }],
    )

    # spawn entity über ros-gz
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'maze_bot',
                   '-topic', '/robot_description',
                   '-x', '0.5', '-y', '-0.4', '-z', '0.05',
                   '-Y', '1.5708'],  # nach norden ausgerichtet
    )

    # bridge: gz topics <-> ros topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        size_arg, seed_arg, world_arg,
        gz_sim,
        TimerAction(period=2.0, actions=[robot_state_pub, bridge]),
        TimerAction(period=4.0, actions=[spawn]),
    ])


def open_xacro(path):
    """xacro -> string. simpel gehalten, nutzt xacro CLI."""
    import subprocess
    res = subprocess.run(['xacro', path], capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"xacro failed: {res.stderr}")
    return res.stdout
