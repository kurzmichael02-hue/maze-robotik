"""startet gazebo + ros-gz bridge.

bot ist direkt in der welt-sdf (kein extra spawn-step nötig - das macht
ros_gz_sim/create oft probleme bei urdf->sdf konvertierung).

usage:
    ros2 launch maze_worlds maze_world.launch.py
"""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('maze_worlds')
    bridge_yaml = os.path.join(pkg, 'config', 'bridge.yaml')
    urdf_xacro = os.path.join(pkg, 'urdf', 'maze_bot.urdf.xacro')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'maze.sdf'),
        description='path to .sdf world file (must contain maze_bot)')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', LaunchConfiguration('world')],
        output='screen'
    )

    # robot_state_publisher: liefert TF für sub-links (lidar etc)
    # bot selbst kommt aus dem sdf, aber wir brauchen das urdf für rviz
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open_xacro(urdf_xacro),
        }],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        TimerAction(period=2.0, actions=[robot_state_pub, bridge]),
    ])


def open_xacro(path):
    res = subprocess.run(['xacro', path], capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"xacro failed: {res.stderr}")
    return res.stdout
