# Adapted from https://github.com/turtlebot/turtlebot4_simulator/blob/humble/turtlebot4_ignition_bringup/launch/turtlebot4_ignition.launch.py
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='robot1',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'], description='Auto start the nav2 stack.'),
    DeclareLaunchArgument('description', default_value='true',
                          choices=['true', 'false'], description='Launch tb3 description server'),
    DeclareLaunchArgument('use_respawn', default_value='true',
                          choices=['true', 'false'], description='Launch tb3 description server'),
    DeclareLaunchArgument('world', default_value='maze',
                          description='Ignition World'),
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('localization', default_value='true',
                          choices=['true', 'false'], description='Start localization.'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'], description='Start nav2.'),
    DeclareLaunchArgument('params_file', default_value=f'{get_package_share_directory("gazebo_simulation")}/params/nav2_params.yaml',
                          description='Full path to Nav2 params file.'),
    DeclareLaunchArgument('map',
                          default_value=f'{get_package_share_directory("gazebo_simulation")}/maps/maze.yaml',
                          description='Full path to map yaml.'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_gazebo_simulation = get_package_share_directory("gazebo_simulation")

    # Paths
    ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'ignition.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_gazebo_simulation, 'turtlebot4_spawn.launch.py'])

    ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ignition_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('autostart', LaunchConfiguration('autostart')),
            ('description', LaunchConfiguration('description')),
            ('use_respawn', LaunchConfiguration('use_respawn')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
            ('map', LaunchConfiguration('map')),
            ('nav2', LaunchConfiguration('nav2')),
            ('localization', LaunchConfiguration('localization')),
            ('params_file', LaunchConfiguration('params_file')),]
    )

#   robot_spawn2 = IncludeLaunchDescription(
#       PythonLaunchDescriptionSource([robot_spawn_launch]),
#       launch_arguments=[
#           ('namespace', 'robot2'),
#           ('rviz', LaunchConfiguration('rviz')),
#           ('autostart', LaunchConfiguration('autostart')),
#           ('description', LaunchConfiguration('description')),
#           ('use_respawn', LaunchConfiguration('use_respawn')),
#           ('x', '-2'),
#           ('y', '-2'),
#           ('z', LaunchConfiguration('z')),
#           ('yaw', LaunchConfiguration('yaw')),
#           ('map', LaunchConfiguration('map')),
#           ('nav2', LaunchConfiguration('nav2')),
#           ('localization', LaunchConfiguration('localization')),
#           ('params_file', LaunchConfiguration('params_file')),]
#   )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ignition)
    ld.add_action(robot_spawn)
#   ld.add_action(robot_spawn2)
    return ld
