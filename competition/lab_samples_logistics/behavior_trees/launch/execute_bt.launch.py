import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


pkg_rome_bt = get_package_share_directory('behavior_trees')
ARGUMENTS = [
    DeclareLaunchArgument('bt',
                          description='Behavior Tree XML to execute'),
    DeclareLaunchArgument('drawer_stuck', default_value="false",
                          description='Behavior Tree XML to execute'),
    DeclareLaunchArgument('tick_rate_ms', default_value="500",
                          description='Rate in milliseconds between ticks'),
    DeclareLaunchArgument('namespace', default_value="",
                          description='Namespace for behavior tree'),
]


def generate_launch_description():
    execute_bt_cmd = Node(
        package="behavior_trees",
        executable="bt_executor",
        name="bt_executor",
        namespace=LaunchConfiguration("namespace"),
        parameters=[{"bt": LaunchConfiguration('bt'),
                     "tick_rate_ms": LaunchConfiguration("tick_rate_ms"),
                     "drawer_stuck": LaunchConfiguration("drawer_stuck"),}]
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(execute_bt_cmd)
    return ld

