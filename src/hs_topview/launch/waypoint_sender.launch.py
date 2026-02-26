#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_waypoints_file = os.path.join(
        get_package_share_directory("hs_topview"),
        "params",
        "waypoints.yaml",
    )

    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=default_waypoints_file,
        description="Path to waypoint sender YAML file.",
    )
    selected_waypoints_arg = DeclareLaunchArgument(
        "selected_waypoints",
        default_value="",
        description="Comma-separated regular waypoint names to execute (e.g. A,C,E). Empty means run all.",
    )
    use_astar_arg = DeclareLaunchArgument(
        "use_astar",
        default_value="false",
        description="Enable A* planning mode using astar_waypoints from start_waypoint to goal_waypoint.",
    )
    start_waypoint_arg = DeclareLaunchArgument(
        "start_waypoint",
        default_value="",
        description="A* start waypoint name (e.g. R1C1). Used only when use_astar is true.",
    )
    goal_waypoint_arg = DeclareLaunchArgument(
        "goal_waypoint",
        default_value="",
        description="A* goal waypoint name (e.g. R3C5). Used only when use_astar is true.",
    )

    waypoint_sender_node = Node(
        package="hs_topview",
        executable="waypoint_sender",
        name="waypoint_sender",
        output="screen",
        parameters=[
            {"waypoints_file": LaunchConfiguration("waypoints_file")},
            {"selected_waypoints": LaunchConfiguration("selected_waypoints")},
            {"use_astar": LaunchConfiguration("use_astar")},
            {"start_waypoint": LaunchConfiguration("start_waypoint")},
            {"goal_waypoint": LaunchConfiguration("goal_waypoint")},
        ],
    )

    return LaunchDescription([
        waypoints_file_arg,
        selected_waypoints_arg,
        use_astar_arg,
        start_waypoint_arg,
        goal_waypoint_arg,
        waypoint_sender_node,
    ])
