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
        description="Comma-separated waypoint names to execute (e.g. A,C,F). Empty means run all.",
    )

    waypoint_sender_node = Node(
        package="hs_topview",
        executable="waypoint_sender",
        name="waypoint_sender",
        output="screen",
        parameters=[
            {"waypoints_file": LaunchConfiguration("waypoints_file")},
            {"selected_waypoints": LaunchConfiguration("selected_waypoints")},
        ],
    )

    return LaunchDescription([
        waypoints_file_arg,
        selected_waypoints_arg,
        waypoint_sender_node,
    ])
