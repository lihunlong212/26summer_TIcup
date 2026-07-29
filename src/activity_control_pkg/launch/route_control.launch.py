import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_routes = os.path.join(
        get_package_share_directory("activity_control_pkg"),
        "config",
        "routes.yaml",
    )
    routes_file = LaunchConfiguration("routes_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "routes_file",
                default_value=default_routes,
                description="ROS parameter YAML containing the two D-task routes.",
            ),
            Node(
                package="activity_control_pkg",
                executable="route_target_publisher_node",
                name="route_target_publisher",
                output="screen",
                parameters=[routes_file],
            ),
        ]
    )
