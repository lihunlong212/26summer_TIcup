import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_launch(package_name: str, filename: str) -> str:
    package_share = get_package_share_directory(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    default_config = os.path.join(
        get_package_share_directory("my_launch"),
        "config",
        "flight.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="PID, vision-control, height-source and waypoint parameters.",
            ),
            Node(
                package="domain_bridge_pkg",
                executable="domain_bridge",
                name="domain_bridge",
                output="screen",
                parameters=[config_file],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _package_launch("my_carto_pkg", "fly_carto.launch.py")
                )
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="uart_to_stm32",
                        executable="uart_to_stm32_node",
                        name="uart_to_stm32",
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="laser_array_pkg",
                        executable="laser_array_ground_node",
                        name="laser_array_ground_node",
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="drone_camera_pkg",
                        executable="drone_camera_node",
                        name="drone_camera_node",
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="activity_control_pkg",
                        executable="height_source_mux_node",
                        name="height_source_mux",
                        output="screen",
                        parameters=[config_file],
                    )
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="pid_control_pkg",
                        executable="position_pid_controller",
                        name="position_pid_controller",
                        output="screen",
                        parameters=[config_file],
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="activity_control_pkg",
                        executable="route_target_publisher_node",
                        name="route_target_publisher",
                        output="screen",
                        parameters=[config_file],
                    )
                ],
            ),
        ]
    )
