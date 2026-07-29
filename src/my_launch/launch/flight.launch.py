import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _package_launch(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description() -> LaunchDescription:
    routes_file = LaunchConfiguration("routes_file")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    height_config_file = LaunchConfiguration("height_config_file")
    default_routes = os.path.join(
        FindPackageShare("activity_control_pkg").find("activity_control_pkg"),
        "config",
        "routes.yaml",
    )
    default_height_config = os.path.join(
        FindPackageShare("my_launch").find("my_launch"),
        "config",
        "height_source.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("routes_file", default_value=default_routes),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyS6"),
            DeclareLaunchArgument("baud_rate", default_value="921600"),
            DeclareLaunchArgument(
                "height_config_file", default_value=default_height_config,
                description="Height-source YAML: laser_array (default) or stm32.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _package_launch("my_carto_pkg", "fly_carto.launch.py")
                )
            ),
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _package_launch(
                                "uart_to_stm32", "uart_to_stm32.launch.py"
                            )
                        ),
                        launch_arguments={
                            "serial_port": serial_port,
                            "baud_rate": baud_rate,
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _package_launch(
                                "laser_array_pkg",
                                "laser_array_ground.launch.py",
                            )
                        ),
                        launch_arguments={
                            "height_topic": "/height_laser_array",
                            "uart_height_fusion_enabled": "false",
                        }.items(),
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
                        parameters=[height_config_file],
                    )
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _package_launch(
                                "pid_control_pkg",
                                "position_pid_controller.launch.py",
                            )
                        )
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _package_launch(
                                "activity_control_pkg",
                                "route_control.launch.py",
                            )
                        ),
                        launch_arguments={"routes_file": routes_file}.items(),
                    )
                ],
            ),
        ]
    )
