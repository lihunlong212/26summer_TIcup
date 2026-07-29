import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("bluesea2").find("bluesea2"),
                "launch",
                "uart_lidar.launch",
            )
        )
    )

    package_share = FindPackageShare("my_carto_pkg").find("my_carto_pkg")
    cartographer = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": False}],
        arguments=[
            "-configuration_directory",
            os.path.join(package_share, "configuration_files"),
            "-configuration_basename",
            "amphi.lua",
        ],
        remappings=[("scan", "scan")],
        output="screen",
    )

    return LaunchDescription(
        [
            lidar_launch,
            TimerAction(period=10.0, actions=[cartographer]),
        ]
    )
