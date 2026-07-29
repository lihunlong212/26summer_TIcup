from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    height_topic = LaunchConfiguration("height_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyS6"),
            DeclareLaunchArgument("baud_rate", default_value="921600"),
            DeclareLaunchArgument("height_topic", default_value="/height_stm32"),
            Node(
                package="uart_to_stm32",
                executable="uart_to_stm32_node",
                name="uart_to_stm32",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baud_rate": ParameterValue(baud_rate, value_type=int),
                        "height_topic": height_topic,
                    }
                ],
                output="screen",
            ),
        ]
    )
