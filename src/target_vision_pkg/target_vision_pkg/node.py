"""ROS 2 console entry point for the target-pattern detector."""

import sys

from target_vision_pkg.vision_target import ros2_main


def main() -> int:
    """Start the ROS 2 target vision node."""
    return ros2_main(sys.argv)
