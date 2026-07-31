"""Load the mutually exclusive vision source from the flight configuration."""

import yaml


def load_visual_settings(config_path):
    """Return the selected source and the shared low-height threshold."""
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}

    source_parameters = (
        config.get("vision_source", {}).get("ros__parameters", {})
    )
    visual_mode = str(source_parameters.get("visual_mode", "apriltag")).lower()
    if visual_mode not in ("apriltag", "target"):
        raise RuntimeError(
            "vision_source.visual_mode must be 'apriltag' or 'target', got "
            + repr(visual_mode)
        )

    pid_parameters = (
        config.get("position_pid_controller", {}).get("ros__parameters", {})
    )
    low_height_threshold_cm = float(
        pid_parameters.get("visual_low_height_threshold_cm", 70.0)
    )
    if low_height_threshold_cm <= 0.0:
        raise RuntimeError(
            "visual_low_height_threshold_cm must be positive"
        )
    return visual_mode, low_height_threshold_cm
