import pytest
import yaml

from my_launch.vision_selection import load_visual_settings


def _write_config(path, visual_mode):
    content = {
        "vision_source": {
            "ros__parameters": {"visual_mode": visual_mode}
        },
        "position_pid_controller": {
            "ros__parameters": {
                "visual_low_height_threshold_cm": 73.0
            }
        },
    }
    path.write_text(yaml.safe_dump(content), encoding="utf-8")


def test_apriltag_and_target_are_mutually_exclusive(tmp_path):
    config_path = tmp_path / "flight.yaml"

    _write_config(config_path, "apriltag")
    assert load_visual_settings(config_path) == ("apriltag", 73.0)

    _write_config(config_path, "target")
    assert load_visual_settings(config_path) == ("target", 73.0)


def test_invalid_visual_mode_is_rejected(tmp_path):
    config_path = tmp_path / "flight.yaml"
    _write_config(config_path, "both")

    with pytest.raises(RuntimeError):
        load_visual_settings(config_path)
