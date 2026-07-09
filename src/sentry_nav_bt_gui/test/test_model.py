import json

import yaml

from sentry_nav_bt_gui.model import (
    config_values,
    load_controller_plugins,
    load_waypoints,
    normalize_controller_plugins,
)


def test_load_waypoints(tmp_path):
    waypoint_file = tmp_path / "waypoints.json"
    waypoint_file.write_text(
        json.dumps({"waypoints": [{"name": "init", "x": 1, "y": 2, "yaw": 0.5}]}),
        encoding="utf-8",
    )
    assert load_waypoints(waypoint_file) == {"init": (1.0, 2.0, 0.5)}


def test_load_controller_plugins(tmp_path):
    params_file = tmp_path / "nav2_params.yaml"
    params_file.write_text(
        yaml.safe_dump(
            {
                "controller_server": {
                    "ros__parameters": {
                        "controller_plugins": [
                            "FollowPath",
                            "AdaptiveMppi",
                            "FollowPath",
                            "",
                        ]
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    assert load_controller_plugins(params_file) == ["FollowPath", "AdaptiveMppi"]


def test_load_controller_plugins_uses_default_for_empty_config(tmp_path):
    params_file = tmp_path / "nav2_params.yaml"
    params_file.write_text("controller_server:\n  ros__parameters: {}\n", encoding="utf-8")
    assert load_controller_plugins(params_file, default=("Fallback",)) == ["Fallback"]


def test_normalize_controller_plugins_accepts_string_value():
    assert normalize_controller_plugins(" AdaptiveMppi ") == ["AdaptiveMppi"]


def test_config_values_normalizes_types():
    values = config_values("init", False, 0, 1, 2, 3, 1, " FollowPath ", 0.25, 5)
    assert values["runtime_controller"] == "FollowPath"
    assert values["runtime_move_posture"] == 3
    assert values["runtime_wait_time_threshold"] == 5.0
