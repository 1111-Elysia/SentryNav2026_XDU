import json

from sentry_nav_bt_gui.model import (
    config_values,
    load_waypoints,
)


def test_load_waypoints(tmp_path):
    waypoint_file = tmp_path / "waypoints.json"
    waypoint_file.write_text(
        json.dumps({"waypoints": [{"name": "init", "x": 1, "y": 2, "yaw": 0.5}]}),
        encoding="utf-8",
    )
    assert load_waypoints(waypoint_file) == {"init": (1.0, 2.0, 0.5)}


def test_config_values_normalizes_types():
    values = config_values("init", False, 0, 1, 2, 3, 1, 0.25, 5)
    assert values["runtime_move_posture"] == 3
    assert values["runtime_wait_time_threshold"] == 5.0
