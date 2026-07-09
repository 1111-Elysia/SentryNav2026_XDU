import json

import yaml


PARAMETER_NAMES = (
    "runtime_goal_name",
    "runtime_use_custom_pose",
    "runtime_goal_x",
    "runtime_goal_y",
    "runtime_goal_yaw",
    "runtime_move_posture",
    "runtime_wait_posture",
    "runtime_controller",
    "runtime_reach_threshold",
    "runtime_wait_time_threshold",
)

DEFAULT_CONTROLLERS = ("FollowPath",)


def normalize_controller_plugins(plugins, default=DEFAULT_CONTROLLERS):
    if isinstance(plugins, str):
        plugins = [plugins]

    controllers = []
    for plugin in plugins or []:
        controller = str(plugin).strip()
        if controller and controller not in controllers:
            controllers.append(controller)

    return controllers or list(default)


def load_waypoints(json_path):
    with open(json_path, "r", encoding="utf-8") as waypoint_file:
        document = json.load(waypoint_file)

    waypoints = {}
    for item in document.get("waypoints", []):
        name = item.get("name")
        if not name:
            continue
        waypoints[name] = (
            float(item["x"]),
            float(item["y"]),
            float(item["yaw"]),
        )
    return waypoints


def load_controller_plugins(yaml_path, default=DEFAULT_CONTROLLERS):
    with open(yaml_path, "r", encoding="utf-8") as params_file:
        document = yaml.safe_load(params_file) or {}

    return normalize_controller_plugins(
        document.get("controller_server", {})
        .get("ros__parameters", {})
        .get("controller_plugins", []),
        default,
    )


def config_values(
    goal_name,
    use_custom_pose,
    x,
    y,
    yaw,
    move_posture,
    wait_posture,
    controller,
    reach_threshold,
    wait_time_threshold,
):
    return {
        "runtime_goal_name": str(goal_name),
        "runtime_use_custom_pose": bool(use_custom_pose),
        "runtime_goal_x": float(x),
        "runtime_goal_y": float(y),
        "runtime_goal_yaw": float(yaw),
        "runtime_move_posture": int(move_posture),
        "runtime_wait_posture": int(wait_posture),
        "runtime_controller": str(controller).strip(),
        "runtime_reach_threshold": float(reach_threshold),
        "runtime_wait_time_threshold": float(wait_time_threshold),
    }
