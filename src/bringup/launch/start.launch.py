from pathlib import Path
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PATH_VALUE_FLAGS = {"--config", "--params-file", "--file", "-f"}


def _load_sequence(config_path: Path) -> List[Dict[str, Any]]:
    if not config_path.is_file():
        raise FileNotFoundError(f"sequence config not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as cfg:
        data = yaml.safe_load(cfg) or {}
    nodes = data.get("sequential_nodes", [])
    if not isinstance(nodes, list) or not nodes:
        raise ValueError("`sequential_nodes` must be a non-empty list")
    return nodes


def _create_action(entry: Dict[str, Any]):
    entry_type = entry.get("type", "node")
    if entry_type == "execute":
        cmd = entry.get("cmd")
        if not cmd:
            raise ValueError(f"entry {entry} missing `cmd` for execute type")
        return ExecuteProcess(cmd=cmd, output="screen", shell=False, cwd=entry.get("cwd"))
    package = entry["package"]
    executable = entry["executable"]
    node_kwargs = {
        "package": package,
        "executable": executable,
        "name": entry.get("name"),
        "namespace": entry.get("namespace"),
        "arguments": entry.get("arguments"),
        "parameters": entry.get("parameters"),
        "respawn": entry.get("respawn", False),
        "respawn_delay": entry.get("respawn_delay", 2.0),
        "prefix": entry.get("prefix"),
        "output": entry.get("output", "screen"),
        "emulate_tty": entry.get("emulate_tty", True),
    }
    return Node(**{k: v for k, v in node_kwargs.items() if v is not None})


def _infer_workspace_root(config_path: Path) -> Path:
    for parent in config_path.parents:
        if parent.name in {"src", "install", "build"}:
            return parent.parent
    return config_path.parents[-1]


def _make_absolute_path(path_str: str, base_dir: Path) -> str:
    candidate = Path(path_str)
    if candidate.is_absolute():
        return str(candidate)
    for base in (config_dir, workspace_root):
        resolved = (base / candidate).resolve()
        if resolved.exists():
            return str(resolved)
    return str((config_dir / candidate).resolve())


def _maybe_resolve_path(path_str: str, config_dir: Path, workspace_root: Path, *, strict: bool = False) -> str:
    candidate = Path(path_str)
    if candidate.is_absolute():
        return str(candidate)

    indicators = candidate.suffix.lower() in {".yaml", ".yml", ".json", ".launch", ".py"} or "/" in path_str
    if not indicators and not strict:
        return path_str

    for base in (config_dir, workspace_root):
        resolved = (base / candidate).resolve()
        if resolved.exists():
            return str(resolved)

    if strict:
        raise FileNotFoundError(
            f"file '{path_str}' not found relative to '{config_dir}' or workspace root '{workspace_root}'"
        )
    return path_str


def _resolve_cmd_tokens(cmd: Any, config_dir: Path, workspace_root: Path):
    if not isinstance(cmd, list):
        return cmd
    resolved = []
    i = 0
    while i < len(cmd):
        token = cmd[i]
        if token in PATH_VALUE_FLAGS and i + 1 < len(cmd):
            resolved.append(token)
            resolved.append(_maybe_resolve_path(cmd[i + 1], config_dir, workspace_root))
            i += 2
            continue
        matched_flag = next((flag for flag in PATH_VALUE_FLAGS if token.startswith(f"{flag}=")), None)
        if matched_flag:
            _, value = token.split("=", 1)
            resolved.append(f"{matched_flag}={_maybe_resolve_path(value, config_dir, workspace_root)}")
            i += 1
            continue
        if ":=" in token:
            key, value = token.split(":=", 1)
            resolved.append(f"{key}:={_maybe_resolve_path(value, config_dir, workspace_root)}")
        else:
            resolved.append(token)
        i += 1
    return resolved


def _resolve_cli_token(token: Any, config_dir: Path, workspace_root: Path) -> Any:
    if not isinstance(token, str):
        return token
    if ":=" in token:
        key, value = token.split(":=", 1)
        return f"{key}:={_maybe_resolve_path(value, config_dir, workspace_root)}"
    return _maybe_resolve_path(token, config_dir, workspace_root)


def _normalize_entry(entry: Dict[str, Any], config_dir: Path, workspace_root: Path) -> Dict[str, Any]:
    normalized = dict(entry)
    entry_type = normalized.get("type", "node")
    if normalized.pop("use_config_dir_as_cwd", False):
        normalized["cwd"] = str(config_dir)
    cwd_value = normalized.get("cwd")
    if isinstance(cwd_value, str):
        normalized["cwd"] = _maybe_resolve_path(cwd_value, config_dir, workspace_root)
    parameters = normalized.get("parameters")
    if isinstance(parameters, list):
        normalized["parameters"] = [
            _maybe_resolve_path(param, config_dir, workspace_root, strict=True) if isinstance(param, str) else param
            for param in parameters
        ]
    if entry_type == "execute":
        normalized["cmd"] = _resolve_cmd_tokens(normalized.get("cmd"), config_dir, workspace_root)
    return normalized


def _sequential_launch(context, config_path_cfg, delay_cfg, wait_after_cfg):
    cfg_path = Path(config_path_cfg.perform(context)).expanduser().resolve()
    cfg_dir = cfg_path.parent
    workspace_root = _infer_workspace_root(cfg_path)
    initial_delay = max(float(delay_cfg.perform(context)), 0.0)
    default_post_delay = max(float(wait_after_cfg.perform(context)), 0.0)
    actions = []
    accumulated_delay = initial_delay
    for entry in _load_sequence(cfg_path):
        normalized_entry = _normalize_entry(entry, cfg_dir, workspace_root)
        action = _create_action(normalized_entry)
        label = normalized_entry.get("label", "entry")
        try:
            entry_wait = max(float(normalized_entry.get("wait_after", default_post_delay)), 0.0)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"`wait_after` for {label} must be numeric") from exc
        if accumulated_delay > 0:
            actions.append(TimerAction(period=accumulated_delay, actions=[action]))
        else:
            actions.append(action)
        accumulated_delay += entry_wait
    return actions


def generate_launch_description():
    default_cfg = Path(
        get_package_share_directory("bringup")
    ) / "config" / "sequential_nodes.yaml"

    sequence_arg = DeclareLaunchArgument(
        "sequence_config",
        default_value=str(default_cfg),
        description="YAML file describing the ordered ROS nodes/processes.",
    )
    delay_arg = DeclareLaunchArgument(
        "start_delay",
        default_value="0.0",
        description="Initial delay (seconds) before the first entry starts.",
    )
    post_delay_arg = DeclareLaunchArgument(
        "post_start_delay",
        default_value="2.0",
        description="Seconds to wait after each entry before launching the next (unless overridden).",
    )

    return LaunchDescription(
        [
            sequence_arg,
            delay_arg,
            post_delay_arg,
            OpaqueFunction(
                function=_sequential_launch,
                args=[
                    LaunchConfiguration("sequence_config"),
                    LaunchConfiguration("start_delay"),
                    LaunchConfiguration("post_start_delay"),
                ],
            ),
        ]
    )
