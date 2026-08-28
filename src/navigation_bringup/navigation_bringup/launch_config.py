"""Helpers for loading navigation-system launch defaults from YAML."""

from pathlib import Path
from typing import Any, Dict

import yaml


def load_navigation_system_defaults(config_path: str) -> Dict[str, str]:
    path = Path(config_path).resolve()
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    try:
        values = document["navigation_system"]["launch_arguments"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"{path} must contain navigation_system.launch_arguments"
        ) from exc

    if not isinstance(values, dict):
        raise RuntimeError(
            f"navigation_system.launch_arguments in {path} must be a mapping"
        )

    defaults = {name: _to_launch_string(name, value) for name, value in values.items()}
    if "map_pcd_path" in defaults:
        defaults["map_pcd_path"] = _resolve_config_path(
            path, defaults["map_pcd_path"]
        )
    return defaults


def load_node_parameters(config_path: str, node_name: str) -> Dict[str, Any]:
    path = Path(config_path).resolve()
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    try:
        parameters = document[node_name]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"{path} must contain {node_name}.ros__parameters"
        ) from exc
    if not isinstance(parameters, dict):
        raise RuntimeError(
            f"{node_name}.ros__parameters in {path} must be a mapping"
        )
    return parameters


def _resolve_config_path(config_path: Path, configured_path: str) -> str:
    path = Path(configured_path).expanduser()
    if not path.is_absolute():
        path = config_path.parent / path
    path = path.resolve()
    if not path.is_file():
        raise RuntimeError(
            f"map_pcd_path does not point to a file after resolution: {path}"
        )
    return str(path)


def _to_launch_string(name: str, value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return ""
    if isinstance(value, (str, int, float)):
        return str(value)
    raise RuntimeError(
        f"launch argument '{name}' must be a scalar, got {type(value).__name__}"
    )
