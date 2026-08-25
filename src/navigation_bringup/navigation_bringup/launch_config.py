"""Helpers for loading navigation-system launch defaults from YAML."""

from pathlib import Path
from typing import Any, Dict

import yaml


def load_navigation_system_defaults(config_path: str) -> Dict[str, str]:
    path = Path(config_path)
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

    return {name: _to_launch_string(name, value) for name, value in values.items()}


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
