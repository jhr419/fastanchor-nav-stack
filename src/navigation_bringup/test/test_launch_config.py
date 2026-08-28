from pathlib import Path

import pytest
import yaml

from navigation_bringup.launch_config import (
    load_navigation_system_defaults,
    load_node_parameters,
)


def write_config(path: Path, map_path: str) -> None:
    path.write_text(
        yaml.safe_dump(
            {
                "navigation_system": {
                    "launch_arguments": {
                        "map_pcd_path": map_path,
                        "use_sim_time": False,
                    }
                }
            }
        ),
        encoding="utf-8",
    )


def test_resolves_map_relative_to_config_file(tmp_path: Path) -> None:
    map_path = tmp_path / "maps" / "map.pcd"
    map_path.parent.mkdir()
    map_path.touch()
    config_path = tmp_path / "config" / "navigation_system.yaml"
    config_path.parent.mkdir()
    write_config(config_path, "../maps/map.pcd")

    defaults = load_navigation_system_defaults(str(config_path))

    assert defaults["map_pcd_path"] == str(map_path)
    assert defaults["use_sim_time"] == "false"


def test_rejects_missing_map(tmp_path: Path) -> None:
    config_path = tmp_path / "navigation_system.yaml"
    write_config(config_path, "missing.pcd")

    with pytest.raises(RuntimeError, match="map_pcd_path does not point to a file"):
        load_navigation_system_defaults(str(config_path))


def test_loads_node_parameters_from_combined_config(tmp_path: Path) -> None:
    config_path = tmp_path / "navigation_system.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "fast_anchor_ekf": {
                    "ros__parameters": {
                        "frequency": 50.0,
                        "odom0_config": [True, False],
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    parameters = load_node_parameters(str(config_path), "fast_anchor_ekf")

    assert parameters["frequency"] == 50.0
    assert parameters["odom0_config"] == [True, False]
