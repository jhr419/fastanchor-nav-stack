"""Project-relative path resolution for the standalone planner workspace."""

import os
from pathlib import Path
from typing import Optional


def _is_fast_planner_root(path: Path) -> bool:
    return (path / 'src' / 'fast_global_planner').is_dir()


def _is_map_processor_root(path: Path) -> bool:
    return (
        (path / 'src' / 'map_processor_core').is_dir()
        and (path / 'maps' / 'output').is_dir()
        and (path / 'maps' / 'tomogram').is_dir()
    )


def find_fast_planner_root() -> Path:
    configured = os.environ.get('FAST_PLANNER_ROOT', '').strip()
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if _is_fast_planner_root(candidate):
            return candidate
        raise RuntimeError('FAST_PLANNER_ROOT is not a FastPlanner workspace: %s' % candidate)

    cwd = Path.cwd().resolve()
    for candidate in (cwd, *cwd.parents):
        if _is_fast_planner_root(candidate):
            return candidate

    source_path = Path(__file__).resolve()
    for candidate in source_path.parents:
        if _is_fast_planner_root(candidate):
            return candidate

    raise RuntimeError(
        'Cannot locate FastPlanner root. Run from the workspace or set FAST_PLANNER_ROOT.'
    )


def find_map_processor_root(configured_root: Optional[str] = None) -> Path:
    raw = str(configured_root or os.environ.get('MAP_PROCESSOR_ROOT', '')).strip()
    if raw:
        candidate = Path(raw).expanduser().resolve()
        if _is_map_processor_root(candidate):
            return candidate
        raise RuntimeError('MAP_PROCESSOR_ROOT is not a MapProcessor workspace: %s' % candidate)

    try:
        fast_root = find_fast_planner_root()
        sibling = (fast_root.parent / 'MapProcessor').resolve()
        if _is_map_processor_root(sibling):
            return sibling
    except RuntimeError:
        pass

    cwd = Path.cwd().resolve()
    for candidate in (cwd, *cwd.parents):
        sibling = candidate / 'MapProcessor'
        if _is_map_processor_root(sibling):
            return sibling.resolve()
        if _is_map_processor_root(candidate):
            return candidate

    raise RuntimeError(
        'Cannot locate MapProcessor. Set MAP_PROCESSOR_ROOT or keep it beside FastPlanner.'
    )


def resolve_fast_planner_path(value: str) -> Path:
    path = Path(str(value)).expanduser()
    return path.resolve() if path.is_absolute() else (find_fast_planner_root() / path).resolve()


def resolve_map_processor_path(value: str, configured_root: Optional[str] = None) -> Path:
    path = Path(str(value)).expanduser()
    return path.resolve() if path.is_absolute() else (
        find_map_processor_root(configured_root) / path
    ).resolve()
