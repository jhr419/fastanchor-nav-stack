"""Reusable map inspection and portable tomogram A* planning helpers."""

from dataclasses import dataclass, field
import heapq
import math
from pathlib import Path
import pickle
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class PlannerTestCase:
    name: str
    start: np.ndarray
    goal: np.ndarray
    error: str = ''


@dataclass
class PlannerTestResult:
    name: str
    start: List[float]
    goal: List[float]
    backend: str
    success: bool = False
    failure_reason: str = ''
    planning_time_ms: float = 0.0
    path_length: float = 0.0
    path_size: int = 0
    expanded_nodes: int = 0
    start_status: Dict = field(default_factory=dict)
    goal_status: Dict = field(default_factory=dict)
    path: Optional[np.ndarray] = field(default=None, repr=False)

    def as_dict(self) -> Dict:
        return {
            'name': self.name,
            'start': list(self.start),
            'goal': list(self.goal),
            'backend': self.backend,
            'success': bool(self.success),
            'failure_reason': self.failure_reason,
            'planning_time_ms': float(self.planning_time_ms),
            'path_length': float(self.path_length),
            'path_size': int(self.path_size),
            'expanded_nodes': int(self.expanded_nodes),
            'start_status': self.start_status,
            'goal_status': self.goal_status,
        }


def vector3(value: Sequence[float], label: str) -> np.ndarray:
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != 3:
        raise ValueError('%s must contain exactly three numeric values' % label)
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError('%s contains a non-finite value' % label)
    return result


def path_length(path: Optional[np.ndarray]) -> float:
    if path is None or len(path) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(np.asarray(path), axis=0), axis=1).sum())


def pcd_statistics(points: np.ndarray) -> Dict:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError('PCD reader returned an invalid point array: %s' % (points.shape,))
    finite = np.isfinite(points[:, :3]).all(axis=1)
    finite_points = points[finite, :3]
    if finite_points.size == 0:
        raise ValueError('PCD contains no finite XYZ points')
    minimum = finite_points.min(axis=0)
    maximum = finite_points.max(axis=0)
    return {
        'loaded': True,
        'point_count': int(points.shape[0]),
        'finite_point_count': int(finite_points.shape[0]),
        'nonfinite_point_count': int(points.shape[0] - finite_points.shape[0]),
        'bounds': {
            'min_x': float(minimum[0]), 'max_x': float(maximum[0]),
            'min_y': float(minimum[1]), 'max_y': float(maximum[1]),
            'min_z': float(minimum[2]), 'max_z': float(maximum[2]),
        },
    }


class TomogramMap:
    """Validated view of the PCT pickle layout used by the original planner."""

    REQUIRED_KEYS = {'data', 'resolution', 'center', 'slice_h0', 'slice_dh'}

    def __init__(self, payload: Dict, traversable_cost_threshold: float):
        missing = self.REQUIRED_KEYS.difference(payload)
        if missing:
            raise ValueError('Tomogram is missing required keys: %s' % sorted(missing))

        data = np.asarray(payload['data'])
        if data.ndim != 4 or data.shape[0] < 5:
            raise ValueError(
                'Tomogram data must have shape [>=5, slices, x, y], got %s' % (data.shape,)
            )
        self.data = data
        self.trav = np.asarray(data[0], dtype=np.float32)
        self.ground = np.asarray(data[3], dtype=np.float32)
        self.ceiling = np.asarray(data[4], dtype=np.float32)
        self.resolution = float(payload['resolution'])
        self.center = np.asarray(payload['center'], dtype=np.float64).reshape(-1)
        self.slice_h0 = float(payload['slice_h0'])
        self.slice_dh = float(payload['slice_dh'])
        self.cost_threshold = float(traversable_cost_threshold)
        if self.resolution <= 0.0 or self.slice_dh <= 0.0:
            raise ValueError('Tomogram resolution and slice_dh must be positive')
        if self.center.size != 2 or not np.isfinite(self.center).all():
            raise ValueError('Tomogram center must contain two finite values')
        if self.trav.shape != self.ground.shape or self.trav.shape != self.ceiling.shape:
            raise ValueError('Tomogram traversal, ground, and ceiling layers do not match')

        self.n_slice, self.size_x, self.size_y = self.trav.shape
        self.offset_x = self.size_x // 2
        self.offset_y = self.size_y // 2
        self.known = np.isfinite(self.ground) & (self.ground > -1.0e5)
        self.free = (
            self.known & np.isfinite(self.trav) & (self.trav <= self.cost_threshold)
        )

    @classmethod
    def load(cls, path: Path, traversable_cost_threshold: float):
        with Path(path).open('rb') as handle:
            payload = pickle.load(handle)
        if not isinstance(payload, dict):
            raise ValueError('Tomogram root object must be a dictionary')
        return cls(payload, traversable_cost_threshold)

    def statistics(self) -> Dict:
        total = int(self.trav.size)
        free = int(np.count_nonzero(self.free))
        known = int(np.count_nonzero(self.known))
        occupied = known - free
        unknown = total - known
        min_x = self.center[0] - self.offset_x * self.resolution
        max_x = self.center[0] + (self.size_x - 1 - self.offset_x) * self.resolution
        min_y = self.center[1] - self.offset_y * self.resolution
        max_y = self.center[1] + (self.size_y - 1 - self.offset_y) * self.resolution
        finite_ground = self.ground[self.known]
        min_z = float(np.min(finite_ground)) if finite_ground.size else self.slice_h0
        max_z = float(np.max(finite_ground)) if finite_ground.size else (
            self.slice_h0 + (self.n_slice - 1) * self.slice_dh
        )
        return {
            'loaded': True,
            'resolution': self.resolution,
            'shape': [int(value) for value in self.data.shape],
            'map_size_cells': {
                'slices': self.n_slice, 'x': self.size_x, 'y': self.size_y,
            },
            'map_size_m': {
                'x': float(self.size_x * self.resolution),
                'y': float(self.size_y * self.resolution),
                'z': float(max_z - min_z),
            },
            'center': [float(value) for value in self.center],
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
            'bounds': {
                'min_x': float(min_x), 'max_x': float(max_x),
                'min_y': float(min_y), 'max_y': float(max_y),
                'min_z': min_z, 'max_z': max_z,
            },
            'traversable_cost_threshold': self.cost_threshold,
            'cell_counts': {
                'total': total,
                'free': free,
                'occupied': occupied,
                'unknown': unknown,
            },
            'cell_classification': (
                'free=finite ground and traversal cost <= threshold; '
                'occupied=finite ground above threshold; unknown=no finite ground'
            ),
        }

    def xy_in_bounds(self, pose: Sequence[float]) -> bool:
        x, y = float(pose[0]), float(pose[1])
        index_x = int(np.rint((x - self.center[0]) / self.resolution)) + self.offset_x
        index_y = int(np.rint((y - self.center[1]) / self.resolution)) + self.offset_y
        return 0 <= index_x < self.size_x and 0 <= index_y < self.size_y

    def pose_to_index(self, pose: Sequence[float]) -> Optional[Tuple[int, int, int]]:
        pose = np.asarray(pose, dtype=np.float64)
        index_x = int(np.rint((pose[0] - self.center[0]) / self.resolution)) + self.offset_x
        index_y = int(np.rint((pose[1] - self.center[1]) / self.resolution)) + self.offset_y
        if not (0 <= index_x < self.size_x and 0 <= index_y < self.size_y):
            return None
        valid_layers = np.flatnonzero(self.free[:, index_x, index_y])
        if valid_layers.size == 0:
            valid_layers = np.flatnonzero(self.known[:, index_x, index_y])
        if valid_layers.size == 0:
            return (0, index_x, index_y)
        heights = self.ground[valid_layers, index_x, index_y]
        layer = int(valid_layers[np.argmin(np.abs(heights - pose[2]))])
        return (layer, index_x, index_y)

    def pose_status(self, pose: Sequence[float], pcd_bounds: Optional[Dict] = None) -> Dict:
        pose = np.asarray(pose, dtype=np.float64)
        pcd_inside = True
        if pcd_bounds:
            pcd_inside = bool(
                pcd_bounds['min_x'] <= pose[0] <= pcd_bounds['max_x']
                and pcd_bounds['min_y'] <= pose[1] <= pcd_bounds['max_y']
                and pcd_bounds['min_z'] <= pose[2] <= pcd_bounds['max_z']
            )
        index = self.pose_to_index(pose)
        if index is None:
            return {
                'pcd_bounds_ok': pcd_inside,
                'tomogram_bounds_ok': False,
                'traversable': False,
                'reason': 'pose is outside tomogram XY bounds',
            }
        layer, index_x, index_y = index
        cost = float(self.trav[layer, index_x, index_y])
        ground = float(self.ground[layer, index_x, index_y])
        known = bool(self.known[layer, index_x, index_y])
        traversable = bool(self.free[layer, index_x, index_y])
        reasons = []
        if not pcd_inside:
            reasons.append('pose is outside PCD XYZ bounds')
        if not known:
            reasons.append('tomogram cell has no finite ground (unknown)')
        elif not np.isfinite(cost):
            reasons.append('tomogram cell has a non-finite traversal cost')
        elif cost > self.cost_threshold:
            reasons.append(
                'tomogram traversal cost %.3f exceeds threshold %.3f'
                % (cost, self.cost_threshold)
            )
        return {
            'pcd_bounds_ok': pcd_inside,
            'tomogram_bounds_ok': True,
            'traversable': traversable,
            'reason': '; '.join(reasons),
            'grid_index': [int(layer), int(index_x), int(index_y)],
            'traversal_cost': cost,
            'ground_z': ground,
        }

    def index_to_world(self, index: Tuple[int, int, int]) -> np.ndarray:
        layer, index_x, index_y = index
        return np.array([
            self.center[0] + (index_x - self.offset_x) * self.resolution,
            self.center[1] + (index_y - self.offset_y) * self.resolution,
            float(self.ground[layer, index_x, index_y]),
        ], dtype=np.float64)

    def traversable(self, index: Tuple[int, int, int]) -> bool:
        layer, index_x, index_y = index
        return bool(
            0 <= layer < self.n_slice
            and 0 <= index_x < self.size_x
            and 0 <= index_y < self.size_y
            and self.free[layer, index_x, index_y]
        )

    def neighbors(
        self,
        index: Tuple[int, int, int],
        max_ground_step: float,
    ) -> Iterable[Tuple[Tuple[int, int, int], float]]:
        layer, index_x, index_y = index
        current_z = float(self.ground[layer, index_x, index_y])
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                next_x = index_x + dx
                next_y = index_y + dy
                if not (0 <= next_x < self.size_x and 0 <= next_y < self.size_y):
                    continue
                for next_layer in range(max(0, layer - 1), min(self.n_slice, layer + 2)):
                    next_index = (next_layer, next_x, next_y)
                    if not self.traversable(next_index):
                        continue
                    next_z = float(self.ground[next_layer, next_x, next_y])
                    dz = abs(next_z - current_z)
                    if dz > max_ground_step:
                        continue
                    horizontal = self.resolution * math.hypot(dx, dy)
                    distance = math.hypot(horizontal, dz)
                    yield next_index, distance


class TomogramAStarPlanner:
    """Portable A* used when the platform-specific PCT pybind runtime is unavailable."""

    def __init__(
        self,
        tomogram: TomogramMap,
        max_planning_time_sec: float = 10.0,
        max_expanded_nodes: int = 500000,
        max_ground_step: float = 0.45,
        traversability_weight: float = 0.5,
    ):
        self.map = tomogram
        self.max_planning_time_sec = max(0.01, float(max_planning_time_sec))
        self.max_expanded_nodes = max(1, int(max_expanded_nodes))
        self.max_ground_step = max(0.0, float(max_ground_step))
        self.traversability_weight = max(0.0, float(traversability_weight))

    def _heuristic(self, left: Tuple[int, int, int], right: Tuple[int, int, int]) -> float:
        left_world = self.map.index_to_world(left)
        right_world = self.map.index_to_world(right)
        return float(np.linalg.norm(left_world - right_world))

    def plan(self, start: Sequence[float], goal: Sequence[float]):
        start_index = self.map.pose_to_index(start)
        goal_index = self.map.pose_to_index(goal)
        if start_index is None or not self.map.traversable(start_index):
            return None, 0, 'start is not on a traversable tomogram cell'
        if goal_index is None or not self.map.traversable(goal_index):
            return None, 0, 'goal is not on a traversable tomogram cell'

        started = time.monotonic()
        sequence = 0
        open_heap = [(self._heuristic(start_index, goal_index), sequence, start_index)]
        g_score = {start_index: 0.0}
        parent = {}
        closed = set()
        expanded = 0

        while open_heap:
            if time.monotonic() - started > self.max_planning_time_sec:
                return None, expanded, 'planning exceeded %.3f seconds' % self.max_planning_time_sec
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal_index:
                indices = [current]
                while current in parent:
                    current = parent[current]
                    indices.append(current)
                indices.reverse()
                path = np.vstack([self.map.index_to_world(value) for value in indices])
                return path, expanded, ''

            closed.add(current)
            expanded += 1
            if expanded >= self.max_expanded_nodes:
                return None, expanded, 'planning reached max_expanded_nodes=%d' % self.max_expanded_nodes

            current_g = g_score[current]
            for neighbor, distance in self.map.neighbors(current, self.max_ground_step):
                if neighbor in closed:
                    continue
                layer, index_x, index_y = neighbor
                cost = max(0.0, float(self.map.trav[layer, index_x, index_y]))
                normalized = cost / max(self.map.cost_threshold, 1.0e-6)
                tentative = current_g + distance * (1.0 + self.traversability_weight * normalized)
                if tentative >= g_score.get(neighbor, math.inf):
                    continue
                parent[neighbor] = current
                g_score[neighbor] = tentative
                sequence += 1
                priority = tentative + self._heuristic(neighbor, goal_index)
                heapq.heappush(open_heap, (priority, sequence, neighbor))

        return None, expanded, 'no connected traversable path was found'
