"""MapProcessor-compatible pre/post path feasibility validation."""

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from .pcd_io import read_pcd_xyz
from .planner_test_core import TomogramMap, pcd_statistics

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - exercised on minimal ROS installations
    cKDTree = None


def _finite_or_text(value: float):
    return float(value) if math.isfinite(value) else 'inf'


@dataclass
class FeasibilityPoint:
    index: int
    source_segment: int
    xyz: Tuple[float, float, float]
    feasible: bool
    collision: bool
    clearance: float
    traversable: bool
    traversal_cost: float
    ground_z: float
    pcd_bounds_ok: bool
    tomogram_bounds_ok: bool
    reason: str = ''

    def as_dict(self) -> Dict:
        return {
            'index': int(self.index),
            'source_segment': int(self.source_segment),
            'x': float(self.xyz[0]),
            'y': float(self.xyz[1]),
            'z': float(self.xyz[2]),
            'feasible': bool(self.feasible),
            'collision': bool(self.collision),
            'clearance': _finite_or_text(self.clearance),
            'traversable': bool(self.traversable),
            'traversal_cost': _finite_or_text(self.traversal_cost),
            'ground_z': _finite_or_text(self.ground_z),
            'pcd_bounds_ok': bool(self.pcd_bounds_ok),
            'tomogram_bounds_ok': bool(self.tomogram_bounds_ok),
            'reason': self.reason,
        }


@dataclass
class FeasibilityReport:
    check_type: str
    feasible: bool
    reasons: List[str] = field(default_factory=list)
    points: List[FeasibilityPoint] = field(default_factory=list)
    original_point_count: int = 0
    sampled_point_count: int = 0
    infeasible_point_count: int = 0
    collision_point_count: int = 0
    discontinuity_count: int = 0
    minimum_clearance: float = math.inf
    maximum_segment_length: float = 0.0

    def as_dict(self) -> Dict:
        return {
            'check_type': self.check_type,
            'feasible': bool(self.feasible),
            'reasons': list(self.reasons),
            'original_point_count': int(self.original_point_count),
            'sampled_point_count': int(self.sampled_point_count),
            'infeasible_point_count': int(self.infeasible_point_count),
            'collision_point_count': int(self.collision_point_count),
            'discontinuity_count': int(self.discontinuity_count),
            'minimum_clearance': _finite_or_text(self.minimum_clearance),
            'maximum_segment_length': float(self.maximum_segment_length),
            'points': [point.as_dict() for point in self.points],
        }


class PcdClearanceIndex:
    """Height-aware obstacle distance queries over MapProcessor's PCD result."""

    def __init__(self, points: np.ndarray, fallback_resolution: float = 0.14):
        points = np.asarray(points, dtype=np.float32)
        finite = np.isfinite(points[:, :3]).all(axis=1)
        self.points = np.ascontiguousarray(points[finite, :3])
        self.resolution = max(0.05, float(fallback_resolution))
        self.xy_tree = cKDTree(self.points[:, :2]) if cKDTree is not None else None
        self.xy_cells = None
        if self.xy_tree is None:
            self.xy_cells = {}
            indices = np.floor(self.points[:, :2] / self.resolution).astype(np.int32)
            for point_index, cell in enumerate(indices):
                self.xy_cells.setdefault((int(cell[0]), int(cell[1])), []).append(point_index)

    def query(
        self,
        point: Sequence[float],
        obstacle_min_relative_z: float,
        obstacle_max_relative_z: float,
        search_radius: float,
    ) -> float:
        xyz = np.asarray(point, dtype=np.float64)
        radius = max(0.01, float(search_radius))
        if self.xy_tree is not None:
            candidates = self.xy_tree.query_ball_point(xyz[:2], radius)
        else:
            candidates = []
            center = np.floor(xyz[:2] / self.resolution).astype(np.int32)
            cell_radius = int(math.ceil(radius / self.resolution))
            for dx in range(-cell_radius, cell_radius + 1):
                for dy in range(-cell_radius, cell_radius + 1):
                    candidates.extend(
                        self.xy_cells.get((int(center[0] + dx), int(center[1] + dy)), ())
                    )

        if not candidates:
            return math.inf
        nearby = self.points[np.asarray(candidates, dtype=np.int64)]
        relative_z = nearby[:, 2] - xyz[2]
        body = (
            (relative_z >= float(obstacle_min_relative_z))
            & (relative_z <= float(obstacle_max_relative_z))
        )
        if not np.any(body):
            return math.inf
        delta_xy = nearby[body, :2].astype(np.float64) - xyz[:2]
        return float(np.linalg.norm(delta_xy, axis=1).min())


class FeasibilityChecker:
    """Validate poses and paths against tomogram traversability and PCD clearance."""

    def __init__(
        self,
        tomogram_file: str,
        pcd_file: str,
        octomap_file: str = '',
        traversable_cost_threshold: float = 25.0,
        clearance_threshold: float = 0.2,
        path_sample_resolution: float = 0.10,
        max_path_segment_length: float = 1.0,
        max_ground_z_error: float = 0.50,
        obstacle_min_relative_z: float = 0.20,
        obstacle_max_relative_z: float = 1.40,
        clearance_search_radius: float = 1.50,
    ):
        self.tomogram_file = str(tomogram_file)
        self.pcd_file = str(pcd_file)
        self.octomap_file = str(octomap_file)
        self.traversable_cost_threshold = float(traversable_cost_threshold)
        self.clearance_threshold = max(0.0, float(clearance_threshold))
        self.path_sample_resolution = max(0.02, float(path_sample_resolution))
        self.max_path_segment_length = max(0.0, float(max_path_segment_length))
        self.max_ground_z_error = max(0.0, float(max_ground_z_error))
        self.obstacle_min_relative_z = float(obstacle_min_relative_z)
        self.obstacle_max_relative_z = float(obstacle_max_relative_z)
        self.clearance_search_radius = max(
            self.clearance_threshold, float(clearance_search_radius)
        )
        self.tomogram: Optional[TomogramMap] = None
        self.pcd_index: Optional[PcdClearanceIndex] = None
        self.pcd_stats: Optional[Dict] = None
        self.loaded = False
        self.load_error = ''

    def load(self) -> bool:
        try:
            self.tomogram = TomogramMap.load(
                Path(self.tomogram_file), self.traversable_cost_threshold
            )
            points = read_pcd_xyz(self.pcd_file)
            self.pcd_stats = pcd_statistics(points)
            self.pcd_index = PcdClearanceIndex(points, self.tomogram.resolution)
            self.loaded = True
            self.load_error = ''
        except Exception as exc:  # noqa: BLE001 - caller needs a complete startup reason
            self.tomogram = None
            self.pcd_index = None
            self.pcd_stats = None
            self.loaded = False
            self.load_error = str(exc)
        return self.loaded

    def map_summary(self) -> Dict:
        return {
            'loaded': bool(self.loaded),
            'load_error': self.load_error,
            'pcd_file': self.pcd_file,
            'tomogram_file': self.tomogram_file,
            'octomap_file': self.octomap_file,
            'octomap_exists': bool(self.octomap_file and Path(self.octomap_file).is_file()),
            'pcd': self.pcd_stats or {},
            'tomogram': self.tomogram.statistics() if self.tomogram is not None else {},
        }

    @staticmethod
    def _sample(points: np.ndarray, resolution: float):
        sampled = []
        for index, point in enumerate(points):
            if index == 0:
                sampled.append((point.copy(), 0))
                continue
            previous = points[index - 1]
            length = float(np.linalg.norm(point - previous))
            steps = max(1, int(math.ceil(length / resolution)))
            for step in range(1, steps + 1):
                ratio = step / float(steps)
                sampled.append((previous + ratio * (point - previous), index - 1))
        return sampled

    def check(
        self,
        points: Sequence[Sequence[float]],
        check_type: str = 'post',
        check_connectivity: bool = True,
    ) -> FeasibilityReport:
        array = np.asarray(points, dtype=np.float64)
        report = FeasibilityReport(check_type=check_type, feasible=False)
        if array.ndim != 2 or array.shape[1] != 3 or len(array) == 0:
            report.reasons.append('path contains no valid XYZ points')
            return report
        report.original_point_count = int(len(array))
        if not np.isfinite(array).all():
            report.reasons.append('path contains non-finite XYZ coordinates')
            return report
        if not self.loaded or self.tomogram is None or self.pcd_index is None:
            report.reasons.append('feasibility map is not loaded: %s' % self.load_error)
            return report

        if len(array) > 1:
            lengths = np.linalg.norm(np.diff(array, axis=0), axis=1)
            report.maximum_segment_length = float(lengths.max())
            if check_connectivity and self.max_path_segment_length > 0.0:
                report.discontinuity_count = int(
                    np.count_nonzero(lengths > self.max_path_segment_length)
                )
                if report.discontinuity_count:
                    report.reasons.append(
                        '%d path segment(s) exceed maximum length %.3fm'
                        % (report.discontinuity_count, self.max_path_segment_length)
                    )

        sampled = (
            self._sample(array, self.path_sample_resolution)
            if check_connectivity
            else [(point.copy(), index) for index, point in enumerate(array)]
        )
        report.sampled_point_count = len(sampled)
        pcd_bounds = self.pcd_stats.get('bounds', {}) if self.pcd_stats else {}
        for sample_index, (point, segment_index) in enumerate(sampled):
            status = self.tomogram.pose_status(point, pcd_bounds)
            clearance = self.pcd_index.query(
                point,
                self.obstacle_min_relative_z,
                self.obstacle_max_relative_z,
                self.clearance_search_radius,
            )
            collision = math.isfinite(clearance) and clearance < self.clearance_threshold
            ground_z = float(status.get('ground_z', math.inf))
            ground_ok = (
                math.isfinite(ground_z)
                and abs(float(point[2]) - ground_z) <= self.max_ground_z_error
            )
            reasons = []
            if not status.get('pcd_bounds_ok', False):
                reasons.append('outside PCD bounds')
            if not status.get('tomogram_bounds_ok', False):
                reasons.append('outside tomogram bounds')
            elif not status.get('traversable', False):
                reasons.append(status.get('reason') or 'tomogram cell is not traversable')
            if status.get('tomogram_bounds_ok', False) and not ground_ok:
                reasons.append(
                    'z differs from tomogram ground by more than %.3fm'
                    % self.max_ground_z_error
                )
            if collision:
                reasons.append(
                    'clearance %.3fm is below threshold %.3fm'
                    % (clearance, self.clearance_threshold)
                )
            feasible = not reasons
            point_result = FeasibilityPoint(
                index=sample_index,
                source_segment=segment_index,
                xyz=(float(point[0]), float(point[1]), float(point[2])),
                feasible=feasible,
                collision=collision,
                clearance=clearance,
                traversable=bool(status.get('traversable', False)),
                traversal_cost=float(status.get('traversal_cost', math.inf)),
                ground_z=ground_z,
                pcd_bounds_ok=bool(status.get('pcd_bounds_ok', False)),
                tomogram_bounds_ok=bool(status.get('tomogram_bounds_ok', False)),
                reason='; '.join(reasons),
            )
            report.points.append(point_result)
            if not feasible:
                report.infeasible_point_count += 1
            if collision:
                report.collision_point_count += 1
            if math.isfinite(clearance):
                report.minimum_clearance = min(report.minimum_clearance, clearance)

        if report.infeasible_point_count:
            report.reasons.append(
                '%d sampled point(s) are infeasible' % report.infeasible_point_count
            )
        report.feasible = not report.reasons
        return report

    def check_poses(self, start: Sequence[float], goal: Sequence[float]) -> FeasibilityReport:
        return self.check([start, goal], check_type='pre', check_connectivity=False)
