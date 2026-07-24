import pickle

import numpy as np

from fast_planner_common.feasibility_checker import FeasibilityChecker
from fast_planner_common.pcd_io import write_pcd_xyz
from fast_planner_common.planner_test_core import TomogramAStarPlanner, TomogramMap


def fixture_files(tmp_path):
    data = np.zeros((5, 1, 7, 7), dtype=np.float32)
    data[0] = 1.0
    data[3] = 0.0
    data[4] = 2.0
    payload = {
        'data': data,
        'resolution': 0.5,
        'center': np.array([0.0, 0.0], dtype=np.float32),
        'slice_h0': 0.2,
        'slice_dh': 0.2,
    }
    tomogram = tmp_path / 'map.pickle'
    with tomogram.open('wb') as handle:
        pickle.dump(payload, handle)
    points = np.array(
        [
            [-1.5, -1.5, 0.0],
            [-1.5, 1.5, 0.0],
            [1.5, -1.5, 0.0],
            [1.5, 1.5, 0.0],
            [0.0, 0.0, 0.5],
        ],
        dtype=np.float32,
    )
    pcd = tmp_path / 'map.pcd'
    write_pcd_xyz(str(pcd), points)
    return tomogram, pcd


def test_clearance_collision_and_safe_path(tmp_path):
    tomogram, pcd = fixture_files(tmp_path)
    checker = FeasibilityChecker(
        str(tomogram),
        str(pcd),
        clearance_threshold=0.25,
        path_sample_resolution=0.1,
        obstacle_min_relative_z=0.2,
        obstacle_max_relative_z=1.0,
    )
    assert checker.load(), checker.load_error

    collision = checker.check([[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0]])
    assert not collision.feasible
    assert collision.collision_point_count > 0

    safe = checker.check([[-0.5, 1.0, 0.0], [0.5, 1.0, 0.0]])
    assert safe.feasible
    assert safe.collision_point_count == 0


def test_precheck_does_not_treat_start_goal_as_one_segment(tmp_path):
    tomogram, pcd = fixture_files(tmp_path)
    checker = FeasibilityChecker(str(tomogram), str(pcd), max_path_segment_length=0.1)
    assert checker.load()
    report = checker.check_poses([-1.0, 1.0, 0.0], [1.0, 1.0, 0.0])
    assert report.discontinuity_count == 0
    assert report.sampled_point_count == 2


def test_migrated_tomogram_astar_returns_xyz_start_to_goal(tmp_path):
    tomogram_path, _ = fixture_files(tmp_path)
    tomogram = TomogramMap.load(tomogram_path, 25.0)
    planner = TomogramAStarPlanner(tomogram)
    path, _expanded, reason = planner.plan([-1.0, -1.0, 0.0], [1.0, 1.0, 0.0])
    assert reason == ''
    assert path is not None
    assert path.shape[1] == 3
    assert np.linalg.norm(path[0, :2] - [-1.0, -1.0]) < 0.4
    assert np.linalg.norm(path[-1, :2] - [1.0, 1.0]) < 0.4
