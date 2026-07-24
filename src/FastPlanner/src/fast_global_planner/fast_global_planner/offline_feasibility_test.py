"""Finite migrated MapProcessor planner test plus FastPlanner path validation."""

import argparse
import csv
from pathlib import Path
import sys
import time

import yaml

from fast_planner_common.feasibility_checker import FeasibilityChecker
from fast_planner_common.planner_test_core import (
    PlannerTestCase,
    PlannerTestResult,
    TomogramAStarPlanner,
    TomogramMap,
    path_length,
    vector3,
)
from fast_planner_common.project_paths import (
    find_fast_planner_root,
    find_map_processor_root,
)


def _cases(path: Path):
    with path.open('r', encoding='utf-8') as handle:
        payload = yaml.safe_load(handle) or {}
    result = []
    for index, item in enumerate(payload.get('test_cases', [])):
        name = str(item.get('name', 'case_%d' % index))
        try:
            result.append(
                PlannerTestCase(name, vector3(item.get('start'), 'start'), vector3(item.get('goal'), 'goal'))
            )
        except ValueError as exc:
            result.append(PlannerTestCase(name, vector3([0, 0, 0], 'start'), vector3([0, 0, 0], 'goal'), str(exc)))
    if not result:
        raise ValueError('No test_cases found in %s' % path)
    return result


def main(argv=None):
    map_root = find_map_processor_root()
    fast_root = find_fast_planner_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', default=str(map_root / 'maps/output/map_preprocessed.pcd'))
    parser.add_argument('--tomogram', default=str(map_root / 'maps/tomogram/map_preprocessed.pickle'))
    parser.add_argument('--octomap', default=str(map_root / 'maps/output/map_preprocessed.bt'))
    parser.add_argument(
        '--cases', default=str(fast_root / 'src/fast_global_planner/config/planner_test_cases.yaml')
    )
    parser.add_argument('--output-dir', default=str(fast_root / 'debug/feasibility'))
    parser.add_argument('--traversable-cost-threshold', type=float, default=25.0)
    parser.add_argument('--clearance-threshold', type=float, default=0.2)
    parser.add_argument('--max-planning-time-sec', type=float, default=10.0)
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    checker = FeasibilityChecker(
        args.tomogram,
        args.pcd,
        args.octomap,
        traversable_cost_threshold=args.traversable_cost_threshold,
        clearance_threshold=args.clearance_threshold,
    )
    if not checker.load():
        print('Map load failed: %s' % checker.load_error, file=sys.stderr)
        return 2
    tomogram = TomogramMap.load(Path(args.tomogram), args.traversable_cost_threshold)
    planner = TomogramAStarPlanner(tomogram, max_planning_time_sec=args.max_planning_time_sec)
    cases = _cases(Path(args.cases))
    results = []
    all_points = []
    for test_case in cases:
        result = PlannerTestResult(
            name=test_case.name,
            start=test_case.start.tolist(),
            goal=test_case.goal.tolist(),
            backend='astar',
        )
        if test_case.error:
            result.failure_reason = test_case.error
            results.append({'planner': result.as_dict(), 'feasibility': {}})
            continue
        started = time.perf_counter()
        path, expanded, reason = planner.plan(test_case.start, test_case.goal)
        result.planning_time_ms = (time.perf_counter() - started) * 1000.0
        result.expanded_nodes = expanded
        if path is None:
            result.failure_reason = reason
            results.append({'planner': result.as_dict(), 'feasibility': {}})
            continue
        result.success = True
        result.path = path
        result.path_size = len(path)
        result.path_length = path_length(path)
        feasibility = checker.check(path, check_type='offline_post')
        results.append({'planner': result.as_dict(), 'feasibility': feasibility.as_dict()})
        for point in feasibility.points:
            row = point.as_dict()
            row['test_case'] = test_case.name
            all_points.append(row)

    payload = {
        'map': checker.map_summary(),
        'clearance_threshold': args.clearance_threshold,
        'test_case_count': len(results),
        'overall_success': all(
            item['planner'].get('success') and item['feasibility'].get('feasible')
            for item in results
        ),
        'test_results': results,
    }
    (output_dir / 'feasibility_report.yaml').write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding='utf-8'
    )
    with (output_dir / 'feasibility_points.csv').open('w', newline='', encoding='utf-8') as handle:
        fields = [
            'test_case', 'index', 'source_segment', 'x', 'y', 'z', 'feasible',
            'collision', 'clearance', 'traversable', 'traversal_cost', 'ground_z',
            'pcd_bounds_ok', 'tomogram_bounds_ok', 'reason',
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(all_points)
    print('Feasibility report: %s' % (output_dir / 'feasibility_report.yaml'))
    print('Feasibility points: %s' % (output_dir / 'feasibility_points.csv'))
    print('Overall success: %s' % payload['overall_success'])
    return 0 if payload['overall_success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
