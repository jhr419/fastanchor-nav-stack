import os
from pathlib import Path
import pickle

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _fast_planner_root():
    configured = os.environ.get('FAST_PLANNER_ROOT', '').strip()
    if configured:
        candidate = Path(configured).expanduser().resolve()
        if (candidate / 'src' / 'fast_global_planner').is_dir():
            return candidate
        raise RuntimeError('FAST_PLANNER_ROOT is invalid: %s' % candidate)
    share = Path(get_package_share_directory('fast_global_planner')).resolve()
    for candidate in (Path.cwd().resolve(), *Path.cwd().resolve().parents, share, *share.parents):
        if (candidate / 'src' / 'fast_global_planner').is_dir():
            return candidate
    raise RuntimeError('Cannot locate FastPlanner root; set FAST_PLANNER_ROOT.')


def _map_processor_root(requested):
    value = str(requested or os.environ.get('MAP_PROCESSOR_ROOT', '')).strip()
    if value:
        candidate = Path(value).expanduser().resolve()
    else:
        candidate = (_fast_planner_root().parent / 'MapProcessor').resolve()
    if not (candidate / 'maps' / 'output').is_dir() or not (
        candidate / 'maps' / 'tomogram'
    ).is_dir():
        raise RuntimeError('MapProcessor outputs are not available under: %s' % candidate)
    return candidate


def _resolve_map_file(root, value):
    path = Path(str(value)).expanduser()
    return path.resolve() if path.is_absolute() else (root / path).resolve()


def _prepare_native_pct_tomogram(source: Path, fast_root: Path) -> Path:
    """Write a NumPy-1/2-neutral NPZ cache for the NumPy-1 native PCT node."""
    cache_dir = fast_root / 'debug' / 'native_pct'
    cache_dir.mkdir(parents=True, exist_ok=True)
    output = cache_dir / (source.stem + '_numpy_compat.npz')
    if output.is_file() and output.stat().st_mtime_ns >= source.stat().st_mtime_ns:
        return output
    with source.open('rb') as handle:
        payload = pickle.load(handle)
    np.savez(
        output,
        data=np.asarray(payload['data']),
        resolution=np.asarray(payload['resolution']),
        center=np.asarray(payload['center']),
        slice_h0=np.asarray(payload['slice_h0']),
        slice_dh=np.asarray(payload['slice_dh']),
    )
    return output


def _launch_setup(context, *args, **kwargs):
    algorithm = LaunchConfiguration('algorithm').perform(context).strip().lower()
    if algorithm not in ('astar', 'pct', 'jie_octomap'):
        raise RuntimeError("algorithm must be one of: astar, pct, jie_octomap")

    map_root = _map_processor_root(LaunchConfiguration('map_processor_root').perform(context))
    fast_root = _fast_planner_root()
    pcd_file = _resolve_map_file(map_root, LaunchConfiguration('pcd_file').perform(context))
    tomogram_file = _resolve_map_file(
        map_root, LaunchConfiguration('tomogram_file').perform(context)
    )
    octomap_file = _resolve_map_file(
        map_root, LaunchConfiguration('octomap_file').perform(context)
    )
    for label, path in (('PCD', pcd_file), ('tomogram', tomogram_file)):
        if not path.is_file():
            raise RuntimeError('%s map output is missing: %s' % (label, path))

    config_file = LaunchConfiguration('config_file').perform(context)
    start_source = LaunchConfiguration('start_source').perform(context)
    backend_start_source = 'tf' if start_source == 'tf' else 'topic'
    raw_path_topic = '/fast_global_planner/raw_path'
    raw_marker_topic = '/fast_global_planner/raw_path_marker'
    validated_pose_topic = '/fast_global_planner/validated_goal_pose_3d'
    validated_point_topic = '/fast_global_planner/validated_goal_point_3d'
    validated_rviz_topic = '/fast_global_planner/validated_goal_pose_2d'
    common_backend = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'start_source': backend_start_source,
        'goal_pose_topic': validated_pose_topic,
        'goal_point_topic': validated_point_topic,
        'rviz_2d_goal_topic': validated_rviz_topic,
        'publish_path_topic': raw_path_topic,
        'publish_alias_path_topic': '',
        'publish_marker_topic': raw_marker_topic,
    }
    actions = [
        LogInfo(msg=['FastPlanner backend: ', algorithm]),
        LogInfo(msg=['MapProcessor root: ', str(map_root)]),
        LogInfo(
            msg=[
                'Feasibility check: ',
                LaunchConfiguration('enable_feasibility_check'),
            ]
        ),
        LogInfo(msg=['Mode: ', LaunchConfiguration('feasibility_check_mode')]),
    ]

    launch_map_publisher = _as_bool(
        LaunchConfiguration('launch_map_publisher').perform(context)
    )
    if launch_map_publisher and algorithm in ('astar', 'pct'):
        pct_config = os.path.join(
            get_package_share_directory('pct_global_planner'), 'config', 'pct_global_planner.yaml'
        )
        actions.append(
            Node(
                package='pct_global_planner',
                executable='pct_global_map_publisher_node',
                name='pct_global_map_publisher',
                output='screen',
                parameters=[
                    pct_config,
                    {
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        ),
                        'map_frame': 'map',
                        'pcd_file': str(pcd_file),
                        'tomogram_dir': str(tomogram_file.parent),
                        'tomogram_file': tomogram_file.stem,
                        'publish_raw_cloud': ParameterValue(
                            LaunchConfiguration('publish_raw_cloud'), value_type=bool
                        ),
                        'publish_tomogram_cloud': True,
                    },
                ],
            )
        )

    if algorithm == 'astar':
        backend_config = os.path.join(
            get_package_share_directory('nav3d_global_planning'),
            'config',
            'astar_global_planner.yaml',
        )
        actions.append(
            Node(
                package='nav3d_global_planning',
                executable='astar_global_planner_node',
                name='astar_global_planner_node',
                output='screen',
                parameters=[
                    backend_config,
                    {
                        **common_backend,
                        'pcd_file': str(pcd_file),
                        'octomap_file': str(octomap_file),
                        'start_pose_topic': '/fast_global_planner/backend_start_pose',
                    },
                ],
            )
        )
    elif algorithm == 'pct':
        use_native_pct = _as_bool(
            LaunchConfiguration('use_native_pct_backend').perform(context)
        )
        planner_tomogram_dir = tomogram_file.parent
        planner_tomogram_stem = tomogram_file.stem
        if use_native_pct:
            native_tomogram = _prepare_native_pct_tomogram(tomogram_file, fast_root)
            planner_tomogram_dir = native_tomogram.parent
            planner_tomogram_stem = native_tomogram.stem
            actions.append(
                LogInfo(msg=['Native PCT NumPy compatibility cache: ', str(native_tomogram)])
            )
        backend_config = os.path.join(
            get_package_share_directory('pct_global_planner'),
            'config',
            'pct_global_planner.yaml',
        )
        actions.append(
            Node(
                package='pct_global_planner',
                executable='pct_global_planner_node',
                name='pct_global_planner_node',
                output='screen',
                parameters=[
                    backend_config,
                    {
                        **common_backend,
                        'map_file': str(pcd_file),
                        'pcd_file': str(pcd_file),
                        'tomogram_dir': str(planner_tomogram_dir),
                        'tomogram_file': planner_tomogram_stem,
                        'planner_lib_dir': str(fast_root / 'vendor' / 'pct_planner' / 'lib'),
                        'pct_ros2_source_dir': str(fast_root / 'src' / 'pct_planner_ros2'),
                        'use_native_pct_backend': use_native_pct,
                        'pct_marker_goal_pose_topic': validated_pose_topic,
                        'start_pose_topic': '/fast_global_planner/backend_start_pose',
                    },
                ],
                additional_env={'PYTHONNOUSERSITE': '1'} if use_native_pct else {},
            )
        )
    else:
        backend_config = os.path.join(
            get_package_share_directory('octo_planner'),
            'config',
            'jie_3d_global_planner.yaml',
        )
        if not octomap_file.is_file():
            actions.extend(
                [
                    LogInfo(
                        msg=['[FastPlanner] OctoMap missing; building from PCD: ', str(pcd_file)]
                    ),
                    Node(
                        package='jie_octomap',
                        executable='pcd_to_octomap_node',
                        name='jie_3d_pcd_to_octomap',
                        output='screen',
                        parameters=[
                            backend_config,
                            {
                                'pcd_file': str(pcd_file),
                                'octomap_topic': '/octomap',
                                'frame_id': 'map',
                            },
                        ],
                    ),
                ]
            )
        actions.extend(
            [
                Node(
                    package='octo_planner',
                    executable='jie_path_node',
                    name='jie_path_node',
                    output='screen',
                    parameters=[backend_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
                ),
                Node(
                    package='octo_planner',
                    executable='jie_3d_global_planner_adapter_node',
                    name='jie_3d_global_planner_adapter',
                    output='screen',
                    parameters=[
                        backend_config,
                        {
                            **common_backend,
                            'octomap_file': str(octomap_file),
                            'map_pcd_file': str(pcd_file),
                            'manual_start_pose_topic': '/fast_global_planner/backend_start_pose',
                        },
                    ],
                ),
            ]
        )

    actions.append(
        Node(
            package='fast_global_planner',
            executable='fast_global_planner_node',
            name='fast_global_planner_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    ),
                    'start_source': start_source,
                    'pcd_file': str(pcd_file),
                    'tomogram_file': str(tomogram_file),
                    'octomap_file': str(octomap_file),
                    'debug_output_dir': str(fast_root / 'debug' / 'feasibility'),
                    'enable_feasibility_check': ParameterValue(
                        LaunchConfiguration('enable_feasibility_check'), value_type=bool
                    ),
                    'feasibility_check_mode': LaunchConfiguration('feasibility_check_mode'),
                    'feasibility_clearance_threshold': ParameterValue(
                        LaunchConfiguration('feasibility_clearance_threshold'), value_type=float
                    ),
                    'fail_on_infeasible_path': ParameterValue(
                        LaunchConfiguration('fail_on_infeasible_path'), value_type=bool
                    ),
                    'publish_feasibility_debug': ParameterValue(
                        LaunchConfiguration('publish_feasibility_debug'), value_type=bool
                    ),
                },
            ],
        )
    )

    if _as_bool(LaunchConfiguration('launch_goal_marker').perform(context)):
        marker_config = os.path.join(
            get_package_share_directory('pct_global_planner'), 'config', 'pct_global_planner.yaml'
        )
        actions.append(
            Node(
                package='pct_global_planner',
                executable='pct_goal_marker_node',
                name='pct_start_goal_marker',
                output='screen',
                parameters=[marker_config],
            )
        )

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='fast_planner_rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        )
    )
    return actions


def generate_launch_description():
    share = get_package_share_directory('fast_global_planner')
    return LaunchDescription(
        [
            DeclareLaunchArgument('algorithm', default_value='astar'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'config_file', default_value=os.path.join(share, 'config', 'global_planner.yaml')
            ),
            DeclareLaunchArgument('map_processor_root', default_value=''),
            DeclareLaunchArgument('pcd_file', default_value='maps/output/map_preprocessed.pcd'),
            DeclareLaunchArgument(
                'tomogram_file', default_value='maps/tomogram/map_preprocessed.pickle'
            ),
            DeclareLaunchArgument('octomap_file', default_value='maps/output/map_preprocessed.bt'),
            DeclareLaunchArgument('start_source', default_value='tf'),
            DeclareLaunchArgument('use_native_pct_backend', default_value='true'),
            DeclareLaunchArgument('enable_feasibility_check', default_value='true'),
            DeclareLaunchArgument('feasibility_check_mode', default_value='post'),
            DeclareLaunchArgument('feasibility_clearance_threshold', default_value='0.2'),
            DeclareLaunchArgument('fail_on_infeasible_path', default_value='true'),
            DeclareLaunchArgument('publish_feasibility_debug', default_value='true'),
            DeclareLaunchArgument('launch_map_publisher', default_value='true'),
            DeclareLaunchArgument('publish_raw_cloud', default_value='true'),
            DeclareLaunchArgument('launch_goal_marker', default_value='true'),
            DeclareLaunchArgument('launch_rviz', default_value='false'),
            DeclareLaunchArgument(
                'rviz_config', default_value=os.path.join(share, 'rviz', 'fast_planner.rviz')
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
