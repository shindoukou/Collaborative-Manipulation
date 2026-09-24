#!/usr/bin/env python3
"""Generate isolated Catch-V1 and Move-V2 trajectories for the new UR3 base."""

import argparse
import datetime as dt
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sample_kirara import sample_pair


PACKAGE = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_ball')
GENERATED = PACKAGE / 'generated'
DEVEL_LIB = Path('/home/mas-2/3rd_party/CM_ws/devel/lib/cm_kirara_ball')
MOVE_TARGET = np.array([0.34299429, 0.25787200, 0.58009389])


def dh_matrix(theta, d, alpha, a):
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def chain_transform(joints, d, a):
    transform = np.eye(4)
    alpha = [0.5 * np.pi, 0.0, 0.0, 0.5 * np.pi, -0.5 * np.pi, 0.0]
    for values in zip(joints, d, alpha, a):
        transform = transform.dot(dh_matrix(*values))
    return transform


def move_tool_midpoint(joints):
    joints = np.asarray(joints, dtype=float)
    left = np.diag([-1.0, -1.0, 1.0, 1.0]).dot(chain_transform(
        joints[:6], [0.1625, 0, 0, 0.1333, 0.0997, 0.1434],
        [0, -0.425, -0.39225, 0, 0, 0]))
    right_base = np.eye(4)
    right_base[:3, 3] = [0.0, 0.437, 0.0]
    right = right_base.dot(chain_transform(
        joints[6:], [0.1519, 0, 0, 0.11235, 0.08535, 0.1257],
        [0, -0.24365, -0.21325, 0, 0, 0]))
    return 0.5 * (left[:3, 3] + right[:3, 3])


def configure_outputs(config, run_dir, phase, stamp):
    config['kirara_path']['left'] = str(run_dir / '{}_left_coeff_{}.txt'.format(phase, stamp))
    config['kirara_path']['right'] = str(run_dir / '{}_right_coeff_{}.txt'.format(phase, stamp))
    config['planned_path']['left'] = str(
        run_dir / '{}_left_planned_path_{}.txt'.format(phase, stamp))
    config['planned_path']['right'] = str(
        run_dir / '{}_right_planned_path_{}.txt'.format(phase, stamp))
    config['goal_generator']['debug_file'] = str(run_dir / '{}_ik_{}.log'.format(phase, stamp))
    config['goal_generator']['output_file'] = str(run_dir / '{}_goal_q_{}.yaml'.format(phase, stamp))


def run_planner(executable, config_path, log_path):
    command = [str(DEVEL_LIB / executable), str(config_path)]
    started = time.monotonic()
    with log_path.open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    elapsed = time.monotonic() - started
    if result.returncode != 0:
        raise RuntimeError('{} failed with code {}; see {}'.format(
            executable, result.returncode, log_path))
    return elapsed


def require_file(path):
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError('planner did not create {}'.format(path))


def write_failure_manifest(run_dir, stamp, phase, error, log_path):
    manifest = {
        'timestamp': stamp,
        'status': 'failed',
        'failed_phase': phase,
        'error': str(error),
        'planner_log': str(log_path),
        'run_dir': str(run_dir),
        'ur3_base_in_ur5_installation_frame_m': [0.0, -0.437, 0.0],
        'ur3_base_in_world_dh_frame_m': [0.0, 0.437, 0.0],
    }
    path = run_dir / 'failure_manifest_{}.json'.format(stamp)
    path.write_text(json.dumps(manifest, indent=2) + '\n')
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stamp', help='YYYYMMDD_HHMMSS; defaults to current time')
    parser.add_argument('--sample-dt', type=float, default=0.01)
    parser.add_argument('--speed-scale', type=float, default=4.0,
                        help='requested geometric time scaling; safety limits may reduce it')
    parser.add_argument('--reuse-catch-run', type=Path,
                        help='reuse a previously collision-revalidated Catch result')
    args = parser.parse_args()

    stamp = args.stamp or dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = GENERATED / ('run_' + stamp)
    run_dir.mkdir(parents=True, exist_ok=False)

    catch = yaml.safe_load((PACKAGE / 'config' / 'catch_template.yaml').read_text())
    move = yaml.safe_load((PACKAGE / 'config' / 'move_template.yaml').read_text())
    configure_outputs(catch, run_dir, 'catch', stamp)
    configure_outputs(move, run_dir, 'move', stamp)

    catch_config = run_dir / 'catch_config_{}.yaml'.format(stamp)
    catch_config.write_text(yaml.safe_dump(catch, sort_keys=False))
    catch_left = Path(catch['kirara_path']['left'])
    catch_right = Path(catch['kirara_path']['right'])
    catch_log = run_dir / 'catch_planner_{}.log'.format(stamp)
    reused_catch_source = None
    if args.reuse_catch_run:
        reused_catch_source = args.reuse_catch_run.resolve()
        source_manifest_path = next(
            reused_catch_source.glob('trajectory_manifest_*.json'))
        source_manifest = json.loads(source_manifest_path.read_text())
        shutil.copy2(next(reused_catch_source.glob('catch_left_coeff_*.txt')), catch_left)
        shutil.copy2(next(reused_catch_source.glob('catch_right_coeff_*.txt')), catch_right)
        shutil.copy2(
            next(reused_catch_source.glob('catch_left_planned_path_*.txt')),
            Path(catch['planned_path']['left']))
        shutil.copy2(
            next(reused_catch_source.glob('catch_right_planned_path_*.txt')),
            Path(catch['planned_path']['right']))
        shutil.copy2(next(reused_catch_source.glob('catch_left_samples_*.txt')),
                     run_dir / 'catch_left_samples_{}.txt'.format(stamp))
        shutil.copy2(next(reused_catch_source.glob('catch_right_samples_*.txt')),
                     run_dir / 'catch_right_samples_{}.txt'.format(stamp))
        catch_sample = dict(source_manifest['catch'])
        catch_sample.pop('planner_wall_time_s', None)
        catch_time = 0.0
        catch_log.write_text(
            'Reused Catch from {} after FCL safety-shell revalidation.\n'.format(
                reused_catch_source))
    else:
        try:
            catch_time = run_planner(
                'CM_KIRARA_BALL_CATCH', catch_config, catch_log)
        except Exception as error:
            failure = write_failure_manifest(
                run_dir, stamp, 'catch', error, catch_log)
            print('Failure manifest: {}'.format(failure), file=sys.stderr)
            raise
        require_file(catch_left)
        require_file(catch_right)
        catch_sample = sample_pair(
            catch_left, catch_right,
            run_dir / 'catch_left_samples_{}.txt'.format(stamp),
            run_dir / 'catch_right_samples_{}.txt'.format(stamp),
            args.sample_dt,
            requested_speed_scale=args.speed_scale,
            rewrite_coefficients=True)

    
    for index, value in enumerate(catch_sample['goal_q'], 1):
        move['Joint_init']['joint_{}'.format(index)] = float(value)
    move_config = run_dir / 'move_config_{}.yaml'.format(stamp)
    move_config.write_text(yaml.safe_dump(move, sort_keys=False))
    move_log = run_dir / 'move_planner_{}.log'.format(stamp)
    try:
        move_time = run_planner(
            'CM_KIRARA_BALL_MOVE', move_config, move_log)
    except Exception as error:
        failure = write_failure_manifest(
            run_dir, stamp, 'move', error, move_log)
        print('Failure manifest: {}'.format(failure), file=sys.stderr)
        raise

    move_left = Path(move['kirara_path']['left'])
    move_right = Path(move['kirara_path']['right'])
    require_file(move_left)
    require_file(move_right)
    move_sample = sample_pair(
        move_left, move_right,
        run_dir / 'move_left_samples_{}.txt'.format(stamp),
        run_dir / 'move_right_samples_{}.txt'.format(stamp),
        args.sample_dt,
        requested_speed_scale=args.speed_scale,
        rewrite_coefficients=True)
    move_goal_midpoint = move_tool_midpoint(move_sample['goal_q'])
    move_goal_target_error = float(np.linalg.norm(
        move_goal_midpoint - MOVE_TARGET))
    if move_goal_target_error > 0.01:
        error = RuntimeError(
            'Move endpoint misses target by {:.6f} m; approximate paths are invalid'.format(
                move_goal_target_error))
        failure = write_failure_manifest(
            run_dir, stamp, 'move_endpoint', error, move_log)
        print('Failure manifest: {}'.format(failure), file=sys.stderr)
        raise error

    manifest = {
        'timestamp': stamp,
        'run_dir': str(run_dir),
        'ur3_base_in_ur5_installation_frame_m': [0.0, -0.437, 0.0],
        'ur3_base_in_world_dh_frame_m': [0.0, 0.437, 0.0],
        'relative_rpy_rad': [0.0, 0.0, 3.1415926],
        'task_layout_offset_m': [-0.07, 0.15, 0.0],
        'gazebo_ball_position_m': [0.490314844, 0.207087123, 0.602334435],
        'gazebo_ball_adjustment_m': [0.0, -0.05, 0.0],
        'gazebo_closed_table': {
            'center_world_m': [0.62986639, 0.167242173, 0.2723345],
            'size_m': [0.625, 0.994, 0.53],
            'joined_inner_bounds_xy_m': {
                'x': [0.31736639, 0.94236639],
                'y': [-0.329757827, 0.664242173],
            },
        },
        'collision_policy': ('all Gazebo-aligned obstacles checked with a 5 mm '
                             'per-side FCL safety shell; only adjacent '
                             'arm links are excluded, and MOVE additionally allows '
                             'the intended carried-ball contact with wrist3/FT links'),
        'planning_obstacle_safety_margin_m': 0.005,
        'post_smoothing_validation_period_s': 0.01,
        'requested_speed_scale': args.speed_scale,
        'reused_catch_source': (str(reused_catch_source)
                                if reused_catch_source else None),
        'catch': dict(catch_sample, planner_wall_time_s=catch_time),
        'move': dict(move_sample, planner_wall_time_s=move_time),
        'move_target_m': MOVE_TARGET.tolist(),
        'move_goal_midpoint_m': move_goal_midpoint.tolist(),
        'move_goal_target_error_m': move_goal_target_error,
        'catch_to_move_joint_gap_rad': max(
            abs(a - b) for a, b in zip(catch_sample['goal_q'], move_sample['start_q'])),
    }
    manifest_path = run_dir / 'trajectory_manifest_{}.json'.format(stamp)
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    (GENERATED / 'latest_run.txt').write_text(str(run_dir) + '\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
