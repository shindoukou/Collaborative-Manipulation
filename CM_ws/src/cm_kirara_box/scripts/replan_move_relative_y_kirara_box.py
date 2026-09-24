#!/usr/bin/env python3
"""Create and select an kirara_box Move constrained near the world Y axis.

Catch and Stage2 are reused unchanged.  The collision-free center route from
the selected planning/KIRARA result is projected onto the box transport manifold;
the result is therefore intentionally recorded as PLANNING_ONLY_TASK_MANIFOLD.
"""

from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
from scipy.interpolate import PchipInterpolator

import plan_kirara_box as planner


DEFAULT_REFERENCE = (planner.GENERATED /
    'run_20260807_213738_move220mm_tightgrip160')
TERMINAL_BLEND_WAYPOINTS = 40
MAX_NOMINAL_JOINT_SPEED_RAD_S = 0.35
MIN_WAYPOINT_TIME_S = 0.04
SAMPLE_PERIOD_S = 0.01


def latest_one(directory, pattern):
    matches = sorted(directory.glob(pattern))
    if not matches:
        raise RuntimeError('missing {} in {}'.format(pattern, directory))
    return matches[-1]


def main():
    reference = DEFAULT_REFERENCE
    if not reference.is_dir():
        raise RuntimeError('reference run does not exist: {}'.format(reference))
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = planner.GENERATED / (
        'run_{}_move220mm_relativeY_PLANNING_ONLY'.format(stamp))
    run_dir.mkdir(parents=True, exist_ok=False)
    staging = planner.GENERATED / 'staging' / ('relative_y_' + stamp)
    command = [
        'python3', str(planner.SCRIPT_DIR / 'generate_task_manifold_move.py'),
        '--samples', '181', '--reference-blend', '1.0',
        '--reference-run', str(reference), '--output-dir', str(staging)]
    subprocess.check_call(command)

    coarse = np.c_[np.loadtxt(str(staging / 'move_left.txt')),
                   np.loadtxt(str(staging / 'move_right.txt'))]
    count = TERMINAL_BLEND_WAYPOINTS
    first, final = coarse[-count-1].copy(), coarse[-1].copy()
    blend = np.linspace(0.0, 1.0, count + 1)
    blend = blend * blend * (3.0 - 2.0 * blend)
    coarse[-count-1:] = first + (final - first) * blend[:, None]

    segment_delta = np.max(np.abs(np.diff(coarse, axis=0)), axis=1)
    segment_time = np.maximum(
        MIN_WAYPOINT_TIME_S,
        segment_delta / MAX_NOMINAL_JOINT_SPEED_RAD_S)
    waypoint_time = np.r_[0.0, np.cumsum(segment_time)]
    sample_time = np.arange(0.0, waypoint_time[-1], SAMPLE_PERIOD_S)
    if waypoint_time[-1] - sample_time[-1] > 1e-8:
        sample_time = np.r_[sample_time, waypoint_time[-1]]
    samples = PchipInterpolator(
        waypoint_time, coarse, axis=0)(sample_time)

    left_path = run_dir / 'move_left_samples_{}.txt'.format(stamp)
    right_path = run_dir / 'move_right_samples_{}.txt'.format(stamp)
    np.savetxt(str(left_path), samples[:, :6], fmt='%.12g')
    np.savetxt(str(right_path), samples[:, 6:], fmt='%.12g')
    subprocess.check_call([
        str(planner.MOVE_EXE), '--validate-samples',
        str(left_path), str(right_path)])

    vectors = np.array([planner.move_dh_vector(row) for row in samples])
    distances = np.linalg.norm(vectors, axis=1)
    transverse = np.hypot(vectors[:, 0], vectors[:, 2])
    y_cos = vectors[:, 1] / distances
    velocity = np.diff(samples, axis=0) / np.diff(sample_time)[:, None]
    wrist = [3, 4, 5, 9, 10, 11]

    shutil.copy2(str(latest_one(reference, 'catch_left_samples_*.txt')),
                 str(run_dir / 'catch_left_samples_{}.txt'.format(stamp)))
    shutil.copy2(str(latest_one(reference, 'catch_right_samples_*.txt')),
                 str(run_dir / 'catch_right_samples_{}.txt'.format(stamp)))
    shutil.copy2(str(planner.CONFIG_MOVE), str(run_dir / 'move_config.yaml'))
    catch_config = reference / 'catch_config.yaml'
    if catch_config.is_file():
        shutil.copy2(str(catch_config),
                     str(run_dir / 'catch_config_REUSED_UNCHANGED.yaml'))

    manifest = {
        'timestamp': stamp,
        'method': 'PLANNING_ONLY_TASK_MANIFOLD_RELATIVE_Y_PCHIP',
        'catch_replanned': False,
        'catch_reused_from': str(reference),
        'move_center_route_reused_from': str(reference),
        'kirara_applied_to_new_move': False,
        'move_points': int(samples.shape[0]),
        'move_duration_s': float(sample_time[-1]),
        'sample_period_s': SAMPLE_PERIOD_S,
        'max_abs_joint_velocity_rad_s': float(np.max(np.abs(velocity))),
        'max_wrist_change_from_start_rad': float(np.max(
            np.abs(samples[:, wrist] - samples[0, wrist][None, :]))),
        'distance_observed_m': [float(distances.min()),
                                float(distances.max())],
        'relative_vector_start_m': vectors[0].tolist(),
        'relative_vector_goal_m': vectors[-1].tolist(),
        'max_transverse_m': float(transverse.max()),
        'max_abs_z_m': float(np.abs(vectors[:, 2]).max()),
        'min_y_cos': float(y_cos.min()),
        'max_angle_from_y_deg': float(np.degrees(
            np.arccos(np.clip(y_cos.min(), -1.0, 1.0)))),
        'native_collision_and_task_validation': 'PASS',
    }
    (run_dir / 'trajectory_manifest_{}.json'.format(stamp)).write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    (planner.GENERATED / 'latest_run.txt').write_text(str(run_dir) + '\n')
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    print('selected={}'.format(run_dir))


if __name__ == '__main__':
    main()
