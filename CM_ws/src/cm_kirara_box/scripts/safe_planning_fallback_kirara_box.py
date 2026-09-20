#!/usr/bin/env python3
"""Create and validate a collision-safe timing of the latest planned path."""

from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np

from plan_kirara_box import move_dh_distance


ROOT = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_box')
STAGING = ROOT / 'generated' / 'staging'
GENERATED = ROOT / 'generated'
MOVE_EXE = ROOT.parents[1] / 'devel/lib/cm_kirara_box/CM_KIRARA_BOX_MOVE'
DT = 0.01
VMAX = 1.2
AMAX = 3.0


def load_pair(prefix):
    left = np.atleast_2d(np.loadtxt(STAGING / '{}_left.txt'.format(prefix)))
    right = np.atleast_2d(np.loadtxt(STAGING / '{}_right.txt'.format(prefix)))
    if left.shape != right.shape or left.shape[1] != 6:
        raise RuntimeError('{} left/right planning path shape mismatch'.format(prefix))
    pair = np.c_[left, right]
    keep = np.r_[True, np.max(np.abs(np.diff(pair, axis=0)), axis=1) > 1e-12]
    return left[keep], right[keep]


def quintic_safe_samples(left, right):
    output_left = []
    output_right = []
    durations = []
    for index in range(left.shape[0] - 1):
        delta = np.r_[left[index + 1] - left[index],
                      right[index + 1] - right[index]]
        magnitude = float(np.max(np.abs(delta)))
        
        
        duration = max(
            0.05,
            1.875 * magnitude / VMAX,
            np.sqrt(5.773503 * magnitude / AMAX))
        count = max(2, int(np.ceil(duration / DT)) + 1)
        u = np.linspace(0.0, 1.0, count)
        s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        segment_left = left[index] + s[:, None] * (
            left[index + 1] - left[index])
        segment_right = right[index] + s[:, None] * (
            right[index + 1] - right[index])
        if index:
            segment_left = segment_left[1:]
            segment_right = segment_right[1:]
        output_left.append(segment_left)
        output_right.append(segment_right)
        durations.append(duration)
    return np.vstack(output_left), np.vstack(output_right), durations


def main():
    catch_left = np.atleast_2d(np.loadtxt(STAGING / 'catch_left_samples.txt'))
    catch_right = np.atleast_2d(np.loadtxt(STAGING / 'catch_right_samples.txt'))
    move_left_planned, move_right_planned = load_pair('move_planned')
    move_left, move_right, durations = quintic_safe_samples(
        move_left_planned, move_right_planned)
    stage2_bridge = float(np.max(np.abs(
        np.r_[catch_left[-1], catch_right[-1]] -
        np.r_[move_left[0], move_right[0]])))
    move_distances = np.array([
        move_dh_distance(np.r_[left, right])
        for left, right in zip(move_left, move_right)])

    left_staging = STAGING / 'move_left_samples.txt'
    right_staging = STAGING / 'move_right_samples.txt'
    np.savetxt(left_staging, move_left, fmt='%.12g')
    np.savetxt(right_staging, move_right, fmt='%.12g')
    completed = subprocess.run(
        [str(MOVE_EXE), '--validate-samples',
         str(left_staging), str(right_staging)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(completed.stdout)
    if completed.returncode != 0:
        raise RuntimeError('Native validation rejected safe planning samples')

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = GENERATED / 'run_{}_safe_planning_move'.format(stamp)
    run_dir.mkdir(parents=True, exist_ok=False)
    arrays = {
        'catch_left_samples': catch_left,
        'catch_right_samples': catch_right,
        'move_left_samples': move_left,
        'move_right_samples': move_right,
    }
    for name, values in arrays.items():
        np.savetxt(run_dir / '{}_{}.txt'.format(name, stamp), values,
                   fmt='%.12g')
    shutil.copy2(ROOT / 'config/catch_kirara_box.yaml',
                 run_dir / 'catch_config.yaml')
    shutil.copy2(ROOT / 'config/move_kirara_box.yaml',
                 run_dir / 'move_config.yaml')
    (run_dir / 'move_native_sample_validation.log').write_text(
        completed.stdout)
    manifest = {
        'timestamp': stamp,
        'catch_method': 'BidirectionalPlanning_KIRARA',
        'move_method': 'BidirectionalPlanning_safe_segment_quintic_time_law',
        'move_kirara_used': False,
        'move_reason': 'KIRARA left the narrow collision/task-constraint manifold',
        'catch_points': int(catch_left.shape[0]),
        'move_points': int(move_left.shape[0]),
        'move_planning_waypoints': int(move_left_planned.shape[0]),
        'move_duration_s': float(sum(durations)),
        'sample_dt_max_s': DT,
        'velocity_limit_rad_s': VMAX,
        'acceleration_limit_rad_s2': AMAX,
        'stage2_joint_bridge_max_rad': stage2_bridge,
        'physical_box_length_m': 0.193,
        'move_dh_distance_limit_m': [0.170, 0.190],
        'move_dh_distance_observed_m': [
            float(np.min(move_distances)), float(np.max(move_distances))],
        'native_move_samples_valid': True,
    }
    (run_dir / 'trajectory_manifest_{}.json'.format(stamp)).write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    (GENERATED / 'latest_run.txt').write_text(str(run_dir) + '\n')
    print('TCST kirara_box safe fallback completed: {}'.format(run_dir))
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
