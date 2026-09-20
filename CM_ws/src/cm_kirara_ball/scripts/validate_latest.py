#!/usr/bin/env python3
"""Validate the latest timestamped Catch/Move controller inputs."""

import json
from pathlib import Path

import numpy as np


PACKAGE = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_ball')


def load_one(run_dir, pattern):
    matches = list(run_dir.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError('{} matched {} files'.format(pattern, len(matches)))
    values = np.loadtxt(str(matches[0]))
    if values.ndim != 2 or values.shape[1] != 6:
        raise RuntimeError('{} has shape {}, expected N x 6'.format(
            matches[0], values.shape))
    if not np.isfinite(values).all():
        raise RuntimeError('{} contains NaN or Inf'.format(matches[0]))
    return values


def main():
    run_dir = Path((PACKAGE / 'generated' / 'latest_run.txt').read_text().strip())
    if not run_dir.is_dir():
        raise RuntimeError('latest run does not exist: {}'.format(run_dir))
    stamp = run_dir.name.replace('run_', '', 1)
    manifest_path = run_dir / 'trajectory_manifest_{}.json'.format(stamp)
    manifest = json.loads(manifest_path.read_text())

    catch_left = load_one(run_dir, 'catch_left_samples_*.txt')
    catch_right = load_one(run_dir, 'catch_right_samples_*.txt')
    move_left = load_one(run_dir, 'move_left_samples_*.txt')
    move_right = load_one(run_dir, 'move_right_samples_*.txt')
    if catch_left.shape[0] != catch_right.shape[0]:
        raise RuntimeError('Catch left/right sample counts differ')
    if move_left.shape[0] != move_right.shape[0]:
        raise RuntimeError('Move left/right sample counts differ')

    gap = float(np.max(np.abs(np.concatenate([
        catch_left[-1] - move_left[0],
        catch_right[-1] - move_right[0],
    ]))))
    estimate_tracking = (
        manifest.get('reference_mode') ==
        '0505_estimate_tracking_no_replanning' and
        manifest.get('move_initial_state_source') ==
        'catch_terminal_estimate')
    allowed_gap = (float(manifest.get('maximum_tracking_bridge_rad', 0.0))
                   if estimate_tracking else 1.0e-9)
    if gap > allowed_gap:
        raise RuntimeError(
            'Catch-to-Move tracking bridge {} exceeds {}'.format(
                gap, allowed_gap))

    result = {
        'status': 'valid',
        'run_dir': str(run_dir),
        'catch_shape': list(catch_left.shape),
        'move_shape': list(move_left.shape),
        'finite': True,
        'reference_mode': manifest.get('reference_mode', 'planned_continuous'),
        'move_initial_state_source': manifest.get(
            'move_initial_state_source', 'move_first_sample'),
        'catch_to_move_joint_gap_rad': gap,
        'allowed_tracking_bridge_rad': allowed_gap,
        'manifest_gap_rad': manifest['catch_to_move_joint_gap_rad'],
    }
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
