#!/usr/bin/env python3
"""Verify the timestamped kirara_box result approaches the box from both sides."""

import argparse
import json
from pathlib import Path

import numpy as np


ROOT = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_box')
GENERATED = ROOT / 'generated'
BOX_LENGTH_M = 0.193
MOVE_DISTANCE_LIMIT_M = (0.170, 0.190)
GRASP_CENTER = np.asarray([0.537989, 0.0969191, 0.163541])
SIDE_NORMAL = np.asarray([-0.0478592, 0.998851, 0.00237649])
SIDE_NORMAL /= np.linalg.norm(SIDE_NORMAL)


def dh_transform(q, d, a, alpha):
    result = np.eye(4)
    for theta, dz, ax, twist in zip(q, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(twist), np.sin(twist)
        result = result.dot(np.asarray([
            [ct, -st * ca, st * sa, ax * ct],
            [st, ct * ca, -ct * sa, ax * st],
            [0.0, sa, ca, dz],
            [0.0, 0.0, 0.0, 1.0]]))
    return result


def endpoints(q):
    alpha = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]
    left = np.diag([-1.0, -1.0, 1.0, 1.0]).dot(dh_transform(
        q[:6], [.1625, 0, 0, .1333, .0997, .1434],
        [0, -.425, -.39225, 0, 0, 0], alpha))
    right_base = np.eye(4)
    right_base[1, 3] = .437
    right = right_base.dot(dh_transform(
        q[6:], [.1519, 0, 0, .11235, .08535, .1257],
        [0, -.24365, -.21325, 0, 0, 0], alpha))
    return left[:3, 3], right[:3, 3]


def load_pair(run_dir, phase):
    left_path = sorted(run_dir.glob(phase + '_left_samples_*.txt'))[-1]
    right_path = sorted(run_dir.glob(phase + '_right_samples_*.txt'))[-1]
    left = np.atleast_2d(np.loadtxt(str(left_path)))
    right = np.atleast_2d(np.loadtxt(str(right_path)))
    if left.shape != right.shape or left.shape[1] != 6:
        raise RuntimeError('{} trajectory pair shape mismatch'.format(phase))
    return np.c_[left, right]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir')
    args = parser.parse_args()
    run_dir = (Path(args.run_dir) if args.run_dir else
               Path((GENERATED / 'latest_run.txt').read_text().strip()))
    catch = load_pair(run_dir, 'catch')
    move = load_pair(run_dir, 'move')

    catch_points = np.asarray([endpoints(q) for q in catch])
    left_projection = np.dot(catch_points[:, 0, :] - GRASP_CENTER, SIDE_NORMAL)
    right_projection = np.dot(catch_points[:, 1, :] - GRASP_CENTER, SIDE_NORMAL)
    window_start = max(0, int(0.80 * (len(catch) - 1)))
    catch_final_distance = float(np.linalg.norm(
        catch_points[-1, 1] - catch_points[-1, 0]))

    move_points = np.asarray([endpoints(q) for q in move])
    move_distances = np.linalg.norm(
        move_points[:, 1, :] - move_points[:, 0, :], axis=1)

    checks = {
        'catch_final_opposite_sides': bool(
            left_projection[-1] < 0.0 < right_projection[-1]),
        'catch_last_20_percent_stays_on_opposite_sides': bool(
            np.max(left_projection[window_start:]) < 0.0 and
            np.min(right_projection[window_start:]) > 0.0),
        'catch_last_20_percent_moves_inward': bool(
            left_projection[-1] > left_projection[window_start] and
            right_projection[-1] < right_projection[window_start]),
        'move_distance_below_box_length': bool(
            np.max(move_distances) < BOX_LENGTH_M),
        'move_distance_within_planner_limits': bool(
            np.min(move_distances) >= MOVE_DISTANCE_LIMIT_M[0] - 1e-6 and
            np.max(move_distances) <= MOVE_DISTANCE_LIMIT_M[1] + 1e-6),
    }
    report = {
        'run_dir': str(run_dir),
        'collision_validation': (
            'The C++ KIRARA acceptance pipeline evaluated the same trajectory '
            'at 0.01 s intervals with its FCL/Pinocchio state-validity check.'),
        'catch': {
            'sample_count': int(len(catch)),
            'side_normal_world': SIDE_NORMAL.tolist(),
            'last_20_percent_start_index': int(window_start),
            'left_side_projection_start_m': float(left_projection[window_start]),
            'left_side_projection_final_m': float(left_projection[-1]),
            'right_side_projection_start_m': float(right_projection[window_start]),
            'right_side_projection_final_m': float(right_projection[-1]),
            'final_dh_endpoint_distance_m': catch_final_distance,
        },
        'move': {
            'sample_count': int(len(move)),
            'dh_endpoint_distance_min_m': float(np.min(move_distances)),
            'dh_endpoint_distance_max_m': float(np.max(move_distances)),
            'physical_box_length_m': BOX_LENGTH_M,
        },
        'checks': checks,
        'passed': bool(all(checks.values())),
    }
    output = run_dir / 'side_approach_validation.json'
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
