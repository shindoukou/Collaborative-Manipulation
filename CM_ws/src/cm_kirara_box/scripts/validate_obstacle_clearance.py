#!/usr/bin/env python3
"""Report minimum FCL clearance from generated arms/carried box to obstacles."""

import argparse
import json
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml
from cm_box_attachment import BoxAttachment


PACKAGE = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_box')
OBSTACLE_PREFIXES = (
    'obs_box_0', 'obs_box_son_0', 'obs_box_platform_0',
    'obs_box_left_0', 'obs_box_right_0', 'obs_box_middle_0',
    'obs_box_middle_in_0', 'other_box_1_0')


def dh_matrix(theta, d, alpha, a):
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0]])


def chain_transform(q, d, alpha, a):
    transform = np.eye(4)
    for values in zip(q, d, alpha, a):
        transform = transform.dot(dh_matrix(*values))
    return transform


def move_configuration(joints, model, data, attachment):
    q = pin.neutral(model)
    q[:12] = joints
    if model.nq >= 19:
        attachment.apply(model, data, q)
    return q


def validate_phase(run_dir, phase, config_path=None):
    config_path = config_path or PACKAGE / 'config' / ('{}_kirara_box.yaml'.format(phase))
    config = yaml.safe_load(Path(config_path).read_text())
    urdf = Path(config['file_path']['pinocchio_file'])
    model = pin.buildModelFromUrdf(str(urdf))
    geometry = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION)
    geometry.addAllCollisionPairs()
    data = model.createData()
    attachment = BoxAttachment(config, model) if phase == 'move' else None
    geometry_data = pin.GeometryData(geometry)
    left_candidates = list(run_dir.glob(
        '{0}/{0}_left_smoothed_path.txt'.format(phase)))
    right_candidates = list(run_dir.glob(
        '{0}/{0}_right_smoothed_path.txt'.format(phase)))
    if not left_candidates:
        left_candidates = list(run_dir.glob(
            '{}_left_samples_*.txt'.format(phase)))
        right_candidates = list(run_dir.glob(
            '{}_right_samples_*.txt'.format(phase)))
    left = np.loadtxt(str(left_candidates[0]))
    right = np.loadtxt(str(right_candidates[0]))
    rows = np.hstack((np.atleast_2d(left), np.atleast_2d(right)))

    selected_pairs = []
    for index, pair in enumerate(geometry.collisionPairs):
        first = geometry.geometryObjects[pair.first].name
        second = geometry.geometryObjects[pair.second].name
        if {first, second} == {'target_box_0', 'other_box_1_0'}:
            continue
        first_obstacle = first.startswith(OBSTACLE_PREFIXES)
        second_obstacle = second.startswith(OBSTACLE_PREFIXES)
        other = second if first_obstacle else first
        if first_obstacle == second_obstacle:
            continue
        if (other.startswith('left_') or other.startswith('right_') or
                (phase == 'move' and other == 'target_box_0')):
            selected_pairs.append((index, first, second))

    minimum = np.inf
    minimum_pair = None
    minimum_sample = None
    collision_count = 0
    for sample, joints in enumerate(rows):
        q = (move_configuration(
            joints, model, data, attachment) if phase == 'move'
             else np.asarray(joints))
        pin.forwardKinematics(model, data, q)
        pin.updateGeometryPlacements(model, data, geometry, geometry_data, q)
        for index, first, second in selected_pairs:
            distance = pin.computeDistance(
                geometry, geometry_data, index).min_distance
            if distance < minimum:
                minimum = float(distance)
                minimum_pair = [first, second]
                minimum_sample = sample
            if distance <= 0.0:
                collision_count += 1
    return {
        'phase': phase,
        'sample_count': int(rows.shape[0]),
        'minimum_clearance_m': minimum,
        'minimum_pair': minimum_pair,
        'minimum_sample': minimum_sample,
        'collision_results_at_or_below_zero': collision_count,
        'valid': collision_count == 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--phase', choices=('catch', 'move'), default='move')
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    result = {'run_dir': str(run_dir), args.phase: validate_phase(run_dir, args.phase, args.config)}
    (run_dir / '{}_obstacle_clearance_report.json'.format(args.phase)).write_text(
        json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if not result[args.phase]['valid']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
