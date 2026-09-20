#!/usr/bin/env python3
"""Report minimum FCL clearance from generated arms/carried ball to boxes."""

import json
from pathlib import Path

import numpy as np
import pinocchio as pin


PACKAGE = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_ball')
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


def move_configuration(joints, nq):
    q = np.zeros(nq)
    q[:12] = joints
    d_left = [0.1625, 0, 0, 0.1333, 0.0997, 0.1434]
    a_left = [0, -0.425, -0.39225, 0, 0, 0]
    d_right = [0.1519, 0, 0, 0.11235, 0.08535, 0.1257]
    a_right = [0, -0.24365, -0.21325, 0, 0, 0]
    alpha = [0.5 * np.pi, 0, 0, 0.5 * np.pi, -0.5 * np.pi, 0]
    left_base = np.diag([-1.0, -1.0, 1.0, 1.0])
    right_base = np.eye(4)
    right_base[:3, 3] = [0.0, 0.437, 0.0]
    left = left_base.dot(chain_transform(joints[:6], d_left, alpha, a_left))
    right = right_base.dot(chain_transform(joints[6:], d_right, alpha, a_right))
    if nq >= 19:
        q[12:15] = 0.5 * (left[:3, 3] + right[:3, 3])
        q[15:19] = [0.0, 0.0, 0.0, 1.0]
    return q


def validate_phase(run_dir, phase):
    urdf = PACKAGE / 'models' / 'urdf' / ('{}_newbase.urdf'.format(phase))
    model = pin.buildModelFromUrdf(str(urdf))
    geometry = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION)
    geometry.addAllCollisionPairs()
    data = model.createData()
    geometry_data = pin.GeometryData(geometry)
    left = np.loadtxt(str(next(run_dir.glob('{}_left_samples_*.txt'.format(phase)))))
    right = np.loadtxt(str(next(run_dir.glob('{}_right_samples_*.txt'.format(phase)))))
    rows = np.hstack((np.atleast_2d(left), np.atleast_2d(right)))

    selected_pairs = []
    for index, pair in enumerate(geometry.collisionPairs):
        first = geometry.geometryObjects[pair.first].name
        second = geometry.geometryObjects[pair.second].name
        first_obstacle = first.startswith(OBSTACLE_PREFIXES)
        second_obstacle = second.startswith(OBSTACLE_PREFIXES)
        other = second if first_obstacle else first
        if first_obstacle == second_obstacle:
            continue
        if (other.startswith('left_') or other.startswith('right_') or
                (phase == 'move' and other == 'target_ball_0')):
            selected_pairs.append((index, first, second))

    minimum = np.inf
    minimum_pair = None
    minimum_sample = None
    collision_count = 0
    for sample, joints in enumerate(rows):
        q = (move_configuration(joints, model.nq) if phase == 'move'
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
    run_dir = Path((PACKAGE / 'generated' / 'latest_run.txt').read_text().strip())
    result = {
        'run_dir': str(run_dir),
        'closed_table_geometry': {
            'gazebo_center_world_m': [0.62986639, 0.167242173, 0.2723345],
            'gazebo_size_m': [0.625, 0.994, 0.53],
            'planning_center_robot_world_m': [
                0.62986639, 0.167242173, -0.2276655],
            'planning_size_with_5mm_shell_m': [0.635, 1.004, 0.54],
            'joined_inner_bounds_xy_m': {
                'x': [0.31736639, 0.94236639],
                'y': [-0.329757827, 0.664242173],
            },
        },
        'trimmed_main_obs_box': {
            'gazebo_center_robot_world_m': [
                0.34236639, 0.034242173, 0.1503430295],
            'gazebo_size_m': [0.05, 0.48, 0.222017059],
            'gap_above_table_m': 0.002,
            'top_height_unchanged': True,
            'planning_size_with_5mm_shell_m': [
                0.06, 0.49, 0.232017059],
        },
        'catch': validate_phase(run_dir, 'catch'),
        'move': validate_phase(run_dir, 'move'),
    }
    (run_dir / 'obstacle_clearance_report.json').write_text(
        json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if not result['catch']['valid'] or not result['move']['valid']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
