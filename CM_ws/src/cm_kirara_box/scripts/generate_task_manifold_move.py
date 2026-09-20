#!/usr/bin/env python3
"""Generate a continuous dual-arm Move seed on the box task manifold."""

import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

import plan_kirara_box as planner


ALPHA = [np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0]
LEFT_D = [.1625, 0, 0, .1333, .0997, .1434]
LEFT_A = [0, -.425, -.39225, 0, 0, 0]
RIGHT_D = [.1519, 0, 0, .11235, .08535, .1257]
RIGHT_A = [0, -.24365, -.21325, 0, 0, 0]
LEFT_BASE = np.diag([-1.0, -1.0, 1.0, 1.0])
RIGHT_BASE = np.eye(4)
RIGHT_BASE[1, 3] = .437


def fk(q, side):
    if side == 'left':
        return LEFT_BASE.dot(planner.dh_transform(q, LEFT_D, LEFT_A, ALPHA))
    return RIGHT_BASE.dot(planner.dh_transform(q, RIGHT_D, RIGHT_A, ALPHA))


def pose_residual(q, side, target_position, target_z, reference):
    transform = fk(q, side)
    position_error = transform[:3, 3] - target_position
    
    
    
    rotation_error = transform[:3, 2] - target_z
    continuity = q - reference
    return np.r_[500.0 * position_error, 30.0 * rotation_error,
                 0.01 * continuity]


def solve_pose(seed, side, position, target_z, q_start):
    lower = np.array([-2*np.pi]*3 + [-np.pi]*3, dtype=float)
    upper = np.array([2*np.pi]*3 + [np.pi]*3, dtype=float)
    lower[3:] = np.maximum(lower[3:], q_start[3:] - 1.20)
    upper[3:] = np.minimum(upper[3:], q_start[3:] + 1.20)
    
    
    lower = np.maximum(lower, seed - 0.18)
    upper = np.minimum(upper, seed + 0.18)
    result = least_squares(
        pose_residual, np.clip(seed, lower + 1e-9, upper - 1e-9),
        bounds=(lower, upper),
        args=(side, position, target_z, seed),
        xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=300)
    transform = fk(result.x, side)
    pos_error = np.linalg.norm(transform[:3, 3] - position)
    rot_error = np.linalg.norm(transform[:3, 2] - target_z)
    
    
    if pos_error > 3.0e-2 or rot_error > 8.0e-1:
        raise RuntimeError('{} IK failed: success={}, pos={}, rot={}'.format(
            side, result.success, pos_error, rot_error))
    return result.x


def joint_vector(config, section):
    return np.array([config[section]['joint_{}'.format(i)]
                     for i in range(1, 13)], dtype=float)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', type=int, default=121)
    parser.add_argument('--lift-start', type=float, default=.27)
    parser.add_argument('--lift-goal', type=float, default=.10)
    parser.add_argument('--output-dir', default='/tmp/kirara_box_task_manifold')
    parser.add_argument('--reference-run', default='')
    parser.add_argument('--reference-blend', type=float, default=1.0)
    parser.add_argument('--extra-z-bump', type=float, default=0.0)
    parser.add_argument('--extra-x-bump', type=float, default=0.0)
    parser.add_argument('--extra-y-bump', type=float, default=0.0)
    args = parser.parse_args()

    config = planner.load_yaml(planner.CONFIG_MOVE)
    q_start = joint_vector(config, 'Joint_init')
    q_goal = joint_vector(config, 'Joint_final')
    start_left, start_right = fk(q_start[:6], 'left'), fk(q_start[6:], 'right')
    goal_left, goal_right = fk(q_goal[:6], 'left'), fk(q_goal[6:], 'right')
    center_start = .5 * (start_left[:3, 3] + start_right[:3, 3])
    center_goal = .5 * (goal_left[:3, 3] + goal_right[:3, 3])
    relative_start = start_right[:3, 3] - start_left[:3, 3]
    relative_goal = goal_right[:3, 3] - goal_left[:3, 3]

    times = np.linspace(0.0, 1.0, args.samples)
    q_left = np.empty((args.samples, 6))
    q_right = np.empty((args.samples, 6))
    q_left[0], q_right[0] = q_start[:6], q_start[6:]
    control_1 = center_start + np.array([0.0, 0.0, args.lift_start])
    control_2 = center_goal + np.array([0.0, 0.0, args.lift_goal])
    reference_centers = None
    if args.reference_run:
        reference_run = Path(args.reference_run)
        old_left = np.atleast_2d(np.loadtxt(str(sorted(
            reference_run.glob('move_left_samples_*.txt'))[-1])))
        old_right = np.atleast_2d(np.loadtxt(str(sorted(
            reference_run.glob('move_right_samples_*.txt'))[-1])))
        old_centers = np.array([
            .5 * (fk(a, 'left')[:3, 3] + fk(b, 'right')[:3, 3])
            for a, b in zip(old_left, old_right)])
        source_t = np.linspace(0.0, 1.0, old_centers.shape[0])
        reference_centers = np.column_stack([
            np.interp(times, source_t, old_centers[:, axis])
            for axis in range(3)])
        
        reference_centers[0] = center_start
        reference_centers[-1] = center_goal
    for index, t in enumerate(times[1:-1], 1):
        bezier_center = ((1-t)**3 * center_start +
                         3*(1-t)**2*t * control_1 +
                         3*(1-t)*t**2 * control_2 + t**3 * center_goal)
        if reference_centers is None:
            center = bezier_center
        else:
            center = (args.reference_blend * reference_centers[index] +
                      (1.0 - args.reference_blend) * bezier_center)
        center = center.copy()
        center[0] += args.extra_x_bump * np.sin(np.pi * t) ** 2
        center[1] += args.extra_y_bump * np.sin(np.pi * t) ** 2
        center[2] += args.extra_z_bump * np.sin(np.pi * t) ** 2
        relative = (1-t)*relative_start + t*relative_goal
        interior = min(1.0, t / 0.03, (1.0 - t) / 0.03)
        
        
        
        relative[0] *= (1.0 - 0.5 * interior)
        relative[2] *= (1.0 - 0.5 * interior)
        relative[1] -= 0.002 * interior
        relative_direction = relative / np.linalg.norm(relative)
        q_left[index] = solve_pose(
            q_left[index-1], 'left', center - .5*relative,
            relative_direction, q_start[:6])
        q_right[index] = solve_pose(
            q_right[index-1], 'right', center + .5*relative,
            -relative_direction, q_start[6:])
    q_left[-1], q_right[-1] = q_goal[:6], q_goal[6:]

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(str(output / 'move_left.txt'), q_left, fmt='%.12g')
    np.savetxt(str(output / 'move_right.txt'), q_right, fmt='%.12g')
    vectors = np.array([planner.move_dh_vector(np.r_[a, b])
                        for a, b in zip(q_left, q_right)])
    distances = np.linalg.norm(vectors, axis=1)
    transverse = np.hypot(vectors[:, 0], vectors[:, 2])
    print('output={}'.format(output))
    print('distance=[{:.6f},{:.6f}] transverse_max={:.6f} '
          'abs_z_max={:.6f} y_cos_min={:.6f}'.format(
              distances.min(), distances.max(), transverse.max(),
              np.abs(vectors[:, 2]).max(),
              np.min(vectors[:, 1] / distances)))


if __name__ == '__main__':
    main()
