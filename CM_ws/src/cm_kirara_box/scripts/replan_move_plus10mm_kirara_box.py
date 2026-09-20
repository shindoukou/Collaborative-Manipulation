#!/usr/bin/env python3
"""Replan only kirara_box Move with 10 mm more EE separation.

Catch and the exact Stage2 terminal (the first Move sample) are preserved.
The existing Move center route and relative direction are reused.  Separation
ramps smoothly from the unchanged handover to +10 mm near the beginning.
The Move center route, endpoint center, height profile, costs, and Catch files
are preserved.
"""

from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

import plan_kirara_box as planner
import generate_task_manifold_move as manifold


EXTRA_DISTANCE_M = 0.010
RAMP_FRACTION = 0.08
COARSE_COUNT = 2000
MAX_EXECUTION_JOINT_STEP_RAD = 0.006


def latest_one(directory, pattern):
    matches = sorted(directory.glob(pattern))
    if not matches:
        raise RuntimeError('missing {} in {}'.format(pattern, directory))
    return matches[-1]


def q_section(config, section):
    return np.asarray([config[section]['joint_{}'.format(index)]
                       for index in range(1, 13)], dtype=float)


def set_q_section(config, section, q):
    config[section] = {
        'joint_{}'.format(index + 1): float(value)
        for index, value in enumerate(q)}


def smoothstep(value):
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def rotation_with_tool_z(reference_rotation, desired_z):
    """Keep the old roll branch with a small tool-axis alignment correction."""
    desired_z = np.asarray(desired_z, dtype=float)
    desired_z /= np.linalg.norm(desired_z)
    z_axis = 0.945 * reference_rotation[:, 2] + 0.055 * desired_z
    z_axis /= np.linalg.norm(z_axis)
    x_axis = reference_rotation[:, 0].copy()
    x_axis -= z_axis * np.dot(x_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def densify_joint_path(path, max_joint_step):
    """Insert only the samples needed to remove local IK branch jumps.

    Every inserted row is a convex joint-space interpolation between two
    consecutive, natively valid Move rows.  The exact first/last rows and the
    geometric route are retained; the densified result is validated again by
    the original C++ collision/task checker before it can be selected.
    """
    dense = [path[0].copy()]
    inserted = 0
    for start, finish in zip(path[:-1], path[1:]):
        subdivisions = max(
            1, int(np.ceil(
                np.max(np.abs(finish - start)) / max_joint_step)))
        for subdivision in range(1, subdivisions + 1):
            alpha = float(subdivision) / float(subdivisions)
            dense.append(start + alpha * (finish - start))
        inserted += subdivisions - 1
    return np.asarray(dense), inserted


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def locked_pose_residual(q, side, position, rotation, reference):
    transform = manifold.fk(q, side)
    position_error = transform[:3, 3] - position
    rotation_error = Rotation.from_matrix(
        rotation.T.dot(transform[:3, :3])).as_rotvec()
    
    
    return np.r_[4000.0 * position_error,
                 800.0 * rotation_error,
                 0.2 * (q - reference)]


def solve_locked_pose(reference, previous_delta, side, position, rotation):
    seed = reference + previous_delta
    
    
    lower = np.maximum(reference - 0.15, seed - 0.05)
    upper = np.minimum(reference + 0.15, seed + 0.05)
    result = least_squares(
        locked_pose_residual, np.clip(seed, lower + 1e-10, upper - 1e-10),
        bounds=(lower, upper),
        args=(side, position, rotation, seed),
        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=400)
    transform = manifold.fk(result.x, side)
    position_error = float(np.linalg.norm(transform[:3, 3] - position))
    rotation_error = float(np.linalg.norm(Rotation.from_matrix(
        rotation.T.dot(transform[:3, :3])).as_rotvec()))
    if position_error > 3.0e-3 or rotation_error > 2.0e-2:
        raise RuntimeError('{} locked IK failed: position={}, rotation={}'.format(
            side, position_error, rotation_error))
    return result.x


def main():
    latest_file = planner.GENERATED / 'latest_run.txt'
    reference = Path(latest_file.read_text().strip()).resolve()
    if not reference.is_dir():
        raise RuntimeError('selected reference run does not exist: {}'.format(
            reference))
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = planner.GENERATED / (
        'run_{}_move_plus10mm_stage2fixed'.format(stamp))
    run_dir.mkdir(parents=True, exist_ok=False)

    old_left_path = latest_one(reference, 'move_left_samples_*.txt')
    old_right_path = latest_one(reference, 'move_right_samples_*.txt')
    old_left = np.atleast_2d(np.loadtxt(str(old_left_path)))
    old_right = np.atleast_2d(np.loadtxt(str(old_right_path)))
    if old_left.shape != old_right.shape or old_left.shape[1] != 6:
        raise RuntimeError('invalid reference Move pair')
    old_q = np.c_[old_left, old_right]
    old_t = np.linspace(0.0, 1.0, old_q.shape[0])
    coarse_indices = np.unique(np.rint(np.linspace(
        0, old_q.shape[0] - 1, COARSE_COUNT)).astype(int))
    coarse_t = old_t[coarse_indices]
    coarse_old = old_q[coarse_indices]
    coarse_new = np.empty_like(coarse_old)
    coarse_new[0] = coarse_old[0]  

    base_config_path = reference / 'move_config.yaml'
    if not base_config_path.is_file():
        base_config_path = planner.CONFIG_MOVE
    config = yaml.safe_load(base_config_path.read_text())
    q_stage2 = q_section(config, 'Joint_init')
    start_gap = float(np.max(np.abs(coarse_new[0] - q_stage2)))
    if start_gap > 1.0e-5:
        raise RuntimeError('reference Move start is not Stage2 terminal: {}'.format(
            start_gap))

    for index in range(1, coarse_old.shape[0]):
        q_ref = coarse_old[index]
        left_tf = manifold.fk(q_ref[:6], 'left')
        right_tf = manifold.fk(q_ref[6:], 'right')
        center = 0.5 * (left_tf[:3, 3] + right_tf[:3, 3])
        vector = right_tf[:3, 3] - left_tf[:3, 3]
        distance = float(np.linalg.norm(vector))
        ramp = smoothstep(coarse_t[index] / RAMP_FRACTION)
        
        
        
        target_vector = vector.copy()
        target_vector[1] += (np.sign(vector[1]) *
                             EXTRA_DISTANCE_M * ramp)
        
        
        target_vector[2] *= (1.0 - 0.10 * ramp)
        target_direction = target_vector / np.linalg.norm(target_vector)
        left_position = center - 0.5 * target_vector
        right_position = center + 0.5 * target_vector
        left_rotation = rotation_with_tool_z(
            left_tf[:3, :3], target_direction)
        right_rotation = rotation_with_tool_z(
            right_tf[:3, :3], -target_direction)
        left_previous_delta = (coarse_new[index - 1, :6] -
                               coarse_old[index - 1, :6])
        right_previous_delta = (coarse_new[index - 1, 6:] -
                                coarse_old[index - 1, 6:])
        coarse_new[index, :6] = solve_locked_pose(
            q_ref[:6], left_previous_delta, 'left', left_position,
            left_rotation)
        coarse_new[index, 6:] = solve_locked_pose(
            q_ref[6:], right_previous_delta, 'right', right_position,
            right_rotation)

    
    new_q = PchipInterpolator(coarse_t, coarse_new, axis=0)(old_t)
    
    
    correction = new_q - old_q
    ramp_weights = np.asarray([
        smoothstep(value / RAMP_FRACTION) for value in old_t])
    new_q = old_q + correction * ramp_weights[:, None]
    new_q[0] = old_q[0]
    
    
    new_q[-1] = coarse_new[-1]

    old_vectors = np.asarray([planner.move_dh_vector(row) for row in old_q])
    new_vectors = np.asarray([planner.move_dh_vector(row) for row in new_q])
    old_distances = np.linalg.norm(old_vectors, axis=1)
    new_distances = np.linalg.norm(new_vectors, axis=1)
    old_centers = np.asarray([
        0.5 * (manifold.fk(row[:6], 'left')[:3, 3] +
               manifold.fk(row[6:], 'right')[:3, 3]) for row in old_q])
    new_centers = np.asarray([
        0.5 * (manifold.fk(row[:6], 'left')[:3, 3] +
               manifold.fk(row[6:], 'right')[:3, 3]) for row in new_q])
    center_error = np.linalg.norm(new_centers - old_centers, axis=1)
    direction_cos = np.sum(old_vectors * new_vectors, axis=1) / (
        old_distances * new_distances)
    direction_error_deg = np.degrees(np.arccos(np.clip(
        direction_cos, -1.0, 1.0)))
    separation_increase = new_distances - old_distances
    endpoint_center_error = float(np.linalg.norm(
        new_centers[-1] - old_centers[-1]))
    max_height_change = float(np.max(np.abs(
        new_centers[:, 2] - old_centers[:, 2])))
    if endpoint_center_error > 1.0e-3:
        raise RuntimeError(
            'Move endpoint center changed by {:.9f} m'.format(
                endpoint_center_error))
    if max_height_change > 1.0e-3:
        raise RuntimeError(
            'Move height profile changed by {:.9f} m'.format(
                max_height_change))

    
    
    
    
    execution_q, inserted_samples = densify_joint_path(
        new_q, MAX_EXECUTION_JOINT_STEP_RAD)
    if execution_q.shape[0] > 3 * new_q.shape[0]:
        raise RuntimeError(
            'local densification unexpectedly exceeded 3x point count')
    execution_vectors = np.asarray([
        planner.move_dh_vector(row) for row in execution_q])
    execution_distances = np.linalg.norm(execution_vectors, axis=1)
    execution_centers = np.asarray([
        0.5 * (manifold.fk(row[:6], 'left')[:3, 3] +
               manifold.fk(row[6:], 'right')[:3, 3])
        for row in execution_q])
    if np.max(execution_centers[:, 2]) > np.max(old_centers[:, 2]) + 1.0e-3:
        raise RuntimeError(
            'densified Move rises above old route by more than 1 mm')
    if np.linalg.norm(execution_centers[-1] - old_centers[-1]) > 1.0e-3:
        raise RuntimeError('densified Move endpoint center changed')

    left_path = run_dir / 'move_left_samples_{}.txt'.format(stamp)
    right_path = run_dir / 'move_right_samples_{}.txt'.format(stamp)
    np.savetxt(str(left_path), execution_q[:, :6], fmt='%.12g')
    np.savetxt(str(right_path), execution_q[:, 6:], fmt='%.12g')

    
    config['move_grasp_distance_dh']['max_m'] = float(
        execution_distances.max() + 0.0005)
    config['palnning']['upperbound'] = float(
        execution_distances.max() + 0.0005)
    set_q_section(config, 'Joint_init', old_q[0])
    set_q_section(config, 'Joint_final', execution_q[-1])
    move_config = run_dir / 'move_config.yaml'
    move_config.write_text(yaml.safe_dump(config, sort_keys=False))

    environment = os.environ.copy()
    environment['CM_KIRARA_BOX_MOVE_CONFIG'] = str(move_config)
    validation = subprocess.run(
        [str(planner.MOVE_EXE), '--validate-samples',
         str(left_path), str(right_path)], env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (run_dir / 'move_samples_validation.log').write_text(validation.stdout)
    if validation.returncode:
        raise RuntimeError('native validation failed; see {}'.format(
            run_dir / 'move_samples_validation.log'))

    
    catch_left_source = latest_one(reference, 'catch_left_samples_*.txt')
    catch_right_source = latest_one(reference, 'catch_right_samples_*.txt')
    catch_left_copy = run_dir / 'catch_left_samples_{}.txt'.format(stamp)
    catch_right_copy = run_dir / 'catch_right_samples_{}.txt'.format(stamp)
    shutil.copy2(str(catch_left_source), str(catch_left_copy))
    shutil.copy2(str(catch_right_source), str(catch_right_copy))
    catch_hashes = {
        'left_source': sha256(catch_left_source),
        'left_copy': sha256(catch_left_copy),
        'right_source': sha256(catch_right_source),
        'right_copy': sha256(catch_right_copy),
    }
    if (catch_hashes['left_source'] != catch_hashes['left_copy'] or
            catch_hashes['right_source'] != catch_hashes['right_copy']):
        raise RuntimeError('Catch byte-for-byte preservation check failed')
    catch_config = reference / 'catch_config_REUSED_UNCHANGED.yaml'
    if not catch_config.is_file():
        catch_config = reference / 'catch_config.yaml'
    if catch_config.is_file():
        shutil.copy2(str(catch_config),
                     str(run_dir / 'catch_config_REUSED_UNCHANGED.yaml'))

    sample_period = 0.01
    velocity = np.diff(execution_q, axis=0) / sample_period
    manifest = {
        'timestamp': stamp,
        'status': 'COMPLETE_VALIDATED',
        'method': 'MOVE_ONLY_TASK_MANIFOLD_PLUS10MM_LOCAL_DENSIFY',
        'reference_move_run': str(reference),
        'catch_replanned': False,
        'catch_reused_byte_for_byte': True,
        'catch_sha256': catch_hashes,
        'stage2_terminal_changed': False,
        'move_endpoint_center_changed': False,
        'cost_weights_changed': False,
        'nmpc_weights_changed': False,
        'move_start_gap_from_stage2_rad': start_gap,
        'requested_distance_increase_m': EXTRA_DISTANCE_M,
        'ramp_fraction': RAMP_FRACTION,
        'ramp_duration_s': float(RAMP_FRACTION *
                                 (old_q.shape[0] - 1) * sample_period),
        'move_points_before_local_densify': int(new_q.shape[0]),
        'inserted_local_execution_samples': int(inserted_samples),
        'max_execution_joint_step_rad': MAX_EXECUTION_JOINT_STEP_RAD,
        'move_points': int(execution_q.shape[0]),
        'move_duration_s': float(
            (execution_q.shape[0] - 1) * sample_period),
        'old_distance_range_m': old_distances[[np.argmin(old_distances),
                                               np.argmax(old_distances)]].tolist(),
        'new_distance_range_m': [float(execution_distances.min()),
                                 float(execution_distances.max())],
        'increase_after_ramp_m': [
            float(separation_increase[old_t >= RAMP_FRACTION].min()),
            float(separation_increase[old_t >= RAMP_FRACTION].max())],
        'max_center_route_change_m': float(center_error.max()),
        'move_endpoint_center_error_m': endpoint_center_error,
        'max_height_profile_change_m': max_height_change,
        'old_max_center_z_m': float(old_centers[:, 2].max()),
        'new_max_center_z_m': float(execution_centers[:, 2].max()),
        'max_relative_direction_change_deg': float(direction_error_deg.max()),
        'max_abs_joint_velocity_rad_s': float(np.max(np.abs(velocity))),
        'native_collision_and_task_validation': 'PASS',
    }
    if manifest['max_abs_joint_velocity_rad_s'] > 0.65:
        (run_dir / 'STATUS_REJECTED.txt').write_text(
            'REJECTED: excessive joint velocity\n')
        raise RuntimeError('joint velocity exceeds 0.65 rad/s: {}'.format(
            manifest['max_abs_joint_velocity_rad_s']))
    manifest_path = run_dir / 'trajectory_manifest_{}.json'.format(stamp)
    manifest_path.write_text(json.dumps(
        manifest, indent=2, ensure_ascii=False) + '\n')
    (run_dir / 'previous_latest_run.txt').write_text(str(reference) + '\n')
    latest_file.write_text(str(run_dir) + '\n')
    print(json.dumps(manifest, indent=2, ensure_ascii=False))
    print('selected={}'.format(run_dir))


if __name__ == '__main__':
    main()
