#!/usr/bin/env python3
"""Replan only kirara_box Move for the measured real Stage2 handover.

Catch is never executed or regenerated.  The accepted Catch samples from the
currently selected run are copied into a new timestamped run only after the
new Move trajectory passes all native checks.
"""

from datetime import datetime
import json
from pathlib import Path
import shutil
import sys

import numpy as np


ROOT = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_box')
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import plan_kirara_box as planner


FALLBACK_STAGE2_DISTANCE_M = 0.1798
MAX_START_DISTANCE_ERROR_M = 0.00035
MAX_WRIST_CHANGE_RAD = 1.05
BASE_MOVE_START_Q = np.array([
    -0.471234821277, -0.969527836086, 1.66182517944,
    -0.692301598319, -0.471234821277, 0.359920291475,
    0.000368777000258, -3.14159265359, 0.211741292068,
    -2.17735812143, -9.48676900925e-20, -0.0336969929796,
], dtype=float)




POSE_REFERENCE_MOVE_GOAL_Q = np.array([
    -0.296650300503, -1.6071750102, 1.83148646523,
    2.82468683327, 0.436580228629, -3.0017719661,
    0.461412371297, -2.12105212787, -0.315711661475,
    -0.827432720832, -0.325400566338, 0.0603607759665,
], dtype=float)


def selected_run():
    pointer = planner.GENERATED / 'latest_run.txt'
    path = Path(pointer.read_text().strip()).resolve()
    if not path.is_dir():
        raise RuntimeError('selected kirara_box run is missing: {}'.format(path))
    return path


def copy_single(pattern, source_run, destination):
    matches = sorted(source_run.glob(pattern))
    if not matches:
        raise RuntimeError('missing reused Catch file: {}'.format(pattern))
    shutil.copy2(str(matches[-1]), str(destination / matches[-1].name))


def catch_terminal(source_run):
    left = np.atleast_2d(np.loadtxt(str(sorted(
        source_run.glob('catch_left_samples_*.txt'))[-1])))
    right = np.atleast_2d(np.loadtxt(str(sorted(
        source_run.glob('catch_right_samples_*.txt'))[-1])))
    return np.r_[left[-1], right[-1]]


def measured_stage2_distance(source_run):
    values = []
    for path in sorted(source_run.glob('box_catch_control_*.txt'))[-5:]:
        data = np.atleast_2d(np.loadtxt(str(path), skiprows=1))
        if data.shape[1] >= 14 and np.isfinite(data[-1, 13]):
            values.append(float(data[-1, 13]))
    if not values:
        return FALLBACK_STAGE2_DISTANCE_M, []
    return float(np.median(values)), values


def aligned_move_start(source_run, target_distance):
    catch_q = catch_terminal(source_run)
    
    
    
    best = None
    for alpha in np.linspace(0.8, 1.35, 2201):
        candidate = catch_q + alpha * (BASE_MOVE_START_Q - catch_q)
        error = abs(planner.move_dh_distance(candidate) - target_distance)
        if best is None or error < best[0]:
            best = (error, candidate, float(alpha))
    return best[1], best[2], best[0]


def main():
    source_run = selected_run()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = planner.GENERATED / 'run_{}_move220mm'.format(stamp)
    run_dir.mkdir(parents=True, exist_ok=False)
    planner.CURRENT_RUN_DIR = run_dir

    try:
        real_stage2_distance, stage2_samples = measured_stage2_distance(
            source_run)
        move_start_q, closure_alpha, alignment_error = aligned_move_start(
            source_run, real_stage2_distance)
        start_distance = planner.move_dh_distance(move_start_q)
        if alignment_error > MAX_START_DISTANCE_ERROR_M:
            raise RuntimeError(
                'Move start {:.6f} m does not match real Stage2 {:.6f} m'.format(
                    start_distance, real_stage2_distance))
        planner.require_safe_move_distance(
            move_start_q, 'real-aligned Move start')
        planner.set_joint_vector(
            planner.CONFIG_MOVE, 'Joint_init', move_start_q)

        
        
        planner.set_joint_vector(
            planner.CONFIG_MOVE, 'Joint_final',
            POSE_REFERENCE_MOVE_GOAL_Q)
        move_goal_path = (
            planner.STAGING / 'move_vbox_parallel_long_edge_q.yaml')
        if move_goal_path.exists():
            move_goal_path.unlink()
        planner.run_logged(
            [planner.GOAL_EXE, planner.CONFIG_MOVE, '1', '0'],
            run_dir / 'move_goal.log')
        move_goal = planner.unwrap_near(
            planner.generated_goal(move_goal_path), move_start_q)
        wrist_goal_change = float(np.max(np.abs(np.r_[
            move_goal[3:6] - move_start_q[3:6],
            move_goal[9:12] - move_start_q[9:12]])))
        if wrist_goal_change > MAX_WRIST_CHANGE_RAD:
            raise RuntimeError(
                'low-wrist Move goal exceeds wrist limit: {:.6f} rad'.format(
                    wrist_goal_change))
        planner.set_joint_vector(
            planner.CONFIG_MOVE, 'Joint_final', move_goal)
        goal_distance = planner.require_safe_move_distance(
            move_goal, '22 cm box Move goal')

        planner.run_logged(
            [planner.MOVE_EXE, '--validate-only'],
            run_dir / 'move_native_validity.log')
        move_attempt = planner.run_planner_with_retries(
            planner.MOVE_EXE, 'move', run_dir)
        move_sampling = planner.sample_kirara('move', run_dir)
        move_left, move_right = planner.trajectory_pair('move')

        start_gap = float(np.max(np.abs(
            np.r_[move_left[0], move_right[0]] - move_start_q)))
        if start_gap > 1e-5:
            raise RuntimeError(
                'new Move first-point joint gap is {:.9f} rad'.format(start_gap))
        distances = np.array([
            planner.move_dh_distance(np.r_[left, right])
            for left, right in zip(move_left, move_right)])
        observed = [float(np.min(distances)), float(np.max(distances))]
        if (observed[0] < planner.MOVE_DH_DISTANCE_MIN_M - 1e-6 or
                observed[1] > planner.MOVE_DH_DISTANCE_MAX_M + 1e-6):
            raise RuntimeError(
                'Move distance [{:.6f}, {:.6f}] violates [{:.3f}, {:.3f}]'.format(
                    observed[0], observed[1],
                    planner.MOVE_DH_DISTANCE_MIN_M,
                    planner.MOVE_DH_DISTANCE_MAX_M))
        joint_samples = np.c_[move_left, move_right]
        wrist_indices = [3, 4, 5, 9, 10, 11]
        wrist_change_observed = float(np.max(np.abs(
            joint_samples[:, wrist_indices] -
            move_start_q[wrist_indices][None, :])))
        if wrist_change_observed > MAX_WRIST_CHANGE_RAD + 1e-6:
            raise RuntimeError(
                'sampled Move wrist change {:.6f} exceeds {:.6f} rad'.format(
                    wrist_change_observed, MAX_WRIST_CHANGE_RAD))

        np.savetxt(
            run_dir / 'move_left_samples_{}.txt'.format(stamp),
            move_left, fmt='%.12g')
        np.savetxt(
            run_dir / 'move_right_samples_{}.txt'.format(stamp),
            move_right, fmt='%.12g')
        shutil.copy2(
            str(planner.CONFIG_MOVE), str(run_dir / 'move_config.yaml'))
        copy_single('catch_left_samples_*.txt', source_run, run_dir)
        copy_single('catch_right_samples_*.txt', source_run, run_dir)
        if (source_run / 'catch_config.yaml').is_file():
            shutil.copy2(
                str(source_run / 'catch_config.yaml'),
                str(run_dir / 'catch_config_REUSED_UNCHANGED.yaml'))

        manifest = {
            'timestamp': stamp,
            'method': 'REUSED_CATCH_MoveOnly_BidirectionalPlanning_KIRARA_220mm_box',
            'catch_replanned': False,
            'catch_reused_from': str(source_run),
            'real_stage2_distance_from_log_m': real_stage2_distance,
            'real_stage2_final_distance_samples_m': stage2_samples,
            'new_move_start_distance_m': start_distance,
            'stage2_to_move_distance_jump_m': (
                start_distance - real_stage2_distance),
            'stage2_closure_extrapolation_alpha': closure_alpha,
            'physical_box_dimensions_m': [0.17, 0.220, 0.101],
            'move_dh_distance_limit_m': [
                planner.MOVE_DH_DISTANCE_MIN_M,
                planner.MOVE_DH_DISTANCE_MAX_M],
            'move_dh_distance_observed_m': observed,
            'move_goal_distance_m': goal_distance,
            'move_goal_max_wrist_change_rad': wrist_goal_change,
            'move_sampled_max_wrist_change_rad': wrist_change_observed,
            'move_first_point_joint_gap_rad': start_gap,
            'move_points': int(move_left.shape[0]),
            'move_duration_s': float(move_sampling['duration_s']),
            'move_planner_attempt': int(move_attempt),
            'auto_ft300_wrapper_modified': False,
        }
        (run_dir / 'trajectory_manifest_{}.json'.format(stamp)).write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
        (planner.GENERATED / 'latest_run.txt').write_text(str(run_dir) + '\n')
        print('TCST kirara_box Move-only planning completed: {}'.format(run_dir))
        print('Catch reused unchanged; real Stage2={:.6f} m, Move start={:.6f} m, '
              'Move range=[{:.6f}, {:.6f}] m'.format(
                  real_stage2_distance, start_distance,
                  observed[0], observed[1]))
        planner.CURRENT_RUN_DIR = None
    except Exception as error:
        planner.mark_planning_only(run_dir, error)
        print('TCST kirara_box Move-only planning error: {}'.format(error),
              file=sys.stderr)
        raise


if __name__ == '__main__':
    main()
