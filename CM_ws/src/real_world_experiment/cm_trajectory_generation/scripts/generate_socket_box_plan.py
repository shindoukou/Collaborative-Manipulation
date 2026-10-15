#!/usr/bin/env python3

import argparse
import datetime
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
import yaml


DEFAULT_WORKSPACE = Path('/home/mas-2/3rd_party/CM_ws')


def run(command):
    print('+ {}'.format(' '.join(str(item) for item in command)), flush=True)
    subprocess.run([str(item) for item in command], check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--workspace', type=Path,
        default=Path(os.environ.get('CM_WORKSPACE', str(DEFAULT_WORKSPACE))))
    parser.add_argument('--catch-config', type=Path)
    parser.add_argument('--move-config', type=Path)
    parser.add_argument('--catch-guide-left', type=Path)
    parser.add_argument('--catch-guide-right', type=Path)
    parser.add_argument('--move-guide-left', type=Path)
    parser.add_argument('--move-guide-right', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--catch-port', type=int, default=55205)
    parser.add_argument('--move-port', type=int, default=55206)
    parser.add_argument('--catch-goal-port', type=int, default=55201)
    parser.add_argument('--move-goal-port', type=int, default=55202)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--no-update-latest', action='store_true')
    args = parser.parse_args()

    package = args.workspace / 'src/real_world_experiment/cm_trajectory_generation'
    ball_bin = args.workspace / 'devel/lib/cm_trajectory_generation'
    box_bin = args.workspace / 'devel/lib/cm_trajectory_generation'
    catch_config = args.catch_config or package / 'config/box/catch_kirara_box.yaml'
    move_config = args.move_config or package / 'config/box/move_kirara_box.yaml'
    if args.output_dir:
        output = args.output_dir
    else:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        output = package / 'generated/box/socket_run_{}'.format(stamp)
    goals = output / 'goals'
    catch_output = output / 'catch'
    move_output = output / 'move'
    for directory in (goals, catch_output, move_output):
        directory.mkdir(parents=True, exist_ok=True)

    goal_pair_runner = ball_bin / 'run_distributed_goal_pair.py'
    goal_node = ball_bin / 'cm_distributed_goal_node.py'
    pair_runner = ball_bin / 'run_socket_planning_pair.py'
    planner_node = ball_bin / 'cm_socket_planner_node.py'
    goal_executable = box_bin / 'CM_KIRARA_BOX_GOAL'
    validator = box_bin / 'CM_SOCKET_BOX_VALIDATOR'
    required = (
        goal_pair_runner, goal_node, pair_runner, planner_node,
        goal_executable, validator)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError('build CM_ws first; missing: {}'.format(', '.join(missing)))

    catch_prefix = goals / 'catch'
    move_prefix = goals / 'move'
    catch_guide_left = args.catch_guide_left or (
        package / 'generated/box/staging/catch_planned_left.txt')
    catch_guide_right = args.catch_guide_right or (
        package / 'generated/box/staging/catch_planned_right.txt')
    if catch_guide_left.exists() != catch_guide_right.exists():
        raise RuntimeError('both Catch guide paths must exist together')
    use_catch_guide = catch_guide_left.exists() and catch_guide_right.exists()
    move_guide_left = args.move_guide_left or (
        package / 'generated/box/socket_latest/move_left_planned_path.txt')
    move_guide_right = args.move_guide_right or (
        package / 'generated/box/socket_latest/move_right_planned_path.txt')
    if not move_guide_left.exists() and not move_guide_right.exists():
        move_guide_left = (
            package / 'generated/box/paper_move_reference_latest_valid/'
            'move_left_planned_path.txt')
        move_guide_right = (
            package / 'generated/box/paper_move_reference_latest_valid/'
            'move_right_planned_path.txt')
    if move_guide_left.exists() != move_guide_right.exists():
        raise RuntimeError('both Move guide paths must exist together')
    use_move_guide = move_guide_left.exists() and move_guide_right.exists()

    catch_goal_output = output / 'catch_goal'
    move_goal_output = output / 'move_goal'
    common_goal = [
        '--goal-node', goal_node,
        '--goal-executable', goal_executable,
        '--validator', validator,
    ]
    run([
        goal_pair_runner, '--phase', 'catch', '--config', catch_config,
        '--output-dir', catch_goal_output, '--output-prefix', catch_prefix,
        '--port', args.catch_goal_port, '--timeout', args.timeout,
    ] + common_goal)
    catch_command = [
        pair_runner, '--phase', 'catch', '--config', catch_config,
        '--output-dir', catch_output, '--port', args.catch_port,
        '--left-endpoints', goals / 'catch_left.txt',
        '--right-endpoints', goals / 'catch_right.txt',
        '--max-iterations', args.max_iterations, '--timeout', args.timeout,
        '--planner-node', planner_node, '--validator', validator,
    ]
    if use_catch_guide:
        catch_command.extend([
            '--left-guide', catch_guide_left,
            '--right-guide', catch_guide_right,
        ])
    run(catch_command)

    move_config_data = yaml.safe_load(move_config.read_text())
    move_start_pair = np.asarray([
        move_config_data['Joint_init']['joint_{}'.format(index + 1)]
        for index in range(12)
    ], dtype=float)
    move_start_path = goals / 'move_stage2_start_pair.txt'
    np.savetxt(str(move_start_path), move_start_pair.reshape(1, 12), fmt='%.17g')
    run([
        goal_pair_runner, '--phase', 'move', '--config', move_config,
        '--output-dir', move_goal_output, '--output-prefix', move_prefix,
        '--reference-pair', move_start_path,
        '--port', args.move_goal_port, '--timeout', args.timeout,
    ] + common_goal)
    move_command = [
        pair_runner, '--phase', 'move', '--config', move_config,
        '--output-dir', move_output, '--port', args.move_port,
        '--left-endpoints', goals / 'move_left.txt',
        '--right-endpoints', goals / 'move_right.txt',
        '--max-iterations', args.max_iterations, '--timeout', args.timeout,
        '--planner-node', planner_node, '--validator', validator,
    ]
    if use_move_guide:
        move_command.extend([
            '--left-guide', move_guide_left,
            '--right-guide', move_guide_right,
        ])
    run(move_command)

    summary = {
        'status': 'success',
        'object': 'box',
        'state_exchange': '6D_left_to_6D_right_over_TCP',
        'terminal_goal_search': {
            'architecture': 'two_process_distributed_local_IK_candidate_exchange',
            'centralized_dual_arm_goal_generator_used': False,
            'catch': json.loads((
                catch_goal_output / 'catch_distributed_goal_summary.json').read_text()),
            'move': json.loads((
                move_goal_output / 'move_distributed_goal_summary.json').read_text()),
        },
        'planning': {
            'architecture': 'two_process_distributed_dual_tree_connect',
            'centralized_12D_tree_stored': False,
            'catch': json.loads((
                catch_output / 'catch_socket_summary.json').read_text()),
            'move': json.loads((
                move_output / 'move_socket_summary.json').read_text()),
        },
        'stage2_boundary': {
            'catch_terminal': str(goals / 'catch_pair.txt'),
            'move_start_after_stage2': str(move_start_path),
            'stage2_is_controller_transition_not_planning': True,
        },
        'move_sampling_heuristic': {
            'source': 'previous_distributed_move_path' if use_move_guide else None,
            'planner_state_or_validity_authority': False,
        },
        'catch_sampling_heuristic': {
            'source': 'previous_collision_checked_path' if use_catch_guide else None,
            'planner_state_or_validity_authority': False,
        },
    }
    summary_path = output / 'planning_summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    if not args.no_update_latest:
        latest = package / 'generated/box/socket_latest'
        latest.mkdir(parents=True, exist_ok=True)
        for phase, phase_output in (
                ('catch', catch_output), ('move', move_output)):
            for role in ('left', 'right'):
                name = '{}_{}_planned_path.txt'.format(phase, role)
                shutil.copy2(phase_output / name, latest / name)
        shutil.copy2(summary_path, latest / 'planning_summary.json')
    print('complete Box distributed planning summary: {}'.format(summary_path))


if __name__ == '__main__':
    main()
