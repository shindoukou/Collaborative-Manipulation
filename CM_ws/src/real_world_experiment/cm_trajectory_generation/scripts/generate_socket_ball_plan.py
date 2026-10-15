#!/usr/bin/env python3

import argparse
import datetime
import json
import os
import shutil
import subprocess
from pathlib import Path


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
    parser.add_argument('--move-guide-left', type=Path)
    parser.add_argument('--move-guide-right', type=Path)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--catch-port', type=int, default=55105)
    parser.add_argument('--move-port', type=int, default=55106)
    parser.add_argument('--catch-goal-port', type=int, default=55101)
    parser.add_argument('--move-goal-port', type=int, default=55102)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--no-update-latest', action='store_true')
    args = parser.parse_args()

    package = args.workspace / 'src/real_world_experiment/cm_trajectory_generation'
    catch_config = args.catch_config or package / 'config/ball/catch_template.yaml'
    move_config = args.move_config or package / 'config/ball/move_template.yaml'
    if args.output_dir:
        output = args.output_dir
    else:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        output = package / 'generated/ball/socket_run_{}'.format(stamp)
    goals = output / 'goals'
    catch_output = output / 'catch'
    move_output = output / 'move'
    for directory in (goals, catch_output, move_output):
        directory.mkdir(parents=True, exist_ok=True)

    goal_pair_runner = (
        args.workspace /
        'devel/lib/cm_trajectory_generation/run_distributed_goal_pair.py')
    pair_runner = (
        args.workspace /
        'devel/lib/cm_trajectory_generation/run_socket_planning_pair.py')
    catch_prefix = goals / 'catch'
    move_prefix = goals / 'move'
    move_guide_left = args.move_guide_left or (
        package / 'generated/ball/socket_latest/move_left_planned_path.txt')
    move_guide_right = args.move_guide_right or (
        package / 'generated/ball/socket_latest/move_right_planned_path.txt')
    if move_guide_left.exists() != move_guide_right.exists():
        raise RuntimeError('both Move guide paths must exist together')
    use_move_guide = move_guide_left.exists() and move_guide_right.exists()

    catch_goal_output = output / 'catch_goal'
    move_goal_output = output / 'move_goal'
    run([
        goal_pair_runner,
        '--phase', 'catch',
        '--config', catch_config,
        '--output-dir', catch_goal_output,
        '--output-prefix', catch_prefix,
        '--port', args.catch_goal_port,
        '--timeout', args.timeout,
    ])
    run([
        pair_runner,
        '--phase', 'catch',
        '--config', catch_config,
        '--output-dir', catch_output,
        '--port', args.catch_port,
        '--left-endpoints', goals / 'catch_left.txt',
        '--right-endpoints', goals / 'catch_right.txt',
        '--max-iterations', args.max_iterations,
        '--timeout', args.timeout,
    ])

    run([
        goal_pair_runner,
        '--phase', 'move',
        '--config', move_config,
        '--output-dir', move_goal_output,
        '--output-prefix', move_prefix,
        '--reference-pair', goals / 'catch_pair.txt',
        '--port', args.move_goal_port,
        '--timeout', args.timeout,
    ])
    move_planning_command = [
        pair_runner,
        '--phase', 'move',
        '--config', move_config,
        '--output-dir', move_output,
        '--port', args.move_port,
        '--left-endpoints', goals / 'move_left.txt',
        '--right-endpoints', goals / 'move_right.txt',
        '--max-iterations', args.max_iterations,
        '--timeout', args.timeout,
    ]
    if use_move_guide:
        move_planning_command.extend([
            '--left-guide', move_guide_left,
            '--right-guide', move_guide_right,
        ])
    run(move_planning_command)

    summary = {
        'status': 'success',
        'state_exchange': '6D_left_to_6D_right',
        'terminal_goal_search': {
            'architecture': 'two_process_distributed_local_IK_candidate_exchange',
            'centralized_dual_arm_goal_generator_used': False,
            'catch': json.loads((
                catch_goal_output / 'catch_distributed_goal_summary.json').read_text()),
            'move': json.loads((
                move_goal_output / 'move_distributed_goal_summary.json').read_text()),
        },
        'move_sampling_heuristic': {
            'source': 'previous_distributed_move_path' if use_move_guide else None,
            'left': str(move_guide_left) if use_move_guide else None,
            'right': str(move_guide_right) if use_move_guide else None,
            'accepted_as_terminal_goal': False,
            'planner_state_or_validity_authority': False,
            'terminal_goal_source': 'distributed_tcp_goal_search',
        },
        'catch': json.loads(
            (catch_output / 'catch_socket_summary.json').read_text()),
        'move': json.loads(
            (move_output / 'move_socket_summary.json').read_text()),
    }
    summary_path = output / 'planning_summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    if not args.no_update_latest:
        latest = package / 'generated/ball/socket_latest'
        latest.mkdir(parents=True, exist_ok=True)
        for phase, phase_output in (
                ('catch', catch_output), ('move', move_output)):
            for role in ('left', 'right'):
                name = '{}_{}_planned_path.txt'.format(phase, role)
                shutil.copy2(phase_output / name, latest / name)
        shutil.copy2(summary_path, latest / 'planning_summary.json')
    print('complete planning summary: {}'.format(summary_path))


if __name__ == '__main__':
    main()
