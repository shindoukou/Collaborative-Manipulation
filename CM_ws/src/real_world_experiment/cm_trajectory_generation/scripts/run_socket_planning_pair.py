#!/usr/bin/env python3

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_WORKSPACE = Path('/home/mas-2/3rd_party/CM_ws')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=55105)
    parser.add_argument('--left-endpoints', type=Path)
    parser.add_argument('--right-endpoints', type=Path)
    parser.add_argument('--left-guide', type=Path)
    parser.add_argument('--right-guide', type=Path)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--step', type=float, default=0.25)
    parser.add_argument('--edge-resolution', type=float, default=0.005)
    parser.add_argument('--shortcut-attempts', type=int, default=80)
    parser.add_argument('--seed', type=int, default=20260728)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--planner-node', type=Path)
    parser.add_argument('--validator', type=Path)
    args = parser.parse_args()
    if bool(args.left_endpoints) != bool(args.right_endpoints):
        parser.error('--left-endpoints and --right-endpoints must be supplied together')
    if bool(args.left_guide) != bool(args.right_guide):
        parser.error('--left-guide and --right-guide must be supplied together')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    workspace = Path(os.environ.get('CM_WORKSPACE', str(DEFAULT_WORKSPACE)))
    node = args.planner_node or (
        workspace / 'devel/lib/cm_trajectory_generation/cm_socket_planner_node.py')
    base = [
        str(node),
        '--phase', args.phase,
        '--config', str(args.config),
        '--output-dir', str(args.output_dir),
        '--port', str(args.port),
        '--max-iterations', str(args.max_iterations),
        '--step', str(args.step),
        '--edge-resolution', str(args.edge_resolution),
        '--shortcut-attempts', str(args.shortcut_attempts),
        '--seed', str(args.seed),
    ]
    if args.validator:
        base += ['--validator', str(args.validator)]
    right_command = base + ['--role', 'right']
    left_command = base + ['--role', 'left', '--peer-host', '127.0.0.1']
    if args.left_endpoints:
        left_command += ['--endpoint-local-path', str(args.left_endpoints)]
        right_command += ['--endpoint-local-path', str(args.right_endpoints)]
    if args.left_guide:
        left_command += ['--guide-local-path', str(args.left_guide)]
        right_command += ['--guide-local-path', str(args.right_guide)]

    right_log_path = args.output_dir / '{}_right_console.log'.format(args.phase)
    left_log_path = args.output_dir / '{}_left_console.log'.format(args.phase)
    started = time.monotonic()
    with right_log_path.open('w') as right_log, left_log_path.open('w') as left_log:
        right = subprocess.Popen(
            right_command, stdout=right_log, stderr=subprocess.STDOUT)
        time.sleep(0.25)
        left = subprocess.Popen(
            left_command, stdout=left_log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + args.timeout
        while True:
            left_code = left.poll()
            right_code = right.poll()
            if left_code is not None and right_code is not None:
                break
            if ((left_code is not None and left_code != 0) or
                    (right_code is not None and right_code != 0) or
                    time.monotonic() >= deadline):
                for process in (left, right):
                    if process.poll() is None:
                        process.terminate()
                for process in (left, right):
                    try:
                        process.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                left_code = left.returncode
                right_code = right.returncode
                break
            time.sleep(0.1)
    if left_code or right_code:
        raise RuntimeError(
            'Socket planning pair failed: left={} right={}; logs: {}, {}'.format(
                left_code, right_code, left_log_path, right_log_path))

    reports = {}
    for role in ('left', 'right'):
        report_path = args.output_dir / '{}_{}_report.json'.format(
            args.phase, role)
        reports[role] = json.loads(report_path.read_text())
    left_count = reports['left']['planner']['states_after_simplification']
    right_count = reports['right']['planner']['states_after_simplification']
    if left_count != right_count:
        raise RuntimeError('left/right output state counts differ')
    summary = {
        'status': 'success',
        'phase': args.phase,
        'wall_time_s': time.monotonic() - started,
        'left': reports['left'],
        'right': reports['right'],
        'socket_is_real_tcp': True,
        'same_host_test': True,
        'state_exchange': '6D_left_to_6D_right',
        'joint_state_storage_in_planner': False,
        'seed': args.seed,
    }
    summary_path = args.output_dir / '{}_socket_summary.json'.format(args.phase)
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('Socket pair runner error: {}'.format(error), file=sys.stderr)
        raise
