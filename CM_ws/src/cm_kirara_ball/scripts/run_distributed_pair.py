#!/usr/bin/env python3
"""Run the left/right distributed planning nodes on one host for testing."""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


DEVEL = Path('/home/mas-2/3rd_party/CM_ws/devel/lib/cm_kirara_ball')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--port', type=int, default=55105)
    parser.add_argument('--left-input', type=Path)
    parser.add_argument('--right-input', type=Path)
    parser.add_argument('--left-endpoints', type=Path)
    parser.add_argument('--right-endpoints', type=Path)
    parser.add_argument('--left-guide', type=Path)
    parser.add_argument('--right-guide', type=Path)
    parser.add_argument('--planning-only', action='store_true')
    parser.add_argument('--kirara-iterations', type=int, default=120)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--seed', type=int, default=20260728)
    args = parser.parse_args()
    if bool(args.left_input) != bool(args.right_input):
        parser.error('--left-input and --right-input must be supplied together')
    if bool(args.left_endpoints) != bool(args.right_endpoints):
        parser.error('--left-endpoints and --right-endpoints must be supplied together')
    if bool(args.left_guide) != bool(args.right_guide):
        parser.error('--left-guide and --right-guide must be supplied together')
    if args.left_input and args.left_endpoints:
        parser.error('input paths and endpoint paths are mutually exclusive')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    base = [
        str(DEVEL / 'cm_distributed_planner_node.py'),
        '--phase', args.phase,
        '--config', str(args.config),
        '--output-dir', str(args.output_dir),
        '--port', str(args.port),
        '--kirara-iterations', str(args.kirara_iterations),
        '--max-iterations', str(args.max_iterations),
        '--seed', str(args.seed),
    ]
    right_command = base + ['--role', 'right']
    left_command = base + ['--role', 'left', '--peer-host', '127.0.0.1']
    if args.left_input:
        left_command += ['--input-local-path', str(args.left_input)]
        right_command += ['--input-local-path', str(args.right_input)]
    if args.left_endpoints:
        left_command += ['--endpoint-local-path', str(args.left_endpoints)]
        right_command += ['--endpoint-local-path', str(args.right_endpoints)]
    if args.left_guide:
        left_command += ['--guide-local-path', str(args.left_guide)]
        right_command += ['--guide-local-path', str(args.right_guide)]
    if args.planning_only:
        left_command.append('--planning-only')
        right_command.append('--planning-only')

    right_log_path = args.output_dir / '{}_right_console.log'.format(args.phase)
    left_log_path = args.output_dir / '{}_left_console.log'.format(args.phase)
    started = time.monotonic()
    with right_log_path.open('w') as right_log, left_log_path.open('w') as left_log:
        right = subprocess.Popen(
            right_command, stdout=right_log, stderr=subprocess.STDOUT)
        time.sleep(0.25)
        left = subprocess.Popen(
            left_command, stdout=left_log, stderr=subprocess.STDOUT)
        left_code = left.wait()
        right_code = right.wait()
    if left_code or right_code:
        raise RuntimeError(
            'distributed pair failed: left={} right={}; logs: {}, {}'.format(
                left_code, right_code, left_log_path, right_log_path))
    reports = {}
    for role in ('left', 'right'):
        report_path = args.output_dir / '{}_{}_report.json'.format(
            args.phase, role)
        reports[role] = json.loads(report_path.read_text())
    summary = {
        'status': 'success',
        'phase': args.phase,
        'wall_time_s': time.monotonic() - started,
        'left': reports['left'],
        'right': reports['right'],
        'socket_is_real_tcp': True,
        'same_host_test': True,
    }
    summary_path = args.output_dir / '{}_distributed_summary.json'.format(
        args.phase)
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('distributed pair error: {}'.format(error), file=sys.stderr)
        raise
