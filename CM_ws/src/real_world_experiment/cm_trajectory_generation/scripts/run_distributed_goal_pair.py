#!/usr/bin/env python3

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


DEFAULT_WORKSPACE = Path('/home/mas-2/3rd_party/CM_ws')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--output-prefix', type=Path, required=True)
    parser.add_argument('--reference-pair', type=Path)
    parser.add_argument('--port', type=int, default=55101)
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--max-candidates-per-sample', type=int, default=4)
    parser.add_argument('--goal-node', type=Path)
    parser.add_argument('--goal-executable', type=Path)
    parser.add_argument('--validator', type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    workspace = Path(os.environ.get('CM_WORKSPACE', str(DEFAULT_WORKSPACE)))
    node = args.goal_node or (
        workspace / 'devel/lib/cm_trajectory_generation/cm_distributed_goal_node.py')
    base = [
        str(node), '--phase', args.phase,
        '--config', str(args.config),
        '--output-dir', str(args.output_dir),
        '--port', str(args.port),
        '--socket-timeout', str(args.timeout),
        '--max-candidates-per-sample', str(args.max_candidates_per_sample),
    ]
    if args.reference_pair:
        base += ['--reference-pair', str(args.reference_pair)]
    if args.goal_executable:
        base += ['--goal-executable', str(args.goal_executable)]
    if args.validator:
        base += ['--validator', str(args.validator)]
    right_command = base + ['--role', 'right']
    left_command = base + ['--role', 'left', '--peer-host', '127.0.0.1']
    right_log_path = args.output_dir / '{}_right_goal_console.log'.format(
        args.phase)
    left_log_path = args.output_dir / '{}_left_goal_console.log'.format(
        args.phase)
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
            'distributed goal search failed: left={} right={}; logs: {}, {}'.format(
                left_code, right_code, left_log_path, right_log_path))

    reports = {}
    pairs = {}
    for role in ('left', 'right'):
        reports[role] = json.loads((
            args.output_dir / '{}_{}_goal_report.json'.format(
                args.phase, role)).read_text())
        pairs[role] = np.asarray(np.loadtxt(
            str(args.output_dir / '{}_{}_selected_pair.txt'.format(
                args.phase, role))), dtype=float).reshape(-1)
    if (reports['left']['selected_pair_digest'] !=
            reports['right']['selected_pair_digest'] or
            not np.allclose(pairs['left'], pairs['right'], rtol=0.0, atol=1.0e-12)):
        raise RuntimeError('distributed goal reports do not agree')
    pair = pairs['left']
    left_endpoints = args.output_dir / '{}_left_goal_endpoints.txt'.format(
        args.phase)
    right_endpoints = args.output_dir / '{}_right_goal_endpoints.txt'.format(
        args.phase)
    shutil.copy2(left_endpoints, str(args.output_prefix) + '_left.txt')
    shutil.copy2(right_endpoints, str(args.output_prefix) + '_right.txt')
    np.savetxt(
        str(args.output_prefix) + '_pair.txt', pair.reshape(1, 12), fmt='%.17g')
    summary = {
        'status': 'success',
        'architecture': 'two_process_distributed_terminal_ik_search',
        'protocol': reports['left']['protocol'],
        'phase': args.phase,
        'wall_time_s': time.monotonic() - started,
        'centralized_dual_arm_goal_generator_used': False,
        'socket_is_real_tcp': True,
        'same_host_test': True,
        'candidate_state_dimension_transmitted': 6,
        'selected_pair_digest': reports['left']['selected_pair_digest'],
        'left': reports['left'],
        'right': reports['right'],
        'outputs': {
            'left_endpoints': str(args.output_prefix) + '_left.txt',
            'right_endpoints': str(args.output_prefix) + '_right.txt',
            'pair': str(args.output_prefix) + '_pair.txt',
        },
    }
    summary_path = args.output_dir / '{}_distributed_goal_summary.json'.format(
        args.phase)
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('distributed goal runner error: {}'.format(error), file=sys.stderr)
        raise
