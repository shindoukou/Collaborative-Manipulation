#!/usr/bin/env python3

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

PROTOCOL = 'CM_DISTRIBUTED_IK_GOAL_V1'
DEFAULT_WORKSPACE = Path('/home/mas-2/3rd_party/CM_ws')
SOURCE_DIRECTORY = Path(os.environ.get(
    'CM_WORKSPACE', str(DEFAULT_WORKSPACE))) / 'src/real_world_experiment/cm_trajectory_generation/scripts'
if str(SOURCE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIRECTORY))

from cm_socket_planner_node import JsonChannel, NativeValidator


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=('left', 'right'), required=True)
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--reference-pair', type=Path)
    parser.add_argument('--bind-host', default='0.0.0.0')
    parser.add_argument('--peer-host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=55101)
    parser.add_argument('--socket-timeout', type=float, default=900.0)
    parser.add_argument('--max-candidates-per-sample', type=int, default=4)
    parser.add_argument('--goal-executable', type=Path)
    parser.add_argument('--validator', type=Path)
    return parser.parse_args()


def load_reference(config, reference_path=None):
    if reference_path:
        values = np.asarray(np.loadtxt(str(reference_path)), dtype=float).reshape(-1)
        if values.shape != (12,) or not np.all(np.isfinite(values)):
            raise RuntimeError('reference pair must contain 12 finite values')
        return values
    return np.asarray([
        config['Joint_init']['joint_{}'.format(index + 1)]
        for index in range(12)
    ], dtype=float)


def load_candidates(path):
    rows = []
    with path.open() as source:
        header = source.readline().strip().split('\t')
        expected = [
            'sample_index', 'axis_index', 'roll_index', 'position_error',
            'orientation_error', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        if header != expected:
            raise RuntimeError('unexpected local candidate header: {}'.format(header))
        for line in source:
            fields = line.strip().split('\t')
            if not fields or fields == ['']:
                continue
            if len(fields) != 11:
                raise RuntimeError('malformed local candidate row')
            q = np.asarray([float(value) for value in fields[5:]], dtype=float)
            record = {
                'sample_index': int(fields[0]),
                'axis_index': int(fields[1]),
                'roll_index': int(fields[2]),
                'position_error': float(fields[3]),
                'orientation_error': float(fields[4]),
                'q': q.tolist(),
            }
            if not np.all(np.isfinite(q)):
                raise RuntimeError('non-finite local candidate')
            rows.append(record)
    return rows


def periodic_difference(values, reference):
    return np.arctan2(np.sin(values - reference), np.cos(values - reference))


def prune_candidates(rows, reference, weights, maximum):
    grouped = defaultdict(list)
    for record in rows:
        q = np.asarray(record['q'], dtype=float)
        difference = periodic_difference(q, reference)
        local_score = float(np.dot(weights, difference * difference))
        local_score += float(record['position_error'] + record['orientation_error'])
        grouped[int(record['sample_index'])].append((local_score, record))
    result = []
    for sample_index in sorted(grouped):
        entries = sorted(grouped[sample_index], key=lambda item: (
            item[0], tuple(item[1]['q'])))
        exact_reference = [
            record for _, record in entries
            if record['position_error'] == 0.0 and
            record['orientation_error'] == 0.0]
        selected = list(exact_reference)
        for _, record in entries:
            if record in selected:
                continue
            if len(selected) >= maximum:
                break
            selected.append(record)
        result.extend(selected)
    return result


def group_by_sample(rows):
    result = defaultdict(list)
    for row in rows:
        result[int(row['sample_index'])].append(row)
    return result


def select_pair(left_rows, right_rows, reference, config, validator):
    goal = config.get('goal_generator', {})
    score_reference = reference
    box_goal = config.get('box_goal_generator', {})
    if box_goal:
        score_reference = np.asarray([
            config['Joint_final']['joint_{}'.format(index + 1)]
            for index in range(12)
        ], dtype=float)
    joint_weights = np.asarray(
        goal.get('joint_score_weights', [1.0] * 12), dtype=float)
    joint_weight = float(goal.get('joint_score_weight', 1.0))
    pose_weight = float(goal.get('pose_score_weight', 20.0))
    left_by_sample = group_by_sample(left_rows)
    right_by_sample = group_by_sample(right_rows)
    best = None
    tested = 0
    valid = 0
    for sample_index in sorted(set(left_by_sample) & set(right_by_sample)):
        for left in left_by_sample[sample_index]:
            for right in right_by_sample[sample_index]:
                tested += 1
                pair = np.r_[
                    np.asarray(left['q'], dtype=float),
                    np.asarray(right['q'], dtype=float)]
                if not validator.valid(pair, terminal=True):
                    continue
                valid += 1
                difference = periodic_difference(pair, score_reference)
                pose_cost = sum(float(row[key]) for row in (left, right)
                                for key in ('position_error', 'orientation_error'))
                score = (joint_weight * float(np.dot(
                    joint_weights, difference * difference)) +
                    pose_weight * pose_cost)
                key = (score, tuple(pair.tolist()))
                if best is None or key < best[0]:
                    best = (key, pair, sample_index, score)
    if best is None:
        raise RuntimeError(
            'no terminal-valid pair among {} tested combinations'.format(tested))
    return best[1], {
        'sample_index': int(best[2]),
        'score': float(best[3]),
        'tested_pairs': int(tested),
        'valid_pairs': int(valid),
    }


def pair_digest(pair):
    payload = np.asarray(pair, dtype='<f8').tobytes()
    return hashlib.sha256(payload).hexdigest()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(args.config.read_text())
    reference = load_reference(config, args.reference_pair)
    local_offset = 0 if args.role == 'left' else 6
    local_reference = reference[local_offset:local_offset + 6]
    goal = config.get('goal_generator', {})
    joint_weights = np.asarray(
        goal.get('joint_score_weights', [1.0] * 12), dtype=float)
    local_weights = joint_weights[local_offset:local_offset + 6]
    workspace = Path(os.environ.get('CM_WORKSPACE', str(DEFAULT_WORKSPACE)))
    executable = args.goal_executable or (
        workspace / 'devel/lib/cm_trajectory_generation/CM_SOCKET_BALL_GOAL')
    candidate_path = args.output_dir / '{}_{}_local_candidates.tsv'.format(
        args.phase, args.role)
    command = [
        str(executable), 'local', args.phase, args.role,
        str(args.config), str(candidate_path)]
    if args.reference_pair:
        command.append(str(args.reference_pair))
    generation_started = time.monotonic()
    subprocess.run(command, check=True)
    generated = load_candidates(candidate_path)
    local_rows = prune_candidates(
        generated, local_reference, local_weights,
        args.max_candidates_per_sample)

    trace_path = args.output_dir / '{}_{}_goal_socket_trace.jsonl'.format(
        args.phase, args.role)
    channel = JsonChannel(
        args.role, args.bind_host, args.peer_host, args.port,
        args.socket_timeout, trace_path)
    validator = NativeValidator(
        args.phase, args.config,
        args.output_dir / '{}_{}_goal_validator.log'.format(
            args.phase, args.role), args.validator)
    started = time.monotonic()
    try:
        hello = channel.exchange('HELLO', {
            'protocol': PROTOCOL,
            'phase': args.phase,
            'role': args.role,
            'candidate_dimension': 6,
            'reference_digest': pair_digest(reference),
        })
        if (hello.get('protocol') != PROTOCOL or
                hello.get('phase') != args.phase or
                hello.get('reference_digest') != pair_digest(reference)):
            raise RuntimeError('peer goal-search protocol/configuration mismatch')
        peer_payload = channel.exchange('LOCAL_IK_CANDIDATES', {
            'candidates': local_rows,
            'generated_count': len(generated),
        })
        peer_rows = list(peer_payload['candidates'])
        if args.role == 'left':
            left_rows, right_rows = local_rows, peer_rows
        else:
            left_rows, right_rows = peer_rows, local_rows
        pair, metrics = select_pair(
            left_rows, right_rows, reference, config, validator)
        agreement = channel.exchange('SELECTED_TERMINAL_PAIR', {
            'digest': pair_digest(pair),
            'pair': pair.tolist(),
            'sample_index': metrics['sample_index'],
        })
        peer_pair = np.asarray(agreement.get('pair'), dtype=float)
        if (agreement.get('digest') != pair_digest(pair) or
                peer_pair.shape != (12,) or
                not np.allclose(peer_pair, pair, rtol=0.0, atol=1.0e-12)):
            raise RuntimeError('left/right terminal-pair selections disagree')

        local_goal = pair[local_offset:local_offset + 6]
        endpoint_path = args.output_dir / '{}_{}_goal_endpoints.txt'.format(
            args.phase, args.role)
        np.savetxt(
            str(endpoint_path), np.vstack([local_reference, local_goal]),
            fmt='%.17g')
        selected_pair_path = args.output_dir / '{}_{}_selected_pair.txt'.format(
            args.phase, args.role)
        np.savetxt(str(selected_pair_path), pair.reshape(1, 12), fmt='%.17g')
        report = {
            'status': 'success',
            'protocol': PROTOCOL,
            'phase': args.phase,
            'role': args.role,
            'local_ik_only': True,
            'local_candidate_dimension': 6,
            'peer_joint_candidates_received_over_tcp': True,
            'centralized_dual_arm_ik_called': False,
            'generated_local_candidates': len(generated),
            'transmitted_local_candidates': len(local_rows),
            'received_peer_candidates': len(peer_rows),
            'selection': metrics,
            'selected_pair_digest': pair_digest(pair),
            'native_terminal_validator_queries': validator.query_count,
            'local_generation_s': started - generation_started,
            'distributed_selection_s': time.monotonic() - started,
            'socket_bytes_sent': channel.bytes_sent,
            'socket_bytes_received': channel.bytes_received,
            'outputs': {
                'local_endpoints': str(endpoint_path),
                'selected_pair': str(selected_pair_path),
            },
        }
        report_path = args.output_dir / '{}_{}_goal_report.json'.format(
            args.phase, args.role)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, separators=(',', ':')))
    finally:
        validator.close()
        channel.close()


if __name__ == '__main__':
    main()
