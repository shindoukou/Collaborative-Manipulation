#!/usr/bin/env python3

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml


PROTOCOL = 'CM_DISTRIBUTED_DUAL_TREE_CONNECT_V1'
DEFAULT_WORKSPACE = Path('/home/mas-2/3rd_party/CM_ws')


def vector6(values, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise RuntimeError('{} must be one finite 6D vector'.format(label))
    return result


def path6(values, label):
    result = np.asarray(values, dtype=float)
    if result.ndim == 1:
        result = result.reshape(1, -1)
    if result.ndim != 2 or result.shape[1] != 6 or not np.all(np.isfinite(result)):
        raise RuntimeError('{} must contain finite 6D rows'.format(label))
    return result


def temporary_pair(role, local, peer):
    """Create a short-lived joint pair only for a native validity query."""
    local = vector6(local, 'local state')
    peer = vector6(peer, 'peer state')
    return np.r_[local, peer] if role == 'left' else np.r_[peer, local]


class JsonChannel:
    def __init__(self, role, bind_host, peer_host, port, timeout, trace_path):
        self.role = role
        self.sequence = 0
        self.bytes_sent = 0
        self.bytes_received = 0
        self.trace = trace_path.open('w')
        if role == 'right':
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((bind_host, port))
            listener.listen(1)
            listener.settimeout(timeout)
            self.sock, address = listener.accept()
            listener.close()
            self.peer_address = '{}:{}'.format(*address)
        else:
            deadline = time.monotonic() + timeout
            last_error = None
            while time.monotonic() < deadline:
                try:
                    self.sock = socket.create_connection(
                        (peer_host, port), timeout=min(2.0, timeout))
                    break
                except OSError as error:
                    last_error = error
                    time.sleep(0.1)
            else:
                raise RuntimeError('cannot connect to {}:{}: {}'.format(
                    peer_host, port, last_error))
            self.peer_address = '{}:{}'.format(peer_host, port)
        self.sock.settimeout(timeout)
        self.reader = self.sock.makefile('r')
        self.writer = self.sock.makefile('w')

    def exchange(self, event, payload):
        packet = {
            'sequence': self.sequence,
            'event': event,
            'role': self.role,
            'payload': payload,
        }
        encoded = json.dumps(packet, separators=(',', ':')) + '\n'
        self.writer.write(encoded)
        self.writer.flush()
        self.bytes_sent += len(encoded.encode('utf-8'))
        reply_line = self.reader.readline()
        if not reply_line:
            raise RuntimeError('peer disconnected during {}'.format(event))
        self.bytes_received += len(reply_line.encode('utf-8'))
        reply = json.loads(reply_line)
        if reply.get('sequence') != self.sequence or reply.get('event') != event:
            raise RuntimeError('Socket sequence mismatch: sent {} {}, got {}'.format(
                self.sequence, event, reply))
        self.trace.write(json.dumps({
            'sent': packet,
            'received': reply,
        }, separators=(',', ':')) + '\n')
        self.trace.flush()
        self.sequence += 1
        return reply['payload']

    def close(self):
        try:
            self.writer.close()
            self.reader.close()
            self.sock.close()
        finally:
            self.trace.close()


class NativeValidator:
    def __init__(self, phase, config_path, log_path, executable=None):
        if executable is None:
            workspace = Path(os.environ.get('CM_WORKSPACE', str(DEFAULT_WORKSPACE)))
            executable = workspace / 'devel/lib/cm_trajectory_generation/CM_SOCKET_BALL_VALIDATOR'
        self.log = log_path.open('w')
        self.process = subprocess.Popen(
            [str(executable), phase, str(config_path)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self.log,
            text=True,
            bufsize=1)
        ready = self.process.stdout.readline().strip()
        if not ready.startswith('READY {} '.format(phase)):
            raise RuntimeError('native validator failed to start: {!r}'.format(ready))
        self.sequence = 0
        self.query_count = 0

    def valid(self, pair, terminal=False):
        pair = np.asarray(pair, dtype=float)
        if pair.shape != (12,) or not np.all(np.isfinite(pair)):
            return False
        command = 'F' if terminal else 'Q'
        request = '{} {} {}\n'.format(
            command,
            self.sequence,
            ' '.join('{:.17g}'.format(value) for value in pair))
        self.process.stdin.write(request)
        self.process.stdin.flush()
        reply = self.process.stdout.readline().strip().split()
        expected = 'F' if terminal else 'V'
        minimum_length = 4 if terminal else 3
        if (len(reply) < minimum_length or reply[0] != expected or
                int(reply[1]) != self.sequence):
            raise RuntimeError('native validator protocol error: {}'.format(reply))
        self.sequence += 1
        self.query_count += 1
        return reply[2] == '1'

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.write('X\n')
                self.process.stdin.flush()
                self.process.wait(timeout=3.0)
            except Exception:
                self.process.kill()
                self.process.wait()
        self.log.close()


class LocalTree:
    """A tree containing this arm's six joint values and shared topology only."""
    def __init__(self, root):
        self.states = [vector6(root, 'tree root').copy()]
        self.parents = [-1]

    def squared_distances(self, target):
        target = vector6(target, 'tree target')
        return np.asarray([
            float(np.dot(state - target, state - target))
            for state in self.states
        ])

    def append(self, state, parent):
        if parent < 0 or parent >= len(self.states):
            raise RuntimeError('invalid shared parent index {}'.format(parent))
        self.states.append(vector6(state, 'new tree state').copy())
        self.parents.append(parent)
        return len(self.states) - 1

    def root_path(self, index):
        result = []
        while index >= 0:
            result.append(self.states[index])
            index = self.parents[index]
        result.reverse()
        return np.asarray(result)


def local_joint_vector(config, section, role):
    offset = 0 if role == 'left' else 6
    return vector6([
        float(config[section]['joint_{}'.format(offset + index)])
        for index in range(1, 7)
    ], '{} {}'.format(section, role))


def load_local_path(path):
    values = np.loadtxt(str(path), dtype=float)
    return path6(values, str(path))


def state_valid(channel, validator, role, local, event):
    local = vector6(local, event)
    reply = channel.exchange(event + '_STATE', {'q_local': local.tolist()})
    peer = vector6(reply['q_local'], event + ' peer')
    own = bool(validator.valid(temporary_pair(role, local, peer)))
    consensus = channel.exchange(event + '_RESULT', {'valid': own})
    return own and bool(consensus['valid'])


def edge_valid(channel, validator, role, local_from, local_to,
               resolution, event):
    local_from = vector6(local_from, event + ' from')
    local_to = vector6(local_to, event + ' to')
    reply = channel.exchange(event + '_STATE', {
        'q_from_local': local_from.tolist(),
        'q_to_local': local_to.tolist(),
    })
    peer_from = vector6(reply['q_from_local'], event + ' peer from')
    peer_to = vector6(reply['q_to_local'], event + ' peer to')
    local_delta = local_to - local_from
    peer_delta = peer_to - peer_from
    sample_count = max(1, int(math.ceil(max(
        float(np.max(np.abs(local_delta))),
        float(np.max(np.abs(peer_delta)))) / resolution)))
    own = True
    for sample in range(1, sample_count + 1):
        ratio = float(sample) / sample_count
        pair = temporary_pair(
            role,
            local_from + ratio * local_delta,
            peer_from + ratio * peer_delta)
        if not validator.valid(pair):
            own = False
            break
    consensus = channel.exchange(event + '_RESULT', {
        'valid': own,
        'samples': sample_count,
    })
    return own and bool(consensus['valid'])


def shared_nearest(channel, tree, target, event):
    local_distances = tree.squared_distances(target)
    reply = channel.exchange(event, {
        'node_count': len(tree.states),
        'squared_distance_local': local_distances.tolist(),
    })
    if int(reply['node_count']) != len(tree.states):
        raise RuntimeError('left/right tree topology diverged')
    peer_distances = np.asarray(reply['squared_distance_local'], dtype=float)
    if peer_distances.shape != local_distances.shape:
        raise RuntimeError('left/right distance table shapes differ')
    return int(np.argmin(local_distances + peer_distances))


def extend_once(channel, validator, role, tree, target, step,
                resolution, event):
    target = vector6(target, event + ' target')
    near_index = shared_nearest(channel, tree, target, event + '_NEAREST')
    q_near = tree.states[near_index]
    delta = target - q_near
    local_distance2 = float(np.dot(delta, delta))
    reply = channel.exchange(event + '_DISTANCE', {
        'squared_distance_local': local_distance2,
    })
    paired_distance = math.sqrt(max(
        0.0, local_distance2 + float(reply['squared_distance_local'])))
    reached = paired_distance <= step
    scale = 1.0 if reached or paired_distance < 1.0e-15 else step / paired_distance
    q_new = q_near + scale * delta
    if not edge_valid(
            channel, validator, role, q_near, q_new,
            resolution, event + '_EDGE'):
        return 'trapped', near_index
    new_index = tree.append(q_new, near_index)
    return ('reached' if reached else 'advanced'), new_index


def connect_toward(channel, validator, role, tree, target, step,
                   resolution, event):
    for extension in range(10000):
        status, index = extend_once(
            channel, validator, role, tree, target, step, resolution,
            '{}_{:05d}'.format(event, extension))
        if status != 'advanced':
            return status, index
    raise RuntimeError('connection extension guard reached')


def simplify_path(channel, validator, role, local_path,
                  resolution, attempts, seed):
    result = [row.copy() for row in path6(local_path, 'planned path')]
    generator = np.random.RandomState(seed)
    for attempt in range(attempts):
        if len(result) <= 2:
            break
        first = int(generator.randint(0, len(result) - 2))
        second = int(generator.randint(first + 2, len(result)))
        if edge_valid(
                channel, validator, role, result[first], result[second],
                resolution, 'SHORTCUT_{:04d}'.format(attempt)):
            result = result[:first + 1] + result[second:]
    return np.asarray(result)


def validate_paired_path(channel, validator, role, local_path,
                         resolution, event):
    local_path = path6(local_path, event)
    reply = channel.exchange(event + '_PATH', {
        'q_path_local': local_path.tolist(),
        'state_count': int(local_path.shape[0]),
    })
    peer_path = path6(reply['q_path_local'], event + ' peer path')
    if peer_path.shape[0] != local_path.shape[0]:
        raise RuntimeError('left/right planned path lengths differ')
    own = True
    for segment in range(local_path.shape[0] - 1):
        local_delta = local_path[segment + 1] - local_path[segment]
        peer_delta = peer_path[segment + 1] - peer_path[segment]
        count = max(1, int(math.ceil(max(
            float(np.max(np.abs(local_delta))),
            float(np.max(np.abs(peer_delta)))) / resolution)))
        for sample in range(count + 1):
            ratio = float(sample) / count
            if not validator.valid(temporary_pair(
                    role,
                    local_path[segment] + ratio * local_delta,
                    peer_path[segment] + ratio * peer_delta)):
                own = False
                break
        if not own:
            break
    consensus = channel.exchange(event + '_RESULT', {'valid': own})
    return own and bool(consensus['valid']), peer_path


def dual_tree_connect(channel, validator, role, local_start, local_goal,
                      config, args, guide=None):
    if not state_valid(
            channel, validator, role, local_start, 'START_VALIDITY'):
        raise RuntimeError('start pair is invalid')
    if not state_valid(
            channel, validator, role, local_goal, 'GOAL_VALIDITY'):
        raise RuntimeError('goal pair is invalid')

    lower = np.array([-math.pi] * 3 + [-2.0 * math.pi] * 3)
    upper = np.array([math.pi] * 3 + [2.0 * math.pi] * 3)
    if role == 'right':
        excursion = float(config.get('goal_generator', {}).get(
            'max_right_wrist_excursion_rad', 2.0 * math.pi))
        lower[3:] = np.maximum(lower[3:], local_start[3:] - excursion)
        upper[3:] = np.minimum(upper[3:], local_start[3:] + excursion)
    generator = np.random.RandomState(args.seed + (0 if role == 'left' else 1))

    start_tree = LocalTree(local_start)
    goal_tree = LocalTree(local_goal)
    tree_a = start_tree
    tree_b = goal_tree
    a_is_start = True
    started = time.monotonic()
    goal_period = max(1, int(round(1.0 / args.goal_bias)))
    for iteration in range(args.max_iterations):
        use_goal = iteration > 0 and iteration % goal_period == 0
        use_guide = guide is not None and not use_goal
        if use_goal:
            local_sample = local_goal if a_is_start else local_start
        elif use_guide:
            if guide.shape[0] <= 2:
                guide_index = 1 if a_is_start else 0
            else:
                depth = 1 + ((iteration // 16) % (guide.shape[0] - 2))
                guide_index = depth if a_is_start else guide.shape[0] - 1 - depth
            local_sample = guide[guide_index]
        else:
            local_sample = generator.uniform(lower, upper)
        sample_reply = channel.exchange('SAMPLE_{:06d}'.format(iteration), {
            'q_local': vector6(local_sample, 'sample').tolist(),
            'goal_biased': use_goal,
            'guide_biased': use_guide,
        })
        vector6(sample_reply['q_local'], 'peer sample')

        status_a, index_a = extend_once(
            channel, validator, role, tree_a, local_sample,
            args.step, args.edge_resolution,
            'EXPAND_A_{:06d}'.format(iteration))
        if status_a != 'trapped':
            local_target = tree_a.states[index_a]
            status_b, index_b = connect_toward(
                channel, validator, role, tree_b, local_target,
                args.step, args.edge_resolution,
                'CONNECT_B_{:06d}'.format(iteration))
            if status_b == 'reached':
                path_a = tree_a.root_path(index_a)
                path_b = tree_b.root_path(index_b)
                if a_is_start:
                    raw_path = np.vstack([path_a, path_b[-2::-1]])
                else:
                    raw_path = np.vstack([path_b, path_a[-2::-1]])
                path = simplify_path(
                    channel, validator, role, raw_path,
                    args.edge_resolution, args.shortcut_attempts,
                    args.seed + 1000)
                valid, peer_path = validate_paired_path(
                    channel, validator, role, path,
                    args.edge_resolution, 'FINAL_VALIDATION')
                if not valid:
                    raise RuntimeError('final paired path failed validation')
                return path, peer_path, {
                    'iterations': iteration + 1,
                    'wall_time_s': time.monotonic() - started,
                    'states_before_simplification': int(raw_path.shape[0]),
                    'states_after_simplification': int(path.shape[0]),
                    'tree_start_size': len(start_tree.states),
                    'tree_goal_size': len(goal_tree.states),
                }
        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start
    raise RuntimeError('dual-tree connection failed after {} iterations'.format(
        args.max_iterations))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=('left', 'right'), required=True)
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--bind-host', default='0.0.0.0')
    parser.add_argument('--peer-host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=55105)
    parser.add_argument('--socket-timeout', type=float, default=180.0)
    parser.add_argument('--seed', type=int, default=20260728)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--goal-bias', type=float, default=0.20)
    parser.add_argument('--step', type=float, default=0.25)
    parser.add_argument('--edge-resolution', type=float, default=0.005)
    parser.add_argument('--shortcut-attempts', type=int, default=80)
    parser.add_argument('--endpoint-local-path', type=Path)
    parser.add_argument('--guide-local-path', type=Path)
    parser.add_argument('--validator', type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(args.config.read_text())
    trace_path = args.output_dir / '{}_{}_socket_trace.jsonl'.format(
        args.phase, args.role)
    channel = JsonChannel(
        args.role, args.bind_host, args.peer_host, args.port,
        args.socket_timeout, trace_path)
    validator = NativeValidator(
        args.phase, args.config,
        args.output_dir / '{}_{}_native_validator.log'.format(
            args.phase, args.role),
        args.validator)
    started = time.monotonic()
    try:
        hello = channel.exchange('HELLO', {
            'protocol': PROTOCOL,
            'state_dimension': 6,
            'phase': args.phase,
        })
        if hello.get('protocol') != PROTOCOL or int(hello.get('state_dimension')) != 6:
            raise RuntimeError('peer protocol or dimension mismatch')

        guide = None
        if args.guide_local_path:
            guide = load_local_path(args.guide_local_path)
            guide_reply = channel.exchange('GUIDE_METADATA', {
                'rows': int(guide.shape[0]),
            })
            if int(guide_reply['rows']) != guide.shape[0]:
                raise RuntimeError('left/right guide path lengths differ')

        if args.endpoint_local_path:
            endpoints = load_local_path(args.endpoint_local_path)
            if endpoints.shape[0] < 2:
                raise RuntimeError('endpoint path needs at least two rows')
            local_start = endpoints[0]
            local_goal = endpoints[-1]
        else:
            local_start = local_joint_vector(config, 'Joint_init', args.role)
            local_goal = local_joint_vector(config, 'Joint_final', args.role)

        local_path, peer_path, metrics = dual_tree_connect(
            channel, validator, args.role, local_start, local_goal,
            config, args, guide)
        output_path = args.output_dir / '{}_{}_planned_path.txt'.format(
            args.phase, args.role)
        np.savetxt(str(output_path), local_path, fmt='%.17g')
        report = {
            'status': 'success',
            'protocol': PROTOCOL,
            'role': args.role,
            'phase': args.phase,
            'config': str(args.config),
            'planner': metrics,
            'state_dimension_local': 6,
            'state_dimension_transmitted': 6,
            'combined_dimension_internal_validation_only': 12,
            'local_tree_contains_peer_states': False,
            'peer_path_states_received_for_final_check': int(peer_path.shape[0]),
            'native_validator_queries': validator.query_count,
            'socket_messages': channel.sequence,
            'socket_bytes_sent': channel.bytes_sent,
            'socket_bytes_received': channel.bytes_received,
            'peer': channel.peer_address,
            'wall_time_s': time.monotonic() - started,
        }
        report_path = args.output_dir / '{}_{}_report.json'.format(
            args.phase, args.role)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        validator.close()
        channel.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('CM Socket planner error: {}'.format(error), file=sys.stderr)
        raise
