#!/usr/bin/env python3
"""Per-arm TCP sampling planner and KIRARA worker matching the paper exchange rules."""

import argparse
import hashlib
import json
import math
import socket
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml


PACKAGE = Path('/home/mas-2/3rd_party/CM_ws/src/cm_kirara_ball')
DEVEL = Path('/home/mas-2/3rd_party/CM_ws/devel/lib/cm_kirara_ball')


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
            'wall_time': time.time(),
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
            raise RuntimeError('socket sequence mismatch: sent {} {}, got {}'.format(
                self.sequence, event, reply))
        self.trace.write(json.dumps({
            'direction': 'exchange',
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
    def __init__(self, phase, config_path, log_path):
        executable = DEVEL / ('CM_KIRARA_BALL_{}_VALIDATOR'.format(
            phase.upper()))
        self.log = log_path.open('w')
        self.process = subprocess.Popen(
            [str(executable), str(config_path)],
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

    def valid(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (12,) or not np.all(np.isfinite(q)):
            return False
        line = 'Q {} {}\n'.format(
            self.sequence, ' '.join('{:.17g}'.format(value) for value in q))
        self.process.stdin.write(line)
        self.process.stdin.flush()
        reply = self.process.stdout.readline().strip().split()
        if len(reply) != 3 or reply[0] != 'V' or int(reply[1]) != self.sequence:
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


class Tree:
    def __init__(self, root):
        self.states = [np.asarray(root, dtype=float).copy()]
        self.parents = [-1]

    def nearest(self, target):
        distances = [np.linalg.norm(state - target) for state in self.states]
        return int(np.argmin(distances))

    def append(self, state, parent):
        self.states.append(np.asarray(state, dtype=float).copy())
        self.parents.append(parent)
        return len(self.states) - 1

    def root_path(self, index):
        result = []
        while index >= 0:
            result.append(self.states[index])
            index = self.parents[index]
        result.reverse()
        return result


def joint_vector(config, section):
    return np.array([
        float(config[section]['joint_{}'.format(index)])
        for index in range(1, 13)
    ])


def local_half(role, full):
    return np.asarray(full[:6] if role == 'left' else full[6:], dtype=float)


def full_pair(role, local, peer):
    if role == 'left':
        return np.r_[local, peer]
    return np.r_[peer, local]


def full_path(role, local_path, peer_path):
    local_path = np.asarray(local_path, dtype=float)
    peer_path = np.asarray(peer_path, dtype=float)
    if local_path.shape != peer_path.shape:
        raise RuntimeError('left/right path shapes differ: {} vs {}'.format(
            local_path.shape, peer_path.shape))
    return (np.c_[local_path, peer_path] if role == 'left'
            else np.c_[peer_path, local_path])


def paired_valid(channel, validator, q, event):
    local = bool(validator.valid(q))
    peer = channel.exchange(event, {'valid': local})
    return local and bool(peer['valid'])


def edge_valid(channel, validator, q0, q1, resolution, event):
    delta = np.asarray(q1) - np.asarray(q0)
    count = max(1, int(math.ceil(np.max(np.abs(delta)) / resolution)))
    local = True
    for index in range(1, count + 1):
        q = q0 + delta * (float(index) / count)
        if not validator.valid(q):
            local = False
            break
    peer = channel.exchange(event, {'valid': local, 'samples': count})
    return local and bool(peer['valid'])


def steer(q_from, q_to, step):
    delta = q_to - q_from
    distance = float(np.linalg.norm(delta))
    if distance <= step:
        return q_to.copy(), True
    return q_from + delta * (step / distance), False


def extend_tree(channel, validator, tree, target, step, resolution, tag):
    near_index = tree.nearest(target)
    q_new, reached = steer(tree.states[near_index], target, step)
    if not edge_valid(
            channel, validator, tree.states[near_index], q_new,
            resolution, '{}_EDGE'.format(tag)):
        return 'trapped', near_index
    new_index = tree.append(q_new, near_index)
    return ('reached' if reached else 'advanced'), new_index


def connect_tree(channel, validator, tree, target, step, resolution, tag):
    while True:
        status, index = extend_tree(
            channel, validator, tree, target, step, resolution, tag)
        if status != 'advanced':
            return status, index


def simplify_path(channel, validator, path, resolution, attempts):
    path = [np.asarray(q, dtype=float) for q in path]
    rng = np.random.RandomState(20260810)
    for attempt in range(attempts):
        if len(path) <= 2:
            break
        first = int(rng.randint(0, len(path) - 2))
        second = int(rng.randint(first + 2, len(path)))
        if edge_valid(
                channel, validator, path[first], path[second], resolution,
                'SHORTCUT_{:04d}'.format(attempt)):
            path = path[:first + 1] + path[second:]
    return np.asarray(path)


def distributed_planning(channel, validator, role, q_start, q_goal, config, args,
                         guide_path=None):
    if not paired_valid(channel, validator, q_start, 'START_VALIDITY'):
        raise RuntimeError('distributed planning start state is invalid')
    if not paired_valid(channel, validator, q_goal, 'GOAL_VALIDITY'):
        raise RuntimeError('distributed planning goal state is invalid')

    local_start = local_half(role, q_start)
    local_goal = local_half(role, q_goal)
    lower = np.array([-math.pi] * 3 + [-2.0 * math.pi] * 3)
    upper = np.array([math.pi] * 3 + [2.0 * math.pi] * 3)
    if role == 'right':
        excursion = float(config.get('goal_generator', {}).get(
            'max_right_wrist_excursion_rad', 2.0 * math.pi))
        lower[3:] = np.maximum(lower[3:], local_start[3:] - excursion)
        upper[3:] = np.minimum(upper[3:], local_start[3:] + excursion)
    rng = np.random.RandomState(args.seed + (0 if role == 'left' else 1))

    start_tree = Tree(q_start)
    goal_tree = Tree(q_goal)
    tree_a = start_tree
    tree_b = goal_tree
    a_is_start = True
    started = time.monotonic()
    solved_path = None
    solved_iteration = None
    for iteration in range(args.max_iterations):
        goal_period = max(1, int(round(1.0 / args.goal_bias)))
        if guide_path is not None:
            goal_period = max(goal_period, 50)
        use_goal = iteration > 0 and iteration % goal_period == 0
        use_guide = guide_path is not None and not use_goal
        guide_index = None
        if use_goal:
            local_sample = local_goal.copy()
        elif use_guide:
            if guide_path.shape[0] <= 2:
                guide_index = 1 if a_is_start else 0
            else:
                # Revisit every guide waypoint long enough for several
                # Range-limited extensions from each search-tree root.
                depth = 1 + ((iteration // 16) % (guide_path.shape[0] - 2))
                guide_index = (depth if a_is_start
                               else guide_path.shape[0] - 1 - depth)
            local_sample = local_half(role, guide_path[guide_index])
        else:
            local_sample = rng.uniform(lower, upper)
        peer_reply = channel.exchange('PLANNING_SAMPLE', {
            'iteration': iteration,
            'q_local': local_sample.tolist(),
            'goal_biased': use_goal,
            'guide_biased': use_guide,
            'guide_index': guide_index,
        })
        peer_sample = np.asarray(peer_reply['q_local'], dtype=float)
        q_random = full_pair(role, local_sample, peer_sample)

        status_a, index_a = extend_tree(
            channel, validator, tree_a, q_random,
            args.step, args.edge_resolution, 'PLANNING_EXTEND_A')
        if status_a != 'trapped':
            q_new = tree_a.states[index_a]
            status_b, index_b = connect_tree(
                channel, validator, tree_b, q_new,
                args.step, args.edge_resolution, 'PLANNING_CONNECT_B')
            if status_b == 'reached':
                path_a = tree_a.root_path(index_a)
                path_b = tree_b.root_path(index_b)
                if a_is_start:
                    solved_path = path_a + list(reversed(path_b))[1:]
                else:
                    solved_path = path_b + list(reversed(path_a))[1:]
                solved_iteration = iteration
                break
        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start

    if solved_path is None:
        raise RuntimeError('distributed planning failed after {} iterations'.format(
            args.max_iterations))
    path = simplify_path(
        channel, validator, solved_path, args.edge_resolution,
        args.shortcut_attempts)
    digest = hashlib.sha256(path.astype('<f8').tobytes()).hexdigest()
    peer = channel.exchange('PLANNING_PATH_DIGEST', {
        'sha256': digest, 'states': int(path.shape[0])})
    if peer['sha256'] != digest or int(peer['states']) != path.shape[0]:
        raise RuntimeError('distributed trees diverged between arms')
    return path, {
        'iterations': solved_iteration + 1,
        'wall_time_s': time.monotonic() - started,
        'states_before_simplification': len(solved_path),
        'states_after_simplification': int(path.shape[0]),
        'tree_start_size': len(start_tree.states),
        'tree_goal_size': len(goal_tree.states),
    }


def minimum_durations(path, velocity_limit, acceleration_limit):
    delta = np.diff(path, axis=0)
    velocity = 1.875 * np.max(np.abs(delta), axis=1) / velocity_limit
    acceleration = np.sqrt(
        5.773502691896258 * np.max(np.abs(delta), axis=1) /
        acceleration_limit)
    return np.maximum(0.20, np.maximum(velocity, acceleration))


def local_cost_gradient(path, times, reference, weights, q_min, q_max,
                        velocity_limit):
    delta = np.diff(path, axis=0)
    inv_time = 1.0 / times
    path_cost = weights['path'] * float(np.sum(delta * delta))
    energy_cost = weights['energy'] * float(np.sum(
        np.sum(delta * delta, axis=1) * inv_time))
    reference_error = path - reference
    reference_cost = 0.5 * weights['reference'] * float(np.sum(
        reference_error * reference_error))
    low_violation = np.maximum(q_min - path, 0.0)
    high_violation = np.maximum(path - q_max, 0.0)
    position_cost = 0.5 * weights['joint'] * float(np.sum(
        low_violation * low_violation + high_violation * high_violation))
    speed = 1.875 * np.abs(delta) * inv_time[:, None]
    velocity_violation = np.maximum(speed - velocity_limit, 0.0)
    velocity_cost = 0.5 * weights['velocity'] * float(np.sum(
        velocity_violation * velocity_violation))
    time_cost = 0.5 * weights['time'] * float(np.sum(times))

    gradient_q = np.zeros_like(path)
    for segment in range(delta.shape[0]):
        base = 2.0 * (
            weights['path'] + weights['energy'] * inv_time[segment]) * delta[segment]
        gradient_q[segment] -= base
        gradient_q[segment + 1] += base
    gradient_q += weights['reference'] * reference_error
    gradient_q += weights['joint'] * (-low_violation + high_violation)
    gradient_q[0] = 0.0
    gradient_q[-1] = 0.0

    gradient_time = (
        -weights['energy'] * np.sum(delta * delta, axis=1) /
        (times * times) + 0.5 * weights['time'])
    return {
        'total': path_cost + energy_cost + reference_cost + position_cost +
                 velocity_cost + time_cost,
        'path': path_cost,
        'energy': energy_cost,
        'reference': reference_cost,
        'joint': position_cost,
        'velocity': velocity_cost,
        'time': time_cost,
    }, gradient_q, gradient_time


def validate_full_path(channel, validator, q_path, resolution, prefix):
    local = True
    for segment in range(q_path.shape[0] - 1):
        delta = q_path[segment + 1] - q_path[segment]
        count = max(1, int(math.ceil(np.max(np.abs(delta)) / resolution)))
        for sample in range(count + 1):
            q = q_path[segment] + delta * (float(sample) / count)
            if not validator.valid(q):
                local = False
                break
        if not local:
            break
    peer = channel.exchange(prefix, {'valid': local})
    return local and bool(peer['valid'])


def write_quintic_coefficients(path, times, output_path):
    with output_path.open('w') as output:
        output.write('# one segment per line\n')
        output.write('# duration, then joint1[a0..a5], ..., joint6[a0..a5]\n')
        for segment, duration in enumerate(times):
            delta = path[segment + 1] - path[segment]
            values = [float(duration)]
            for joint in range(6):
                values.extend([
                    path[segment, joint], 0.0, 0.0,
                    10.0 * delta[joint] / duration ** 3,
                    -15.0 * delta[joint] / duration ** 4,
                    6.0 * delta[joint] / duration ** 5,
                ])
            output.write(' '.join('{:.17g}'.format(value) for value in values) + '\n')


def distributed_kirara(channel, validator, role, local_path, peer_path, args):
    local_path = np.asarray(local_path, dtype=float)
    peer_path = np.asarray(peer_path, dtype=float)
    reference = local_path.copy()
    local_min = minimum_durations(
        local_path, args.velocity_limit, args.acceleration_limit)
    peer_reply = channel.exchange('KIRARA_INITIAL_DURATION', {
        'minimum_s': local_min.tolist()})
    times = np.maximum(local_min, np.asarray(peer_reply['minimum_s'], dtype=float))
    weights = {
        'path': args.weight_path,
        'energy': args.weight_energy,
        'reference': args.weight_reference,
        'joint': args.weight_joint,
        'velocity': args.weight_velocity,
        'time': args.weight_time,
    }
    q_min = np.array([-math.pi] * 3 + [-2.0 * math.pi] * 3)
    q_max = -q_min
    initial_cost = None
    final_cost = None
    accepted_geometry_updates = 0
    rejected_geometry_updates = 0
    learning_rate_q = args.learning_rate_q
    started = time.monotonic()
    for iteration in range(args.kirara_iterations):
        cost, gradient_q, gradient_time = local_cost_gradient(
            local_path, times, reference, weights, q_min, q_max,
            args.velocity_limit)
        if initial_cost is None:
            initial_cost = cost
        peer_gradient = channel.exchange('KIRARA_TIME_GRADIENT', {
            'iteration': iteration,
            'dJdT_local': gradient_time.tolist(),
            'local_cost': cost,
        })
        shared_gradient = 0.5 * (
            gradient_time + np.asarray(peer_gradient['dJdT_local'], dtype=float))

        if iteration % args.geometry_update_period == 0 and local_path.shape[0] > 2:
            proposal = local_path.copy()
            proposal[1:-1] -= learning_rate_q * gradient_q[1:-1]
            proposal = np.minimum(np.maximum(proposal, q_min), q_max)
            peer_proposal = channel.exchange('KIRARA_COEFFICIENT_PROPOSAL', {
                'iteration': iteration,
                'q_local': proposal.tolist(),
            })
            peer_candidate = np.asarray(peer_proposal['q_local'], dtype=float)
            candidate_full = full_path(role, proposal, peer_candidate)
            if validate_full_path(
                    channel, validator, candidate_full,
                    args.edge_resolution, 'KIRARA_CONSTRAINT_CHECK'):
                local_path = proposal
                peer_path = peer_candidate
                accepted_geometry_updates += 1
            else:
                rejected_geometry_updates += 1
                learning_rate_q *= 0.5

        local_min = minimum_durations(
            local_path, args.velocity_limit, args.acceleration_limit)
        peer_min = channel.exchange('KIRARA_KINEMATIC_BOUND', {
            'iteration': iteration,
            'minimum_s': local_min.tolist(),
        })
        shared_min = np.maximum(
            local_min, np.asarray(peer_min['minimum_s'], dtype=float))
        times = np.maximum(shared_min, times - args.learning_rate_time * shared_gradient)
        final_cost = cost
        if float(np.max(np.abs(shared_gradient))) < args.gradient_tolerance:
            break

    final_peer = channel.exchange('KIRARA_FINAL_PATH', {
        'q_local': local_path.tolist(), 'times': times.tolist()})
    peer_path = np.asarray(final_peer['q_local'], dtype=float)
    peer_times = np.asarray(final_peer['times'], dtype=float)
    if np.max(np.abs(peer_times - times)) > 1.0e-9:
        raise RuntimeError('shared time segments diverged between arms')
    q_full = full_path(role, local_path, peer_path)
    if not validate_full_path(
            channel, validator, q_full, args.edge_resolution,
            'KIRARA_FINAL_NATIVE_VALIDATION'):
        raise RuntimeError('distributed KIRARA result failed native validation')
    return local_path, times, {
        'iterations': iteration + 1,
        'wall_time_s': time.monotonic() - started,
        'initial_local_cost': initial_cost,
        'final_local_cost': final_cost,
        'accepted_geometry_updates': accepted_geometry_updates,
        'rejected_geometry_updates': rejected_geometry_updates,
        'shared_duration_s': float(np.sum(times)),
        'time_gradient_exchange_rule': '0.5*(dJ_i/dT_i+dJ_peer/dT_i)',
        'weights': weights,
    }


def load_local_input(role, path):
    values = np.loadtxt(str(path), dtype=float)
    if values.ndim == 1:
        values = values.reshape(1, -1)
    if values.shape[1] == 12:
        return values[:, :6] if role == 'left' else values[:, 6:]
    if values.shape[1] != 6:
        raise RuntimeError('input path must contain 6 or 12 columns')
    return values


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=('left', 'right'), required=True)
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--bind-host', default='0.0.0.0')
    parser.add_argument('--peer-host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=55105)
    parser.add_argument('--socket-timeout', type=float, default=120.0)
    parser.add_argument('--seed', type=int, default=20260728)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--goal-bias', type=float, default=0.20)
    parser.add_argument('--step', type=float, default=0.25)
    parser.add_argument('--edge-resolution', type=float, default=0.005)
    parser.add_argument('--shortcut-attempts', type=int, default=80)
    parser.add_argument('--input-local-path', type=Path)
    parser.add_argument(
        '--endpoint-local-path', type=Path,
        help='use the first/last rows as distributed planning endpoints, but still plan')
    parser.add_argument(
        '--guide-local-path', type=Path,
        help='paper-style neighborhood guide samples; planning trees are still rebuilt')
    parser.add_argument('--planning-only', action='store_true')
    parser.add_argument('--weight-path', type=float, default=1.0)
    parser.add_argument('--weight-energy', type=float, default=1.0)
    parser.add_argument('--weight-reference', type=float, default=20.0)
    parser.add_argument('--weight-joint', type=float, default=100.0)
    parser.add_argument('--weight-velocity', type=float, default=100.0)
    parser.add_argument('--weight-time', type=float, default=0.05)
    parser.add_argument('--velocity-limit', type=float, default=1.2)
    parser.add_argument('--acceleration-limit', type=float, default=3.0)
    parser.add_argument('--kirara-iterations', type=int, default=120)
    parser.add_argument('--geometry-update-period', type=int, default=10)
    parser.add_argument('--learning-rate-q', type=float, default=0.0005)
    parser.add_argument('--learning-rate-time', type=float, default=0.01)
    parser.add_argument('--gradient-tolerance', type=float, default=1.0e-5)
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
            args.phase, args.role))
    started = time.monotonic()
    try:
        hello = channel.exchange('HELLO', {
            'protocol': 'CM_DISTRIBUTED_PLANNING_KIRARA_V1',
            'phase': args.phase,
        })
        if hello['protocol'] != 'CM_DISTRIBUTED_PLANNING_KIRARA_V1':
            raise RuntimeError('peer protocol mismatch')
        if args.input_local_path:
            local_path = load_local_input(args.role, args.input_local_path)
            peer = channel.exchange('INPUT_PATH', {
                'q_local': local_path.tolist()})
            peer_path = np.asarray(peer['q_local'], dtype=float)
            q_path = full_path(args.role, local_path, peer_path)
            if not validate_full_path(
                    channel, validator, q_path, args.edge_resolution,
                    'INPUT_PATH_NATIVE_VALIDATION'):
                raise RuntimeError('input paired path is not native-feasible')
            planning_metrics = {
                'mode': 'validated_input_path',
                'states_after_simplification': int(local_path.shape[0]),
            }
        else:
            guide_full = None
            if args.guide_local_path:
                guide_local = load_local_input(args.role, args.guide_local_path)
                guide_peer = channel.exchange('PLANNING_GUIDE_EXCHANGE', {
                    'q_local': guide_local.tolist(),
                })
                guide_full = full_path(
                    args.role, guide_local,
                    np.asarray(guide_peer['q_local'], dtype=float))
            if args.endpoint_local_path:
                endpoint_path = load_local_input(
                    args.role, args.endpoint_local_path)
                if endpoint_path.shape[0] < 2:
                    raise RuntimeError('endpoint path needs at least two rows')
                endpoint_peer = channel.exchange('PLANNING_ENDPOINT_EXCHANGE', {
                    'q_start_local': endpoint_path[0].tolist(),
                    'q_goal_local': endpoint_path[-1].tolist(),
                })
                q_start = full_pair(
                    args.role, endpoint_path[0],
                    np.asarray(endpoint_peer['q_start_local'], dtype=float))
                q_goal = full_pair(
                    args.role, endpoint_path[-1],
                    np.asarray(endpoint_peer['q_goal_local'], dtype=float))
            else:
                q_start = joint_vector(config, 'Joint_init')
                q_goal = joint_vector(config, 'Joint_final')
            q_path, planning_metrics = distributed_planning(
                channel, validator, args.role, q_start, q_goal, config, args,
                guide_path=guide_full)
            local_path = local_half(args.role, q_path.T).T
            peer_path = local_half(
                'right' if args.role == 'left' else 'left', q_path.T).T

        planning_output = args.output_dir / '{}_{}_planned_path.txt'.format(
            args.phase, args.role)
        np.savetxt(str(planning_output), local_path, fmt='%.17g')
        kirara_metrics = None
        if not args.planning_only:
            local_path, times, kirara_metrics = distributed_kirara(
                channel, validator, args.role, local_path, peer_path, args)
            write_quintic_coefficients(
                local_path, times,
                args.output_dir / '{}_{}_kirara.txt'.format(
                    args.phase, args.role))

        report = {
            'status': 'success',
            'protocol': 'CM_DISTRIBUTED_PLANNING_KIRARA_V1',
            'role': args.role,
            'phase': args.phase,
            'config': str(args.config),
            'planning': planning_metrics,
            'kirara': kirara_metrics,
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
        print('CM distributed planner error: {}'.format(error), file=sys.stderr)
        raise
