#!/usr/bin/env python3
"""Sample TCST quintic KIRARA coefficients into plain six-joint rows."""

import argparse
import json
from pathlib import Path

import numpy as np


def read_coefficients(path):
    segments = []
    with Path(path).open('r') as stream:
        for line_number, line in enumerate(stream, 1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            values = [float(value) for value in line.split()]
            if len(values) != 37:
                raise ValueError('{}:{} requires 37 values, got {}'.format(
                    path, line_number, len(values)))
            duration = values[0]
            if duration <= 0.0:
                raise ValueError('{}:{} has non-positive duration'.format(path, line_number))
            segments.append((duration, np.asarray(values[1:]).reshape(6, 6)))
    if not segments:
        raise ValueError('{} contains no KIRARA segments'.format(path))
    return segments


def sample_segments(segments, sample_dt):
    rows = []
    times = []
    elapsed = 0.0
    for segment_index, (duration, coefficients) in enumerate(segments):
        count = max(1, int(np.ceil(duration / sample_dt)))
        local_times = np.linspace(0.0, duration, count + 1)
        if segment_index:
            local_times = local_times[1:]
        for local_time in local_times:
            powers = np.asarray([local_time ** power for power in range(6)])
            rows.append(coefficients.dot(powers))
            times.append(elapsed + local_time)
        elapsed += duration
    return np.asarray(times), np.asarray(rows)


def kinematic_extrema(segments):
    max_velocity = np.zeros(6)
    max_acceleration = np.zeros(6)
    for duration, coefficients in segments:
        count = max(1001, int(np.ceil(duration / 0.001)) + 1)
        local_time = np.linspace(0.0, duration, count)
        velocity = sum(
            power * coefficients[:, power, None] *
            local_time[None, :] ** (power - 1)
            for power in range(1, 6))
        acceleration = sum(
            power * (power - 1) * coefficients[:, power, None] *
            local_time[None, :] ** (power - 2)
            for power in range(2, 6))
        max_velocity = np.maximum(max_velocity, np.max(np.abs(velocity), axis=1))
        max_acceleration = np.maximum(
            max_acceleration, np.max(np.abs(acceleration), axis=1))
    return max_velocity, max_acceleration


def time_scale_segments(segments, speed_scale):
    scaled = []
    powers = speed_scale ** np.arange(6)
    for duration, coefficients in segments:
        scaled.append((duration / speed_scale, coefficients * powers[None, :]))
    return scaled


def write_coefficients(path, segments):
    with Path(path).open('w') as stream:
        stream.write('# duration then six rows of quintic coefficients a0..a5\n')
        for duration, coefficients in segments:
            values = np.concatenate(([duration], coefficients.reshape(-1)))
            stream.write(' '.join('{:.17g}'.format(value) for value in values) + '\n')


def sample_pair(left_path, right_path, left_output, right_output, sample_dt,
                requested_speed_scale=1.0, max_velocity=1.2,
                max_acceleration=3.0, rewrite_coefficients=False):
    left_segments = read_coefficients(left_path)
    right_segments = read_coefficients(right_path)
    left_durations = np.asarray([item[0] for item in left_segments])
    right_durations = np.asarray([item[0] for item in right_segments])
    if left_durations.shape != right_durations.shape or not np.allclose(
            left_durations, right_durations, atol=1.0e-9, rtol=0.0):
        raise ValueError('left/right KIRARA segment durations are not synchronized')

    left_velocity, left_acceleration = kinematic_extrema(left_segments)
    right_velocity, right_acceleration = kinematic_extrema(right_segments)
    original_velocity = max(np.max(left_velocity), np.max(right_velocity))
    original_acceleration = max(
        np.max(left_acceleration), np.max(right_acceleration))
    velocity_scale = (max_velocity / original_velocity
                      if original_velocity > 0.0 else requested_speed_scale)
    acceleration_scale = (np.sqrt(max_acceleration / original_acceleration)
                          if original_acceleration > 0.0 else requested_speed_scale)
    speed_scale = min(requested_speed_scale, velocity_scale, acceleration_scale)
    if speed_scale <= 0.0:
        raise ValueError('speed scale and kinematic limits must be positive')
    left_segments = time_scale_segments(left_segments, speed_scale)
    right_segments = time_scale_segments(right_segments, speed_scale)
    if rewrite_coefficients:
        write_coefficients(left_path, left_segments)
        write_coefficients(right_path, right_segments)

    times, left = sample_segments(left_segments, sample_dt)
    right_times, right = sample_segments(right_segments, sample_dt)
    if times.shape != right_times.shape or not np.allclose(times, right_times):
        raise ValueError('left/right sampled time grids differ')
    np.savetxt(str(left_output), left, fmt='%.17g')
    np.savetxt(str(right_output), right, fmt='%.17g')
    return {
        'duration_s': float(times[-1]),
        'sample_dt_max_s': float(sample_dt),
        'sample_count': int(times.size),
        'requested_speed_scale': float(requested_speed_scale),
        'applied_speed_scale': float(speed_scale),
        'max_joint_velocity_rad_s': float(original_velocity * speed_scale),
        'max_joint_acceleration_rad_s2': float(
            original_acceleration * speed_scale * speed_scale),
        'velocity_limit_rad_s': float(max_velocity),
        'acceleration_limit_rad_s2': float(max_acceleration),
        'start_q': np.concatenate([left[0], right[0]]).tolist(),
        'goal_q': np.concatenate([left[-1], right[-1]]).tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('left_coeff', type=Path)
    parser.add_argument('right_coeff', type=Path)
    parser.add_argument('left_output', type=Path)
    parser.add_argument('right_output', type=Path)
    parser.add_argument('--dt', type=float, default=0.01)
    parser.add_argument('--speed-scale', type=float, default=1.0)
    parser.add_argument('--metadata', type=Path)
    args = parser.parse_args()
    metadata = sample_pair(
        args.left_coeff, args.right_coeff,
        args.left_output, args.right_output, args.dt,
        requested_speed_scale=args.speed_scale)
    if args.metadata:
        args.metadata.write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
