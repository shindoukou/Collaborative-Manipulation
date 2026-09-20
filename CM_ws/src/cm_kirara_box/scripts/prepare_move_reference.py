#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', type=Path, required=True)
    return parser.parse_args()


def selected_indices(count, stride):
    values = list(range(0, count, stride))
    if values[-1] != count - 1:
        values.append(count - 1)
    return values


def sampled_edges(knots, samples_per_edge=31):
    rows = []
    for index in range(len(knots) - 1):
        for sample in range(samples_per_edge):
            if index and sample == 0:
                continue
            tau = float(sample) / float(samples_per_edge - 1)
            smooth = tau * tau * (3.0 - 2.0 * tau)
            rows.append((1.0 - smooth) * knots[index] + smooth * knots[index + 1])
    return np.asarray(rows)


def main():
    args = parse_args()
    workspace = args.workspace.resolve()
    package = workspace / 'src' / 'cm_kirara_box'
    source = (package / 'generated' /
              'run_20260809_134102_move_plus10mm_stage2fixed')
    left = np.atleast_2d(np.loadtxt(str(
        source / 'move_left_samples_20260809_134102.txt')))
    right = np.atleast_2d(np.loadtxt(str(
        source / 'move_right_samples_20260809_134102.txt')))
    if left.shape != right.shape or left.shape[1] != 6:
        raise RuntimeError('invalid source pair')
    output = package / 'generated' / 'paper_move_reference_latest_valid'
    output.mkdir(parents=True, exist_ok=True)
    validator = workspace / 'devel/lib/cm_kirara_box/CM_KIRARA_BOX_MOVE'
    config = source / 'move_config.yaml'
    tested = []
    selected = None
    for stride in (200, 160, 120, 100, 80, 60, 40, 30, 20, 10, 5, 2, 1):
        indices = selected_indices(left.shape[0], stride)
        left_knots = left[indices]
        right_knots = right[indices]
        left_dense = sampled_edges(left_knots)
        right_dense = sampled_edges(right_knots)
        left_test = output / 'candidate_left_samples.txt'
        right_test = output / 'candidate_right_samples.txt'
        np.savetxt(str(left_test), left_dense, fmt='%.12f')
        np.savetxt(str(right_test), right_dense, fmt='%.12f')
        environment = os.environ.copy()
        environment['CM_KIRARA_BOX_MOVE_CONFIG'] = str(config)
        process = subprocess.run(
            [str(validator), '--validate-samples',
             str(left_test), str(right_test)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, env=environment)
        record = {
            'source_stride': stride,
            'knot_count': len(indices),
            'validation_sample_count': int(left_dense.shape[0]),
            'native_valid': process.returncode == 0,
            'validator_tail': process.stdout.splitlines()[-4:],
        }
        tested.append(record)
        if process.returncode == 0:
            selected = record
            np.savetxt(
                str(output / 'move_left_planned_path.txt'),
                left_knots, fmt='%.12f')
            np.savetxt(
                str(output / 'move_right_planned_path.txt'),
                right_knots, fmt='%.12f')
            break
    for temporary in ('candidate_left_samples.txt',
                      'candidate_right_samples.txt'):
        path = output / temporary
        if path.exists():
            path.unlink()
    if selected is None:
        raise RuntimeError('no native-valid sparse reference found')
    report = {
        'source': str(source),
        'source_samples': int(left.shape[0]),
        'method': 'select existing samples and validate every zero-velocity cubic edge',
        'collision_repair_or_projection_used': False,
        'tested': tested,
        'selected': selected,
    }
    (output / 'reference_preparation_report.json').write_text(
        json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
