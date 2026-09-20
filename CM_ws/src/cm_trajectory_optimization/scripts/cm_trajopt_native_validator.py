#!/usr/bin/env python3
"""Adapter for the existing FCL trajectory-state validator executables."""

import collections
import subprocess

import numpy as np


def _reason_category(reason):
    for separator in (":", "="):
        if separator in reason:
            return reason.split(separator, 1)[0]
    return reason


def validate_sampled_pair(executable, phase, config_path, left_path, right_path):
    left = np.atleast_2d(np.loadtxt(str(left_path), dtype=float))
    right = np.atleast_2d(np.loadtxt(str(right_path), dtype=float))
    if left.shape != right.shape or left.ndim != 2 or left.shape[1] != 6:
        raise ValueError("native validation requires paired Nx6 sampled paths")
    process = subprocess.Popen(
        [str(executable), str(phase), str(config_path)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    ready = process.stdout.readline().strip()
    if not ready.startswith("READY "):
        process.kill()
        raise RuntimeError("native validator did not become ready: {}".format(ready))
    invalid = []
    reason_counts = collections.Counter()
    for sequence, (q_left, q_right) in enumerate(zip(left, right)):
        command = "F" if phase == "catch" and sequence + 1 == left.shape[0] else "D"
        values = " ".join("{:.17g}".format(value) for value in np.r_[q_left, q_right])
        process.stdin.write("{} {} {}\n".format(command, sequence, values))
        process.stdin.flush()
        reply = process.stdout.readline().strip()
        fields = reply.split(" ", 3)
        accepted = (len(fields) >= 3 and fields[0] == command
                    and fields[1] == str(sequence) and fields[2] == "1")
        if not accepted:
            reason = fields[3] if len(fields) >= 4 else reply
            reason_counts[_reason_category(reason)] += 1
            if len(invalid) < 20:
                invalid.append({"sample": sequence, "reply": reply})
    process.stdin.write("X\n")
    process.stdin.flush()
    return_code = process.wait(timeout=10.0)
    if return_code:
        raise RuntimeError("native validator exited with {}".format(return_code))
    invalid_count = int(sum(reason_counts.values()))
    return {
        "valid": invalid_count == 0,
        "sample_count": int(left.shape[0]),
        "invalid_sample_count": invalid_count,
        "reason_counts": dict(reason_counts),
        "first_invalid_samples": invalid,
        "ready": ready,
    }
