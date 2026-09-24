#!/usr/bin/env python3
"""Build a fixed 20-trajectory catch-reference manifest without filtering."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    trajectories = []
    for index in range(1, 21):
        sample_id = "sample_{:03d}".format(index)
        case = args.root / "samples" / sample_id / "cases" / "x1_baseline"
        left = case / "catch_left_smoothed_path.txt"
        right = case / "catch_right_smoothed_path.txt"
        left_times = case / "catch_left_sample_times.txt"
        right_times = case / "catch_right_sample_times.txt"
        for path in (left, right, left_times, right_times):
            if not path.is_file():
                raise FileNotFoundError(path)
        lt = np.loadtxt(str(left_times), dtype=float).reshape(-1)
        rt = np.loadtxt(str(right_times), dtype=float).reshape(-1)
        if lt.shape != rt.shape or not np.allclose(lt, rt, rtol=0.0, atol=1e-12):
            raise RuntimeError("left/right sample times differ for " + sample_id)
        left_q = np.atleast_2d(np.loadtxt(str(left), dtype=float))
        right_q = np.atleast_2d(np.loadtxt(str(right), dtype=float))
        if left_q.shape != right_q.shape or left_q.shape[0] != lt.size:
            raise RuntimeError("reference shape mismatch for " + sample_id)
        trajectories.append({
            "trajectory_id": sample_id,
            "source": "fixed_preexisting_grasp_x1_reference",
            "planner_seed": 2026082700 + index,
            "reference_left": str(left.resolve()),
            "reference_right": str(right.resolve()),
            "sample_times": str(left_times.resolve()),
            "reference_left_sha256": digest(left),
            "reference_right_sha256": digest(right),
            "sample_times_sha256": digest(left_times),
            "reference_rows": int(left_q.shape[0]),
            "duration_s": float(lt[-1] - lt[0]),
            "samples_per_segment_including_endpoints": 3,
            "interior_optimization_samples_per_segment": 1,
        })
    pairs = {(row["reference_left_sha256"], row["reference_right_sha256"])
             for row in trajectories}
    payload = {
        "schema": "CM_DNMPC_FIXED_GRASP_X1_REFERENCE_POOL_V1",
        "selection_policy": (
            "all twenty pre-existing x1 grasp references in sample-id order; "
            "no DNMPC outcome filtering"),
        "trajectory_count": len(trajectories),
        "unique_reference_pair_count": len(pairs),
        "source_root": str(args.root.resolve()),
        "trajectories": trajectories,
    }
    if len(pairs) != len(trajectories):
        raise RuntimeError("duplicate reference pair")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
