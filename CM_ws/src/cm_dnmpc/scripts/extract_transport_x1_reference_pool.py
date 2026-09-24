#!/usr/bin/env python3
"""Freeze the 20 selected Transporting x1 trajectories for DNMPC."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planning-pool", type=Path, required=True)
    parser.add_argument("--trajopt-results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    planning = json.loads((
        args.planning_pool / "fixed20_input_manifest.json").read_text(
            encoding="utf-8"))
    run_manifest = json.loads((
        args.trajopt_results / "run_manifest.json").read_text(encoding="utf-8"))
    if len(planning["samples"]) != 20 or len(run_manifest["runs"]) != 20:
        raise RuntimeError("exactly 20 paired inputs are required")
    planning_by_id = {row["sample_id"]: row for row in planning["samples"]}
    trajectories = []
    for run in run_manifest["runs"]:
        sample_id = run["sample_id"]
        summary_path = Path(run["summary"])
        sweep = json.loads(summary_path.read_text(encoding="utf-8"))
        if not bool(sweep["x1_metrics"]["successful"]):
            raise RuntimeError(
                sample_id + " x1 did not pass the complete trajectory gate")
        if int(sweep["interior_optimization_samples_per_segment"]) != 1:
            raise RuntimeError(sample_id + " is not internal1")
        source_case = summary_path.parent / "cases/x1_baseline"
        destination = args.output / "trajectories" / sample_id
        planning_destination = destination / "planning"
        optimized_destination = destination / "optimized"
        planning_destination.mkdir(parents=True)
        source_planning = args.planning_pool / sample_id
        for name in ("move_left_planned_path.txt", "move_right_planned_path.txt"):
            shutil.copy2(source_planning / name, planning_destination / name)
        shutil.copytree(source_case, optimized_destination)
        left = optimized_destination / "move_left_smoothed_path.txt"
        right = optimized_destination / "move_right_smoothed_path.txt"
        times = optimized_destination / "move_left_sample_times.txt"
        left_values = np.atleast_2d(np.loadtxt(str(left)))
        right_values = np.atleast_2d(np.loadtxt(str(right)))
        time_values = np.atleast_1d(np.loadtxt(str(times)))
        if left_values.shape != right_values.shape:
            raise RuntimeError(sample_id + " left/right shapes differ")
        if left_values.shape[0] != time_values.size:
            raise RuntimeError(sample_id + " time/reference size differs")
        source = planning_by_id[sample_id]
        trajectories.append({
            "trajectory_id": sample_id,
            "source": source["source"],
            "planner_seed": source.get("planner_seed"),
            "planned_left": str((planning_destination /
                                 "move_left_planned_path.txt").resolve()),
            "planned_right": str((planning_destination /
                                  "move_right_planned_path.txt").resolve()),
            "reference_left": str(left.resolve()),
            "reference_right": str(right.resolve()),
            "sample_times": str(times.resolve()),
            "reference_left_sha256": sha256(left),
            "reference_right_sha256": sha256(right),
            "sample_times_sha256": sha256(times),
            "reference_rows": int(left_values.shape[0]),
            "duration_s": float(time_values[-1]),
            "samples_per_segment_including_endpoints": 3,
            "interior_optimization_samples_per_segment": 1,
            "trajopt_x1_weights": sweep["x1_weights"],
            "trajopt_x1_fixed_Ei": float(sweep["x1_fixed_objective"]["total"]),
        })
    hashes = {
        (row["reference_left_sha256"], row["reference_right_sha256"])
        for row in trajectories}
    if len(hashes) != len(trajectories):
        raise RuntimeError("optimized reference pool contains duplicates")
    payload = {
        "schema": "CM_DNMPC_STAGE3_TRANSPORT_X1_REFERENCE_POOL_V1",
        "selection_policy": (
            "same 20 planning inputs used by the formal Transporting sweep; "
            "only the predeclared x1 output is copied, before DNMPC ablation"),
        "trajectory_count": len(trajectories),
        "unique_reference_pair_count": len(hashes),
        "old_demonstration_trajectory_count": sum(
            int(row["source"] == "existing_demonstration")
            for row in trajectories),
        "newly_planned_trajectory_count": sum(
            int(row["source"] == "new_planning_run")
            for row in trajectories),
        "trajectories": trajectories,
    }
    (args.output / "trajectory_pool_manifest.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    with (args.output / "trajectory_pool_manifest.csv").open(
            "w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(trajectories[0]))
        writer.writeheader()
        writer.writerows(trajectories)
    print(json.dumps({
        "trajectory_count": len(trajectories),
        "unique_reference_pair_count": len(hashes),
        "output": str(args.output.resolve()),
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
