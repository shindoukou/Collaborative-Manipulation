#!/usr/bin/env python3
"""Generate a frozen pool of distinct valid Ball Move planning trajectories."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List

import numpy as np


DEFAULT_WORKSPACE = Path("/home/mas-2/3rd_party/CM_ws")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def available_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
        stream.bind(("127.0.0.1", 0))
        return int(stream.getsockname()[1])


def load_path(path):
    values = np.atleast_2d(np.loadtxt(str(path), dtype=float))
    if values.shape[1] != 6 or values.shape[0] < 2:
        raise ValueError("expected at least two six-joint states: " + str(path))
    return values


def resample(path, count=64):
    delta = np.linalg.norm(np.diff(path, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(delta)))
    if arc[-1] <= 1.0e-12:
        return np.repeat(path[:1], count, axis=0)
    keep = np.concatenate(([True], np.diff(arc) > 1.0e-12))
    arc = arc[keep] / arc[-1]
    path = path[keep]
    query = np.linspace(0.0, 1.0, count)
    return np.column_stack([
        np.interp(query, arc, path[:, joint]) for joint in range(6)])


def pair_signature(left, right):
    return np.column_stack((resample(left), resample(right)))


def pair_rms(first, second):
    return float(np.sqrt(np.mean((first - second) ** 2)))


def write_json(path, payload):
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")


def write_csv(path, rows: List[Dict]):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def sample_record(sample_id, source, seed, directory, signatures):
    left_path = directory / "move_left_planned_path.txt"
    right_path = directory / "move_right_planned_path.txt"
    left = load_path(left_path)
    right = load_path(right_path)
    signature = pair_signature(left, right)
    minimum_rms = None
    if signatures:
        minimum_rms = min(pair_rms(signature, item) for item in signatures)
    return {
        "sample_id": sample_id,
        "source": source,
        "planner_seed": seed,
        "left_path": str(left_path.resolve()),
        "right_path": str(right_path.resolve()),
        "left_sha256": sha256(left_path),
        "right_sha256": sha256(right_path),
        "left_rows": int(left.shape[0]),
        "right_rows": int(right.shape[0]),
        "left_joint_length_rad": float(np.linalg.norm(
            np.diff(left, axis=0), axis=1).sum()),
        "right_joint_length_rad": float(np.linalg.norm(
            np.diff(right, axis=0), axis=1).sum()),
        "minimum_pairwise_resampled_rms_rad": minimum_rms,
    }, signature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--existing-left", type=Path, required=True)
    parser.add_argument("--existing-right", type=Path, required=True)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--seed-base", type=int, default=2026082701)
    parser.add_argument("--maximum-attempts", type=int, default=200)
    parser.add_argument("--minimum-pairwise-rms-rad", type=float, default=0.01)
    parser.add_argument("--max-iterations", type=int, default=20000)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument(
        "--bootstrap-manifest", type=Path,
        help=("copy pre-optimization planning samples from an earlier pool "
              "before generating the remaining samples"))
    parser.add_argument(
        "--exclude-bootstrap-sample", action="append", default=[],
        help="sample_id to omit from --bootstrap-manifest (repeatable)")
    args = parser.parse_args()
    if args.count < 2:
        raise SystemExit("count must be at least 2")

    workspace = args.workspace.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    package = workspace / "src/cm_kirara_ball"
    runner = package / "scripts/run_socket_planning_pair.py"
    config = package / "config/move_template.yaml"
    validator = workspace / "devel/lib/cm_kirara_ball/CM_SOCKET_BALL_VALIDATOR"
    endpoints = output / "frozen_endpoints"
    endpoints.mkdir()
    old_left = load_path(args.existing_left)
    old_right = load_path(args.existing_right)
    np.savetxt(str(endpoints / "move_left.txt"), old_left[[0, -1]], fmt="%.17g")
    np.savetxt(str(endpoints / "move_right.txt"), old_right[[0, -1]], fmt="%.17g")

    manifest = {
        "schema": "CM_BALL_MOVE_PLANNING_POOL_V1",
        "created_at": dt.datetime.now().astimezone().isoformat(),
        "selection_policy": (
            "bootstrap samples, if supplied, are copied before downstream "
            "optimization; remaining samples are accepted in increasing seed "
            "order only when planning succeeds and the frozen pre-DNMPC "
            "diversity threshold is met"),
        "selection_uses_trajectory_optimization_outcome": False,
        "selection_uses_dnmpc_outcome": False,
        "requested_count": args.count,
        "seed_base": args.seed_base,
        "maximum_attempts": args.maximum_attempts,
        "minimum_pairwise_resampled_rms_rad": args.minimum_pairwise_rms_rad,
        "resampling_points_for_diversity": 64,
        "fixed_endpoints": {
            "left": str((endpoints / "move_left.txt").resolve()),
            "right": str((endpoints / "move_right.txt").resolve()),
        },
        "planner": {
            "runner": str(runner.resolve()),
            "config": str(config.resolve()),
            "validator": str(validator.resolve()),
            "guide_is_sampling_heuristic_only": True,
            "max_iterations": args.max_iterations,
        },
        "attempts": [],
        "samples": [],
    }
    manifest_path = output / "fixed20_input_manifest.json"
    write_json(manifest_path, manifest)

    signatures = []
    if args.bootstrap_manifest:
        bootstrap_path = args.bootstrap_manifest.resolve()
        bootstrap = json.loads(bootstrap_path.read_text(encoding="utf-8"))
        excluded = set(args.exclude_bootstrap_sample)
        manifest["bootstrap_manifest"] = str(bootstrap_path)
        manifest["bootstrap_manifest_sha256"] = sha256(bootstrap_path)
        manifest["excluded_bootstrap_samples"] = sorted(excluded)
        for source_record in bootstrap.get("samples") or []:
            original_id = source_record["sample_id"]
            if original_id in excluded:
                continue
            if len(manifest["samples"]) >= args.count:
                break
            sample_id = "sample_{:03d}".format(len(manifest["samples"]) + 1)
            destination = output / sample_id
            source_directory = Path(source_record["left_path"]).resolve().parent
            shutil.copytree(source_directory, destination)
            record, signature = sample_record(
                sample_id, source_record.get("source", "new_planning_run"),
                source_record.get("planner_seed"), destination, signatures)
            record["bootstrap_source_sample_id"] = original_id
            signatures.append(signature)
            manifest["samples"].append(record)
        if not manifest["samples"]:
            raise RuntimeError("bootstrap manifest supplied no usable samples")
    else:
        first = output / "sample_001"
        first.mkdir()
        shutil.copy2(args.existing_left, first / "move_left_planned_path.txt")
        shutil.copy2(args.existing_right, first / "move_right_planned_path.txt")
        record, signature = sample_record(
            "sample_001", "existing_demonstration", None, first, signatures)
        signatures.append(signature)
        manifest["samples"].append(record)
    write_json(manifest_path, manifest)

    started = time.monotonic()
    seed = args.seed_base
    while (len(manifest["samples"]) < args.count and
           len(manifest["attempts"]) < args.maximum_attempts):
        attempt_id = "seed_{:010d}".format(seed)
        attempt = output / "attempts" / attempt_id
        attempt.mkdir(parents=True)
        command = [
            sys.executable, runner,
            "--phase", "move",
            "--config", config,
            "--output-dir", attempt,
            "--port", str(available_port()),
            "--left-endpoints", endpoints / "move_left.txt",
            "--right-endpoints", endpoints / "move_right.txt",
            "--left-guide", args.existing_left,
            "--right-guide", args.existing_right,
            "--max-iterations", str(args.max_iterations),
            "--seed", str(seed),
            "--timeout", str(args.timeout),
            "--validator", validator,
        ]
        attempt_record = {
            "attempt_id": attempt_id,
            "seed": seed,
            "command": [str(item) for item in command],
            "accepted": False,
        }
        console = attempt / "runner_console.log"
        try:
            with console.open("w", encoding="utf-8") as log:
                completed = subprocess.run(
                    [str(item) for item in command],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=args.timeout + 30.0,
                    check=False)
            attempt_record["return_code"] = int(completed.returncode)
            if completed.returncode != 0:
                raise RuntimeError("planning pair failed")
            candidate, signature = sample_record(
                "candidate", "new_planning_run", seed, attempt, signatures)
            diversity = candidate["minimum_pairwise_resampled_rms_rad"]
            attempt_record["minimum_pairwise_resampled_rms_rad"] = diversity
            if diversity < args.minimum_pairwise_rms_rad:
                raise RuntimeError(
                    "planned path is below the frozen diversity threshold")
            sample_id = "sample_{:03d}".format(len(manifest["samples"]) + 1)
            destination = output / sample_id
            shutil.copytree(attempt, destination)
            candidate, signature = sample_record(
                sample_id, "new_planning_run", seed, destination, signatures)
            signatures.append(signature)
            manifest["samples"].append(candidate)
            attempt_record["accepted"] = True
            attempt_record["sample_id"] = sample_id
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            attempt_record["failure"] = str(error)
        attempt_record["elapsed_total_s"] = time.monotonic() - started
        manifest["attempts"].append(attempt_record)
        write_json(manifest_path, manifest)
        print(json.dumps(attempt_record, ensure_ascii=False), flush=True)
        seed += 1

    manifest["completed"] = len(manifest["samples"]) == args.count
    manifest["elapsed_s"] = time.monotonic() - started
    manifest["unique_path_pair_count"] = len({
        (row["left_sha256"], row["right_sha256"])
        for row in manifest["samples"]})
    write_json(manifest_path, manifest)
    write_csv(output / "fixed20_input_manifest.csv", manifest["samples"])
    if not manifest["completed"]:
        raise RuntimeError("only {} of {} paths were accepted".format(
            len(manifest["samples"]), args.count))
    print(json.dumps({
        "completed": True,
        "sample_count": len(manifest["samples"]),
        "unique_path_pair_count": manifest["unique_path_pair_count"],
        "output": str(output),
    }, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
