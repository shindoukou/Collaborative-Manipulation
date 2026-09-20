#!/usr/bin/env python3
"""Run the current CM trajectory-weight sweep on one frozen 20-input pool."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--phase", choices=("grasp", "move"), required=True)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-count", type=int, default=20)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--x1-weight-velocity", type=float, default=1.0)
    parser.add_argument("--x1-weight-time", type=float, default=1.0)
    parser.add_argument("--x1-weights-json")
    parser.add_argument("--maximum-knot-deviation", type=float)
    parser.add_argument("--maximum-segment-duration", type=float)
    parser.add_argument("--task-distance-safety-margin-m", type=float)
    parser.add_argument("--task-orientation-safety-margin-rad", type=float)
    parser.add_argument("--velocity-limit-scale", type=float)
    parser.add_argument("--continue-on-error", action="store_true")
    parser.add_argument("--groups")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    workspace = args.workspace.resolve()
    input_root = args.input_root.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    script_name = (
        "sweep_grasp_trajopt_weights.py"
        if args.phase == "grasp" else "sweep_move_trajopt_weights.py"
    )
    sweep_script = workspace / "src/cm_kirara_ball/scripts" / script_name
    prefix = "catch" if args.phase == "grasp" else "move"

    inputs = []
    for number in range(1, args.sample_count + 1):
        sample_id = f"sample_{number:03d}"
        sample_dir = input_root / sample_id
        left = sample_dir / f"{prefix}_left_planned_path.txt"
        right = sample_dir / f"{prefix}_right_planned_path.txt"
        if not left.is_file() or not right.is_file():
            raise FileNotFoundError(f"missing input pair for {sample_id}")
        inputs.append({
            "sample_id": sample_id,
            "left": str(left),
            "right": str(right),
            "left_sha256": sha256(left),
            "right_sha256": sha256(right),
            "left_rows": sum(1 for line in left.read_text().splitlines() if line.strip()),
            "right_rows": sum(1 for line in right.read_text().splitlines() if line.strip()),
        })

    requested_manifest = {
        "created_at": dt.datetime.now().isoformat(),
        "phase": args.phase,
        "sample_count": len(inputs),
        "input_pool_frozen_before_ablation": True,
        "selection_uses_ablation_outcome": False,
        "samples_per_segment_including_endpoints": 3,
        "interior_optimization_samples_per_segment": 1,
        "sweep_script": str(sweep_script),
        "sweep_script_sha256": sha256(sweep_script),
        "max_iterations": args.max_iterations,
        "x1_weight_velocity": args.x1_weight_velocity,
        "x1_weight_time": args.x1_weight_time,
        "x1_weights_json": args.x1_weights_json,
        "maximum_knot_deviation": args.maximum_knot_deviation,
        "maximum_segment_duration": args.maximum_segment_duration,
        "task_distance_safety_margin_m": args.task_distance_safety_margin_m,
        "task_orientation_safety_margin_rad": (
            args.task_orientation_safety_margin_rad),
        "velocity_limit_scale": args.velocity_limit_scale,
        "groups": args.groups,
        "inputs": inputs,
        "runs": [],
    }
    manifest_path = output / "run_manifest.json"
    if args.resume:
        if not manifest_path.is_file():
            raise FileNotFoundError(
                "--resume requires an existing run_manifest.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        keys_to_match = (
            "phase", "sample_count", "sweep_script", "sweep_script_sha256",
            "max_iterations", "x1_weight_velocity", "x1_weight_time",
            "x1_weights_json", "maximum_knot_deviation",
            "maximum_segment_duration", "task_distance_safety_margin_m",
            "task_orientation_safety_margin_rad", "velocity_limit_scale",
            "groups", "inputs",
        )
        mismatches = [
            key for key in keys_to_match
            if manifest.get(key) != requested_manifest.get(key)
        ]
        if mismatches:
            raise RuntimeError(
                "resume manifest mismatch: " + ", ".join(mismatches))
        manifest["resumed_at"] = dt.datetime.now().isoformat()
        manifest.pop("completed", None)
    else:
        manifest = requested_manifest
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")

    overall_start = time.monotonic()
    prior_elapsed = float(manifest.get("elapsed_s", 0.0))
    failures = sum(
        1 for record in manifest.get("runs", [])
        if record.get("return_code") != 0 or not record.get("summary_exists"))
    completed_ids = {
        record["sample_id"] for record in manifest.get("runs", [])
        if record.get("return_code") == 0 and record.get("summary_exists")
    }
    for index, item in enumerate(inputs, 1):
        sample_id = item["sample_id"]
        sample_output = output / "samples" / sample_id
        summary_path = sample_output / "sweep_summary.json"
        if args.resume and sample_id in completed_ids and summary_path.is_file():
            print(
                f"TRAJOPT20_RESUME_SKIP {index}/{len(inputs)} "
                f"phase={args.phase} sample={sample_id}", flush=True)
            continue
        command = [
            sys.executable,
            str(sweep_script),
            "--workspace", str(workspace),
            "--left-path", item["left"],
            "--right-path", item["right"],
            "--output-dir", str(sample_output),
            "--max-iterations", str(args.max_iterations),
            "--x1-weight-velocity", str(args.x1_weight_velocity),
            "--x1-weight-time", str(args.x1_weight_time),
        ]
        if args.x1_weights_json:
            command.extend(["--x1-weights-json", args.x1_weights_json])
        if args.maximum_knot_deviation is not None:
            command.extend([
                "--maximum-knot-deviation",
                str(args.maximum_knot_deviation),
            ])
        if args.maximum_segment_duration is not None:
            command.extend([
                "--maximum-segment-duration",
                str(args.maximum_segment_duration),
            ])
        if args.task_distance_safety_margin_m is not None:
            command.extend([
                "--task-distance-safety-margin-m",
                str(args.task_distance_safety_margin_m),
            ])
        if args.task_orientation_safety_margin_rad is not None:
            command.extend([
                "--task-orientation-safety-margin-rad",
                str(args.task_orientation_safety_margin_rad),
            ])
        if args.velocity_limit_scale is not None:
            command.extend([
                "--velocity-limit-scale", str(args.velocity_limit_scale),
            ])
        if args.groups:
            command.extend(["--groups", args.groups])
        print(
            f"TRAJOPT20_PROGRESS {index}/{len(inputs)} phase={args.phase} "
            f"sample={sample_id}", flush=True)
        started = time.monotonic()
        completed = subprocess.run(
            command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            check=False)
        wall_time = time.monotonic() - started
        log_path = output / f"{sample_id}.log"
        log_path.write_text(completed.stdout or "", encoding="utf-8")
        baseline_success = False
        if summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                baseline_success = bool(summary["x1_metrics"]["successful"])
            except (KeyError, ValueError, TypeError):
                pass
        if completed.returncode != 0 or not summary_path.is_file():
            failures += 1
        run_record = {
            "sample_id": sample_id,
            "return_code": completed.returncode,
            "wall_time_s": wall_time,
            "summary": str(summary_path),
            "summary_exists": summary_path.is_file(),
            "x1_success": baseline_success,
            "log": str(log_path),
        }
        manifest["runs"].append(run_record)
        manifest["elapsed_s"] = prior_elapsed + time.monotonic() - overall_start
        manifest_path.write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        print(json.dumps(run_record, ensure_ascii=False), flush=True)
        if failures and not args.continue_on_error:
            return 2

    manifest["completed"] = True
    manifest["failures"] = failures
    manifest["elapsed_s"] = prior_elapsed + time.monotonic() - overall_start
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({
        "complete": True,
        "output": str(output),
        "failures": failures,
        "elapsed_s": manifest["elapsed_s"],
    }, ensure_ascii=False), flush=True)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
