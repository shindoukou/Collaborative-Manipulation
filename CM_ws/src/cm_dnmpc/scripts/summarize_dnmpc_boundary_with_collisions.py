#!/usr/bin/env python3
"""Merge Stage-3 task outcomes with Gazebo-geometry collision audits."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


METRICS = (
    "q_rmse_rad",
    "relative_position_rmse_m",
    "orientation_rmse_deg",
    "velocity_rmse_rad_s",
    "force_rmse_n",
    "control_effort",
    "common_eval_cost",
    "terminal_target_error_mm",
    "mean_solve_ms",
    "p95_solve_ms",
    "max_solve_ms",
    "force_min_n",
    "force_max_n",
    "force_slew_max_n_s",
    "ee_distance_min_m",
    "ee_distance_max_m",
    "joint_velocity_max_rad_s",
    "joint_velocity_ratio_max",
    "joint_acceleration_max_rad_s2",
    "command_velocity_max_rad_s",
    "command_jump_max_rad_s",
    "deadline_miss_fraction",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--weights", nargs="+", required=True)
    parser.add_argument("--base-weights-json", required=True)
    parser.add_argument(
        "--terminal-target-error-mm", type=float, default=10.0)
    parser.add_argument(
        "--orientation-rmse-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_collisions(path):
    payload = json.loads(path.read_text(encoding="utf-8"))
    return {
        str(Path(item["trace"]).resolve()): item
        for item in payload["results"]
    }


def truth(value):
    return str(value).strip().lower() == "true"


def mean(rows, key):
    values = [float(row[key]) for row in rows]
    return sum(values) / len(values)


def setting_name(value, base):
    if abs(value) <= 1e-12:
        return "x0"
    if abs(value - base) <= max(1e-12, abs(base) * 1e-9):
        return "x1"
    if abs(value - 2.0 * base) <= max(1e-12, abs(base) * 1e-9):
        return "x2"
    return "value_{}".format(value)


def main():
    args = parse_args()
    base_weights = {
        key: float(value)
        for key, value in json.loads(args.base_weights_json).items()
    }
    merged_rows = []
    grouped = defaultdict(list)

    for weight in args.weights:
        rows = read_csv(args.root / weight / "raw_runs.csv")
        collisions = read_collisions(
            args.root / weight / "collision_audit.json")
        for row in rows:
            trace = str(Path(row["joint_trace"]).resolve())
            audit = collisions[trace]
            if "hard_cycle_failure" in row:
                task_success = truth(row["task_success"])
            else:
                task_success = (
                    not truth(row["solver_failure"])
                    and not truth(row["joint_violation"])
                    and not truth(row["slip_failure"])
                    and not truth(row["overforce_failure"])
                    and float(row["terminal_target_error_mm"])
                    <= args.terminal_target_error_mm
                    and float(row["orientation_rmse_deg"])
                    <= args.orientation_rmse_deg
                )
            collision_free = bool(audit["collision_free"])
            value = float(row["weight_value"])
            label = setting_name(value, base_weights[weight])
            merged = dict(row)
            merged.update({
                "factor_label": label,
                "collision_free": collision_free,
                "collision_frames": int(audit["collision_frames"]),
                "first_collision": json.dumps(
                    audit["first_collision"], ensure_ascii=False),
                "criterion_task_success": task_success,
                "combined_success": task_success and collision_free,
                "combined_failure_reason": (
                    "collision" if not collision_free else
                    row.get("first_failure_reason", "")),
            })
            merged_rows.append(merged)
            grouped[(weight, label, value)].append(merged)

    summary = []
    for (weight, label, value), rows in sorted(grouped.items()):
        item = {
            "weight": weight,
            "factor_label": label,
            "weight_value": value,
            "runs": len(rows),
            "task_successes": sum(
                truth(row["criterion_task_success"]) for row in rows),
            "collision_free_runs": sum(
                truth(row["collision_free"]) for row in rows),
            "combined_successes": sum(
                truth(row["combined_success"]) for row in rows),
        }
        reason_counts = defaultdict(int)
        for row in rows:
            if not truth(row["combined_success"]):
                reason_counts[row["combined_failure_reason"] or "task_failure"] += 1
        item["combined_failure_reason_counts"] = json.dumps(
            dict(reason_counts), ensure_ascii=False, sort_keys=True)
        item["combined_success_rate_percent"] = (
            100.0 * item["combined_successes"] / item["runs"])
        for metric in METRICS:
            item[metric + "_mean"] = mean(rows, metric)
        summary.append(item)

    baselines = {
        item["weight"]: item for item in summary
        if item["factor_label"] == "x1"
    }
    for item in summary:
        baseline = baselines[item["weight"]]
        for metric in METRICS:
            key = metric + "_mean"
            base = baseline[key]
            item[metric + "_change_percent"] = (
                100.0 * (item[key] - base) / base if base else 0.0)

    args.output.mkdir(parents=True, exist_ok=True)
    if merged_rows:
        with (args.output / "combined_raw_runs.csv").open(
                "w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(merged_rows[0]))
            writer.writeheader()
            writer.writerows(merged_rows)
    if summary:
        with (args.output / "combined_summary.csv").open(
                "w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
            writer.writeheader()
            writer.writerows(summary)
    payload = {
        "schema": "CM_DNMPC_BOUNDARY_WITH_COLLISION_V1",
        "root": str(args.root.resolve()),
        "base_weights": base_weights,
        "weights": args.weights,
        "success_criteria": {
            "terminal_target_error_mm": args.terminal_target_error_mm,
            "orientation_rmse_deg": args.orientation_rmse_deg,
            "collision_free": True,
            "solver_failure": False,
            "joint_violation": False,
            "slip_failure": False,
            "overforce_failure": False,
        },
        "summary": summary,
    }
    (args.output / "combined_result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "rows": len(merged_rows),
        "groups": len(summary),
        "x1_combined_successes": {
            key: value["combined_successes"]
            for key, value in baselines.items()
        },
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
