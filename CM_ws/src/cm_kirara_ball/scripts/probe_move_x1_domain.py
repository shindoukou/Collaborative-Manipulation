#!/usr/bin/env python3
"""Probe Ball Move x1 feasibility across fixed knot-deviation domains."""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
from pathlib import Path


def port():
    with socket.socket() as stream:
        stream.bind(("127.0.0.1", 0))
        return stream.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weights-json", required=True)
    parser.add_argument("--deviations", required=True)
    parser.add_argument("--max-iterations", type=int, default=800)
    parser.add_argument("--maximum-segment-duration", type=float)
    args = parser.parse_args()

    ws = args.workspace.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    package = ws / "src/cm_kirara_ball"
    source = Path((package / "generated/latest_controller_run.txt").read_text().strip())
    common = json.loads((source / "move/move_left_resolved_options.json").read_text())
    weights = json.loads(args.weights_json)
    common.update({key: float(value) for key, value in weights.items()})
    if args.maximum_segment_duration is not None:
        common["maximum_segment_duration"] = args.maximum_segment_duration
    common["object"] = "ball"
    common["phase"] = "move"
    if int(common["samples_per_segment"]) != 3:
        raise RuntimeError("probe requires one internal point")

    runner = ws / "devel/lib/cm_trajectory_optimization/run_distributed_trajopt_pair.py"
    validator = ws / "devel/lib/cm_kirara_ball/CM_SOCKET_BALL_VALIDATOR"
    config = package / "config/move_template.yaml"
    rows = []
    for index, deviation in enumerate(float(x) for x in args.deviations.split(",")):
        case = output / "deviation_{:02d}_{:.9f}".format(index, deviation)
        case.mkdir()
        options = dict(common)
        options["maximum_knot_deviation"] = deviation
        options_path = case / "options.json"
        options_path.write_text(json.dumps(options, indent=2) + "\n")
        command = [
            str(runner), "--left-path", str(args.left.resolve()),
            "--right-path", str(args.right.resolve()),
            "--output-dir", str(case), "--object", "ball", "--phase", "move",
            "--port", str(port()), "--options-json", str(options_path),
            "--max-iterations", str(args.max_iterations), "--output-dt", "0.01",
            "--timeout", "120", "--constraint-config", str(config),
            "--native-validator", str(validator), "--validator-config", str(config),
            "--require-native-valid", "--validation-max-joint-step", "0.005",
        ]
        with (case / "console.log").open("w") as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        summary = json.loads((case / "move_distributed_trajopt_summary.json").read_text())
        native = summary.get("native_validation") or {}
        kinematics = summary.get("candidate_kinematics") or {}
        costs = {
            name: sum(float(summary[side]["final_costs"].get(name, 0.0))
                      for side in ("left", "right"))
            for name in ("joint_position", "joint_velocity",
                         "task_position", "task_orientation")
        }
        row = {
            "maximum_knot_deviation": deviation,
            "return_code": completed.returncode,
            "accepted_for_control": bool(summary.get("accepted_for_control")),
            "native_valid": bool(native.get("valid")),
            "native_invalid_samples": int(native.get("invalid_sample_count", -1)),
            "continuous_kinematics_valid": bool(
                kinematics and all(value.get("valid") for value in kinematics.values())),
            "duration_s": float(summary["shared_duration"]),
            "left_iterations": int(summary["left"]["iterations"]),
            "right_iterations": int(summary["right"]["iterations"]),
            **costs,
        }
        rows.append(row)
        print(json.dumps(row, separators=(",", ":")), flush=True)
    (output / "domain_probe_summary.json").write_text(
        json.dumps({"weights": weights, "results": rows}, indent=2) + "\n")


if __name__ == "__main__":
    main()
