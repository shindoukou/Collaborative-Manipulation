#!/usr/bin/env python3

import json
import socket
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def free_port():
    candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    candidate.bind(("127.0.0.1", 0))
    port = candidate.getsockname()[1]
    candidate.close()
    return port


def main():
    root = Path(__file__).resolve().parent
    runner = root / "run_distributed_trajopt_pair.py"
    left = np.array([
        [-1.20, -1.55, 1.50, -1.55, -0.05, 0.05],
        [-0.85, -1.30, 1.48, -1.25, -0.30, 0.08],
        [-0.45, -1.05, 1.45, -0.95, -0.60, 0.10],
    ])
    right = np.array([
        [1.20, -1.55, -1.50, -1.55, -0.05, -0.05],
        [0.85, -1.80, -1.52, -1.85, -0.30, -0.08],
        [0.45, -2.05, -1.55, -2.10, -0.60, -0.10],
    ])
    with tempfile.TemporaryDirectory(prefix="cm_trajopt_") as temporary:
        temporary = Path(temporary)
        left_path = temporary / "left.txt"
        right_path = temporary / "right.txt"
        options_path = temporary / "options.json"
        output = temporary / "output"
        np.savetxt(str(left_path), left)
        np.savetxt(str(right_path), right)
        options_path.write_text(json.dumps({
            "samples_per_segment": 5,
            "initial_segment_duration": 1.0,
            "weight_task_position_grasp": 0.0,
            "weight_task_orientation_grasp": 0.0,
            "weight_task_position_transport": 0.0,
            "weight_task_orientation_transport": 0.0,
            "task_distance_min": 0.0,
            "task_distance_max": 10.0,
            "task_orientation_cos_min": -1.0,
            "spatial_step": 0.01,
            "time_step": 0.01,
        }))
        completed = subprocess.run([
            sys.executable, str(runner),
            "--left-path", str(left_path),
            "--right-path", str(right_path),
            "--output-dir", str(output),
            "--object", "ball",
            "--phase", "move",
            "--port", str(free_port()),
            "--options-json", str(options_path),
            "--max-iterations", "12",
            "--output-dt", "0.02",
            "--timeout", "90",
        ], check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        summary = json.loads((output / "move_distributed_trajopt_summary.json").read_text())
        assert summary["joint_state_transmitted"] is False
        assert summary["left"]["polynomial_degree"] == 3
        assert summary["right"]["polynomial_degree"] == 3
        assert summary["left"]["time_gradient_exchanged"] is True
        assert summary["right"]["time_gradient_exchanged"] is True
        assert summary["optimization_method"] == "explicit_projected_gradient_descent"
        assert summary["collision_policy"]["collision_gradient_used"] is False
        assert summary["collision_policy"]["post_optimization_fcl_check_performed"] is False
        assert summary["collision_policy"]["collision_safe_blend_or_recovery_used"] is False
        assert summary["accepted_for_control"] is False
        assert summary["native_validation"] is None
        assert summary["output_policy"] == (
            "analysis_only_gradient_descent_result_without_fcl_recheck")
        assert summary["left"]["optimization_samples_per_segment_including_endpoints"] == 5
        assert summary["right"]["optimization_samples_per_segment_including_endpoints"] == 5
        assert summary["left"]["interior_optimization_samples_per_segment"] == 3
        assert summary["right"]["interior_optimization_samples_per_segment"] == 3
        assert summary["left"]["socket"]["messages_sent"] == 25
        assert summary["right"]["socket"]["messages_sent"] == 25
        initial_sum = (
            summary["left"]["initial_total_cost"]
            + summary["right"]["initial_total_cost"])
        final_sum = (
            summary["left"]["final_total_cost"]
            + summary["right"]["final_total_cost"])
        assert final_sum < initial_sum
        assert np.array_equal(
            np.asarray(summary["left"]["segment_times"]),
            np.asarray(summary["right"]["segment_times"]))
        print("DISTRIBUTED_TRAJOPT_LOOPBACK_OK", json.dumps({
            "iterations": summary["left"]["iterations"],
            "initial_total_cost": initial_sum,
            "final_total_cost": final_sum,
            "shared_duration": summary["shared_duration"],
            "joint_state_transmitted": False,
            "processes": 2,
        }, separators=(",", ":")))


if __name__ == "__main__":
    main()
