#!/usr/bin/env python3
import argparse
import datetime
import json
import os
import subprocess
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=5)
    parser.add_argument("--lower-deviation", type=float, default=1.0e-6)
    parser.add_argument("--upper-deviation", type=float, default=0.005)
    return parser.parse_args()


def run_candidate(args, output_root, deviation):
    workspace = args.workspace.resolve()
    box = workspace / "src" / "cm_kirara_box"
    trajopt = workspace / "src" / "cm_trajectory_optimization" / "scripts"
    candidate = output_root / "deviation_{:.9f}".format(deviation)
    move = candidate / "move"
    move.mkdir(parents=True, exist_ok=True)
    constraint_config = box / "config" / "move_kirara_box.yaml"

    base_options = json.loads((
        box / "config" / "move_planning_edge_fcl_options.json").read_text())
    base_options["maximum_knot_deviation"] = float(deviation)
    base_options["spatial_step"] = float(0.4 * deviation)
    options_path = candidate / "resolved_search_options.json"
    options_path.write_text(json.dumps(base_options, indent=2) + "\n")

    command = [
        sys.executable,
        str(trajopt / "run_distributed_trajopt_pair.py"),
        "--left-path", str(
            box / "generated" / "socket_latest" /
            "move_left_planned_path.txt"),
        "--right-path", str(
            box / "generated" / "socket_latest" /
            "move_right_planned_path.txt"),
        "--output-dir", str(move),
        "--object", "box",
        "--phase", "move",
        "--options-json", str(options_path),
        "--constraint-config", str(constraint_config),
        "--max-iterations", "80",
        "--output-dt", "0.01",
        "--timeout", "120",
    ]
    with (move / "runner_stdout.json").open("w") as stream:
        subprocess.run(command, check=True, stdout=stream)

    sys.path.insert(0, str(trajopt))
    from cm_trajopt_native_validator import validate_sampled_pair

    validation = validate_sampled_pair(
        workspace / "devel" / "lib" / "cm_kirara_box" /
        "CM_SOCKET_BOX_VALIDATOR",
        "move-fcl",
        constraint_config,
        move / "move_left_smoothed_path.txt",
        move / "move_right_smoothed_path.txt")
    native_environment = os.environ.copy()
    native_config = constraint_config
    native_environment["CM_KIRARA_BOX_MOVE_CONFIG"] = str(native_config)
    native_process = subprocess.run([
        str(workspace / "devel" / "lib" / "cm_kirara_box" /
            "CM_KIRARA_BOX_MOVE"),
        "--validate-samples",
        str(move / "move_left_smoothed_path.txt"),
        str(move / "move_right_smoothed_path.txt"),
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
       text=True, env=native_environment)
    summary = json.loads((move / "move_distributed_trajopt_summary.json").read_text())
    penalty_names = (
        "joint_position", "joint_velocity",
        "task_position", "task_orientation")
    penalties = {
        side: {
            name: float(summary[side]["final_costs"][name])
            for name in penalty_names
        }
        for side in ("left", "right")
    }
    joint_constraints_valid = all(
        penalties[side][name] <= 1.0e-9
        for side in penalties
        for name in ("joint_position", "joint_velocity"))
    result = {
        "maximum_knot_deviation_rad": float(deviation),
        "spatial_step": float(0.4 * deviation),
        "duration_s": float(summary["shared_duration"]),
        "sample_count": int(summary["left"]["sample_count"]),
        "paper_penalty_costs": penalties,
        "paper_constraint_costs_recorded": True,
        "joint_constraints_valid": bool(joint_constraints_valid),
        "task_constraints_are_paper_penalties_not_fcl_acceptance_rules": True,
        "fcl_validation": validation,
        "native_box_constraints": {
            "valid": native_process.returncode == 0,
            "config": str(native_config),
            "validator_tail": native_process.stdout.splitlines()[-5:],
        },
        "accepted": bool(
            joint_constraints_valid and validation["valid"] and
            native_process.returncode == 0),
        "output": str(candidate),
        "collision_repair_or_projection_used": False,
    }
    (candidate / "candidate_validation.json").write_text(
        json.dumps(result, indent=2) + "\n")
    return result


def main():
    args = parse_args()
    if (args.iterations < 1 or args.lower_deviation <= 0.0 or
            args.upper_deviation <= args.lower_deviation):
        raise ValueError("invalid search interval")
    workspace = args.workspace.resolve()
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = (
        workspace / "src" / "cm_kirara_box" / "generated" /
        ("move_fcl_boundary_" + stamp))
    output_root.mkdir(parents=True)

    tested = []
    lower = float(args.lower_deviation)
    upper = float(args.upper_deviation)
    lower_result = run_candidate(args, output_root, lower)
    tested.append(lower_result)
    if not lower_result["accepted"]:
        raise RuntimeError(
            "minimum positive-deviation reference edge failed acceptance")
    upper_result = run_candidate(args, output_root, upper)
    tested.append(upper_result)
    if upper_result["accepted"]:
        lower = upper
        lower_result = upper_result
    else:
        for _ in range(args.iterations):
            middle = 0.5 * (lower + upper)
            result = run_candidate(args, output_root, middle)
            tested.append(result)
            if result["accepted"]:
                lower = middle
                lower_result = result
            else:
                upper = middle

    report = {
        "method": "binary search with post-optimization Pinocchio/hpp-FCL and native Box task acceptance",
        "paper_objective_terms_unchanged": True,
        "planning_input_unchanged": True,
        "time_search_domain_unchanged": True,
        "fcl_in_objective_or_gradient": False,
        "collision_repair_or_projection_used": False,
        "tested": tested,
        "selected": lower_result,
        "first_rejected_upper_bound_rad": float(upper),
        "boundary_resolution_rad": float(upper - lower),
    }
    (output_root / "boundary_search_report.json").write_text(
        json.dumps(report, indent=2) + "\n")
    print(json.dumps({
        "output_root": str(output_root),
        "selected": lower_result,
        "first_rejected_upper_bound_rad": upper,
        "boundary_resolution_rad": upper - lower,
    }, indent=2))


if __name__ == "__main__":
    main()
