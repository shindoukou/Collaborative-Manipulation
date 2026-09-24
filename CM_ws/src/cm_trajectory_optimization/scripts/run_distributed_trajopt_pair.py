#!/usr/bin/env python3
"""Launch two independent gradient-descent trajectory optimizers."""

import argparse
import json
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

SOURCE_DIRECTORY = Path(__file__).resolve().parent
if str(SOURCE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIRECTORY))

from cm_trajopt_native_validator import validate_sampled_pair
from cm_trajopt_safety import (
    load_coefficients,
    sample_pair_for_validation,
    validate_kinematics,
)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-path", type=Path, required=True)
    parser.add_argument("--right-path", type=Path, required=True)
    parser.add_argument("--left-initial-state", type=Path)
    parser.add_argument("--right-initial-state", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--object", choices=("ball", "box"), required=True)
    parser.add_argument("--phase", choices=("catch", "move"), required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=55831)
    parser.add_argument("--options-json", type=Path)
    parser.add_argument("--max-iterations", type=int, default=80)
    parser.add_argument("--output-dt", type=float, default=0.01)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument(
        "--constraint-config", type=Path,
        help=(
            "optional planning YAML used only to import joint, velocity and "
            "task-geometry bounds; it does not start FCL validation"))
    parser.add_argument("--native-validator", type=Path)
    parser.add_argument("--validator-config", type=Path)
    parser.add_argument("--require-native-valid", action="store_true")
    parser.add_argument("--validation-max-joint-step", type=float, default=0.005)
    return parser.parse_args()


def command(args, side, socket_role, input_path, options_path):
    node = Path(__file__).resolve().parent / "cm_trajopt_arm_node.py"
    result = [
        sys.executable, str(node),
        "--side", side,
        "--socket-role", socket_role,
        "--host", args.host,
        "--port", str(args.port),
        "--input-path", str(input_path),
        "--output-dir", str(args.output_dir),
        "--object", args.object,
        "--phase", args.phase,
        "--max-iterations", str(args.max_iterations),
        "--output-dt", str(args.output_dt),
        "--timeout", str(min(args.timeout, 60.0)),
    ]
    if options_path:
        result.extend(["--options-json", str(options_path)])
    initial_state = getattr(args, side + "_initial_state", None)
    if initial_state:
        result.extend(["--initial-state", str(initial_state)])
    return result


def _resolved_options(args, side):
    options = {}
    if args.options_json:
        options.update(json.loads(args.options_json.read_text()))
    options.setdefault("joint_min", [-2.0 * math.pi] * 6)
    options.setdefault("joint_max", [2.0 * math.pi] * 6)
    options.setdefault("velocity_max", [0.5 * math.pi] * 6)
    options.setdefault("maximum_knot_deviation", 0.05)
    if float(options["maximum_knot_deviation"]) <= 0.0:
        raise ValueError(
            "maximum_knot_deviation must be positive: zero freezes all "
            "interior knot positions and disables the paper spatial update")
    options.setdefault("weight_path", 1.0)
    options.setdefault("weight_velocity", 1.0)
    options.setdefault("weight_joint_limit", 100.0)
    options.setdefault("weight_velocity_limit", 100.0)
    options.setdefault("weight_task_position", 100.0)
    options.setdefault("weight_task_orientation", 100.0)
    options.setdefault(
        "weight_task_position_grasp", options["weight_task_position"])
    options.setdefault(
        "weight_task_orientation_grasp", options["weight_task_orientation"])
    options.setdefault(
        "weight_task_position_transport", options["weight_task_position"])
    options.setdefault(
        "weight_task_orientation_transport", options["weight_task_orientation"])
    options.setdefault("weight_time", 0.05)
    constraint_source = "optimizer defaults"
    if args.constraint_config:
        config = yaml.safe_load(args.constraint_config.read_text()) or {}
        urdf_path = Path(config["file_path"]["pinocchio_file"])
        joint_suffixes = (
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")
        joint_elements = {
            element.attrib.get("name"): element
            for element in ET.parse(str(urdf_path)).getroot().findall("joint")
        }
        joint_min = []
        joint_max = []
        velocity_max = []
        for joint_index, suffix in enumerate(joint_suffixes):
            name = "{}_{}".format(side, suffix)
            joint = joint_elements.get(name)
            if joint is None or joint.find("limit") is None:
                raise RuntimeError("URDF joint limit not found: {}".format(name))
            limit = joint.find("limit").attrib
            lower = float(limit["lower"])
            upper = float(limit["upper"])
            # The planner and IK generator intentionally use +/-2pi for the
            # three wrist joints; generated URDFs retain generic +/-pi tags.
            # Keep trajectory validation consistent with the configurations
            # that the upstream planner is authorized to generate.
            if joint_index >= 3:
                lower = min(lower, -2.0 * math.pi)
                upper = max(upper, 2.0 * math.pi)
            joint_min.append(lower)
            joint_max.append(upper)
            velocity_max.append(float(limit["velocity"]))
        velocity_limit_scale = float(
            options.get("velocity_limit_scale", 1.0))
        if not 0.0 < velocity_limit_scale <= 1.0:
            raise ValueError("velocity_limit_scale must be in (0, 1]")
        velocity_max = [
            value * velocity_limit_scale for value in velocity_max]
        options["joint_min"] = joint_min
        options["joint_max"] = joint_max
        options["velocity_max"] = velocity_max
        goal = config.get("goal_generator") or {}
        planning = config.get("planning") or config.get("palnning") or {}
        cost = config.get("cost") or {}
        move_grasp = config.get("move_grasp_distance_dh") or {}
        if args.object == "box" and args.phase == "move" and move_grasp:
            distance_min = move_grasp["min_m"]
            distance_max = move_grasp["max_m"]
            orientation_cos_min = float(
                cost.get("cos" if side == "left" else "cos_2", -1.0))
            options["kinematics_profile"] = "dh_ft300"
            options["task_coordinate_definition"] = (
                "same DH+FT300 endpoint and tool z-axis used by CM DNMPC")
        else:
            distance_min = goal.get(
                "distance_min", planning.get("lowbound", 0.0))
            distance_max = goal.get(
                "distance_max",
                planning.get("upperbound", cost.get("eff", 1.0e6)))
            theta = float(goal.get("theta_{}".format(side), 0.45))
            orientation_cos_min = math.cos(theta)
        task_distance_safety_margin = float(
            options.get("task_distance_safety_margin_m", 0.0))
        if task_distance_safety_margin < 0.0:
            raise ValueError("task distance safety margin must be nonnegative")
        distance_min = float(distance_min) + task_distance_safety_margin
        distance_max = float(distance_max) - task_distance_safety_margin
        if distance_min >= distance_max:
            raise ValueError("task distance safety margin closes the interval")
        task_orientation_safety_margin = float(
            options.get("task_orientation_safety_margin_rad", 0.0))
        if task_orientation_safety_margin < 0.0:
            raise ValueError(
                "task orientation safety margin must be nonnegative")
        base_theta = math.acos(max(-1.0, min(1.0, orientation_cos_min)))
        if task_orientation_safety_margin >= base_theta:
            raise ValueError(
                "task orientation safety margin closes the angular interval")
        orientation_cos_min = math.cos(
            base_theta - task_orientation_safety_margin)
        options["task_distance_min"] = distance_min
        options["task_distance_max"] = distance_max
        options["task_orientation_cos_min"] = orientation_cos_min
        constraint_source = str(args.constraint_config)
    options["phase"] = args.phase
    options["constraint_profile_source"] = constraint_source
    options["weight_provenance"] = (
        "absolute trajectory-smoothing weights are not published in main20260815gpt.tex")
    return options


def _write_resolved_options(args):
    result = {}
    for side in ("left", "right"):
        path = args.output_dir / "{}_{}_resolved_options.json".format(args.phase, side)
        path.write_text(json.dumps(_resolved_options(args, side), indent=2) + "\n")
        result[side] = path
    return result


def main():
    args = parse_args()
    if args.require_native_valid and not args.native_validator:
        raise RuntimeError(
            "--require-native-valid requires --native-validator")
    if args.native_validator and not args.validator_config:
        raise RuntimeError(
            "--validator-config is required with --native-validator")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    resolved_option_paths = _write_resolved_options(args)
    left_log_path = args.output_dir / "{}_left_trajopt_console.log".format(args.phase)
    right_log_path = args.output_dir / "{}_right_trajopt_console.log".format(args.phase)
    with left_log_path.open("w") as left_log, right_log_path.open("w") as right_log:
        left = subprocess.Popen(
            command(args, "left", "server", args.left_path, resolved_option_paths["left"]),
            stdout=left_log, stderr=subprocess.STDOUT)
        right = subprocess.Popen(
            command(args, "right", "client", args.right_path, resolved_option_paths["right"]),
            stdout=right_log, stderr=subprocess.STDOUT)
        try:
            left_code = left.wait(timeout=args.timeout)
            right_code = right.wait(timeout=args.timeout)
        except subprocess.TimeoutExpired:
            left.kill()
            right.kill()
            raise RuntimeError("distributed trajectory optimization timed out")
    if left_code or right_code:
        raise RuntimeError(
            "trajectory optimizer failed left={} right={}; see {} and {}".format(
                left_code, right_code, left_log_path, right_log_path))

    reports = {}
    for side in ("left", "right"):
        path = args.output_dir / "{}_{}_trajopt_report.json".format(args.phase, side)
        reports[side] = json.loads(path.read_text())
    left_times = np.asarray(reports["left"]["segment_times"], dtype=float)
    right_times = np.asarray(reports["right"]["segment_times"], dtype=float)
    if not np.array_equal(left_times, right_times):
        raise RuntimeError("left/right optimized segment times are not identical")
    if reports["left"]["joint_state_transmitted"] or reports["right"]["joint_state_transmitted"]:
        raise RuntimeError("joint-space data crossed the distributed boundary")
    resolved_options = {
        side: json.loads(path.read_text()) for side, path in resolved_option_paths.items()
    }
    optimized_outputs = {
        side: dict(reports[side]["outputs"]) for side in ("left", "right")
    }
    optimization_completed = bool(all(
        reports[side]["iterations"] > 0
        and np.isfinite(reports[side]["final_total_cost"])
        for side in ("left", "right")))
    candidate_kinematics = None
    native_validation = None
    validation_sample_count = None
    postvalidated_for_control = False
    if args.native_validator:
        left_coefficients, coefficient_times = load_coefficients(
            reports["left"]["outputs"]["coefficients"])
        right_coefficients, right_coefficient_times = load_coefficients(
            reports["right"]["outputs"]["coefficients"])
        if (not np.array_equal(coefficient_times, left_times) or
                not np.array_equal(right_coefficient_times, left_times)):
            raise RuntimeError(
                "coefficient files and distributed segment times differ")
        candidate_kinematics = {
            "left": validate_kinematics(
                left_coefficients, left_times,
                resolved_options["left"]["joint_min"],
                resolved_options["left"]["joint_max"],
                resolved_options["left"]["velocity_max"]),
            "right": validate_kinematics(
                right_coefficients, right_times,
                resolved_options["right"]["joint_min"],
                resolved_options["right"]["joint_max"],
                resolved_options["right"]["velocity_max"]),
        }
        validation_times, validation_left, validation_right = (
            sample_pair_for_validation(
                left_coefficients, right_coefficients, left_times,
                args.output_dt, args.validation_max_joint_step))
        validation_sample_count = int(validation_times.size)
        with tempfile.TemporaryDirectory(
                prefix="trajopt_post_fcl_", dir=str(args.output_dir)) as temporary:
            left_validation_file = Path(temporary) / "left.txt"
            right_validation_file = Path(temporary) / "right.txt"
            np.savetxt(
                str(left_validation_file), validation_left, fmt="%.17g")
            np.savetxt(
                str(right_validation_file), validation_right, fmt="%.17g")
            native_validation = validate_sampled_pair(
                args.native_validator, args.phase, args.validator_config,
                left_validation_file, right_validation_file)
        postvalidated_for_control = bool(
            optimization_completed
            and native_validation["valid"]
            and candidate_kinematics["left"]["valid"]
            and candidate_kinematics["right"]["valid"])
    summary = {
        "architecture": "two_process_distributed_cubic_trajectory_optimization",
        "optimization_method": "explicit_projected_gradient_descent",
        "spatial_update_equation": "x_next=Pi_X(x-alpha*grad_x_J)",
        "shared_time_update_equation": (
            "T_next=Pi_[Tmin,Tmax](T-beta*(grad_T_Jl+grad_T_Jr)/2)"),
        "protocol": reports["left"]["protocol"],
        "object": args.object,
        "phase": args.phase,
        "shared_segment_times": left_times.tolist(),
        "shared_duration": float(left_times.sum()),
        "joint_state_transmitted": False,
        "collision_policy": {
            "collision_cost_in_objective": False,
            "collision_gradient_used": False,
            "post_optimization_fcl_check_performed": bool(
                args.native_validator),
            "collision_safe_blend_or_recovery_used": False,
            "dense_sampled_collision_check_passed": bool(
                native_validation and native_validation["valid"]),
            "colliding_optimizer_output_is_permitted": not bool(
                args.require_native_valid),
            "upstream_assumption": (
                "the input planning trajectory was collision-checked by "
                "the planning stage; the optimizer may move away from that "
                "path and collision is neither optimized nor projected"),
        },
        "spatial_position_updates_enabled": bool(all(
            float(resolved_options[side]["maximum_knot_deviation"]) > 0.0
            for side in ("left", "right"))),
        "paper_weight_verification": {
            "matches_paper": None,
            "status": "not_verifiable_from_paper",
            "reason": (
                "main20260815gpt.tex defines weight symbols and 0/x1/x2 ablations "
                "but does not publish the absolute x1 trajectory-smoothing values"),
            "resolved_option_files": {
                side: str(path) for side, path in resolved_option_paths.items()
            },
            "current_software_weights": {
                key: resolved_options["left"][key] for key in (
                    "weight_path", "weight_velocity", "weight_joint_limit",
                    "weight_velocity_limit",
                    "weight_task_position_grasp",
                    "weight_task_orientation_grasp",
                    "weight_task_position_transport",
                    "weight_task_orientation_transport", "weight_time")
            },
        },
        "optimization_completed": optimization_completed,
        "native_validation_grid": {
            "time_step_s": args.output_dt,
            "maximum_joint_step_rad": args.validation_max_joint_step,
            "sample_count": validation_sample_count,
        },
        "native_validation": native_validation,
        "candidate_kinematics": candidate_kinematics,
        "output_policy": (
            "direct_gradient_descent_result_with_post_fcl_acceptance_gate"
            if args.native_validator else
            "analysis_only_gradient_descent_result_without_fcl_recheck"),
        "accepted_segment_times": left_times.tolist(),
        "accepted_outputs": optimized_outputs,
        "accepted_for_control": postvalidated_for_control,
        "acceptance_scope": (
            "continuous joint-limit checks plus dense sampled post-FCL gate; "
            "FCL is not part of the optimizer objective or gradient"),
        "left": reports["left"],
        "right": reports["right"],
    }
    summary_path = args.output_dir / "{}_distributed_trajopt_summary.json".format(args.phase)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, separators=(",", ":")))
    if (not optimization_completed or
            (args.require_native_valid and not postvalidated_for_control)):
        raise SystemExit(4)


if __name__ == "__main__":
    main()
