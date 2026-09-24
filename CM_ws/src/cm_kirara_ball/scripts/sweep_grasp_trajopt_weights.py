#!/usr/bin/env python3
"""Ball Grasp trajectory-optimization x0/x1/x2 sweep on one fixed reference.

Every candidate trajectory is cross-evaluated with the same x1 objective.
Collision checking stays outside the objective and gradient, but is recorded as
the post-optimization binary success gate.
"""

import argparse
import csv
import datetime as dt
import hashlib
import json
import shutil
import socket
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml


DEFAULT_WORKSPACE = Path("/home/mas-2/3rd_party/CM_ws")
WEIGHT_KEYS = (
    "weight_path",
    "weight_velocity",
    "weight_joint_limit",
    "weight_velocity_limit",
    "weight_task_position_grasp",
    "weight_task_orientation_grasp",
    "weight_time",
)
GROUPS = (
    ("path", ("weight_path",), ("path",)),
    ("velocity", ("weight_velocity",), ("velocity",)),
    ("kinematic_limits",
     ("weight_joint_limit", "weight_velocity_limit"),
     ("joint_position", "joint_velocity")),
    ("task_grasp",
     ("weight_task_position_grasp", "weight_task_orientation_grasp"),
     ("task_position", "task_orientation")),
    ("time", ("weight_time",), ("time",)),
)
COST_KEYS = (
    "path", "velocity", "joint_position", "joint_velocity",
    "task_position", "task_orientation", "time",
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative_change_percent(value, baseline):
    baseline = float(baseline)
    if abs(baseline) <= 1.0e-12:
        return None
    return 100.0 * (float(value) - baseline) / abs(baseline)


def available_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as stream:
        stream.bind(("127.0.0.1", 0))
        return int(stream.getsockname()[1])


class FixedObjectiveEvaluator:
    """Re-evaluate any candidate polynomial with one fixed x1 objective."""

    def __init__(self, workspace, inputs, fixed_options):
        sys.path.insert(0, str(
            workspace / "src/cm_trajectory_optimization/scripts"))
        from cm_trajopt_common import LocalCubicTrajectoryOptimizer
        self.optimizer_class = LocalCubicTrajectoryOptimizer
        self.inputs = inputs
        self.fixed_options = dict(fixed_options)

    def _load(self, side, case_dir):
        payload = json.loads((
            case_dir / f"catch_{side}_cubic_coefficients.json").read_text())
        coefficients = np.asarray(payload["coefficients_Mx4x6"], dtype=float)
        times = np.asarray(payload["segment_times"], dtype=float)
        optimizer = self.optimizer_class(
            side, self.inputs[side], self.fixed_options)
        inner_count = optimizer.inner_count
        optimizer.x[: 6 * inner_count] = coefficients[1:, 0, :].reshape(-1)
        optimizer.x[6 * inner_count: optimizer.active_spatial_dim] = (
            coefficients[1:, 1, :] / times[1:, None]).reshape(-1)
        optimizer.times = times
        return optimizer

    def evaluate(self, case_dir):
        left = self._load("left", case_dir)
        right = self._load("right", case_dir)
        left_position, left_orientation = left.cartesian_samples()
        right_position, right_orientation = right.cartesian_samples()
        left_evaluation = left.evaluate(right_position, right_orientation)
        right_evaluation = right.evaluate(left_position, left_orientation)
        components = {
            key: left_evaluation.costs[key] + right_evaluation.costs[key]
            for key in COST_KEYS
        }
        return {
            "total": left_evaluation.total_cost + right_evaluation.total_cost,
            "components": components,
        }


def case_metrics(summary, return_code):
    left = summary["left"]
    right = summary["right"]
    native = summary.get("native_validation") or {}
    kinematics = summary.get("candidate_kinematics") or {}
    return {
        "return_code": int(return_code),
        "optimization_completed": bool(summary.get("optimization_completed")),
        "successful": bool(summary.get("accepted_for_control")),
        "native_fcl_valid": bool(native.get("valid")),
        "native_invalid_samples": int(native.get("invalid_sample_count", -1)),
        "native_failure_reasons": native.get("reason_counts") or {},
        "continuous_kinematics_valid": bool(
            kinematics and all(value.get("valid")
                               for value in kinematics.values())),
        "duration_s": float(summary["shared_duration"]),
        "path_length_sum_rad": float(
            left["optimized_path_length"] + right["optimized_path_length"]),
        "joint_velocity_cost_sum": float(
            left["joint_velocity_squared_integral"] +
            right["joint_velocity_squared_integral"]),
        "optimization_wall_time_max_s": float(max(
            left["optimization_wall_time_s"],
            right["optimization_wall_time_s"])),
        "left_iterations": int(left["iterations"]),
        "right_iterations": int(right["iterations"]),
        "left_converged": bool(left["converged"]),
        "right_converged": bool(right["converged"]),
        "source_sample_count": int(left["sample_count"]),
    }


def copy_selected_controller(source_root, baseline_dir, output, summary):
    destination = output / "selected_x1_controller"
    if destination.exists():
        shutil.rmtree(str(destination))
    shutil.copytree(str(source_root), str(destination))
    shutil.rmtree(str(destination / "catch"))
    shutil.copytree(str(baseline_dir), str(destination / "catch"))
    inputs = destination / "inputs"
    inputs.mkdir(parents=True, exist_ok=True)
    shutil.copy2(str(baseline_dir / "options.json"),
                 str(inputs / "catch_selected_x1_options.json"))
    for side in ("left", "right"):
        shutil.copy2(
            str(summary["input"][side]),
            str(inputs / f"catch_{side}_planned_path.txt"))
    shutil.copy2(
        str(summary["input"]["constraint_config"]),
        str(inputs / "catch_template.yaml"))

    catch_summary_path = destination / "catch/catch_distributed_trajopt_summary.json"
    catch_summary = json.loads(catch_summary_path.read_text())
    for side in ("left", "right"):
        accepted = catch_summary["accepted_outputs"][side]
        accepted["sampled_path"] = str(
            destination / f"catch/catch_{side}_smoothed_path.txt")
        accepted["sample_times"] = str(
            destination / f"catch/catch_{side}_sample_times.txt")
        accepted["coefficients"] = str(
            destination / f"catch/catch_{side}_cubic_coefficients.json")
        accepted["gradient_descent_trace"] = str(
            destination / f"catch/catch_{side}_gradient_descent_trace.tsv")
    catch_summary_path.write_text(json.dumps(catch_summary, indent=2) + "\n")

    manifest_path = destination / "canonical_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    weights = summary["x1_weights"]
    manifest["schema"] = "CM_CANONICAL_BALL_TRAJECTORY_V4"
    manifest["created_local"] = dt.datetime.now().astimezone().isoformat()
    manifest["run_root"] = str(destination)
    manifest["source_controller_run"] = str(source_root)
    catch_optimization = {
        "architecture": "two_process_distributed_cubic_trajectory_optimization",
        "samples_per_segment_including_endpoints": 3,
        "interior_optimization_samples_per_segment": 1,
        "optimizable_internal_trajectory_knots_per_arm": 1,
        "temporal_reference_tube": summary["temporal_reference_tube"],
        "weights": weights,
        "weight_time": weights["weight_time"],
        "collision_cost_in_objective": False,
        "collision_gradient_used": False,
        "x0_x1_x2_sweep_summary": str(output / "sweep_summary.json"),
        "options_sha256": sha256(
            inputs / "catch_selected_x1_options.json"),
    }
    baseline = summary["x1_metrics"]
    catch_execution = {
        "source_dt_s": 0.01,
        "reference_stride": 1,
        "duration_s": baseline["duration_s"],
        "sample_count": baseline["source_sample_count"],
        "left_path_sha256": sha256(
            destination / "catch/catch_left_smoothed_path.txt"),
        "right_path_sha256": sha256(
            destination / "catch/catch_right_smoothed_path.txt"),
        "sample_times_sha256": sha256(
            destination / "catch/catch_left_sample_times.txt"),
    }
    catch_gate = {
        "performed": True,
        "accepted_for_control": baseline["successful"],
        "valid": baseline["native_fcl_valid"],
        "invalid_sample_count": baseline["native_invalid_samples"],
        "role": "post-optimization success criterion only",
        "used_as_cost_or_gradient": False,
    }
    by_phase = manifest.setdefault("trajectory_optimization_by_phase", {})
    if "move" not in by_phase and manifest.get("trajectory_optimization"):
        by_phase["move"] = manifest["trajectory_optimization"]
    by_phase["catch"] = catch_optimization
    execution_by_phase = manifest.setdefault("execution_by_phase", {})
    if "move" not in execution_by_phase and manifest.get("execution"):
        execution_by_phase["move"] = manifest["execution"]
    execution_by_phase["catch"] = catch_execution
    gate_by_phase = manifest.setdefault("post_fcl_acceptance_gate_by_phase", {})
    if "move" not in gate_by_phase and manifest.get("post_fcl_acceptance_gate"):
        gate_by_phase["move"] = manifest["post_fcl_acceptance_gate"]
    gate_by_phase["catch"] = catch_gate
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return destination


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--left-path", type=Path)
    parser.add_argument("--right-path", type=Path)
    parser.add_argument("--reference-total-duration", type=float)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--x1-weight-path", type=float, default=150.0)
    parser.add_argument("--x1-weight-velocity", type=float, default=1.0)
    parser.add_argument("--x1-weight-kinematic", type=float, default=1000.0)
    parser.add_argument("--x1-weight-task", type=float, default=1.0e7)
    parser.add_argument("--x1-weight-time", type=float, default=1.0)
    parser.add_argument("--time-reference-ratio", type=float, default=1.03)
    parser.add_argument(
        "--minimum-x0-target-deterioration-percent", type=float, default=1.0)
    parser.add_argument(
        "--maximum-x2-target-benefit-percent", type=float, default=5.0)
    parser.add_argument("--promote", action="store_true")
    args = parser.parse_args()

    package = args.workspace / "src/cm_kirara_ball"
    selector = package / "generated/latest_controller_run.txt"
    source_root = Path(selector.read_text().strip()).resolve()
    source_catch = source_root / "catch"
    stored_left = source_root / "inputs/catch_left_planned_path.txt"
    stored_right = source_root / "inputs/catch_right_planned_path.txt"
    stored_config = source_root / "inputs/catch_template.yaml"
    left_input = (args.left_path.resolve() if args.left_path else
                  stored_left if stored_left.exists() else
                  package / "generated/socket_latest/catch_left_planned_path.txt")
    right_input = (args.right_path.resolve() if args.right_path else
                   stored_right if stored_right.exists() else
                   package / "generated/socket_latest/catch_right_planned_path.txt")
    constraint_config = (stored_config if stored_config.exists() else
                         package / "config/catch_template.yaml")
    constraint_document = yaml.safe_load(constraint_config.read_text()) or {}
    validation_urdf = Path(
        constraint_document["file_path"]["pinocchio_file"]).resolve()
    gazebo_scene_xacro = (
        package / "models/urdf/gazebo_newbase.xacro").resolve()
    runner = (args.workspace /
              "devel/lib/cm_trajectory_optimization/run_distributed_trajopt_pair.py")
    validator = (args.workspace /
                 "devel/lib/cm_kirara_ball/CM_SOCKET_BALL_VALIDATOR")
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output = (args.output_dir.resolve() if args.output_dir else
              package / "generated" /
              f"trajopt_weight_sweep_grasp_single_insert_{stamp}")
    output.mkdir(parents=True, exist_ok=True)

    common = json.loads(
        (source_catch / "catch_left_resolved_options.json").read_text())
    common["samples_per_segment"] = 3
    common["interior_optimization_samples_per_segment"] = 1
    common.setdefault("spatial_step", 0.02)
    if args.time_reference_ratio < 1.0:
        raise ValueError("time-reference-ratio must be at least 1.0")
    source_catch_summary = json.loads((
        source_catch / "catch_distributed_trajopt_summary.json").read_text())
    source_point_count = int(np.atleast_2d(
        np.loadtxt(str(left_input))).shape[0])
    # samples_per_segment controls quadrature/evaluation density.  It does not
    # split one planned edge into multiple cubic pieces; two planned endpoints
    # therefore still define exactly one time segment.
    piece_count = source_point_count - 1
    if piece_count <= 0:
        raise RuntimeError("catch input must contain at least two points")
    reference_total_duration = (
        float(args.reference_total_duration)
        if args.reference_total_duration is not None else
        float(source_catch_summary["shared_duration"]))
    if reference_total_duration <= 0.0:
        raise ValueError("reference-total-duration must be positive")
    reference_piece_duration = reference_total_duration / piece_count
    common["initial_segment_duration"] = reference_piece_duration
    common["minimum_segment_duration"] = (
        reference_piece_duration / args.time_reference_ratio)
    common["maximum_segment_duration"] = (
        reference_piece_duration * args.time_reference_ratio)
    right_options = json.loads(
        (source_catch / "catch_right_resolved_options.json").read_text())
    for key in WEIGHT_KEYS:
        if float(common[key]) != float(right_options[key]):
            raise RuntimeError(f"left/right x1 option mismatch: {key}")
    x1_weights = {key: float(common[key]) for key in WEIGHT_KEYS}
    x1_weights["weight_path"] = args.x1_weight_path
    x1_weights["weight_velocity"] = args.x1_weight_velocity
    x1_weights["weight_joint_limit"] = args.x1_weight_kinematic
    x1_weights["weight_velocity_limit"] = args.x1_weight_kinematic
    x1_weights["weight_task_position_grasp"] = args.x1_weight_task
    x1_weights["weight_task_orientation_grasp"] = args.x1_weight_task
    x1_weights["weight_time"] = args.x1_weight_time
    fixed_options = dict(common)
    fixed_options.update(x1_weights)
    fixed_options.update({"object": "ball", "phase": "catch"})
    inputs = {
        "left": np.atleast_2d(np.loadtxt(str(left_input))),
        "right": np.atleast_2d(np.loadtxt(str(right_input))),
    }
    internal_knot_count = int(inputs["left"].shape[0] - 2)
    if internal_knot_count != 1 or inputs["right"].shape[0] - 2 != 1:
        raise RuntimeError(
            "current Grasp experiment requires exactly one optimizable "
            "internal trajectory knot per arm")
    objective_evaluator = FixedObjectiveEvaluator(
        args.workspace, inputs, fixed_options)
    all_cases = []

    def run_case(name, weights):
        case_dir = output / "cases" / name
        case_dir.mkdir(parents=True, exist_ok=True)
        options = dict(common)
        options.update(weights)
        options_path = case_dir / "options.json"
        options_path.write_text(json.dumps(options, indent=2) + "\n")
        summary_path = case_dir / "catch_distributed_trajopt_summary.json"
        console_path = case_dir / "runner_console.log"
        command = [
            str(runner), "--left-path", str(left_input),
            "--right-path", str(right_input), "--output-dir", str(case_dir),
            "--object", "ball", "--phase", "catch", "--port",
            str(available_port()), "--options-json", str(options_path),
            "--max-iterations", str(args.max_iterations), "--output-dt", "0.01",
            "--timeout", "120", "--constraint-config", str(constraint_config),
            "--native-validator", str(validator), "--validator-config",
            str(constraint_config), "--require-native-valid",
            "--validation-max-joint-step", "0.005",
        ]
        with console_path.open("w") as console:
            completed = subprocess.run(
                command, stdout=console, stderr=subprocess.STDOUT, check=False)
        if not summary_path.exists():
            raise RuntimeError(f"case {name} produced no summary")
        distributed = json.loads(summary_path.read_text())
        if not distributed.get("optimization_completed"):
            raise RuntimeError(f"case {name} did not complete optimization")
        metrics = case_metrics(distributed, completed.returncode)
        fixed_objective = objective_evaluator.evaluate(case_dir)
        record = {
            "name": name,
            "weights": {key: float(options[key]) for key in WEIGHT_KEYS},
            "metrics": metrics,
            "fixed_x1_objective": fixed_objective,
            "directory": str(case_dir),
        }
        (case_dir / "case_result.json").write_text(
            json.dumps(record, indent=2) + "\n")
        all_cases.append(record)
        print(json.dumps({
            "name": name,
            "duration_s": metrics["duration_s"],
            "velocity_integral": metrics["joint_velocity_cost_sum"],
            "fixed_x1_total_J": fixed_objective["total"],
            "optimization_completed": metrics["optimization_completed"],
            "successful": metrics["successful"],
        }, separators=(",", ":")), flush=True)
        return record

    baseline = run_case("x1_baseline", x1_weights)
    records = {}
    for group, weight_keys, component_keys in GROUPS:
        records[group] = {1.0: baseline}
        for scale in (0.0, 2.0):
            weights = dict(x1_weights)
            for weight_key in weight_keys:
                weights[weight_key] = scale * x1_weights[weight_key]
            records[group][scale] = run_case(
                f"{group}_x{int(scale)}", weights)

    baseline_total = baseline["fixed_x1_objective"]["total"]
    baseline_metrics = baseline["metrics"]
    table = []
    conclusions = {}
    for group, weight_keys, component_keys in GROUPS:
        for scale in (0.0, 1.0, 2.0):
            record = records[group][scale]
            objective = record["fixed_x1_objective"]
            metrics = record["metrics"]
            table.append({
                "group": group,
                "weight_keys": list(weight_keys),
                "scale": scale,
                "candidate_weights": {
                    key: record["weights"][key] for key in weight_keys
                },
                "optimization_completed": metrics["optimization_completed"],
                "successful": metrics["successful"],
                "success_rate_percent": 100.0 if metrics["successful"] else 0.0,
                "native_invalid_samples": metrics["native_invalid_samples"],
                "path_length_sum_rad": metrics["path_length_sum_rad"],
                "path_length_change_vs_x1_percent": relative_change_percent(
                    metrics["path_length_sum_rad"],
                    baseline_metrics["path_length_sum_rad"]),
                "joint_velocity_cost_sum": metrics["joint_velocity_cost_sum"],
                "joint_velocity_cost_change_vs_x1_percent": relative_change_percent(
                    metrics["joint_velocity_cost_sum"],
                    baseline_metrics["joint_velocity_cost_sum"]),
                "execution_time_s": metrics["duration_s"],
                "execution_time_change_vs_x1_percent": relative_change_percent(
                    metrics["duration_s"], baseline_metrics["duration_s"]),
                "optimization_time_s": metrics["optimization_wall_time_max_s"],
                "optimization_time_change_vs_x1_percent": relative_change_percent(
                    metrics["optimization_wall_time_max_s"],
                    baseline_metrics["optimization_wall_time_max_s"]),
                "fixed_x1_total_objective": objective["total"],
                "fixed_x1_objective_components": dict(
                    objective["components"]),
                "total_objective_change_vs_x1_percent": (
                    100.0 * (objective["total"] - baseline_total) /
                    max(abs(baseline_total), 1.0e-12)),
                "fixed_x1_target_component": sum(
                    objective["components"][key] for key in component_keys),
            })
        group_rows = [row for row in table if row["group"] == group]
        by_scale = {row["scale"]: row for row in group_rows}
        gap_x0 = by_scale[0.0]["total_objective_change_vs_x1_percent"]
        gap_x2 = by_scale[2.0]["total_objective_change_vs_x1_percent"]
        target_values = [
            by_scale[scale]["fixed_x1_target_component"]
            for scale in (0.0, 1.0, 2.0)
        ]
        success = {
            scale: by_scale[scale]["successful"]
            for scale in (0.0, 1.0, 2.0)
        }
        component_values = {
            key: [
                by_scale[scale]["fixed_x1_objective_components"][key]
                for scale in (0.0, 1.0, 2.0)
            ]
            for key in COST_KEYS
        }
        varying_components = []
        for key, values in component_values.items():
            scale = max(1.0, max(abs(value) for value in values))
            if max(values) - min(values) > 1.0e-8 * scale:
                varying_components.append(key)
        target_metric_key = {
            "path": "path_length_change_vs_x1_percent",
            "velocity": "joint_velocity_cost_change_vs_x1_percent",
            "time": "execution_time_change_vs_x1_percent",
        }.get(group)
        target_metric_changes = None
        if target_metric_key is not None:
            target_metric_changes = {
                scale: by_scale[scale][target_metric_key]
                for scale in (0.0, 1.0, 2.0)
            }
        inactive = not varying_components
        strict_minimum = gap_x0 > 0.0 and gap_x2 > 0.0
        x2_target_benefit = (
            max(0.0, -target_metric_changes[2.0])
            if target_metric_changes is not None else None)
        x0_worse = (not success[0.0]) or (
            target_metric_changes is not None and gap_x0 > 1.0e-9 and
            (target_metric_changes[0.0] >=
             args.minimum_x0_target_deterioration_percent or
             gap_x0 >= args.minimum_x0_target_deterioration_percent))
        x2_worse = (not success[2.0]) or (
            target_metric_changes is not None and gap_x2 > 1.0e-9 and
            (x2_target_benefit <= args.maximum_x2_target_benefit_percent or
             gap_x2 >= x2_target_benefit))
        supported = success[1.0] and x0_worse and x2_worse
        if not success[1.0]:
            status = "x1_failed_success_gate"
        elif supported:
            status = "supported_by_strict_user_rule"
        elif inactive:
            status = "inactive_on_this_trajectory"
        elif strict_minimum:
            status = "strict_total_minimum_but_user_rule_failed"
        else:
            status = "x1_not_supported"
        conclusions[group] = {
            "weight_keys": list(weight_keys),
            "x1_weights": {
                key: x1_weights[key] for key in weight_keys
            },
            "status": status,
            "x1_is_strict_total_objective_minimum": strict_minimum,
            "x1_supported_by_complete_rule": supported,
            "success": {"x0": success[0.0], "x1": success[1.0], "x2": success[2.0]},
            "x0_total_objective_gap_percent": gap_x0,
            "x2_total_objective_gap_percent": gap_x2,
            "target_metric_change_vs_x1_percent": (
                None if target_metric_changes is None else {
                    "x0": target_metric_changes[0.0],
                    "x1": target_metric_changes[1.0],
                    "x2": target_metric_changes[2.0],
                }),
            "x2_target_benefit_percent": x2_target_benefit,
            "minimum_x0_target_deterioration_percent": (
                args.minimum_x0_target_deterioration_percent),
            "maximum_x2_target_benefit_percent": (
                args.maximum_x2_target_benefit_percent),
            "varying_fixed_objective_components": varying_components,
            "fixed_x1_target_component": {
                "x0": target_values[0],
                "x1": target_values[1],
                "x2": target_values[2],
            },
        }

    baseline_report_total = 0.0
    for side in ("left", "right"):
        report = json.loads((
            Path(baseline["directory"]) /
            f"catch_{side}_trajopt_report.json").read_text())
        baseline_report_total += float(report["final_total_cost"])
    cross_evaluation_error = abs(baseline_report_total - baseline_total)
    if cross_evaluation_error > 1.0e-8:
        raise RuntimeError(
            "fixed-objective reconstruction mismatch: "
            f"{cross_evaluation_error:.3e}")

    required_groups = ("path", "velocity", "time")
    recommended = all(
        conclusions[group]["x1_supported_by_complete_rule"]
        for group in required_groups)
    result = {
        "status": "complete",
        "scope": "Ball Grasp trajectory optimization x0/x1/x2",
        "selection_rule": {
            "objective": (
                "Every x0/x1/x2 trajectory is re-evaluated with the identical "
                "complete x1 paper-form objective J_left + J_right"),
            "x1_supported_when": (
                "x1 passes optimization/kinematics/post-FCL; x0 either fails "
                "or makes either the swept target metric or the complete "
                "fixed-x1 objective materially worse, with the complete "
                "objective required to rise; x2 either fails, has no "
                "more than the allowed marginal target benefit while the "
                "complete objective rises, or its complete-objective rise is "
                "larger than its target benefit"),
            "minimum_x0_target_deterioration_percent": (
                args.minimum_x0_target_deterioration_percent),
            "maximum_x2_target_benefit_percent": (
                args.maximum_x2_target_benefit_percent),
            "required_identifiable_groups": list(required_groups),
            "complete_total_objective_is_not_replaced_by_a_"
            "constant_stripped_score": True,
            "no_normalized_balance_score": True,
            "success_precedes_energy": True,
        },
        "source_controller_run": str(source_root),
        "interior_optimization_samples_per_segment": 1,
        "optimizable_internal_trajectory_knots_per_arm": internal_knot_count,
        "samples_per_segment": 3,
        "temporal_reference_tube": {
            "source_total_duration_s": reference_total_duration,
            "piece_count": piece_count,
            "reference_piece_duration_s": reference_piece_duration,
            "ratio": args.time_reference_ratio,
            "minimum_piece_duration_s": common[
                "minimum_segment_duration"],
            "maximum_piece_duration_s": common[
                "maximum_segment_duration"],
        },
        "input": {
            "left": str(left_input), "left_sha256": sha256(left_input),
            "right": str(right_input), "right_sha256": sha256(right_input),
            "constraint_config": str(constraint_config),
            "constraint_config_sha256": sha256(constraint_config),
            "validation_urdf": str(validation_urdf),
            "validation_urdf_sha256": sha256(validation_urdf),
            "gazebo_scene_xacro": str(gazebo_scene_xacro),
            "gazebo_scene_xacro_sha256": sha256(gazebo_scene_xacro),
        },
        "x1_weights": x1_weights,
        "x1_metrics": baseline["metrics"],
        "x1_fixed_objective": baseline["fixed_x1_objective"],
        "fixed_objective_cross_evaluation_error": cross_evaluation_error,
        "conclusions": conclusions,
        "supported_weight_groups": [
            group for group, value in conclusions.items()
            if value["x1_supported_by_complete_rule"]
        ],
        "not_identifiable_or_not_supported_groups": [
            group for group, value in conclusions.items()
            if not value["x1_supported_by_complete_rule"]
        ],
        "selected_by_success_then_energy_rule": recommended,
        "recommendation_basis": (
            "post-optimization success is checked first; target-metric "
            "deterioration/saturation and the unchanged complete fixed-x1 "
            "objective are then checked using the user's strict rule"),
        "collision_evaluation": {
            "performed": True,
            "part_of_parameter_selection": True,
            "role": "post-optimization binary success gate",
            "collision_cost_in_optimizer": False,
            "collision_gradient_in_optimizer": False,
            "successful_case_count": sum(
                bool(case["metrics"]["successful"]) for case in all_cases),
            "case_count": len(all_cases),
        },
        "x0_x1_x2": table,
        "case_count": len(all_cases),
        "cases": all_cases,
        "selector_updated": False,
    }
    summary_path = output / "sweep_summary.json"
    summary_path.write_text(json.dumps(result, indent=2) + "\n")
    csv_path = output / "x0_x1_x2.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0].keys()))
        writer.writeheader()
        writer.writerows(table)

    if args.promote:
        if not recommended:
            raise RuntimeError(
                "x1 was not promoted because the fixed-objective rule failed")
        selected = copy_selected_controller(
            source_root, Path(baseline["directory"]), output, result)
        result["selector_updated"] = True
        result["selected_controller_run"] = str(selected)
        summary_path.write_text(json.dumps(result, indent=2) + "\n")
        temporary_selector = selector.with_suffix(".tmp")
        temporary_selector.write_text(str(selected) + "\n")
        temporary_selector.replace(selector)

    print(json.dumps({
        "complete": True,
        "output": str(output),
        "summary": str(summary_path),
        "x1_weights": x1_weights,
        "conclusions": conclusions,
        "selected_by_success_then_energy_rule": recommended,
        "selector_updated": result["selector_updated"],
    }, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
