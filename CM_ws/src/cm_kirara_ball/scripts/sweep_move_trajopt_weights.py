#!/usr/bin/env python3
"""Ball Move trajectory-optimization x0/x1/x2 sweep on one fixed reference.

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
    "weight_task_position_transport",
    "weight_task_orientation_transport",
    "weight_time",
)
GROUPS = (
    ("path", ("weight_path",), ("path",)),
    ("velocity", ("weight_velocity",), ("velocity",)),
    ("kinematic_limits",
     ("weight_joint_limit", "weight_velocity_limit"),
     ("joint_position", "joint_velocity")),
    ("task_transport",
     ("weight_task_position_transport", "weight_task_orientation_transport"),
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

    def __init__(self, workspace, inputs, fixed_weights):
        sys.path.insert(0, str(
            workspace / "src/cm_trajectory_optimization/scripts"))
        from cm_trajopt_common import LocalCubicTrajectoryOptimizer
        self.optimizer_class = LocalCubicTrajectoryOptimizer
        self.inputs = inputs
        self.fixed_weights = dict(fixed_weights)

    def _load(self, side, case_dir):
        payload = json.loads((
            case_dir / f"move_{side}_cubic_coefficients.json").read_text())
        coefficients = np.asarray(payload["coefficients_Mx4x6"], dtype=float)
        times = np.asarray(payload["segment_times"], dtype=float)
        # Resolve the fixed objective from the options that the runner actually
        # used for this side.  In particular, the two URDF arms can have
        # different joint-velocity limits.  Reusing the left-side template for
        # both sides is invisible while the limit term is inactive, but gives a
        # wrong cross-evaluation as soon as the velocity constraint is active.
        options = json.loads((
            case_dir / f"move_{side}_resolved_options.json").read_text())
        options.update(self.fixed_weights)
        options.update({"object": "ball", "phase": "move"})
        optimizer = self.optimizer_class(
            side, self.inputs[side], options)
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
    shutil.rmtree(str(destination / "move"))
    shutil.copytree(str(baseline_dir), str(destination / "move"))
    shutil.copy2(str(baseline_dir / "options.json"),
                 str(destination / "inputs/move_selected_x1_options.json"))

    move_summary_path = destination / "move/move_distributed_trajopt_summary.json"
    move_summary = json.loads(move_summary_path.read_text())
    for side in ("left", "right"):
        accepted = move_summary["accepted_outputs"][side]
        accepted["sampled_path"] = str(
            destination / f"move/move_{side}_smoothed_path.txt")
        accepted["sample_times"] = str(
            destination / f"move/move_{side}_sample_times.txt")
    move_summary_path.write_text(json.dumps(move_summary, indent=2) + "\n")

    manifest_path = destination / "canonical_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    weights = summary["x1_weights"]
    manifest["schema"] = "CM_CANONICAL_BALL_TRAJECTORY_V3"
    manifest["created_local"] = dt.datetime.now().astimezone().isoformat()
    manifest["run_root"] = str(destination)
    manifest["source_controller_run"] = str(source_root)
    manifest["trajectory_optimization"].update({
        "samples_per_segment_including_endpoints": 3,
        "interior_optimization_samples_per_segment": 1,
        "weights": weights,
        "weight_time": weights["weight_time"],
        "collision_cost_in_objective": False,
        "collision_gradient_used": False,
        "x0_x1_x2_sweep_summary": str(output / "sweep_summary.json"),
        "options_sha256": sha256(
            destination / "inputs/move_selected_x1_options.json"),
    })
    baseline = summary["x1_metrics"]
    manifest["execution"].update({
        "source_dt_s": 0.01,
        "reference_stride": 1,
        "duration_s": baseline["duration_s"],
        "sample_count": baseline["source_sample_count"],
        "left_path_sha256": sha256(
            destination / "move/move_left_smoothed_path.txt"),
        "right_path_sha256": sha256(
            destination / "move/move_right_smoothed_path.txt"),
        "sample_times_sha256": sha256(
            destination / "move/move_left_sample_times.txt"),
    })
    manifest["post_fcl_acceptance_gate"] = {
        "performed": True,
        "accepted_for_control": baseline["successful"],
        "valid": baseline["native_fcl_valid"],
        "invalid_sample_count": baseline["native_invalid_samples"],
        "role": "post-optimization success criterion only",
        "used_as_cost_or_gradient": False,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return destination


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--left-path", type=Path)
    parser.add_argument("--right-path", type=Path)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--x1-weight-velocity", type=float, default=1.0)
    parser.add_argument("--x1-weight-time", type=float, default=1.0)
    parser.add_argument(
        "--x1-weights-json",
        help=("JSON object overriding any complete x1 weight; keys must be "
              "members of the trajectory objective weight set"))
    parser.add_argument("--maximum-knot-deviation", type=float)
    parser.add_argument("--maximum-segment-duration", type=float)
    parser.add_argument("--task-distance-safety-margin-m", type=float)
    parser.add_argument("--task-orientation-safety-margin-rad", type=float)
    parser.add_argument("--velocity-limit-scale", type=float)
    parser.add_argument("--material-gap-percent", type=float, default=1.0)
    parser.add_argument(
        "--groups",
        default=",".join(group[0] for group in GROUPS),
        help="Comma-separated ablation groups; useful for calibration runs")
    parser.add_argument("--promote", action="store_true")
    args = parser.parse_args()

    requested_groups = tuple(
        value.strip() for value in args.groups.split(",") if value.strip())
    known_groups = {group[0] for group in GROUPS}
    unknown_groups = sorted(set(requested_groups) - known_groups)
    if unknown_groups:
        raise ValueError("unknown groups: " + ", ".join(unknown_groups))
    selected_groups = tuple(
        group for group in GROUPS if group[0] in requested_groups)
    if not selected_groups:
        raise ValueError("at least one ablation group is required")

    package = args.workspace / "src/cm_kirara_ball"
    selector = package / "generated/latest_controller_run.txt"
    source_root = Path(selector.read_text().strip()).resolve()
    source_move = source_root / "move"
    stored_left = source_root / "inputs/move_left_planned_path.txt"
    stored_right = source_root / "inputs/move_right_planned_path.txt"
    left_input = args.left_path.resolve() if args.left_path else stored_left
    right_input = args.right_path.resolve() if args.right_path else stored_right
    constraint_config = source_root / "inputs/move_template.yaml"
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
    output = args.output_dir or (
        package / "generated" / f"trajopt_weight_sweep_single_insert_{stamp}")
    output.mkdir(parents=True, exist_ok=True)

    common = json.loads(
        (source_move / "move_left_resolved_options.json").read_text())
    if args.maximum_knot_deviation is not None:
        if args.maximum_knot_deviation <= 0.0:
            raise ValueError("maximum knot deviation must be positive")
        common["maximum_knot_deviation"] = args.maximum_knot_deviation
    if args.maximum_segment_duration is not None:
        if args.maximum_segment_duration <= 0.0:
            raise ValueError("maximum segment duration must be positive")
        if args.maximum_segment_duration < float(
                common.get("minimum_segment_duration", 0.0)):
            raise ValueError("maximum segment duration is below the minimum")
        common["maximum_segment_duration"] = args.maximum_segment_duration
    if args.task_distance_safety_margin_m is not None:
        margin = args.task_distance_safety_margin_m
        if margin < 0.0:
            raise ValueError("task distance safety margin must be nonnegative")
        common["task_distance_safety_margin_m"] = margin
        common["task_distance_min"] = float(common["task_distance_min"]) + margin
        common["task_distance_max"] = float(common["task_distance_max"]) - margin
        if common["task_distance_min"] >= common["task_distance_max"]:
            raise ValueError("task distance safety margin closes the interval")
    if args.task_orientation_safety_margin_rad is not None:
        margin = args.task_orientation_safety_margin_rad
        if margin < 0.0:
            raise ValueError(
                "task orientation safety margin must be nonnegative")
        common["task_orientation_safety_margin_rad"] = margin
    if args.velocity_limit_scale is not None:
        scale = args.velocity_limit_scale
        if not 0.0 < scale <= 1.0:
            raise ValueError("velocity limit scale must be in (0, 1]")
        common["velocity_limit_scale"] = scale
        common["velocity_max"] = [
            float(value) * scale for value in common["velocity_max"]]
    if common.get("samples_per_segment") != 3:
        raise RuntimeError("samples_per_segment must be 3")
    if common.get("interior_optimization_samples_per_segment") != 1:
        raise RuntimeError("interior optimization samples must be 1")
    right_options = json.loads(
        (source_move / "move_right_resolved_options.json").read_text())
    for key in WEIGHT_KEYS:
        if float(common[key]) != float(right_options[key]):
            raise RuntimeError(f"left/right x1 option mismatch: {key}")
    x1_weights = {key: float(common[key]) for key in WEIGHT_KEYS}
    x1_weights["weight_velocity"] = args.x1_weight_velocity
    x1_weights["weight_time"] = args.x1_weight_time
    if args.x1_weights_json:
        overrides = json.loads(args.x1_weights_json)
        unknown = sorted(set(overrides) - set(WEIGHT_KEYS))
        if unknown:
            raise ValueError("unknown x1 weight keys: " + ", ".join(unknown))
        for key, value in overrides.items():
            value = float(value)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError("x1 weights must be finite and nonnegative")
            x1_weights[key] = value
    inputs = {
        "left": np.atleast_2d(np.loadtxt(str(left_input))),
        "right": np.atleast_2d(np.loadtxt(str(right_input))),
    }
    objective_evaluator = FixedObjectiveEvaluator(
        args.workspace, inputs, x1_weights)
    all_cases = []

    def run_case(name, weights):
        case_dir = output / "cases" / name
        case_dir.mkdir(parents=True, exist_ok=True)
        options = dict(common)
        options.update(weights)
        options_path = case_dir / "options.json"
        options_path.write_text(json.dumps(options, indent=2) + "\n")
        summary_path = case_dir / "move_distributed_trajopt_summary.json"
        console_path = case_dir / "runner_console.log"
        command = [
            str(runner), "--left-path", str(left_input),
            "--right-path", str(right_input), "--output-dir", str(case_dir),
            "--object", "ball", "--phase", "move", "--port",
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
    for group, weight_keys, component_keys in selected_groups:
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
    for group, weight_keys, component_keys in selected_groups:
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
        inactive = max(abs(value) for value in target_values) <= 1.0e-12
        strict_minimum = gap_x0 > 0.0 and gap_x2 > 0.0
        material_minimum = (
            strict_minimum and
            min(gap_x0, gap_x2) >= args.material_gap_percent)
        x0_worse = (not success[0.0]) or gap_x0 >= args.material_gap_percent
        x2_worse = (not success[2.0]) or gap_x2 >= args.material_gap_percent
        supported = success[1.0] and x0_worse and x2_worse
        if not success[1.0]:
            status = "x1_failed_success_gate"
        elif supported:
            status = "supported_by_success_and_or_total_objective"
        elif inactive:
            status = "inactive_on_this_trajectory"
        elif strict_minimum:
            status = "strict_but_not_material"
        else:
            status = "x1_not_supported"
        conclusions[group] = {
            "weight_keys": list(weight_keys),
            "x1_weights": {
                key: x1_weights[key] for key in weight_keys
            },
            "status": status,
            "x1_is_strict_total_objective_minimum": strict_minimum,
            "x1_is_material_total_objective_minimum": material_minimum,
            "x1_supported_by_complete_rule": supported,
            "success": {"x0": success[0.0], "x1": success[1.0], "x2": success[2.0]},
            "x0_total_objective_gap_percent": gap_x0,
            "x2_total_objective_gap_percent": gap_x2,
            "materiality_threshold_percent": args.material_gap_percent,
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
            f"move_{side}_trajopt_report.json").read_text())
        baseline_report_total += float(report["final_total_cost"])
    cross_evaluation_error = abs(baseline_report_total - baseline_total)
    if cross_evaluation_error > 1.0e-8:
        raise RuntimeError(
            "fixed-objective reconstruction mismatch: "
            f"{cross_evaluation_error:.3e}")

    required_groups = tuple(
        group for group in ("velocity", "time")
        if group in requested_groups)
    recommended = all(
        conclusions[group]["x1_supported_by_complete_rule"]
        for group in required_groups)
    result = {
        "status": "complete",
        "scope": "Ball Move trajectory optimization x0/x1/x2",
        "selection_rule": {
            "objective": (
                "Every x0/x1/x2 trajectory is re-evaluated with the identical "
                "complete x1 paper-form objective J_left + J_right"),
            "x1_supported_when": (
                "x1 passes optimization/kinematics/post-FCL; each neighbor "
                "either fails that success gate or has materially higher "
                "complete fixed-x1 objective"),
            "materiality_threshold_percent": args.material_gap_percent,
            "no_normalized_balance_score": True,
            "success_precedes_energy": True,
        },
        "source_controller_run": str(source_root),
        "interior_optimization_samples_per_segment": 1,
        "samples_per_segment": 3,
        "task_distance_safety_margin_m": float(
            common.get("task_distance_safety_margin_m", 0.0)),
        "task_orientation_safety_margin_rad": float(
            common.get("task_orientation_safety_margin_rad", 0.0)),
        "velocity_limit_scale": float(
            common.get("velocity_limit_scale", 1.0)),
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
            "post-optimization success is checked first; successful neighbors "
            "are then compared using the same complete x1 objective"),
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
