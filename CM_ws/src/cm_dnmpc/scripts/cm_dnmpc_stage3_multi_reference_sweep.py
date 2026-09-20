#!/usr/bin/env python3
"""Stage-3 DNMPC x0/x1/x2 sweep on one frozen multi-trajectory pool."""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import csv
import hashlib
import json
import math
import time
from pathlib import Path

from cm_dnmpc_common import (
    LocalDNMPC,
    ReferenceTrajectory,
    WEIGHT_NAMES,
    load_json,
    resolve_stage_weight_error_scales,
    resolve_stage_weights,
    validate_weights,
)
from cm_dnmpc_stage3_sweep import (
    add_realistic_robustness_args,
    aggregate,
    run_trial,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--reference-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--trace-output", type=Path,
        help="optional directory for per-cycle joint traces")
    parser.add_argument("--weight", choices=WEIGHT_NAMES, required=True)
    parser.add_argument(
        "--values", required=True,
        help="comma-separated absolute controller weights")
    parser.add_argument("--base-weights-json")
    parser.add_argument("--evaluation-weights-json")
    parser.add_argument(
        "--steps", type=int, default=150,
        help="Stage-3 control cycles evaluated on every trajectory")
    parser.add_argument(
        "--start-fraction", type=float, default=0.42,
        help="relative start of the fixed Stage-3 evaluation window")
    parser.add_argument("--initial-error-std-rad", type=float, default=0.001)
    parser.add_argument(
        "--measurement-noise-std-rad", type=float, default=0.0)
    parser.add_argument(
        "--acceleration-limit-rad-s2", type=float, default=0.0,
        help=("optional per-joint plant acceleration limit; zero keeps the "
              "paper's ideal joint-velocity plant (default)"))
    parser.add_argument("--desired-force-n", type=float, default=15.0)
    parser.add_argument("--object-stiffness-n-m", type=float, default=1000.0)
    parser.add_argument("--object-damping-n-s-m", type=float, default=8.0)
    parser.add_argument(
        "--force-equilibrium-mode",
        choices=("reference_desired", "runtime_absolute"),
        default="reference_desired",
        help=("reference_desired preserves the historical sweep model; "
              "runtime_absolute uses the same absolute object-length "
              "compression model as the Gazebo controller"))
    parser.add_argument(
        "--object-length-m", type=float,
        help=("undeformed object length for runtime_absolute; defaults to "
              "paper_elastic.ball.move.object_length_m from the config"))
    parser.add_argument("--force-delay-cycles", type=int, default=1)
    parser.add_argument("--bhat-left", type=float, default=8.0)
    parser.add_argument("--bhat-right", type=float, default=10.0)
    parser.add_argument("--wb-left", type=float, default=8.0)
    parser.add_argument("--wb-right", type=float, default=10.0)
    parser.add_argument("--wk-left", type=float, default=800.0)
    parser.add_argument("--wk-right", type=float, default=1000.0)
    parser.add_argument("--k0", type=float, default=1000.0)
    parser.add_argument(
        "--max-aic-displacement-m", type=float,
        help=("optional public AIC Cartesian displacement saturation; "
              "defaults to paper_elastic.ball.move.max_displacement_m"))
    parser.add_argument("--disturbance-start-fraction", type=float, default=0.45)
    parser.add_argument("--disturbance-duration-s", type=float, default=0.30)
    parser.add_argument("--disturbance-amplitude-rad-s", type=float, default=0.45)
    parser.add_argument(
        "--orientation-disturbance-amplitude-rad-s",
        type=float, default=0.0)
    parser.add_argument(
        "--slip-onset-force-n", type=float, default=10.0,
        help="measured slip-onset force; this is not the drop threshold")
    parser.add_argument(
        "--drop-force-multiplier", type=float, default=1.5,
        help=("drop threshold multiplier applied to the measured slip-onset "
              "force; default 1.5 gives 15 N for a 10 N onset force"))
    parser.add_argument("--overforce-limit-n", type=float, default=25.0)
    parser.add_argument("--slip-hold-cycles", type=int, default=1)
    parser.add_argument("--overforce-hold-cycles", type=int, default=1)
    parser.add_argument("--grace-cycles", type=int, default=0)
    parser.add_argument("--terminal-target-error-mm", type=float, default=30.0)
    parser.add_argument("--orientation-rmse-deg", type=float, default=30.0)
    parser.add_argument(
        "--jobs", type=int, default=1,
        help="parallel independent trajectory trials; one preserves legacy order")
    add_realistic_robustness_args(parser)
    return parser.parse_args()


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_parallel_trial(payload):
    (args, config, item, value_index, value, base_weights,
     evaluation_weights, weight_error_scales) = payload
    controller_weights = dict(base_weights)
    controller_weights[args.weight] = value
    label = "v{:02d}".format(value_index)
    references = {
        side: ReferenceTrajectory(
            Path(item["reference_" + side]), config["source_dt"],
            sample_times=(Path(item["sample_times"])
                          if item.get("sample_times") else None))
        for side in ("left", "right")}
    solvers = {
        side: LocalDNMPC(
            side, int(config["horizon"]), config,
            weights=controller_weights,
            weight_error_scales=weight_error_scales,
            solver_label="{}_{}_{}_{}".format(
                args.weight, label, item["trajectory_id"], side))
        for side in ("left", "right")}
    trial_args = copy.copy(args)
    trial_args.weight = args.weight
    trajectory_index = int(item["fixed_trial_index"])
    trace_rows = [] if args.trace_output else None
    row = run_trial(
        trial_args, config, references, controller_weights,
        evaluation_weights, trajectory_index, label, solvers,
        weight_error_scales=weight_error_scales,
        trace_rows=trace_rows)
    if trace_rows is not None:
        trace_path = (
            args.trace_output / label /
            "{}_joint_trace.csv".format(item["trajectory_id"]))
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        write_csv(trace_path, trace_rows)
        row["joint_trace"] = str(trace_path.resolve())
    row.update({
        "trajectory_id": item["trajectory_id"],
        "planner_seed": item.get("planner_seed"),
        "reference_left_sha256": item["reference_left_sha256"],
        "reference_right_sha256": item["reference_right_sha256"],
        "reference_duration_s": min(
            ref.duration for ref in references.values()),
        "control_steps": row.get("control_steps", trial_args.steps),
        "value_index": value_index,
        "trajectory_index": trajectory_index,
    })
    return row


def run_parallel_group(payload):
    (args, config, items, value_index, value, base_weights,
     evaluation_weights, weight_error_scales) = payload
    controller_weights = dict(base_weights)
    controller_weights[args.weight] = value
    label = "v{:02d}".format(value_index)
    solvers = {
        side: LocalDNMPC(
            side, int(config["horizon"]), config,
            weights=controller_weights,
            weight_error_scales=weight_error_scales,
            solver_label="{}_{}_group_{}".format(args.weight, label, side))
        for side in ("left", "right")}
    group_rows = []
    for item in items:
        references = {
            side: ReferenceTrajectory(
                Path(item["reference_" + side]), config["source_dt"],
                sample_times=(Path(item["sample_times"])
                              if item.get("sample_times") else None))
            for side in ("left", "right")}
        trajectory_index = int(item["fixed_trial_index"])
        trace_rows = [] if args.trace_output else None
        row = run_trial(
            args, config, references, controller_weights,
            evaluation_weights, trajectory_index, label, solvers,
            weight_error_scales=weight_error_scales,
            trace_rows=trace_rows)
        if trace_rows is not None:
            trace_path = (
                args.trace_output / label /
                "{}_joint_trace.csv".format(item["trajectory_id"]))
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            write_csv(trace_path, trace_rows)
            row["joint_trace"] = str(trace_path.resolve())
        row.update({
            "trajectory_id": item["trajectory_id"],
            "planner_seed": item.get("planner_seed"),
            "reference_left_sha256": item["reference_left_sha256"],
            "reference_right_sha256": item["reference_right_sha256"],
            "reference_duration_s": min(
                ref.duration for ref in references.values()),
            "control_steps": row.get("control_steps", args.steps),
            "value_index": value_index,
            "trajectory_index": trajectory_index,
        })
        group_rows.append(row)
    return group_rows


def main():
    args = parse_args()
    config = load_json(args.config)
    config["dnmpc_force_stiffness_n_m"] = args.object_stiffness_n_m
    base_weights = resolve_stage_weights(config, "ball", 3)
    if args.base_weights_json:
        base_weights = validate_weights(
            json.loads(args.base_weights_json), "base")
    evaluation_weights = dict(base_weights)
    if args.evaluation_weights_json:
        evaluation_weights = validate_weights(
            json.loads(args.evaluation_weights_json), "evaluation")
    weight_error_scales = resolve_stage_weight_error_scales(
        config, "ball", 3)
    values = [float(value) for value in args.values.split(",")]
    if any(not math.isfinite(value) or value < 0.0 for value in values):
        raise SystemExit("weight values must be finite and nonnegative")
    if (not math.isfinite(args.acceleration_limit_rad_s2)
            or args.acceleration_limit_rad_s2 < 0.0):
        raise SystemExit("acceleration limit must be finite and nonnegative")
    if (args.object_length_m is not None
            and (not math.isfinite(args.object_length_m)
                 or args.object_length_m <= 0.0)):
        raise SystemExit("object length must be finite and positive")
    if args.jobs < 1:
        raise SystemExit("jobs must be positive")

    pool = json.loads(args.reference_manifest.read_text(encoding="utf-8"))
    trajectories = list(pool.get("trajectories") or [])
    if len(trajectories) < 2:
        raise RuntimeError("multi-trajectory manifest contains fewer than 2 paths")
    if int(pool.get("unique_reference_pair_count", len(trajectories))) != len(
            trajectories):
        raise RuntimeError("reference pool contains duplicate trajectory pairs")
    for trajectory_index, item in enumerate(trajectories):
        item["fixed_trial_index"] = trajectory_index
        if int(item["interior_optimization_samples_per_segment"]) != 1:
            raise RuntimeError("reference pool contains a non-internal1 trajectory")
        for side in ("left", "right"):
            path = Path(item["reference_" + side])
            if sha256(path) != item["reference_" + side + "_sha256"]:
                raise RuntimeError("reference hash mismatch: " + str(path))

    args.output.mkdir(parents=True, exist_ok=False)
    raw = []
    started = time.time()
    dt = float(config["Ts"])
    horizon = int(config["horizon"])
    chunks_per_value = max(1, min(
        len(trajectories), args.jobs // max(1, len(values))))
    trajectory_chunks = [
        trajectories[index::chunks_per_value]
        for index in range(chunks_per_value)]
    tasks = [
        (args, config, chunk, value_index, value, base_weights,
         evaluation_weights, weight_error_scales)
        for value_index, value in enumerate(values)
        for chunk in trajectory_chunks if chunk]
    if args.jobs == 1:
        grouped_rows = map(run_parallel_group, tasks)
        for group in grouped_rows:
            for row in group:
                raw.append(row)
                print(
                    "RUN {} trajectory={} success={} Ei={:.9g}".format(
                        row["setting_label"], row["trajectory_id"],
                        int(row["task_success"]), row["common_eval_cost"]),
                    flush=True)
    else:
        with concurrent.futures.ProcessPoolExecutor(
                max_workers=args.jobs) as executor:
            futures = [executor.submit(run_parallel_group, task)
                       for task in tasks]
            for future in concurrent.futures.as_completed(futures):
                for row in future.result():
                    raw.append(row)
                    print(
                        "RUN {} trajectory={} success={} Ei={:.9g}".format(
                            row["setting_label"], row["trajectory_id"],
                            int(row["task_success"]), row["common_eval_cost"]),
                        flush=True)
    raw.sort(key=lambda row: (
        int(row["value_index"]), int(row["trajectory_index"])))

    summary = aggregate(raw)
    provenance = {
        "schema": "CM_DNMPC_STAGE3_MULTI_REFERENCE_SWEEP_V1",
        "created_unix": time.time(),
        "elapsed_s": time.time() - started,
        "config": str(args.config.resolve()),
        "reference_manifest": str(args.reference_manifest.resolve()),
        "reference_manifest_sha256": sha256(args.reference_manifest),
        "trajectory_count": len(trajectories),
        "trajectory_ids": [row["trajectory_id"] for row in trajectories],
        "trajectory_hashes": [{
            "trajectory_id": row["trajectory_id"],
            "left": row["reference_left_sha256"],
            "right": row["reference_right_sha256"],
        } for row in trajectories],
        "old_demonstration_trajectory_count": sum(
            int(row.get("source") == "existing_demonstration")
            for row in trajectories),
        "newly_planned_trajectory_count": sum(
            int(row.get("source") == "new_planning_run")
            for row in trajectories),
        "controller_base_weights": base_weights,
        "fixed_evaluation_weights": evaluation_weights,
        "weight_error_scales": weight_error_scales,
        "swept_weight": args.weight,
        "values": values,
        "parallel_jobs": args.jobs,
        "Ts_s": dt,
        "horizon": horizon,
        "control_steps_per_trajectory": (
            "full_remaining_reference_plus_{}_settle_cycles".format(
                args.settle_cycles)
            if args.run_full_reference else args.steps),
        "reference_start_fraction": args.start_fraction,
        "initial_error_std_rad": args.initial_error_std_rad,
        "measurement_noise_std_rad": args.measurement_noise_std_rad,
        "plant_model": {
            "type": ("per_joint_velocity_state_with_acceleration_saturation"
                     if args.acceleration_limit_rad_s2 > 0.0
                     else "ideal_joint_velocity_input"),
            "acceleration_limit_rad_s2": args.acceleration_limit_rad_s2,
            "initial_velocity": "frozen_reference_velocity_at_window_start",
            "source": ("explicit experiment option; 3.0 matches the Ball "
                       "trajectory generator and resampler when enabled"),
        },
        "force_model": {
            "equilibrium": args.force_equilibrium_mode,
            "desired_force_n": args.desired_force_n,
            "object_stiffness_n_m": args.object_stiffness_n_m,
            "object_damping_n_s_m": args.object_damping_n_s_m,
            "object_length_m": (
                args.object_length_m
                if args.object_length_m is not None else
                config["paper_elastic"]["ball"]["move"][
                    "object_length_m"]),
            "initialization": (
                "damped Cartesian least-squares compression to desired force"
                if args.force_equilibrium_mode == "runtime_absolute" else
                "reference state plus the declared initial joint error"),
            "force_delay_cycles": args.force_delay_cycles,
            "sampled_stiffness_range_n_m": [
                args.object_stiffness_min_n_m,
                args.object_stiffness_max_n_m],
            "sampled_damping_range_n_s_m": [
                args.object_damping_min_n_s_m,
                args.object_damping_max_n_s_m],
            "force_noise_std_n": args.force_noise_std_n,
            "sampled_force_delay_range_cycles": [
                args.force_delay_min_cycles,
                args.force_delay_max_cycles],
            "aic": {
                "bhat_left": args.bhat_left,
                "bhat_right": args.bhat_right,
                "wb_left": args.wb_left,
                "wb_right": args.wb_right,
                "wk_left": args.wk_left,
                "wk_right": args.wk_right,
                "k0": args.k0,
                "max_displacement_m": (
                    args.max_aic_displacement_m
                    if args.max_aic_displacement_m is not None else
                    config["paper_elastic"]["ball"]["move"][
                        "max_displacement_m"]),
            },
        },
        "disturbance": {
            "start_fraction": args.disturbance_start_fraction,
            "duration_s": args.disturbance_duration_s,
            "amplitude_rad_s": args.disturbance_amplitude_rad_s,
            "orientation_amplitude_rad_s": (
                args.orientation_disturbance_amplitude_rad_s),
            "smooth_window": args.smooth_disturbance,
        },
        "real_robot_perturbations": {
            "peer_delay_range_cycles": [
                args.peer_delay_min_cycles, args.peer_delay_max_cycles],
            "actuation_delay_range_cycles": [
                args.actuation_delay_min_cycles,
                args.actuation_delay_max_cycles],
            "period_jitter_std_fraction": args.period_jitter_std_fraction,
            "period_jitter_limit_fraction": args.period_jitter_limit_fraction,
            "fixed_by_trial_index": True,
            "identical_across_x0_x1_x2": True,
        },
        "success_thresholds": {
            "slip_onset_force_n": args.slip_onset_force_n,
            "drop_force_multiplier": args.drop_force_multiplier,
            "drop_force_threshold_n": (
                args.slip_onset_force_n * args.drop_force_multiplier),
            "drop_definition": (
                "any checked cycle below the threshold is a drop failure"),
            "overforce_limit_n": args.overforce_limit_n,
            "slip_hold_cycles": args.slip_hold_cycles,
            "overforce_hold_cycles": args.overforce_hold_cycles,
            "grace_cycles": args.grace_cycles,
            "terminal_target_error_mm": args.terminal_target_error_mm,
            "orientation_rmse_deg": args.orientation_rmse_deg,
            "hard_distance_min_m": args.hard_distance_min_m,
            "hard_distance_max_m": args.hard_distance_max_m,
            "joint_velocity_limits_rad_s": config.get(
                "velocity_max", config["input_max_rad_s"]),
            "hard_joint_velocity_scale": args.hard_joint_velocity_scale,
            "hard_joint_acceleration_rad_s2": (
                args.hard_joint_acceleration_rad_s2),
            "hard_command_jump_rad_s": args.hard_command_jump_rad_s,
            "hard_force_slew_n_s": args.hard_force_slew_n_s,
            "deadline_ms": args.deadline_ms,
            "deadline_miss_hold_cycles": args.deadline_miss_hold_cycles,
            "any_hard_cycle_violation_fails_trial": True,
        },
        "Ei_definition": (
            "mean over control cycles of both arms' complete five-step "
            "prediction objectives, re-evaluated with fixed x1 weights"),
        "collision_checking_during_dnmpc": "not used",
        "paired_comparison": (
            "every x0/x1/x2 setting uses the identical frozen trajectory pool"),
    }
    payload = {"provenance": provenance, "summary": summary, "raw_runs": raw}
    (args.output / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    write_csv(args.output / "summary.csv", summary)
    write_csv(args.output / "raw_runs.csv", raw)
    print("RESULT " + json.dumps(summary, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
