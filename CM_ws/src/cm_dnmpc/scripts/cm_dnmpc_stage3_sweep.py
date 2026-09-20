#!/usr/bin/env python3
"""Offline Stage-3 sweep using the current CM distributed DNMPC objective.

The controller weight may vary between runs, while every reported E_i is
re-evaluated with one frozen x1 profile.  This keeps x0/x1/x2 comparable.
No collision library is imported or called by this experiment.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np

from cm_dnmpc_common import (
    AICHorizonState,
    LocalDNMPC,
    ReferenceTrajectory,
    WEIGHT_NAMES,
    load_json,
    resolve_stage_weight_error_scales,
    resolve_stage_weights,
    validate_weights,
)
from paper_elastic3 import PaperElastic3


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--left-reference", type=Path, required=True)
    parser.add_argument("--right-reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weight", choices=WEIGHT_NAMES, required=True)
    parser.add_argument(
        "--values", required=True,
        help="comma-separated absolute controller weights")
    parser.add_argument(
        "--base-weights-json",
        help="controller base profile; defaults to resolved ball Stage 3")
    parser.add_argument(
        "--evaluation-weights-json",
        help="frozen E_i profile; defaults to controller base profile")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--steps", type=int, default=150)
    parser.add_argument("--start-fraction", type=float, default=0.42)
    parser.add_argument("--initial-error-std-rad", type=float, default=0.001)
    parser.add_argument(
        "--measurement-noise-std-rad", type=float, default=0.0,
        help="zero-mean joint measurement noise; shared by every weight setting",
    )
    parser.add_argument(
        "--acceleration-limit-rad-s2", type=float, default=0.0,
        help=("optional per-joint plant acceleration limit; zero keeps the "
              "paper's ideal joint-velocity plant (default)"),
    )
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
        "--orientation-disturbance-amplitude-rad-s", type=float, default=0.25)
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
    add_realistic_robustness_args(parser)
    return parser.parse_args()


def add_realistic_robustness_args(parser):
    """Add optional real-robot perturbations and per-cycle hard guards.

    Every option is disabled by default so historical experiment commands keep
    exactly their original behavior.  The multi-reference runner imports this
    helper and exposes the same protocol.
    """
    parser.add_argument("--object-stiffness-min-n-m", type=float)
    parser.add_argument("--object-stiffness-max-n-m", type=float)
    parser.add_argument(
        "--controller-force-stiffness-n-m", type=float,
        help=("fixed stiffness used inside the DNMPC distance/force target; "
              "when omitted it follows the sampled plant stiffness for "
              "backward compatibility"))
    parser.add_argument("--object-damping-min-n-s-m", type=float)
    parser.add_argument("--object-damping-max-n-s-m", type=float)
    parser.add_argument("--force-noise-std-n", type=float, default=0.0)
    parser.add_argument("--force-delay-min-cycles", type=int)
    parser.add_argument("--force-delay-max-cycles", type=int)
    parser.add_argument("--peer-delay-min-cycles", type=int, default=0)
    parser.add_argument("--peer-delay-max-cycles", type=int, default=0)
    parser.add_argument(
        "--peer-position-noise-std-m", type=float, default=0.0,
        help=("independent Cartesian prediction noise seen by each receiving "
              "arm; zero preserves the historical ideal communication model"))
    parser.add_argument("--actuation-delay-min-cycles", type=int, default=0)
    parser.add_argument("--actuation-delay-max-cycles", type=int, default=0)
    parser.add_argument("--period-jitter-std-fraction", type=float, default=0.0)
    parser.add_argument("--period-jitter-limit-fraction", type=float, default=0.0)
    parser.add_argument("--smooth-disturbance", action="store_true")
    parser.add_argument("--hard-distance-min-m", type=float, default=0.0)
    parser.add_argument("--hard-distance-max-m", type=float, default=0.0)
    parser.add_argument("--hard-joint-velocity-scale", type=float, default=0.0)
    parser.add_argument("--hard-joint-acceleration-rad-s2", type=float, default=0.0)
    parser.add_argument("--hard-command-jump-rad-s", type=float, default=0.0)
    parser.add_argument("--hard-force-slew-n-s", type=float, default=0.0)
    parser.add_argument("--deadline-ms", type=float, default=20.0)
    parser.add_argument(
        "--deadline-miss-hold-cycles", type=int, default=0,
        help="zero records deadline misses without making them task failures")
    parser.add_argument(
        "--run-full-reference", action="store_true",
        help=("run each trajectory through its complete remaining reference "
              "duration instead of truncating every trajectory to --steps"))
    parser.add_argument(
        "--settle-cycles", type=int, default=0,
        help=("extra fixed control cycles after the nominal reference end; "
              "used only with --run-full-reference"))
    parser.add_argument(
        "--preserve-initial-clamp-at-terminal", action="store_true",
        help=("keep the Stage-3 initial end-effector separation at the phase "
              "terminal target while retaining the RRT terminal midpoint "
              "and inter-arm direction"))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rms(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values * values)))


def angle_error_deg(left_axis, right_axis):
    cosine = float(np.clip(-np.dot(left_axis, right_axis), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def make_elastic(values, dt):
    return PaperElastic3(dt=dt, **values)


def sampled_float(rng, low, high, fallback, name):
    if low is None and high is None:
        return float(fallback)
    low = float(fallback if low is None else low)
    high = float(fallback if high is None else high)
    if not math.isfinite(low) or not math.isfinite(high) or low > high:
        raise ValueError("invalid {} range [{}, {}]".format(name, low, high))
    return float(rng.uniform(low, high)) if high > low else low


def sampled_int(rng, low, high, fallback, name):
    if low is None and high is None:
        return int(fallback)
    low = int(fallback if low is None else low)
    high = int(fallback if high is None else high)
    if low < 0 or low > high:
        raise ValueError("invalid {} range [{}, {}]".format(name, low, high))
    return int(rng.integers(low, high + 1)) if high > low else low


def initialize_force_equilibrium(solvers, q_reference, object_length_m,
                                 desired_force_n, stiffness_n_m, joint_min,
                                 joint_max):
    """Compress both arms to the runtime force equilibrium.

    Stage 3 starts after Stage 2 has established the desired grasp force.  The
    frozen Move references do not contain that Cartesian compression, so the
    offline runtime-force model reconstructs it without changing the stored
    trajectories.
    """
    q = {side: np.asarray(q_reference[side], dtype=float).copy()
         for side in ("left", "right")}
    target_distance = max(
        1.0e-6,
        float(object_length_m)
        - float(desired_force_n) / float(stiffness_n_m))
    low = np.asarray(joint_min, dtype=float)
    high = np.asarray(joint_max, dtype=float)
    for _ in range(20):
        positions = {side: solvers[side].pose(q[side])[0]
                     for side in ("left", "right")}
        line = positions["right"] - positions["left"]
        distance = float(np.linalg.norm(line))
        if distance <= 1.0e-9:
            break
        distance_error = distance - target_distance
        if abs(distance_error) <= 1.0e-7:
            break
        direction = line / distance
        deltas = {
            "left": 0.5 * distance_error * direction,
            "right": -0.5 * distance_error * direction,
        }
        for side in ("left", "right"):
            jacobian, _ = solvers[side].jacobians(q[side])
            regularizer = 1.0e-6 * np.eye(3)
            dq = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + regularizer, deltas[side])
            q[side] = np.clip(
                q[side] + np.clip(dq, -0.05, 0.05), low, high)
    return q


def evaluate_prediction_objective(solver, solution, qref, qdref,
                                  peer_positions, peer_orientations,
                                  aic_state, terminal_target, weights,
                                  weight_error_scales, dt):
    """Re-evaluate one solved horizon with fixed x1 objective weights."""
    positions = solution.position_prediction
    axes = solution.orientation_prediction
    joints = solution.joint_prediction
    controls = solution.controls
    scales = weight_error_scales
    initial_position = positions[0]
    initial_reference_position = solver.pose(qref[0])[0]
    components = {name: 0.0 for name in
                  ("Jd", "Jt", "Jo", "Ju", "Jq", "Jv", "JF")}
    for stage in range(len(controls)):
        position = positions[stage]
        axis = axes[stage]
        peer_line = peer_positions[stage] - position
        peer_distance = math.sqrt(float(np.sum(peer_line ** 2)) + 1.0e-12)
        peer_direction = peer_line / peer_distance
        desired_peer_distance = max(
            1.0e-6,
            aic_state.object_length_m
            - aic_state.desired_force_n
            / max(aic_state.force_scale_n_m, 1.0e-6))
        relative_position_error = (
            peer_line - peer_direction * desired_peer_distance)
        reference_position = solver.pose(qref[stage])[0]
        force_target = (
            initial_position
            + (reference_position - initial_reference_position)
            + peer_direction * aic_state.desired_displacement_m)
        components["Jt"] += dt * weights["Wt"] * float(np.sum(
            (relative_position_error / scales["Wt"]) ** 2))
        components["Jo"] += dt * weights["Wo"] * float(np.sum(
            ((axis + peer_orientations[stage]) / scales["Wo"]) ** 2))
        components["Ju"] += dt * weights["Wu"] * float(np.sum(
            (controls[stage] / scales["Wu"]) ** 2))
        components["Jq"] += dt * weights["Wq"] * float(np.sum(
            ((joints[stage] - qref[stage]) / scales["Wq"]) ** 2))
        components["Jv"] += dt * weights["Wv"] * float(np.sum(
            ((controls[stage] - qdref[stage]) / scales["Wv"]) ** 2))
        components["JF"] += (
            dt * weights["WF"] * (1.0 if aic_state.active else 0.0)
            * float(np.sum(
                ((position - force_target) / scales["WF"]) ** 2)))
    components["Jd"] = weights["Wd"] * float(np.sum(
        ((positions[-1] - terminal_target) / scales["Wd"]) ** 2))
    return components


def run_trial(args, config, references, controller_weights,
              evaluation_weights, seed, setting_label, solvers,
              weight_error_scales=None, trace_rows=None):
    dt = float(config["Ts"])
    horizon = int(config["horizon"])
    count = horizon + 1
    rng = np.random.default_rng(seed)
    environment_rng = np.random.default_rng(1000003 + int(seed))
    elastic_values = config["paper_elastic"]["ball"]["move"]
    max_aic_displacement = getattr(args, "max_aic_displacement_m", None)
    if max_aic_displacement is None:
        max_aic_displacement = elastic_values["max_displacement_m"]
    max_aic_displacement = float(max_aic_displacement)
    if not math.isfinite(max_aic_displacement) or max_aic_displacement < 0.0:
        raise ValueError("max AIC displacement must be finite and nonnegative")
    stiffness = sampled_float(
        environment_rng,
        getattr(args, "object_stiffness_min_n_m", None),
        getattr(args, "object_stiffness_max_n_m", None),
        args.object_stiffness_n_m, "object stiffness")
    controller_force_stiffness = getattr(
        args, "controller_force_stiffness_n_m", None)
    if controller_force_stiffness is None:
        controller_force_stiffness = stiffness
    controller_force_stiffness = float(controller_force_stiffness)
    if (not math.isfinite(controller_force_stiffness)
            or controller_force_stiffness <= 0.0):
        raise ValueError(
            "controller force stiffness must be finite and positive")
    damping = sampled_float(
        environment_rng,
        getattr(args, "object_damping_min_n_s_m", None),
        getattr(args, "object_damping_max_n_s_m", None),
        args.object_damping_n_s_m, "object damping")
    force_delay_cycles = sampled_int(
        environment_rng,
        getattr(args, "force_delay_min_cycles", None),
        getattr(args, "force_delay_max_cycles", None),
        args.force_delay_cycles, "force delay")
    peer_delay_cycles = sampled_int(
        environment_rng,
        getattr(args, "peer_delay_min_cycles", 0),
        getattr(args, "peer_delay_max_cycles", 0),
        0, "peer delay")
    actuation_delay_cycles = sampled_int(
        environment_rng,
        getattr(args, "actuation_delay_min_cycles", 0),
        getattr(args, "actuation_delay_max_cycles", 0),
        0, "actuation delay")
    desired_force = float(args.desired_force_n)
    if weight_error_scales is None:
        weight_error_scales = resolve_stage_weight_error_scales(
            config, "ball", 3)
    force_mode = str(getattr(
        args, "force_equilibrium_mode", "reference_desired"))
    object_length = getattr(args, "object_length_m", None)
    if object_length is None:
        object_length = elastic_values["object_length_m"]
    object_length = float(object_length)
    if (not math.isfinite(object_length) or object_length <= 0.0
            or not math.isfinite(stiffness) or stiffness <= 0.0):
        raise ValueError("runtime force model requires positive finite geometry")
    for solver in solvers.values():
        solver.warm = np.zeros((horizon, 6))
    elastic = {}
    for side in ("left", "right"):
        side_values = dict(elastic_values)
        side_values.update({
            "desired_force_n": desired_force,
            "k0": float(args.k0),
            "b0": float(getattr(args, "bhat_" + side)),
            "wb": float(getattr(args, "wb_" + side)),
            "wk": float(getattr(args, "wk_" + side)),
            "max_displacement_m": max_aic_displacement,
        })
        elastic[side] = make_elastic(side_values, dt)
    duration = min(ref.duration for ref in references.values())
    start_time = float(args.start_fraction) * duration
    trial_steps = int(args.steps)
    if getattr(args, "run_full_reference", False):
        remaining_duration = max(0.0, duration - start_time)
        trial_steps = int(math.ceil(remaining_duration / dt))
        trial_steps += int(getattr(args, "settle_cycles", 0))
        trial_steps = max(1, trial_steps)
    qref0 = {}
    plant_velocity_state = {}
    for side in ("left", "right"):
        qref0[side], qdref0 = references[side].sample(start_time)
        # Start from the frozen reference velocity so that the reported
        # response is not dominated by an artificial zero-speed transient.
        plant_velocity_state[side] = qdref0.copy()
    q_initial = qref0
    if force_mode == "runtime_absolute":
        q_initial = initialize_force_equilibrium(
            solvers, qref0, object_length, desired_force, stiffness,
            config["joint_min"], config["joint_max"])
    q = {
        side: q_initial[side] + rng.normal(
            0.0, args.initial_error_std_rad, size=6)
        for side in ("left", "right")
    }
    predictions = {}
    terminal_targets = {}
    nominal_terminal_positions = {
        side: solvers[side].pose(references[side].q[-1])[0]
        for side in ("left", "right")}
    if getattr(args, "preserve_initial_clamp_at_terminal", False):
        initial_positions = {
            side: solvers[side].pose(q_initial[side])[0]
            for side in ("left", "right")}
        initial_separation = float(np.linalg.norm(
            initial_positions["right"] - initial_positions["left"]))
        terminal_line = (
            nominal_terminal_positions["right"]
            - nominal_terminal_positions["left"])
        terminal_line_norm = float(np.linalg.norm(terminal_line))
        if terminal_line_norm <= 1.0e-9:
            raise RuntimeError("terminal end-effector direction is singular")
        terminal_direction = terminal_line / terminal_line_norm
        terminal_midpoint = 0.5 * (
            nominal_terminal_positions["left"]
            + nominal_terminal_positions["right"])
        terminal_targets["left"] = (
            terminal_midpoint - 0.5 * initial_separation * terminal_direction)
        terminal_targets["right"] = (
            terminal_midpoint + 0.5 * initial_separation * terminal_direction)
    else:
        terminal_targets.update(nominal_terminal_positions)
    for side in ("left", "right"):
        qref, _ = references[side].horizon(start_time, count, dt)
        predictions[side] = solvers[side].cartesian_horizon(qref)
    prediction_history = {
        side: [(predictions[side][0].copy(), predictions[side][1].copy())]
        for side in ("left", "right")}

    disturb_start = int(round(
        args.disturbance_start_fraction * trial_steps))
    disturb_count = max(1, int(round(args.disturbance_duration_s / dt)))
    drop_force_threshold_n = (
        args.slip_onset_force_n * args.drop_force_multiplier)
    slip_run = 0
    overforce_run = 0
    max_slip_run = 0
    max_overforce_run = 0
    slip_failure = False
    overforce_failure = False
    solver_failure = False
    joint_violation = False
    rows = []
    previous_force_e = None
    previous_true_force = None
    force_buffer = []
    command_history = {
        side: [plant_velocity_state[side].copy()]
        for side in ("left", "right")}
    previous_command = {
        side: plant_velocity_state[side].copy()
        for side in ("left", "right")}
    previous_actual_velocity = {
        side: plant_velocity_state[side].copy()
        for side in ("left", "right")}
    elapsed_time = 0.0
    deadline_run = 0
    maximum_deadline_run = 0
    hard_failures = {}

    def mark_hard_failure(reason, cycle, value, limit):
        if reason not in hard_failures:
            hard_failures[reason] = {
                "cycle": int(cycle),
                "sim_time_s": float(elapsed_time),
                "value": float(value),
                "limit": float(limit),
            }

    for cycle in range(trial_steps):
        jitter_std = max(
            0.0, float(getattr(args, "period_jitter_std_fraction", 0.0)))
        jitter_limit = max(
            0.0, float(getattr(args, "period_jitter_limit_fraction", 0.0)))
        period_scale = 1.0
        if jitter_std > 0.0:
            period_scale += float(environment_rng.normal(0.0, jitter_std))
        if jitter_limit > 0.0:
            period_scale = float(np.clip(
                period_scale, 1.0 - jitter_limit, 1.0 + jitter_limit))
        cycle_dt = max(1.0e-6, dt * period_scale)
        reference_time = min(start_time + elapsed_time, duration)
        qrefs = {}
        qdrefs = {}
        positions = {}
        axes = {}
        ref_positions = {}
        ref_axes = {}
        for side in ("left", "right"):
            qrefs[side], qdrefs[side] = references[side].horizon(
                reference_time, count, dt)
            positions[side], axes[side] = solvers[side].pose(q[side])
            ref_positions[side], ref_axes[side] = solvers[side].pose(
                qrefs[side][0])

        measured_q = {
            side: q[side] + rng.normal(
                0.0, args.measurement_noise_std_rad, size=6)
            for side in ("left", "right")
        }

        distance = float(np.linalg.norm(positions["right"] - positions["left"]))
        reference_distance = float(np.linalg.norm(
            ref_positions["right"] - ref_positions["left"]))
        if force_mode == "runtime_absolute":
            # Match cm_dnmpc_arm_node.simulated_clamp_force.  The optional
            # damping term is zero in the Gazebo-aligned formal protocol.
            force_e = object_length - distance
            force_e_dot = (
                0.0 if previous_force_e is None else
                (force_e - previous_force_e) / cycle_dt)
            true_force = max(
                0.0, stiffness * force_e + damping * force_e_dot)
            elastic_object_length = object_length
        else:
            # Backward-compatible historical model: every nominal reference
            # sample is the desired-force equilibrium.
            force_e = 0.5 * (reference_distance - distance)
            force_e_dot = (
                0.0 if previous_force_e is None else
                (force_e - previous_force_e) / cycle_dt)
            true_force = max(
                0.0,
                desired_force + stiffness * force_e
                + damping * force_e_dot)
            elastic_object_length = reference_distance
        previous_force_e = force_e
        measured_force = max(
            0.0, true_force + float(environment_rng.normal(
                0.0, max(0.0, getattr(args, "force_noise_std_n", 0.0)))))
        force_buffer.append(measured_force)
        delayed_index = max(
            0, len(force_buffer) - 1 - force_delay_cycles)
        aic_force = force_buffer[delayed_index]
        aic_states = {}
        force_targets = {}
        for side, peer in (("left", "right"), ("right", "left")):
            elastic[side].object_length = elastic_object_length
            force_targets[side], state = elastic[side].update(
                positions[side], positions[peer], aic_force)
            aic_states[side] = AICHorizonState(
                active=True,
                object_length_m=elastic_object_length,
                measured_force_n=aic_force,
                desired_force_n=desired_force,
                b_hat=state["b_hat"],
                k_hat=state["k_hat"],
                current_e_m=state["e"],
                max_displacement_m=max_aic_displacement,
                desired_displacement_m=state["displacement"],
                force_scale_n_m=controller_force_stiffness,
            )

        communicated_peer_predictions = {}
        for side, peer in (("left", "right"), ("right", "left")):
            peer_history_index = max(
                0, len(prediction_history[peer]) - 1 - peer_delay_cycles)
            peer_prediction = prediction_history[peer][peer_history_index]
            peer_position_noise_std = max(
                0.0, float(getattr(
                    args, "peer_position_noise_std_m", 0.0)))
            communicated_peer_predictions[side] = (
                peer_prediction[0] + environment_rng.normal(
                    0.0, peer_position_noise_std,
                    size=peer_prediction[0].shape),
                peer_prediction[1])

        solutions = {}
        for side, peer in (("left", "right"), ("right", "left")):
            peer_prediction = communicated_peer_predictions[side]
            solutions[side] = solvers[side].solve(
                measured_q[side], qrefs[side], qdrefs[side][:-1],
                peer_prediction[0], peer_prediction[1],
                aic_states[side],
                terminal_target=terminal_targets[side])
        if not all(solution.success for solution in solutions.values()):
            solver_failure = True
            mark_hard_failure("solver_failure", cycle, 1.0, 0.0)

        fixed_prediction_components = {
            name: 0.0 for name in
            ("Jd", "Jt", "Jo", "Ju", "Jq", "Jv", "JF")}
        for side, peer in (("left", "right"), ("right", "left")):
            peer_prediction = communicated_peer_predictions[side]
            local_components = evaluate_prediction_objective(
                solvers[side], solutions[side], qrefs[side],
                qdrefs[side][:-1], peer_prediction[0],
                peer_prediction[1], aic_states[side],
                terminal_targets[side], evaluation_weights,
                weight_error_scales, dt)
            for name, value in local_components.items():
                fixed_prediction_components[name] += value

        controls = {side: solutions[side].controls[0].copy()
                    for side in ("left", "right")}
        for side in ("left", "right"):
            command_history[side].append(controls[side].copy())
        delayed_controls = {
            side: command_history[side][max(
                0, len(command_history[side]) - 1 - actuation_delay_cycles)]
            for side in ("left", "right")}
        requested_acceleration = {
            side: (delayed_controls[side] - plant_velocity_state[side])
            / cycle_dt
            for side in ("left", "right")
        }
        if args.acceleration_limit_rad_s2 > 0.0:
            applied_acceleration = {
                side: np.clip(
                    requested_acceleration[side],
                    -args.acceleration_limit_rad_s2,
                    args.acceleration_limit_rad_s2)
                for side in ("left", "right")
            }
        else:
            applied_acceleration = {
                side: requested_acceleration[side].copy()
                for side in ("left", "right")
            }
        actuator_velocity = {
            side: plant_velocity_state[side]
            + cycle_dt * applied_acceleration[side]
            for side in ("left", "right")
        }
        plant_velocity = {side: actuator_velocity[side].copy()
                          for side in ("left", "right")}
        if disturb_start <= cycle < disturb_start + disturb_count:
            disturbance_gain = 1.0
            if getattr(args, "smooth_disturbance", False):
                phase = (
                    cycle - disturb_start + 0.5) / float(disturb_count)
                disturbance_gain = math.sin(math.pi * phase)
            opening = positions["right"] - positions["left"]
            norm = float(np.linalg.norm(opening))
            if norm > 1.0e-12:
                opening /= norm
                jacobian, _ = solvers["right"].jacobians(q["right"])
                direction = jacobian.T @ opening
                direction_norm = float(np.linalg.norm(direction))
                if direction_norm > 1.0e-12:
                    direction /= direction_norm
                    plant_velocity["right"] += (
                        disturbance_gain
                        * args.disturbance_amplitude_rad_s * direction)
            # Independently excite orientation coordination.  This bounded
            # direction maximizes instantaneous EE-axis rotation.
            _, axis_jacobian = solvers["right"].jacobians(q["right"])
            _, _, vh = np.linalg.svd(axis_jacobian, full_matrices=False)
            plant_velocity["right"] += (
                disturbance_gain
                * args.orientation_disturbance_amplitude_rad_s * vh[0])

        actual_acceleration = {
            side: (plant_velocity[side] - previous_actual_velocity[side])
            / cycle_dt
            for side in ("left", "right")}
        command_jump = {
            side: controls[side] - previous_command[side]
            for side in ("left", "right")}
        force_slew = (
            0.0 if previous_true_force is None else
            (true_force - previous_true_force) / cycle_dt)
        previous_true_force = true_force

        q_next = {}
        for side in ("left", "right"):
            raw = q[side] + cycle_dt * plant_velocity[side]
            low = np.asarray(config["joint_min"], dtype=float)
            high = np.asarray(config["joint_max"], dtype=float)
            if np.any(raw < low) or np.any(raw > high):
                joint_violation = True
                violation = max(
                    float(np.max(low - raw)), float(np.max(raw - high)))
                mark_hard_failure(
                    "joint_position_limit", cycle, max(0.0, violation), 0.0)
            q_next[side] = np.clip(raw, low, high)
            predictions[side] = (
                solutions[side].position_prediction.copy(),
                solutions[side].orientation_prediction.copy())
            prediction_history[side].append((
                predictions[side][0].copy(), predictions[side][1].copy()))

        relative_error = (
            (positions["left"] - positions["right"])
            - (ref_positions["left"] - ref_positions["right"]))
        target = 0.5 * (positions["left"] + positions["right"])
        target_ref = 0.5 * (ref_positions["left"] + ref_positions["right"])
        max_joint_velocity = max(
            float(np.max(np.abs(plant_velocity[side])))
            for side in ("left", "right"))
        velocity_limits = np.asarray(
            config.get("velocity_max", config["input_max_rad_s"]),
            dtype=float)
        joint_velocity_ratio = max(
            float(np.max(
                np.abs(plant_velocity[side]) / velocity_limits))
            for side in ("left", "right"))
        max_joint_acceleration = max(
            float(np.max(np.abs(actual_acceleration[side])))
            for side in ("left", "right"))
        max_command_velocity = max(
            float(np.max(np.abs(controls[side])))
            for side in ("left", "right"))
        max_command_jump = max(
            float(np.max(np.abs(command_jump[side])))
            for side in ("left", "right"))
        solve_ms = max(solution.solve_ms for solution in solutions.values())
        deadline_run = (
            deadline_run + 1
            if solve_ms > float(getattr(args, "deadline_ms", 20.0)) else 0)
        maximum_deadline_run = max(maximum_deadline_run, deadline_run)

        distance_min = float(getattr(args, "hard_distance_min_m", 0.0))
        distance_max = float(getattr(args, "hard_distance_max_m", 0.0))
        if distance_min > 0.0 and distance < distance_min:
            mark_hard_failure(
                "relative_distance_below_min", cycle, distance, distance_min)
        if distance_max > 0.0 and distance > distance_max:
            mark_hard_failure(
                "relative_distance_above_max", cycle, distance, distance_max)
        velocity_guard_scale = float(getattr(
            args, "hard_joint_velocity_scale", 0.0))
        if (velocity_guard_scale > 0.0
                and joint_velocity_ratio > velocity_guard_scale):
            mark_hard_failure(
                "joint_velocity_limit", cycle,
                joint_velocity_ratio, velocity_guard_scale)
        acceleration_guard = float(getattr(
            args, "hard_joint_acceleration_rad_s2", 0.0))
        if (acceleration_guard > 0.0
                and max_joint_acceleration > acceleration_guard):
            mark_hard_failure(
                "joint_acceleration_limit", cycle,
                max_joint_acceleration, acceleration_guard)
        command_jump_guard = float(getattr(
            args, "hard_command_jump_rad_s", 0.0))
        if command_jump_guard > 0.0 and max_command_jump > command_jump_guard:
            mark_hard_failure(
                "command_jump_limit", cycle,
                max_command_jump, command_jump_guard)
        force_slew_guard = float(getattr(
            args, "hard_force_slew_n_s", 0.0))
        if force_slew_guard > 0.0 and abs(force_slew) > force_slew_guard:
            mark_hard_failure(
                "force_slew_limit", cycle, abs(force_slew), force_slew_guard)
        deadline_hold = int(getattr(args, "deadline_miss_hold_cycles", 0))
        if deadline_hold > 0 and deadline_run >= deadline_hold:
            mark_hard_failure(
                "solver_deadline_miss", cycle, deadline_run, deadline_hold)
        row = {
            "cycle": cycle,
            "cycle_dt_s": cycle_dt,
            "q_error_sq": sum(float(np.sum(
                (q[side] - qrefs[side][0]) ** 2))
                for side in ("left", "right")),
            "q_count": 12,
            "relative_error_sq": float(np.sum(relative_error ** 2)),
            "target_position_error_sq": sum(float(np.sum(
                (positions[side] - ref_positions[side]) ** 2))
                for side in ("left", "right")),
            "orientation_error_deg": angle_error_deg(
                axes["left"], axes["right"]),
            "velocity_error_sq": sum(float(np.sum(
                (plant_velocity[side] - qdrefs[side][0]) ** 2))
                for side in ("left", "right")),
            # The paper's W_v term is part of the controller objective and
            # therefore uses commanded qdot.  The reported joint-velocity
            # RMSE above still uses actual plant velocity, including the
            # common external disturbance.
            "command_velocity_error_sq": sum(float(np.sum(
                (controls[side] - qdrefs[side][0]) ** 2))
                for side in ("left", "right")),
            "velocity_count": 12,
            "force_error_n": measured_force - desired_force,
            "measured_force_n": measured_force,
            "true_force_n": true_force,
            "force_slew_n_s": force_slew,
            "ee_distance_m": distance,
            "control_sq": sum(float(np.sum(controls[side] ** 2))
                              for side in ("left", "right")),
            "max_joint_velocity_rad_s": max_joint_velocity,
            "max_joint_velocity_ratio": joint_velocity_ratio,
            "max_joint_acceleration_rad_s2": max_joint_acceleration,
            "max_command_velocity_rad_s": max_command_velocity,
            "max_command_jump_rad_s": max_command_jump,
            "requested_acceleration_sq": sum(float(np.sum(
                requested_acceleration[side] ** 2))
                for side in ("left", "right")),
            "applied_acceleration_sq": sum(float(np.sum(
                applied_acceleration[side] ** 2))
                for side in ("left", "right")),
            "acceleration_count": 12,
            "acceleration_limited_count": (
                sum(int(np.count_nonzero(
                    np.abs(requested_acceleration[side])
                    > args.acceleration_limit_rad_s2 + 1.0e-12))
                    for side in ("left", "right"))
                if args.acceleration_limit_rad_s2 > 0.0 else 0),
            "orientation_objective_twice": 2.0 * float(np.sum(
                (axes["left"] + axes["right"]) ** 2)),
            "force_target_error_sq": sum(float(np.sum(
                (positions[side] - force_targets[side]) ** 2))
                for side in ("left", "right")),
            "target_error_mm": 1000.0 * float(np.linalg.norm(target-target_ref)),
            "solve_ms": solve_ms,
            "deadline_miss": int(
                solve_ms > float(getattr(args, "deadline_ms", 20.0))),
            "fixed_prediction_objective": float(sum(
                fixed_prediction_components.values())),
        }
        if trace_rows is not None:
            trace = {
                "cycle": cycle,
                "sim_time_s": elapsed_time,
                "cycle_dt_s": cycle_dt,
                "reference_time_s": reference_time,
                "measured_force_n": measured_force,
                "true_force_n": true_force,
                "force_slew_n_s": force_slew,
                "ee_distance_m": distance,
                "max_joint_velocity_rad_s": max_joint_velocity,
                "max_joint_acceleration_rad_s2": max_joint_acceleration,
                "max_command_velocity_rad_s": max_command_velocity,
                "max_command_jump_rad_s": max_command_jump,
                "solve_ms": solve_ms,
                "ball_x_m": float(target[0]),
                "ball_y_m": float(target[1]),
                "ball_z_m": float(target[2]),
            }
            for side in ("left", "right"):
                for joint, value in enumerate(q[side], start=1):
                    trace["{}_q{}".format(side, joint)] = float(value)
                for joint, value in enumerate(controls[side], start=1):
                    trace["{}_command_qd{}".format(side, joint)] = float(value)
                for joint, value in enumerate(
                        plant_velocity[side], start=1):
                    trace["{}_actual_qd{}".format(side, joint)] = float(value)
                for joint, value in enumerate(
                        actual_acceleration[side], start=1):
                    trace["{}_actual_qdd{}".format(side, joint)] = float(value)
                for axis_name, value in zip("xyz", positions[side]):
                    trace["{}_ee_{}_m".format(side, axis_name)] = float(value)
            trace_rows.append(trace)
        row.update({
            "fixed_prediction_{}".format(name): value
            for name, value in fixed_prediction_components.items()})
        rows.append(row)
        if cycle >= args.grace_cycles:
            slip_run = (
                slip_run + 1
                if true_force < drop_force_threshold_n else 0)
            overforce_run = overforce_run + 1 if true_force > args.overforce_limit_n else 0
            max_slip_run = max(max_slip_run, slip_run)
            max_overforce_run = max(max_overforce_run, overforce_run)
            slip_failure = slip_failure or slip_run >= args.slip_hold_cycles
            overforce_failure = (
                overforce_failure
                or overforce_run >= args.overforce_hold_cycles)
            if slip_run >= args.slip_hold_cycles:
                mark_hard_failure(
                    "drop_force_below_1p5_onset", cycle,
                    true_force, drop_force_threshold_n)
            if overforce_run >= args.overforce_hold_cycles:
                mark_hard_failure(
                    "force_above_limit", cycle,
                    true_force, args.overforce_limit_n)
        q = q_next
        plant_velocity_state = actuator_velocity
        previous_actual_velocity = {
            side: plant_velocity[side].copy()
            for side in ("left", "right")}
        previous_command = {
            side: controls[side].copy()
            for side in ("left", "right")}
        elapsed_time += cycle_dt

    final_time = min(start_time + elapsed_time, duration)
    final_reference = {
        side: references[side].sample(final_time)[0]
        for side in ("left", "right")
    }
    terminal_sq = 0.0
    terminal_midpoint_error_mm = 0.0
    final_positions = {}
    final_ref_positions = {}
    for side in ("left", "right"):
        final_positions[side] = solvers[side].pose(q[side])[0]
        final_ref_positions[side] = solvers[side].pose(final_reference[side])[0]
        terminal_sq += float(np.sum(
            (final_positions[side] - terminal_targets[side]) ** 2))
    terminal_goal_midpoint_error_mm = 1000.0 * float(np.linalg.norm(
        0.5 * (final_positions["left"] + final_positions["right"])
        - 0.5 * (terminal_targets["left"] + terminal_targets["right"])))
    terminal_midpoint_error_mm = 1000.0 * float(np.linalg.norm(
        0.5 * (final_positions["left"] + final_positions["right"])
        - 0.5 * (final_ref_positions["left"] + final_ref_positions["right"])))

    q_rmse = math.sqrt(sum(row["q_error_sq"] for row in rows)
                       / sum(row["q_count"] for row in rows))
    velocity_rmse = math.sqrt(sum(row["velocity_error_sq"] for row in rows)
                             / sum(row["velocity_count"] for row in rows))
    relative_rmse = math.sqrt(float(np.mean(
        [row["relative_error_sq"] for row in rows])))
    orientation_rmse = rms([row["orientation_error_deg"] for row in rows])
    force_rmse = rms([row["force_error_n"] for row in rows])
    command_cost = sum(
        row["cycle_dt_s"] * row["control_sq"] for row in rows)
    acceleration_limited_fraction = (
        sum(row["acceleration_limited_count"] for row in rows)
        / float(sum(row["acceleration_count"] for row in rows)))
    realized_components = {
        "Jd": evaluation_weights["Wd"] * terminal_sq
        / weight_error_scales["Wd"] ** 2,
        "Jt": evaluation_weights["Wt"] * sum(
            row["cycle_dt_s"] * row["target_position_error_sq"]
            for row in rows)
        / weight_error_scales["Wt"] ** 2,
        "Jo": evaluation_weights["Wo"] * sum(
            row["cycle_dt_s"] * row["orientation_objective_twice"]
            for row in rows)
        / weight_error_scales["Wo"] ** 2,
        "Ju": evaluation_weights["Wu"] * command_cost
        / weight_error_scales["Wu"] ** 2,
        "Jq": evaluation_weights["Wq"] * sum(
            row["cycle_dt_s"] * row["q_error_sq"] for row in rows)
        / weight_error_scales["Wq"] ** 2,
        "Jv": evaluation_weights["Wv"] * sum(
            row["cycle_dt_s"] * row["command_velocity_error_sq"]
            for row in rows)
        / weight_error_scales["Wv"] ** 2,
        "JF": evaluation_weights["WF"] * sum(
            row["cycle_dt_s"] * row["force_target_error_sq"]
            for row in rows)
        / weight_error_scales["WF"] ** 2,
    }
    components = {
        name: float(np.mean([
            row["fixed_prediction_{}".format(name)] for row in rows]))
        for name in ("Jd", "Jt", "Jo", "Ju", "Jq", "Jv", "JF")
    }
    if terminal_midpoint_error_mm > args.terminal_target_error_mm:
        mark_hard_failure(
            "terminal_target_error", trial_steps - 1,
            terminal_midpoint_error_mm, args.terminal_target_error_mm)
    if orientation_rmse > args.orientation_rmse_deg:
        mark_hard_failure(
            "orientation_rmse", trial_steps - 1,
            orientation_rmse, args.orientation_rmse_deg)
    success = bool(
        not solver_failure and not joint_violation
        and not slip_failure and not overforce_failure
        and not hard_failures
        and terminal_midpoint_error_mm <= args.terminal_target_error_mm
        and orientation_rmse <= args.orientation_rmse_deg)
    ordered_failures = sorted(
        hard_failures.items(),
        key=lambda item: (item[1]["cycle"], item[0]))
    first_failure_reason = ordered_failures[0][0] if ordered_failures else ""
    first_failure_cycle = (
        ordered_failures[0][1]["cycle"] if ordered_failures else -1)
    return {
        "seed": seed,
        "weight": args.weight,
        "setting_label": setting_label,
        "weight_value": float(controller_weights[args.weight]),
        "task_success": success,
        "solver_failure": solver_failure,
        "joint_violation": joint_violation,
        "slip_failure": slip_failure,
        "slip_onset_force_n": float(args.slip_onset_force_n),
        "drop_force_multiplier": float(args.drop_force_multiplier),
        "drop_force_threshold_n": float(drop_force_threshold_n),
        "overforce_failure": overforce_failure,
        "hard_cycle_failure": bool(hard_failures),
        "hard_failure_reasons": ";".join(sorted(hard_failures)),
        "first_failure_reason": first_failure_reason,
        "first_failure_cycle": first_failure_cycle,
        "max_slip_duration_s": max_slip_run * dt,
        "max_overforce_duration_s": max_overforce_run * dt,
        "terminal_target_error_mm": terminal_midpoint_error_mm,
        "terminal_goal_error_mm": terminal_goal_midpoint_error_mm,
        "q_rmse_rad": q_rmse,
        "relative_position_rmse_m": relative_rmse,
        "orientation_rmse_deg": orientation_rmse,
        "velocity_rmse_rad_s": velocity_rmse,
        "force_rmse_n": force_rmse,
        "force_min_n": float(min(row["true_force_n"] for row in rows)),
        "force_max_n": float(max(row["true_force_n"] for row in rows)),
        "force_slew_max_n_s": float(max(
            abs(row["force_slew_n_s"]) for row in rows)),
        "ee_distance_min_m": float(min(row["ee_distance_m"] for row in rows)),
        "ee_distance_max_m": float(max(row["ee_distance_m"] for row in rows)),
        "joint_velocity_max_rad_s": float(max(
            row["max_joint_velocity_rad_s"] for row in rows)),
        "joint_velocity_ratio_max": float(max(
            row["max_joint_velocity_ratio"] for row in rows)),
        "joint_acceleration_max_rad_s2": float(max(
            row["max_joint_acceleration_rad_s2"] for row in rows)),
        "command_velocity_max_rad_s": float(max(
            row["max_command_velocity_rad_s"] for row in rows)),
        "command_jump_max_rad_s": float(max(
            row["max_command_jump_rad_s"] for row in rows)),
        "deadline_miss_cycles": int(sum(
            row["deadline_miss"] for row in rows)),
        "deadline_miss_fraction": float(np.mean([
            row["deadline_miss"] for row in rows])),
        "maximum_consecutive_deadline_misses": int(maximum_deadline_run),
        "control_effort": command_cost,
        "acceleration_limited_fraction": acceleration_limited_fraction,
        "requested_acceleration_rmse_rad_s2": math.sqrt(
            sum(row["requested_acceleration_sq"] for row in rows)
            / sum(row["acceleration_count"] for row in rows)),
        "applied_acceleration_rmse_rad_s2": math.sqrt(
            sum(row["applied_acceleration_sq"] for row in rows)
            / sum(row["acceleration_count"] for row in rows)),
        "common_eval_cost": float(sum(components.values())),
        "realized_common_eval_cost": float(sum(realized_components.values())),
        "mean_solve_ms": float(np.mean([row["solve_ms"] for row in rows])),
        "p95_solve_ms": float(np.percentile(
            [row["solve_ms"] for row in rows], 95)),
        "max_solve_ms": float(max(row["solve_ms"] for row in rows)),
        "sampled_object_stiffness_n_m": stiffness,
        "controller_force_stiffness_n_m": controller_force_stiffness,
        "sampled_object_damping_n_s_m": damping,
        "sampled_force_delay_cycles": force_delay_cycles,
        "sampled_peer_delay_cycles": peer_delay_cycles,
        "sampled_actuation_delay_cycles": actuation_delay_cycles,
        "control_steps": trial_steps,
        **components,
    }


def aggregate(raw):
    metric_names = (
        "q_rmse_rad", "relative_position_rmse_m", "orientation_rmse_deg",
        "velocity_rmse_rad_s", "force_rmse_n", "control_effort",
        "common_eval_cost", "realized_common_eval_cost",
        "acceleration_limited_fraction",
        "requested_acceleration_rmse_rad_s2",
        "applied_acceleration_rmse_rad_s2",
        "terminal_target_error_mm",
        "terminal_goal_error_mm", "mean_solve_ms",
        "p95_solve_ms", "force_min_n", "force_max_n",
        "max_slip_duration_s", "max_overforce_duration_s",
        "force_slew_max_n_s", "ee_distance_min_m", "ee_distance_max_m",
        "joint_velocity_max_rad_s", "joint_velocity_ratio_max",
        "joint_acceleration_max_rad_s2", "command_velocity_max_rad_s",
        "command_jump_max_rad_s", "deadline_miss_cycles",
        "deadline_miss_fraction", "maximum_consecutive_deadline_misses",
        "max_solve_ms", "sampled_object_stiffness_n_m",
        "sampled_object_damping_n_s_m", "sampled_force_delay_cycles",
        "sampled_peer_delay_cycles", "sampled_actuation_delay_cycles",
        "Jd", "Jt", "Jo", "Ju", "Jq", "Jv", "JF")
    grouped = {}
    for row in raw:
        grouped.setdefault((row["setting_label"], row["weight_value"]), []).append(row)
    summary = []
    for (label, value), rows in grouped.items():
        item = {
            "weight": rows[0]["weight"],
            "setting_label": label,
            "weight_value": value,
            "runs": len(rows),
            "successes": sum(int(row["task_success"]) for row in rows),
            "success_rate_percent": 100.0 * float(np.mean(
                [row["task_success"] for row in rows])),
            "slip_failures": sum(int(row["slip_failure"]) for row in rows),
            "overforce_failures": sum(int(row["overforce_failure"]) for row in rows),
            "solver_failures": sum(int(row["solver_failure"]) for row in rows),
            "hard_cycle_failures": sum(
                int(row["hard_cycle_failure"]) for row in rows),
        }
        failure_counts = {}
        for row in rows:
            for reason in filter(None, row["hard_failure_reasons"].split(";")):
                failure_counts[reason] = failure_counts.get(reason, 0) + 1
        item["hard_failure_reason_counts"] = json.dumps(
            failure_counts, ensure_ascii=False, sort_keys=True)
        for name in metric_names:
            values = np.asarray([row[name] for row in rows], dtype=float)
            item[name + "_mean"] = float(np.mean(values))
            item[name + "_std"] = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        summary.append(item)
    return summary


def main():
    args = parse_args()
    if args.runs < 1 or args.steps < 1:
        raise SystemExit("runs and steps must be positive")
    if args.settle_cycles < 0:
        raise SystemExit("settle cycles must be nonnegative")
    if (not math.isfinite(args.acceleration_limit_rad_s2)
            or args.acceleration_limit_rad_s2 < 0.0):
        raise SystemExit("acceleration limit must be finite and nonnegative")
    if (args.object_length_m is not None
            and (not math.isfinite(args.object_length_m)
                 or args.object_length_m <= 0.0)):
        raise SystemExit("object length must be finite and positive")
    config = load_json(args.config)
    config["dnmpc_force_stiffness_n_m"] = args.object_stiffness_n_m
    base_weights = resolve_stage_weights(config, "ball", 3)
    if args.base_weights_json:
        base_weights = validate_weights(json.loads(args.base_weights_json), "base")
    evaluation_weights = base_weights
    if args.evaluation_weights_json:
        evaluation_weights = validate_weights(
            json.loads(args.evaluation_weights_json), "evaluation")
    weight_error_scales = resolve_stage_weight_error_scales(
        config, "ball", 3)
    values = [float(value) for value in args.values.split(",")]
    if any(not math.isfinite(value) or value < 0.0 for value in values):
        raise SystemExit("weight values must be finite and nonnegative")
    references = {
        "left": ReferenceTrajectory(args.left_reference, config["source_dt"]),
        "right": ReferenceTrajectory(args.right_reference, config["source_dt"]),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    raw = []
    started = time.time()
    for value_index, value in enumerate(values):
        weights = dict(base_weights)
        weights[args.weight] = value
        label = "v{:02d}".format(value_index)
        print("BUILD {} {}={:.12g}".format(label, args.weight, value), flush=True)
        solvers = {
            side: LocalDNMPC(
                side, int(config["horizon"]), config, weights=weights,
                weight_error_scales=weight_error_scales,
                solver_label="{}_{}_{}".format(args.weight, label, side))
            for side in ("left", "right")
        }
        for seed in range(args.runs):
            row = run_trial(
                args, config, references, weights, evaluation_weights,
                seed, label, solvers,
                weight_error_scales=weight_error_scales)
            raw.append(row)
            print("RUN {} seed={} success={} Ei={:.9g}".format(
                label, seed, int(row["task_success"]), row["common_eval_cost"]),
                flush=True)
    summary = aggregate(raw)
    provenance = {
        "schema": "CM_DNMPC_STAGE3_CURRENT_OBJECTIVE_SWEEP_V1",
        "created_unix": time.time(),
        "elapsed_s": time.time() - started,
        "config": str(args.config.resolve()),
        "left_reference": str(args.left_reference.resolve()),
        "right_reference": str(args.right_reference.resolve()),
        "left_sha256": sha256(args.left_reference),
        "right_sha256": sha256(args.right_reference),
        "reference_raw_points": {
            side: ref.raw_count for side, ref in references.items()},
        "reference_duration_s": {
            side: ref.duration for side, ref in references.items()},
        "controller_base_weights": base_weights,
        "fixed_evaluation_weights": evaluation_weights,
        "weight_error_scales": weight_error_scales,
        "swept_weight": args.weight,
        "values": values,
        "runs": args.runs,
        "steps": args.steps,
        "Ts_s": float(config["Ts"]),
        "horizon": int(config["horizon"]),
        "start_fraction": args.start_fraction,
        "disturbance": {
            "start_fraction": args.disturbance_start_fraction,
            "duration_s": args.disturbance_duration_s,
            "amplitude_rad_s": args.disturbance_amplitude_rad_s,
            "orientation_amplitude_rad_s": (
                args.orientation_disturbance_amplitude_rad_s),
        },
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
            "slip_onset_force_n": args.slip_onset_force_n,
            "drop_force_multiplier": args.drop_force_multiplier,
            "drop_force_threshold_n": (
                args.slip_onset_force_n * args.drop_force_multiplier),
            "drop_definition": (
                "any checked cycle with force below multiplier times the "
                "measured slip-onset force is a drop failure"),
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
        "terminal_cost_definition": (
            "RRT terminal midpoint and inter-arm direction with the Stage-3 "
            "initial force-equilibrium end-effector separation preserved"
            if args.preserve_initial_clamp_at_terminal else
            "global phase-end target, as in the paper's Wd term"),
        "Wt_target_definition": (
            "communicated peer predicted end-effector position x_pi_prime "
            "minus the local predicted position, compared with the desired "
            "grasp offset of magnitude object_length-desired_force/stiffness; "
            "this preserves object size and does not attract both EEs to the "
            "same point"),
        "AIC_force_target_definition": (
            "current force-equilibrium EE position plus the local RRT/FK "
            "horizon displacement increment plus the PaperElastic3 force "
            "correction; the target is not frozen at the cycle start"),
        "preserve_initial_clamp_at_terminal": bool(
            args.preserve_initial_clamp_at_terminal),
        "Ei_definition": (
            "mean over control cycles of both arms' complete five-step "
            "prediction objectives, re-evaluated with fixed x1 weights"),
        "success_terminal_definition": (
            "tracking error at the end of the evaluated Stage-3 window"),
        "collision_checking": "not used",
    }
    payload = {"provenance": provenance, "summary": summary, "raw_runs": raw}
    (args.output / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    for filename, rows in (("summary.csv", summary), ("raw_runs.csv", raw)):
        if rows:
            with (args.output / filename).open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    print("RESULT " + json.dumps(summary, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
