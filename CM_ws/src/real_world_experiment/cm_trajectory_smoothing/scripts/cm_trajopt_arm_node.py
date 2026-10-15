#!/usr/bin/env python3

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

SOURCE_DIRECTORY = Path(__file__).resolve().parent
if str(SOURCE_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SOURCE_DIRECTORY))

from cm_trajopt_common import LocalCubicTrajectoryOptimizer
from cm_trajopt_protocol import (
    PROTOCOL,
    make_cartesian_state,
    make_time_gradient,
    parse_cartesian_state,
    parse_time_gradient,
)
from cm_trajopt_transport import FramedSocket


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--socket-role", choices=("server", "client"), required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--input-path", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--object", choices=("ball", "box"), required=True)
    parser.add_argument("--phase", choices=("catch", "move"), required=True)
    parser.add_argument("--options-json", type=Path)
    parser.add_argument("--initial-state", type=Path)
    parser.add_argument("--max-iterations", type=int, default=80)
    parser.add_argument("--output-dt", type=float, default=0.01)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def load_options(args):
    options = {}
    if args.options_json:
        options.update(json.loads(args.options_json.read_text()))
    options["phase"] = args.phase
    return options


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    waypoints = np.atleast_2d(np.loadtxt(str(args.input_path), dtype=float))
    resolved_options = load_options(args)
    optimizer = LocalCubicTrajectoryOptimizer(args.side, waypoints, resolved_options)
    initialization = {"method": "zero_inner_velocities"}
    if args.initial_state:
        seed = json.loads(args.initial_state.read_text())
        if seed.get("side") != args.side:
            raise ValueError("initial state belongs to the other arm")
        optimizer.set_initial_state(seed["segment_times"], seed["inner_velocities"])
        initialization = {"method": "explicit_local_warm_start",
                          "path": str(args.initial_state),
                          "sha256": hashlib.sha256(args.initial_state.read_bytes()).hexdigest()}
    peer_side = optimizer.peer_side
    history = []
    converged = False
    started = time.monotonic()

    with FramedSocket(args.socket_role, args.host, args.port, args.timeout) as transport:
        iterations_run = 0
        peer_positions = None
        peer_orientations = None
        for iteration in range(max(args.max_iterations, 1)):
            local_positions, local_orientations = optimizer.cartesian_samples()
            peer_packet = transport.exchange(make_cartesian_state(
                args.side, iteration, optimizer.times,
                local_positions, local_orientations))
            peer_times, peer_positions, peer_orientations = parse_cartesian_state(
                peer_packet, peer_side, iteration,
                optimizer.piece_count, optimizer.sample_count)
            if not np.allclose(peer_times, optimizer.times, rtol=0.0, atol=1.0e-9):
                raise RuntimeError("left/right shared segment times diverged")

            evaluation = optimizer.evaluate(peer_positions, peer_orientations)
            local_spatial_norm = 0.0
            if optimizer.active_spatial_dim:
                local_spatial_norm = float(np.max(np.abs(
                    evaluation.spatial_gradient[: optimizer.active_spatial_dim])))
            peer_gradient_packet = transport.exchange(make_time_gradient(
                args.side, iteration, evaluation.time_gradient,
                local_spatial_norm, evaluation.costs))
            peer_time_gradient, peer_spatial_norm, peer_costs = parse_time_gradient(
                peer_gradient_packet, peer_side, iteration, optimizer.piece_count)
            averaged_time_gradient = 0.5 * (
                evaluation.time_gradient + peer_time_gradient)
            local_converged, spatial_norm, time_norm = optimizer.convergence(
                evaluation, averaged_time_gradient)
            converged = (
                local_converged
                and peer_spatial_norm < optimizer.epsilon_c
            )
            segment_times_before_update = optimizer.times.tolist()
            spatial_turn = (
                (iteration % 2 == 0 and args.side == "left")
                or (iteration % 2 == 1 and args.side == "right"))
            spatial_descent = {
                "method": "projected_gradient_descent_with_backtracking",
                "equation": "x_next=Pi_X(x-alpha*grad_x_J)",
                "scheduled": bool(
                    spatial_turn and not converged
                    and optimizer.active_spatial_dim),
                "attempted": False,
                "accepted": False,
                "configured_step": float(optimizer.spatial_step),
                "accepted_step": 0.0,
                "line_search_trials": 0,
                "cost_before": float(evaluation.total_cost),
                "cost_after": float(evaluation.total_cost),
                "gradient_inf": float(spatial_norm),
                "maximum_variable_change": 0.0,
            }
            time_descent = {
                "method": "projected_shared_time_gradient_descent",
                "equation": (
                    "T_next=Pi_[Tmin,Tmax](T-beta*"
                    "(grad_T_Jl+grad_T_Jr)/2)"),
                "applied": False,
                "step": float(optimizer.time_step),
                "gradient_inf": float(time_norm),
                "maximum_time_change": 0.0,
                "clipped_segment_count": 0,
            }
            if not converged:
                if spatial_turn and optimizer.active_spatial_dim:
                    spatial_descent = optimizer.update_spatial(
                        evaluation, peer_positions, peer_orientations)
                    spatial_descent["scheduled"] = True
                time_descent = optimizer.update_shared_time(
                    averaged_time_gradient)
            history.append({
                "iteration": iteration,
                "local_total_cost": evaluation.total_cost,
                "local_costs": evaluation.costs,
                "peer_costs": peer_costs,
                "spatial_gradient_inf": spatial_norm,
                "peer_spatial_gradient_inf": peer_spatial_norm,
                "averaged_time_gradient_inf": time_norm,
                "segment_times_before_update": segment_times_before_update,
                "segment_times_after_update": optimizer.times.tolist(),
                "segment_times": optimizer.times.tolist(),
                "spatial_descent": spatial_descent,
                "time_descent": time_descent,
                "collision_gradient_used": False,
            })
            iterations_run = iteration + 1
            if converged:
                break

        final_positions, final_orientations = optimizer.cartesian_samples()
        peer_packet = transport.exchange(make_cartesian_state(
            args.side, iterations_run, optimizer.times,
            final_positions, final_orientations))
        peer_times, peer_positions, peer_orientations = parse_cartesian_state(
            peer_packet, peer_side, iterations_run,
            optimizer.piece_count, optimizer.sample_count)
        if not np.allclose(peer_times, optimizer.times, rtol=0.0, atol=1.0e-9):
            raise RuntimeError("final shared segment times diverged")
        final_evaluation = optimizer.evaluate(peer_positions, peer_orientations)
        socket_statistics = transport.statistics()

    sample_times, sampled_path = optimizer.sample_uniform(args.output_dt)
    initial_path_length = float(np.linalg.norm(
        np.diff(waypoints, axis=0), axis=1).sum())
    optimized_path_length = float(np.linalg.norm(
        np.diff(sampled_path, axis=0), axis=1).sum())
    sampled_velocity = np.gradient(sampled_path, sample_times, axis=0)
    velocity_squared_integral = float(np.trapz(
        np.sum(sampled_velocity * sampled_velocity, axis=1), sample_times))
    stem = "{}_{}".format(args.phase, args.side)
    sampled_path_file = args.output_dir / "{}_smoothed_path.txt".format(stem)
    time_file = args.output_dir / "{}_sample_times.txt".format(stem)
    coefficient_file = args.output_dir / "{}_cubic_coefficients.json".format(stem)
    report_file = args.output_dir / "{}_trajopt_report.json".format(stem)
    gradient_trace_file = args.output_dir / "{}_gradient_descent_trace.tsv".format(stem)
    np.savetxt(str(sampled_path_file), sampled_path, fmt="%.17g")
    np.savetxt(str(time_file), sample_times, fmt="%.17g")
    coefficient_file.write_text(json.dumps({
        "basis": "q(tau)=a0+a1*tau+a2*tau^2+a3*tau^3",
        "degree": 3,
        "side": args.side,
        "segment_times": optimizer.times.tolist(),
        "coefficients_Mx4x6": optimizer.coefficients().tolist(),
    }, indent=2) + "\n")
    gradient_trace = np.asarray([
        [
            row["iteration"],
            row["local_total_cost"],
            row["spatial_gradient_inf"],
            int(row["spatial_descent"]["scheduled"]),
            int(row["spatial_descent"]["accepted"]),
            row["spatial_descent"]["accepted_step"],
            row["spatial_descent"]["cost_after"],
            row["averaged_time_gradient_inf"],
            int(row["time_descent"]["applied"]),
            row["time_descent"]["step"],
            row["time_descent"]["maximum_time_change"],
        ]
        for row in history
    ], dtype=float)
    np.savetxt(
        str(gradient_trace_file), gradient_trace, delimiter="\t", fmt="%.17g",
        header=(
            "iteration\tlocal_total_cost\tspatial_gradient_inf\t"
            "spatial_scheduled\tspatial_accepted\tspatial_accepted_step\t"
            "spatial_cost_after\tshared_time_gradient_inf\t"
            "time_update_applied\ttime_step\tmaximum_time_change"),
        comments="")
    report = {
        "protocol": PROTOCOL,
        "architecture": "one_local_six_joint_optimizer_per_process",
        "optimization_method": {
            "name": "explicit_projected_gradient_descent",
            "spatial_variable": (
                "inner polynomial boundary positions and velocities; "
                "c=M(x,T)"),
            "spatial_update": "x_next=Pi_X(x-alpha*grad_x_J)",
            "shared_time_update": (
                "T_next=Pi_[Tmin,Tmax](T-beta*(grad_T_Jl+grad_T_Jr)/2)"),
            "spatial_backtracking": True,
            "collision_cost_present": False,
            "collision_gradient_used": False,
            "post_optimization_fcl_check": False,
        },
        "paper_cost_mapping": {
            "J_pi": "adjacent ordered samples of cubic P_im(t)",
            "J_ui": "analytic integral of squared cubic velocity",
            "J_Ki": {
                "J_qi": "nonnegative piecewise joint-position violation",
                "J_vi": "nonnegative piecewise joint-velocity violation",
            },
            "J_ci": {
                "grasp": "terminal polynomial point only",
                "transport": "all polynomial quadrature nodes",
                "four_independent_weights": True,
            },
            "J_T": "0.5*w_Ti*sum_m(T_im)",
            "collision_term": False,
        },
        "side": args.side,
        "peer_side": peer_side,
        "object": args.object,
        "phase": args.phase,
        "resolved_options": resolved_options,
        "initialization": initialization,
        "weight_verification": {
            "matches_paper": None,
            "status": "not_verifiable_from_paper",
            "reason": (
                "the paper defines trajectory-smoothing weight symbols and relative "
                "0/x1/x2 ablations, but gives no absolute x1 values"),
        },
        "input_path": str(args.input_path),
        "piece_count": optimizer.piece_count,
        "optimization_samples_per_segment_including_endpoints": (
            optimizer.sample_count),
        "interior_optimization_samples_per_segment": (
            optimizer.interior_sample_count),
        "polynomial_degree": 3,
        "iterations": iterations_run,
        "converged": converged,
        "initial_total_cost": history[0]["local_total_cost"],
        "initial_costs": history[0]["local_costs"],
        "final_total_cost": final_evaluation.total_cost,
        "cost_reduction_percent": 100.0 * (
            history[0]["local_total_cost"] - final_evaluation.total_cost
        ) / max(abs(history[0]["local_total_cost"]), 1.0e-12),
        "final_costs": final_evaluation.costs,
        "initial_discrete_path_length": initial_path_length,
        "optimized_path_length": optimized_path_length,
        "path_length_change_percent": 100.0 * (
            optimized_path_length - initial_path_length
        ) / max(initial_path_length, 1.0e-12),
        "joint_velocity_squared_integral": velocity_squared_integral,
        "segment_times": optimizer.times.tolist(),
        "duration": float(np.sum(optimizer.times)),
        "optimization_wall_time_s": time.monotonic() - started,
        "joint_state_transmitted": False,
        "cartesian_state_transmitted": True,
        "time_gradient_exchanged": True,
        "socket": socket_statistics,
        "sample_period_s": args.output_dt,
        "sample_count": int(sampled_path.shape[0]),
        "outputs": {
            "sampled_path": str(sampled_path_file),
            "sample_times": str(time_file),
            "coefficients": str(coefficient_file),
            "gradient_descent_trace": str(gradient_trace_file),
        },
        "history": history,
    }
    report_file.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, separators=(",", ":")))


if __name__ == "__main__":
    main()
