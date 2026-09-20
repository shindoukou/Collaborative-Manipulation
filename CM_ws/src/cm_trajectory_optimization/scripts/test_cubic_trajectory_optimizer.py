#!/usr/bin/env python3

import numpy as np

from cm_trajopt_common import LocalCubicTrajectoryOptimizer


def make_paths():
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
    return left, right


def options():
    return {
        "phase": "move",
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
    }


def main():
    left_path, right_path = make_paths()
    left = LocalCubicTrajectoryOptimizer("left", left_path, options())
    right = LocalCubicTrajectoryOptimizer("right", right_path, options())
    right_positions, right_orientations = right.cartesian_samples()
    evaluation = left.evaluate(right_positions, right_orientations)
    assert evaluation.spatial_gradient.shape == (left.spatial_dim,)
    assert evaluation.time_gradient.shape == (left.piece_count,)
    assert np.isfinite(evaluation.total_cost)
    assert set((
        "path", "velocity", "joint_position", "joint_velocity",
        "task_position", "task_orientation", "kinematic", "task",
        "time")).issubset(evaluation.costs)
    assert abs(evaluation.costs["time"]
               - 0.5 * options().get("weight_time", 0.05)
               * np.sum(left.times)) < 1.0e-12
    assert not hasattr(left, "peer_fk")
    assert not hasattr(left, "peer_kinematics")

    epsilon = 1.0e-6
    for piece in range(left.piece_count):
        plus = left.times.copy()
        minus = left.times.copy()
        plus[piece] += epsilon
        minus[piece] -= epsilon
        numerical = (
            left.evaluate(right_positions, right_orientations, times=plus).total_cost
            - left.evaluate(right_positions, right_orientations, times=minus).total_cost
        ) / (2.0 * epsilon)
        assert abs(numerical - evaluation.time_gradient[piece]) < 2.0e-4

    coefficients = left.coefficients()
    assert coefficients.shape == (2, 4, 6)
    assert np.allclose(coefficients[0, 0], left_path[0])
    assert np.allclose(coefficients[-1].sum(axis=0), left_path[-1])
    end_velocity = (
        coefficients[0, 1] + 2.0 * coefficients[0, 2]
        + 3.0 * coefficients[0, 3]) / left.times[0]
    start_velocity = coefficients[1, 1] / left.times[1]
    assert np.allclose(end_velocity, start_velocity, atol=1.0e-10)

    # J_pi must be evaluated on polynomial samples: changing an inner knot
    # velocity changes the sampled path cost while all knot positions remain
    # fixed.
    changed_x = left.x.copy()
    changed_x[6 * left.inner_count] = 0.25
    changed_path_cost = left.evaluate(
        right_positions, right_orientations, x=changed_x).costs["path"]
    assert abs(changed_path_cost - evaluation.costs["path"]) > 1.0e-8

    # J_ui is the analytic cubic-polynomial velocity-squared integral.
    dense_integral = 0.0
    for piece, coefficient in enumerate(coefficients):
        tau = np.linspace(0.0, 1.0, 20001)
        derivative_tau = (
            coefficient[1][None, :]
            + 2.0 * tau[:, None] * coefficient[2][None, :]
            + 3.0 * (tau * tau)[:, None] * coefficient[3][None, :])
        qdot = derivative_tau / left.times[piece]
        dense_integral += np.trapz(
            np.sum(qdot * qdot, axis=1), tau) * left.times[piece]
    assert abs(dense_integral - evaluation.costs["velocity"]) < 1.0e-7

    # Exercise every paper constraint component with deliberately tight
    # bounds; all violation costs must be nonnegative and active.
    strict_options = options()
    strict_options.update({
        "joint_min": [-0.1] * 6,
        "joint_max": [0.1] * 6,
        "velocity_max": [0.01] * 6,
        "task_distance_min": 0.0,
        "task_distance_max": 0.01,
        "task_orientation_cos_min": 1.0,
        "weight_task_position_grasp": 0.0,
        "weight_task_orientation_grasp": 0.0,
        "weight_task_position_transport": 3.0,
        "weight_task_orientation_transport": 4.0,
    })
    strict_left = LocalCubicTrajectoryOptimizer(
        "left", left_path, strict_options)
    strict_evaluation = strict_left.evaluate(
        right_positions, right_orientations)
    for component in (
            "joint_position", "joint_velocity",
            "task_position", "task_orientation"):
        assert strict_evaluation.costs[component] > 0.0

    before = evaluation.total_cost
    descent = left.update_spatial(
        evaluation, right_positions, right_orientations)
    after = left.evaluate(right_positions, right_orientations).total_cost
    assert descent["method"] == "projected_gradient_descent_with_backtracking"
    assert descent["equation"] == "x_next=Pi_X(x-alpha*grad_x_J)"
    assert descent["attempted"] is True
    assert descent["accepted"] is True
    assert descent["maximum_knot_position_change"] > 0.0
    assert after <= before + 1.0e-12
    print("CUBIC_LOCAL_TRAJOPT_OK", {
        "degree": 3,
        "pieces": left.piece_count,
        "initial_cost": before,
        "after_spatial_update": after,
        "time_gradient_max_error": 2.0e-4,
        "paper_cost_components": sorted(evaluation.costs),
        "active_violation_components": {
            key: strict_evaluation.costs[key] for key in (
                "joint_position", "joint_velocity",
                "task_position", "task_orientation")},
        "peer_model_loaded": False,
    })


if __name__ == "__main__":
    main()
