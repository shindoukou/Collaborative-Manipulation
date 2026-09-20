#!/usr/bin/env python3
"""Unit tests for hard checks and the verified recovery geometry."""

import numpy as np

from cm_trajopt_safety import (
    blend_coefficients,
    reference_coefficients,
    retime_for_velocity,
    sample_coefficients,
    sample_pair_for_validation,
    validate_kinematics,
)


def main():
    waypoints = np.array([
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [1.0, -0.4, 0.2, 0.0, 0.0, 0.0],
        [1.2, -0.1, 0.0, 0.1, 0.0, 0.0],
    ])
    reference = reference_coefficients(waypoints)
    for piece in range(reference.shape[0]):
        start = np.array([1.0, 0.0, 0.0, 0.0]).dot(reference[piece])
        finish = np.array([1.0, 1.0, 1.0, 1.0]).dot(reference[piece])
        assert np.allclose(start, waypoints[piece])
        assert np.allclose(finish, waypoints[piece + 1])
        assert np.allclose(reference[piece, 1], 0.0)
        assert np.allclose(
            reference[piece, 1] + 2.0 * reference[piece, 2]
            + 3.0 * reference[piece, 3], 0.0)

    optimized = reference.copy()
    optimized[0, 1, 0] = 4.0
    blended = blend_coefficients(reference, optimized, 0.5)
    assert np.allclose(blended, 0.5 * (reference + optimized))

    base_times = np.array([0.1, 0.1])
    velocity_limit = np.full(6, 0.5)
    times = retime_for_velocity(
        reference, reference, base_times, velocity_limit, velocity_limit)
    validation = validate_kinematics(
        reference, times, np.full(6, -2.0), np.full(6, 2.0), velocity_limit)
    assert validation["valid"], validation
    assert np.any(times > base_times)

    sample_times, sampled = sample_coefficients(reference, times, 0.01)
    assert np.allclose(sampled[0], waypoints[0])
    assert np.allclose(sampled[-1], waypoints[-1])
    assert sample_times[-1] == np.sum(times)
    validation_times, validation_left, validation_right = sample_pair_for_validation(
        reference, reference, times, 0.1, 0.002)
    assert validation_times.size > sample_times.size
    assert np.max(np.abs(np.diff(validation_left, axis=0))) <= 0.00201
    assert np.allclose(validation_left, validation_right)

    overshoot = reference.copy()
    overshoot[0, 1, 0] = 20.0
    invalid = validate_kinematics(
        overshoot, np.ones(2), np.full(6, -2.0), np.full(6, 2.0),
        np.full(6, 100.0))
    assert not invalid["valid"]
    assert invalid["max_joint_position_violation_rad"] > 0.0
    print("trajectory safety tests passed")


if __name__ == "__main__":
    main()
