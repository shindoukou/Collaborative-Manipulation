#!/usr/bin/env python3

import copy

import numpy as np

from cm_trajopt_protocol import (
    PROTOCOL,
    make_cartesian_state,
    make_time_gradient,
    parse_cartesian_state,
    parse_time_gradient,
)


def expect_failure(callback):
    try:
        callback()
    except ValueError:
        return
    raise AssertionError("invalid packet was accepted")


def main():
    times = np.array([0.8, 1.1])
    positions = np.arange(30, dtype=float).reshape(2, 5, 3) * 0.01
    orientations = np.zeros_like(positions)
    orientations[:, :, 2] = 1.0
    state = make_cartesian_state("left", 3, times, positions, orientations)
    parsed_times, parsed_positions, parsed_orientations = parse_cartesian_state(
        state, "left", 3, 2, 5)
    assert np.array_equal(parsed_times, times)
    assert np.array_equal(parsed_positions, positions)
    assert np.array_equal(parsed_orientations, orientations)
    assert state["protocol"] == PROTOCOL
    assert state["joint_state_transmitted"] is False
    for forbidden in ("q", "joint_state", "waypoints", "coefficients", "controls"):
        assert forbidden not in state

    contaminated = copy.deepcopy(state)
    contaminated["q"] = [0.0] * 6
    expect_failure(lambda: parse_cartesian_state(contaminated, "left", 3, 2, 5))

    gradient = make_time_gradient(
        "right", 3, np.array([0.2, -0.4]), 0.1,
        {"path": 1.0, "velocity": 2.0})
    parsed_gradient, spatial_norm, costs = parse_time_gradient(
        gradient, "right", 3, 2)
    assert np.array_equal(parsed_gradient, np.array([0.2, -0.4]))
    assert spatial_norm == 0.1
    assert costs["velocity"] == 2.0
    print("TRAJOPT_PROTOCOL_OK", {
        "protocol": PROTOCOL,
        "cartesian_dimensions": 6,
        "time_gradient_length": 2,
        "joint_state_transmitted": False,
    })


if __name__ == "__main__":
    main()
