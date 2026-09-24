#!/usr/bin/env python3
"""Wire contract for Cartesian trajectory state and shared-time gradients."""

import numpy as np


PROTOCOL = "CM_TRAJOPT_CARTESIAN_TIME_GRADIENT_V1"


def _finite_array(value, trailing_shape, label):
    result = np.asarray(value, dtype=float)
    if result.ndim != len(trailing_shape) or tuple(result.shape[1:]) != tuple(trailing_shape[1:]):
        raise ValueError("{} has invalid shape {}".format(label, result.shape))
    if trailing_shape[0] is not None and result.shape[0] != trailing_shape[0]:
        raise ValueError("{} has invalid first dimension {}".format(label, result.shape[0]))
    if not np.all(np.isfinite(result)):
        raise ValueError("{} contains non-finite values".format(label))
    return result


def make_cartesian_state(side, iteration, segment_times, positions, orientations):
    times = np.asarray(segment_times, dtype=float).reshape(-1)
    positions = np.asarray(positions, dtype=float)
    orientations = np.asarray(orientations, dtype=float)
    if side not in ("left", "right"):
        raise ValueError(side)
    if times.size == 0 or np.any(times <= 0.0) or not np.all(np.isfinite(times)):
        raise ValueError("segment_times must be finite and positive")
    if positions.ndim != 3 or positions.shape[0] != times.size or positions.shape[2] != 3:
        raise ValueError("positions must have shape MxSx3")
    if orientations.shape != positions.shape:
        raise ValueError("orientations must match positions")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(orientations)):
        raise ValueError("Cartesian samples must be finite")
    return {
        "protocol": PROTOCOL,
        "kind": "cartesian_state",
        "side": side,
        "iteration": int(iteration),
        "segment_times": times.tolist(),
        "positions": positions.tolist(),
        "orientations": orientations.tolist(),
        "joint_state_transmitted": False,
    }


def parse_cartesian_state(payload, peer_side, iteration, piece_count, sample_count):
    if payload.get("protocol") != PROTOCOL or payload.get("kind") != "cartesian_state":
        raise ValueError("unexpected trajectory optimizer Cartesian message")
    if payload.get("side") != peer_side or int(payload.get("iteration", -1)) != int(iteration):
        raise ValueError("trajectory optimizer Cartesian sequence mismatch")
    if payload.get("joint_state_transmitted") is not False:
        raise ValueError("joint state must not be transmitted")
    forbidden = {"q", "joint_state", "waypoints", "coefficients", "controls"}
    if forbidden.intersection(payload):
        raise ValueError("joint-space data found in Cartesian exchange")
    times = np.asarray(payload["segment_times"], dtype=float).reshape(-1)
    positions = np.asarray(payload["positions"], dtype=float)
    orientations = np.asarray(payload["orientations"], dtype=float)
    if times.shape != (piece_count,) or np.any(times <= 0.0) or not np.all(np.isfinite(times)):
        raise ValueError("invalid peer segment times")
    expected = (piece_count, sample_count, 3)
    if positions.shape != expected or orientations.shape != expected:
        raise ValueError("invalid peer Cartesian sample shape")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(orientations)):
        raise ValueError("non-finite peer Cartesian samples")
    return times, positions, orientations


def make_time_gradient(side, iteration, time_gradient, spatial_gradient_norm, costs):
    gradient = np.asarray(time_gradient, dtype=float).reshape(-1)
    if side not in ("left", "right") or gradient.size == 0 or not np.all(np.isfinite(gradient)):
        raise ValueError("invalid local time gradient")
    clean_costs = {str(key): float(value) for key, value in costs.items()}
    if not np.all(np.isfinite(list(clean_costs.values()))):
        raise ValueError("non-finite cost components")
    return {
        "protocol": PROTOCOL,
        "kind": "time_gradient",
        "side": side,
        "iteration": int(iteration),
        "time_gradient": gradient.tolist(),
        "spatial_gradient_norm": float(spatial_gradient_norm),
        "costs": clean_costs,
    }


def parse_time_gradient(payload, peer_side, iteration, piece_count):
    if payload.get("protocol") != PROTOCOL or payload.get("kind") != "time_gradient":
        raise ValueError("unexpected trajectory optimizer gradient message")
    if payload.get("side") != peer_side or int(payload.get("iteration", -1)) != int(iteration):
        raise ValueError("trajectory optimizer gradient sequence mismatch")
    gradient = np.asarray(payload["time_gradient"], dtype=float).reshape(-1)
    if gradient.shape != (piece_count,) or not np.all(np.isfinite(gradient)):
        raise ValueError("invalid peer time gradient")
    return gradient, float(payload["spatial_gradient_norm"]), dict(payload["costs"])
