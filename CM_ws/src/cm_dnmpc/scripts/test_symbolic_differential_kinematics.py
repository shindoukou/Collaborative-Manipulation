#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np

from cm_dnmpc_common import (
    LocalDNMPC,
    differential_kinematics_function,
    frame_function,
    load_json,
)


ROOT = Path("/home/mas-2/3rd_party/CM_ws")
CONFIG = load_json(ROOT / "src/cm_dnmpc/config/controller.json")
RNG = np.random.default_rng(20260817)
EPSILON = 1.0e-7
records = []

for side in ("left", "right"):
    differential = differential_kinematics_function(side)
    frame = frame_function(side)
    for sample in range(20):
        q = RNG.uniform(-2.0, 2.0, 6)
        qdot = RNG.uniform(-0.5, 0.5, 6)
        derivative, position_jacobian, axis_jacobian = differential(q, qdot)
        derivative = np.asarray(derivative).reshape(12)
        position_jacobian = np.asarray(position_jacobian).reshape(3, 6)
        axis_jacobian = np.asarray(axis_jacobian).reshape(3, 6)
        position_plus, rotation_plus = frame(q + EPSILON * qdot)
        position_minus, rotation_minus = frame(q - EPSILON * qdot)
        position_difference = (
            np.asarray(position_plus).reshape(3)
            - np.asarray(position_minus).reshape(3)
        ) / (2.0 * EPSILON)
        axis_difference = (
            np.asarray(rotation_plus).reshape(3, 3)[:, 2]
            - np.asarray(rotation_minus).reshape(3, 3)[:, 2]
        ) / (2.0 * EPSILON)
        position_error = float(np.max(np.abs(
            derivative[6:9] - position_difference
        )))
        axis_error = float(np.max(np.abs(
            derivative[9:12] - axis_difference
        )))
        multiplication_error = max(
            float(np.max(np.abs(
                derivative[6:9] - position_jacobian.dot(qdot)
            ))),
            float(np.max(np.abs(
                derivative[9:12] - axis_jacobian.dot(qdot)
            )))
        )
        if position_error > 2.0e-8 or axis_error > 2.0e-8:
            raise RuntimeError(
                "finite-difference mismatch side={} sample={} position={} axis={}".format(
                    side, sample, position_error, axis_error
                )
            )
        if multiplication_error > 1.0e-12:
            raise RuntimeError("jacobian multiplication mismatch")
        records.append({
            "side": side,
            "sample": sample,
            "position_error": position_error,
            "axis_error": axis_error,
        })

for side in ("left", "right"):
    solver = LocalDNMPC(side, CONFIG["horizon"], CONFIG)
    q = RNG.uniform(-1.0, 1.0, 6)
    qdot = RNG.uniform(-0.2, 0.2, 6)
    derivative = solver.state_derivative(q, qdot)
    position_jacobian, axis_jacobian = solver.jacobians(q)
    if not np.allclose(derivative[:6], qdot):
        raise RuntimeError("joint differential mismatch")
    if not np.allclose(derivative[6:9], position_jacobian.dot(qdot)):
        raise RuntimeError("position differential mismatch")
    if not np.allclose(derivative[9:12], axis_jacobian.dot(qdot)):
        raise RuntimeError("axis differential mismatch")

print("CM_SYMBOLIC_DIFFERENTIAL_KINEMATICS_OK " + json.dumps({
    "samples": len(records),
    "max_position_error": max(item["position_error"] for item in records),
    "max_axis_error": max(item["axis_error"] for item in records),
}, separators=(",", ":")))
