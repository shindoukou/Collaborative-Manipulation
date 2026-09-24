#!/usr/bin/env python3
"""Hard-constraint checks and collision-safe recovery for cubic trajectories."""

import json
import math
import tempfile
from pathlib import Path

import numpy as np


def load_coefficients(path):
    payload = json.loads(Path(path).read_text())
    coefficients = np.asarray(payload["coefficients_Mx4x6"], dtype=float)
    times = np.asarray(payload["segment_times"], dtype=float).reshape(-1)
    if (coefficients.ndim != 3 or coefficients.shape[1:] != (4, 6)
            or coefficients.shape[0] != times.size):
        raise ValueError("coefficient file must contain Mx4x6 coefficients and M times")
    if not np.all(np.isfinite(coefficients)) or not np.all(np.isfinite(times)):
        raise ValueError("coefficient file contains non-finite values")
    if np.any(times <= 0.0):
        raise ValueError("segment times must be positive")
    return coefficients, times


def reference_coefficients(waypoints):
    """C1 stop-at-waypoint cubics following every checked joint-space edge."""
    waypoints = np.atleast_2d(np.asarray(waypoints, dtype=float))
    if waypoints.ndim != 2 or waypoints.shape[1] != 6 or waypoints.shape[0] < 2:
        raise ValueError("waypoints must have shape Nx6 with N >= 2")
    delta = np.diff(waypoints, axis=0)
    result = np.zeros((delta.shape[0], 4, 6), dtype=float)
    result[:, 0, :] = waypoints[:-1]
    result[:, 2, :] = 3.0 * delta
    result[:, 3, :] = -2.0 * delta
    return result


def blend_coefficients(reference, optimized, alpha):
    reference = np.asarray(reference, dtype=float)
    optimized = np.asarray(optimized, dtype=float)
    alpha = float(alpha)
    if reference.shape != optimized.shape or reference.ndim != 3:
        raise ValueError("reference and optimized coefficients must have equal shape")
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be in [0, 1]")
    return reference + alpha * (optimized - reference)


def _quadratic_roots(a, b, c):
    if abs(a) < 1.0e-14:
        if abs(b) < 1.0e-14:
            return []
        return [-c / b]
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return []
    root = math.sqrt(max(discriminant, 0.0))
    return [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)]


def _position_extrema(coefficient):
    values = []
    for joint in range(6):
        a0, a1, a2, a3 = coefficient[:, joint]
        taus = [0.0, 1.0]
        taus.extend(
            value for value in _quadratic_roots(3.0 * a3, 2.0 * a2, a1)
            if 0.0 < value < 1.0)
        values.append([a0 + a1 * tau + a2 * tau ** 2 + a3 * tau ** 3
                       for tau in taus])
    return values


def _normalized_velocity_max(coefficient):
    maxima = np.zeros(6, dtype=float)
    for joint in range(6):
        _, a1, a2, a3 = coefficient[:, joint]
        taus = [0.0, 1.0]
        if abs(a3) > 1.0e-14:
            stationary = -a2 / (3.0 * a3)
            if 0.0 < stationary < 1.0:
                taus.append(stationary)
        maxima[joint] = max(
            abs(a1 + 2.0 * a2 * tau + 3.0 * a3 * tau ** 2)
            for tau in taus)
    return maxima


def retime_for_velocity(left_coefficients, right_coefficients, times,
                        left_velocity_max, right_velocity_max):
    times = np.asarray(times, dtype=float).reshape(-1)
    left_velocity_max = np.asarray(left_velocity_max, dtype=float).reshape(6)
    right_velocity_max = np.asarray(right_velocity_max, dtype=float).reshape(6)
    if np.any(left_velocity_max <= 0.0) or np.any(right_velocity_max <= 0.0):
        raise ValueError("velocity limits must be positive")
    result = times.copy()
    for piece in range(times.size):
        left_required = np.max(
            _normalized_velocity_max(left_coefficients[piece]) / left_velocity_max)
        right_required = np.max(
            _normalized_velocity_max(right_coefficients[piece]) / right_velocity_max)
        result[piece] = max(result[piece], left_required, right_required)
    return result


def validate_kinematics(coefficients, times, joint_min, joint_max, velocity_max,
                        tolerance=1.0e-9):
    coefficients = np.asarray(coefficients, dtype=float)
    times = np.asarray(times, dtype=float).reshape(-1)
    joint_min = np.asarray(joint_min, dtype=float).reshape(6)
    joint_max = np.asarray(joint_max, dtype=float).reshape(6)
    velocity_max = np.asarray(velocity_max, dtype=float).reshape(6)
    maximum_position_violation = 0.0
    maximum_velocity_violation = 0.0
    for piece in range(times.size):
        for joint, values in enumerate(_position_extrema(coefficients[piece])):
            maximum_position_violation = max(
                maximum_position_violation,
                max(values) - joint_max[joint],
                joint_min[joint] - min(values),
            )
        actual_velocity = _normalized_velocity_max(coefficients[piece]) / times[piece]
        maximum_velocity_violation = max(
            maximum_velocity_violation,
            float(np.max(actual_velocity - velocity_max)),
        )
    maximum_position_violation = max(0.0, maximum_position_violation)
    maximum_velocity_violation = max(0.0, maximum_velocity_violation)
    return {
        "valid": bool(
            maximum_position_violation <= tolerance
            and maximum_velocity_violation <= tolerance),
        "continuous_polynomial_check": True,
        "max_joint_position_violation_rad": float(maximum_position_violation),
        "max_joint_velocity_violation_rad_s": float(maximum_velocity_violation),
    }


def sample_coefficients(coefficients, times, sample_period):
    coefficients = np.asarray(coefficients, dtype=float)
    times = np.asarray(times, dtype=float).reshape(-1)
    sample_period = float(sample_period)
    if sample_period <= 0.0:
        raise ValueError("sample period must be positive")
    total = float(np.sum(times))
    sample_times = np.arange(0.0, total, sample_period)
    if sample_times.size == 0 or total - sample_times[-1] > 1.0e-10:
        sample_times = np.append(sample_times, total)
    return sample_at_times(coefficients, times, sample_times)


def sample_at_times(coefficients, times, sample_times):
    coefficients = np.asarray(coefficients, dtype=float)
    times = np.asarray(times, dtype=float).reshape(-1)
    sample_times = np.asarray(sample_times, dtype=float).reshape(-1)
    boundaries = np.concatenate([[0.0], np.cumsum(times)])
    path = np.zeros((sample_times.size, 6), dtype=float)
    for index, current in enumerate(sample_times):
        piece = min(
            int(np.searchsorted(boundaries, current, side="right") - 1),
            times.size - 1)
        tau = float(np.clip((current - boundaries[piece]) / times[piece], 0.0, 1.0))
        path[index] = np.array([1.0, tau, tau ** 2, tau ** 3]).dot(
            coefficients[piece])
    return sample_times, path


def sample_pair_for_validation(left_coefficients, right_coefficients, times,
                               sample_period, maximum_joint_step):
    """Combine time and conservative joint-space resolution grids."""
    times = np.asarray(times, dtype=float).reshape(-1)
    maximum_joint_step = float(maximum_joint_step)
    if maximum_joint_step <= 0.0:
        raise ValueError("maximum_joint_step must be positive")
    total = float(np.sum(times))
    points = [np.arange(0.0, total, float(sample_period)), np.array([total])]
    boundaries = np.concatenate([[0.0], np.cumsum(times)])
    for piece, duration in enumerate(times):
        derivative_bound = max(
            float(np.max(_normalized_velocity_max(left_coefficients[piece]))),
            float(np.max(_normalized_velocity_max(right_coefficients[piece]))))
        interval_count = max(1, int(math.ceil(
            derivative_bound / maximum_joint_step)))
        points.append(boundaries[piece] + duration * np.linspace(
            0.0, 1.0, interval_count + 1))
    validation_times = np.unique(np.concatenate(points))
    _, left_path = sample_at_times(
        left_coefficients, times, validation_times)
    _, right_path = sample_at_times(
        right_coefficients, times, validation_times)
    return validation_times, left_path, right_path


def path_metrics(path, sample_times):
    path = np.asarray(path, dtype=float)
    sample_times = np.asarray(sample_times, dtype=float)
    velocity = np.gradient(path, sample_times, axis=0)
    return {
        "sample_count": int(path.shape[0]),
        "duration_s": float(sample_times[-1]),
        "joint_path_length": float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
        "joint_velocity_squared_integral": float(np.trapz(
            np.sum(velocity * velocity, axis=1), sample_times)),
    }


def write_accepted_outputs(output_dir, phase, alpha, times,
                           left_coefficients, right_coefficients,
                           sample_times, left_path, right_path):
    output_dir = Path(output_dir)
    outputs = {}
    for side, coefficients, path in (
            ("left", left_coefficients, left_path),
            ("right", right_coefficients, right_path)):
        path_file = output_dir / "{}_{}_accepted_path.txt".format(phase, side)
        coefficient_file = output_dir / "{}_{}_accepted_cubic_coefficients.json".format(
            phase, side)
        np.savetxt(str(path_file), path, fmt="%.17g")
        coefficient_file.write_text(json.dumps({
            "basis": "q(tau)=a0+a1*tau+a2*tau^2+a3*tau^3",
            "degree": 3,
            "side": side,
            "collision_safe_blend_alpha": float(alpha),
            "segment_times": np.asarray(times, dtype=float).tolist(),
            "coefficients_Mx4x6": np.asarray(coefficients, dtype=float).tolist(),
        }, indent=2) + "\n")
        outputs[side] = {
            "sampled_path": str(path_file),
            "coefficients": str(coefficient_file),
            "metrics": path_metrics(path, sample_times),
        }
    time_file = output_dir / "{}_accepted_sample_times.txt".format(phase)
    np.savetxt(str(time_file), sample_times, fmt="%.17g")
    outputs["sample_times"] = str(time_file)
    return outputs


def recover_collision_safe_pair(
        validator, phase, validator_config, output_dir, sample_period,
        left_waypoints, right_waypoints, left_candidate, right_candidate,
        candidate_times, left_limits, right_limits, validate_pair,
        search_iterations=8, maximum_joint_step=0.002):
    """Return the most optimized verified blend found on the reference ray."""
    left_waypoints = np.atleast_2d(np.asarray(left_waypoints, dtype=float))
    right_waypoints = np.atleast_2d(np.asarray(right_waypoints, dtype=float))
    left_reference = reference_coefficients(left_waypoints)
    right_reference = reference_coefficients(right_waypoints)
    attempts = []
    detour_attempts = []

    def evaluate_coefficients(left_coefficients, right_coefficients, times, label):
        times = retime_for_velocity(
            left_coefficients, right_coefficients, times,
            left_limits["velocity_max"], right_limits["velocity_max"])
        left_kinematics = validate_kinematics(
            left_coefficients, times, left_limits["joint_min"],
            left_limits["joint_max"], left_limits["velocity_max"])
        right_kinematics = validate_kinematics(
            right_coefficients, times, right_limits["joint_min"],
            right_limits["joint_max"], right_limits["velocity_max"])
        output_times, left_output = sample_coefficients(
            left_coefficients, times, sample_period)
        _, right_output = sample_coefficients(
            right_coefficients, times, sample_period)
        validation_times, left_validation, right_validation = sample_pair_for_validation(
            left_coefficients, right_coefficients, times,
            sample_period, maximum_joint_step)
        with tempfile.TemporaryDirectory(prefix="trajopt_recovery_", dir=str(output_dir)) as temporary:
            left_file = Path(temporary) / "left.txt"
            right_file = Path(temporary) / "right.txt"
            np.savetxt(str(left_file), left_validation, fmt="%.17g")
            np.savetxt(str(right_file), right_validation, fmt="%.17g")
            native = validate_pair(
                validator, phase, validator_config, left_file, right_file)
        valid = bool(native["valid"] and left_kinematics["valid"]
                     and right_kinematics["valid"])
        record = {
            "label": label,
            "valid": valid,
            "segment_times": times.tolist(),
            "validation_grid": {
                "time_step_s": float(sample_period),
                "maximum_joint_step_rad": float(maximum_joint_step),
                "sample_count": int(validation_times.size),
            },
            "native_validation": native,
            "left_kinematics": left_kinematics,
            "right_kinematics": right_kinematics,
        }
        return record, (left_coefficients, right_coefficients, times,
                        output_times, left_output, right_output,
                        validation_times, left_validation, right_validation)

    def evaluate(alpha):
        left_coefficients = blend_coefficients(left_reference, left_candidate, alpha)
        right_coefficients = blend_coefficients(right_reference, right_candidate, alpha)
        record, data = evaluate_coefficients(
            left_coefficients, right_coefficients, candidate_times,
            "blend_alpha={:.17g}".format(alpha))
        record["alpha"] = float(alpha)
        attempts.append(record)
        return record, data

    def try_local_detour(reference_record, reference_data):
        validation_times = reference_data[6]
        left_validation = reference_data[7]
        right_validation = reference_data[8]
        invalid = reference_record["native_validation"]["first_invalid_samples"]
        if not invalid:
            return None
        invalid_index = int(invalid[0]["sample"])
        if not 0 <= invalid_index < validation_times.size:
            return None
        current_time = validation_times[invalid_index]
        base_times = np.asarray(reference_record["segment_times"], dtype=float)
        boundaries = np.concatenate([[0.0], np.cumsum(base_times)])
        piece = min(int(np.searchsorted(
            boundaries, current_time, side="right") - 1), base_times.size - 1)
        tau = float(np.clip(
            (current_time - boundaries[piece]) / base_times[piece], 0.05, 0.95))
        insert_at = piece + 1
        left_insert = left_validation[invalid_index]
        right_insert = right_validation[invalid_index]
        base_left = np.insert(left_waypoints, insert_at, left_insert, axis=0)
        base_right = np.insert(right_waypoints, insert_at, right_insert, axis=0)
        split_times = np.insert(base_times, piece + 1, base_times[piece] * (1.0 - tau))
        split_times[piece] = base_times[piece] * tau
        reply = invalid[0].get("reply", "")
        if "right_" in reply and "left_" not in reply:
            side_order = ("right", "left")
        elif "left_" in reply and "right_" not in reply:
            side_order = ("left", "right")
        else:
            side_order = ("right", "left")
        joint_order = (4, 3, 5, 2, 1, 0)
        magnitudes = (0.01, 0.02, 0.03, 0.05)
        for side in side_order:
            for joint in joint_order:
                for magnitude in magnitudes:
                    for sign in (-1.0, 1.0):
                        detour_left = base_left.copy()
                        detour_right = base_right.copy()
                        target = detour_left if side == "left" else detour_right
                        target[insert_at, joint] += sign * magnitude
                        left_coefficients = reference_coefficients(detour_left)
                        right_coefficients = reference_coefficients(detour_right)
                        label = "detour_{}_joint{}_offset={:.17g}".format(
                            side, joint + 1, sign * magnitude)
                        record, data = evaluate_coefficients(
                            left_coefficients, right_coefficients, split_times, label)
                        detour_attempts.append({
                            "label": label,
                            "valid": record["valid"],
                            "invalid_sample_count": record["native_validation"][
                                "invalid_sample_count"],
                            "reason_counts": record["native_validation"]["reason_counts"],
                        })
                        if record["valid"]:
                            return record, data, {
                                "side": side,
                                "joint_index_1_based": joint + 1,
                                "offset_rad": sign * magnitude,
                                "inserted_after_piece_1_based": piece + 1,
                                "source_invalid_sample": invalid_index,
                            }
        return None

    reference_record, reference_data = evaluate(0.0)
    if not reference_record["valid"]:
        detour = try_local_detour(reference_record, reference_data)
        if detour is not None:
            best_record, best_data, detour_details = detour
            (left_coefficients, right_coefficients, times,
             sample_times, left_path, right_path, _, _, _) = best_data
            outputs = write_accepted_outputs(
                output_dir, phase, 0.0, times, left_coefficients,
                right_coefficients, sample_times, left_path, right_path)
            return {
                "attempted": True,
                "successful": True,
                "method": "verified_local_joint_detour_from_reference",
                "alpha": 0.0,
                "detour": detour_details,
                "retimed": True,
                "accepted_segment_times": times.tolist(),
                "accepted_validation": best_record["native_validation"],
                "accepted_validation_grid": best_record["validation_grid"],
                "accepted_kinematics": {
                    "left": best_record["left_kinematics"],
                    "right": best_record["right_kinematics"],
                },
                "outputs": outputs,
                "attempts": attempts,
                "detour_attempts": detour_attempts,
            }
        return {
            "attempted": True,
            "successful": False,
            "reason": "reference and bounded one-joint detours failed hard validation",
            "attempts": attempts,
            "detour_attempts": detour_attempts,
        }

    best_record = reference_record
    best_data = reference_data
    lower = 0.0
    upper = 1.0
    for _ in range(max(0, int(search_iterations))):
        alpha = 0.5 * (lower + upper)
        record, data = evaluate(alpha)
        if record["valid"]:
            lower = alpha
            best_record = record
            best_data = data
        else:
            upper = alpha
    (left_coefficients, right_coefficients, times,
     sample_times, left_path, right_path, _, _, _) = best_data
    outputs = write_accepted_outputs(
        output_dir, phase, best_record["alpha"], times,
        left_coefficients, right_coefficients, sample_times, left_path, right_path)
    return {
        "attempted": True,
        "successful": True,
        "method": "verified_reference_to_optimized_coefficient_blend",
        "alpha": best_record["alpha"],
        "search_interval": [lower, upper],
        "retimed": not np.array_equal(times, np.asarray(candidate_times, dtype=float)),
        "accepted_segment_times": times.tolist(),
        "accepted_validation": best_record["native_validation"],
        "accepted_validation_grid": best_record["validation_grid"],
        "accepted_kinematics": {
            "left": best_record["left_kinematics"],
            "right": best_record["right_kinematics"],
        },
        "outputs": outputs,
        "attempts": attempts,
    }
