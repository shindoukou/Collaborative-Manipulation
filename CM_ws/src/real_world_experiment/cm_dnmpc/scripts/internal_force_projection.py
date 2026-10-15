#!/usr/bin/env python3
import math

import numpy as np


def rotation_matrix_from_rpy(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=float).reshape(3)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def _rotation(value, name):
    matrix = np.asarray(value, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("{} contains nonfinite values".format(name))
    if not np.allclose(matrix.T.dot(matrix), np.eye(3), atol=1.0e-6):
        raise ValueError("{} is not orthonormal".format(name))
    if np.linalg.det(matrix) < 0.999999:
        raise ValueError("{} is not a proper rotation".format(name))
    return matrix


class ForceFrameCompensator:
    def __init__(self, sensor_rpy_in_ee, tool_mass_kg=0.0,
                 gravity_common_m_s2=(0.0, 0.0, -9.80665)):
        self.rotation_ee_sensor = rotation_matrix_from_rpy(sensor_rpy_in_ee)
        self.gravity_force_common = float(tool_mass_kg) * np.asarray(
            gravity_common_m_s2, dtype=float
        ).reshape(3)
        self.reference_rotation_common_sensor = None

    def rotation_common_sensor(self, rotation_common_ee):
        rotation_common_ee = _rotation(rotation_common_ee, "rotation_common_ee")
        return rotation_common_ee.dot(self.rotation_ee_sensor)

    def set_zero_orientation(self, rotation_common_ee):
        self.reference_rotation_common_sensor = self.rotation_common_sensor(rotation_common_ee)

    def to_common(self, zeroed_force_sensor, rotation_common_ee):
        force_sensor = np.asarray(zeroed_force_sensor, dtype=float).reshape(3)
        if not np.all(np.isfinite(force_sensor)):
            raise ValueError("zeroed_force_sensor contains nonfinite values")
        rotation = self.rotation_common_sensor(rotation_common_ee)
        corrected_sensor = force_sensor.copy()
        if self.reference_rotation_common_sensor is not None and np.any(self.gravity_force_common):
            current_gravity_sensor = rotation.T.dot(self.gravity_force_common)
            zero_gravity_sensor = self.reference_rotation_common_sensor.T.dot(
                self.gravity_force_common
            )
            corrected_sensor -= current_gravity_sensor - zero_gravity_sensor
        return rotation.dot(corrected_sensor)


def project_dual_arm_forces(left_position, right_position,
                            left_force_common, right_force_common,
                            minimum_separation_m=1.0e-6,
                            use_absolute=True,
                            fallback_left_to_right=None):
    left_position = np.asarray(left_position, dtype=float).reshape(3)
    right_position = np.asarray(right_position, dtype=float).reshape(3)
    left_force_common = np.asarray(left_force_common, dtype=float).reshape(3)
    right_force_common = np.asarray(right_force_common, dtype=float).reshape(3)
    values = np.r_[left_position, right_position, left_force_common, right_force_common]
    if not np.all(np.isfinite(values)):
        raise ValueError("force projection input contains nonfinite values")
    separation = right_position - left_position
    distance = float(np.linalg.norm(separation))
    direction_valid = distance > float(minimum_separation_m)
    if direction_valid:
        left_to_right = separation / distance
    elif fallback_left_to_right is not None:
        fallback = np.asarray(fallback_left_to_right, dtype=float).reshape(3)
        fallback_norm = float(np.linalg.norm(fallback))
        if np.all(np.isfinite(fallback)) and fallback_norm > 1.0e-12:
            left_to_right = fallback / fallback_norm
        else:
            left_to_right = np.zeros(3)
    else:
        left_to_right = np.zeros(3)
    right_to_left = -left_to_right
    left_force = float(np.dot(left_to_right, left_force_common))
    right_force = float(np.dot(right_to_left, right_force_common))
    if use_absolute:
        left_force = abs(left_force)
        right_force = abs(right_force)
    return {
        "distance_m": distance,
        "direction_valid": bool(direction_valid),
        "left_to_right": left_to_right,
        "right_to_left": right_to_left,
        "left_force_n": left_force,
        "right_force_n": right_force,
        "internal_force_n": 0.5 * (left_force + right_force),
        "motion_force_n": left_force - right_force
    }
