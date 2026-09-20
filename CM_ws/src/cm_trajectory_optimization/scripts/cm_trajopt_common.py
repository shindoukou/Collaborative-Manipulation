#!/usr/bin/env python3
"""One-arm cubic trajectory optimizer with peer Cartesian samples as parameters."""

import math
from dataclasses import dataclass

import casadi as ca
import numpy as np


def _fixed_transform(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = np.asarray(xyz, dtype=float)
    return ca.DM(result)


def _axis_rotation(theta, axis):
    cosine = ca.cos(theta)
    sine = ca.sin(theta)
    if axis == "y":
        return ca.vertcat(
            ca.horzcat(cosine, 0, sine, 0),
            ca.horzcat(0, 1, 0, 0),
            ca.horzcat(-sine, 0, cosine, 0),
            ca.horzcat(0, 0, 0, 1),
        )
    if axis == "z":
        return ca.vertcat(
            ca.horzcat(cosine, -sine, 0, 0),
            ca.horzcat(sine, cosine, 0, 0),
            ca.horzcat(0, 0, 1, 0),
            ca.horzcat(0, 0, 0, 1),
        )
    raise ValueError(axis)


def _legacy_fk_expression(q, side):
    if side == "left":
        transform = _fixed_transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        origins = [
            ((0.0, 0.0, 0.163), (0.0, 0.0, 0.0)),
            ((0.0, 0.138, 0.0), (0.0, math.pi / 2.0, 0.0)),
            ((0.0, -0.131, 0.425), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.392), (0.0, math.pi / 2.0, 0.0)),
            ((0.0, 0.127, 0.0), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.1), (0.0, 0.0, 0.0)),
        ]
        tool_origin = ((0.0, 0.1, 0.0), (-math.pi / 2.0, 0.0, 0.0))
    elif side == "right":
        transform = _fixed_transform(
            (0.0, 0.437, 0.0), (0.0, 0.0, math.pi))
        origins = [
            ((0.0, 0.0, 0.1519), (0.0, 0.0, 0.0)),
            ((0.0, 0.1198, 0.0), (0.0, math.pi / 2.0, 0.0)),
            ((0.0, -0.0925, 0.24365), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.21325), (0.0, math.pi / 2.0, 0.0)),
            ((0.0, 0.08505, 0.0), (0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.08535), (0.0, 0.0, 0.0)),
        ]
        tool_origin = ((0.0, 0.0819, 0.0), (-math.pi / 2.0, 0.0, 0.0))
    else:
        raise ValueError(side)
    axes = ("z", "y", "y", "y", "z", "y")
    for index, (origin, rpy) in enumerate(origins):
        transform = ca.mtimes(
            ca.mtimes(transform, _fixed_transform(origin, rpy)),
            _axis_rotation(q[index], axes[index]))
    transform = ca.mtimes(transform, _fixed_transform(*tool_origin))
    return transform[:3, 3], transform[:3, 2]


def _dh_transform(theta, d, a, alpha):
    cosine = ca.cos(theta)
    sine = ca.sin(theta)
    cosine_alpha = math.cos(alpha)
    sine_alpha = math.sin(alpha)
    return ca.vertcat(
        ca.horzcat(cosine, -sine * cosine_alpha,
                   sine * sine_alpha, a * cosine),
        ca.horzcat(sine, cosine * cosine_alpha,
                   -cosine * sine_alpha, a * sine),
        ca.horzcat(0, sine_alpha, cosine_alpha, d),
        ca.horzcat(0, 0, 0, 1))


def _dh_ft300_fk_expression(q, side):
    alpha = (math.pi / 2.0, 0.0, 0.0,
             math.pi / 2.0, -math.pi / 2.0, 0.0)
    if side == "left":
        d = (0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.1434)
        a = (0.0, -0.425, -0.39225, 0.0, 0.0, 0.0)
        transform = ca.DM([
            [-1, 0, 0, 0], [0, -1, 0, 0],
            [0, 0, 1, 0], [0, 0, 0, 1]])
    elif side == "right":
        d = (0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.1257)
        a = (0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0)
        transform = ca.DM([
            [1, 0, 0, 0], [0, 1, 0, 0.437],
            [0, 0, 1, 0], [0, 0, 0, 1]])
    else:
        raise ValueError(side)
    for index in range(6):
        transform = ca.mtimes(
            transform, _dh_transform(q[index], d[index], a[index], alpha[index]))
    return transform[:3, 3], transform[:3, 2]


def fk_expression(q, side, profile="legacy_tool_origin"):
    if profile == "dh_ft300":
        return _dh_ft300_fk_expression(q, side)
    if profile == "legacy_tool_origin":
        return _legacy_fk_expression(q, side)
    raise ValueError("unknown kinematics profile: {}".format(profile))


def _paper_violation(value):
    """Nonnegative piecewise violation used by the paper-form penalties."""
    return ca.if_else(value > 0.0, value, 0.0)


@dataclass
class LocalEvaluation:
    total_cost: float
    costs: dict
    spatial_gradient: np.ndarray
    time_gradient: np.ndarray


class LocalCubicTrajectoryOptimizer:
    """Optimizes only one arm; peer joint states and peer kinematics are absent."""

    def __init__(self, side, waypoints, options):
        if side not in ("left", "right"):
            raise ValueError(side)
        self.side = side
        self.peer_side = "right" if side == "left" else "left"
        self.waypoints = np.atleast_2d(np.asarray(waypoints, dtype=float))
        if self.waypoints.ndim != 2 or self.waypoints.shape[1] != 6 or self.waypoints.shape[0] < 2:
            raise ValueError("waypoints must have finite shape Nx6 with N >= 2")
        if not np.all(np.isfinite(self.waypoints)):
            raise ValueError("waypoints contain non-finite values")
        self.options = dict(options)
        self.piece_count = self.waypoints.shape[0] - 1
        self.inner_count = self.waypoints.shape[0] - 2
        self.active_spatial_dim = 12 * self.inner_count
        self.spatial_dim = max(1, self.active_spatial_dim)
        configured_interior_samples = self.options.get(
            "interior_optimization_samples_per_segment")
        if configured_interior_samples is None:
            self.sample_count = int(
                self.options.get("samples_per_segment", 5))
        else:
            self.interior_sample_count = int(configured_interior_samples)
            if self.interior_sample_count < 0:
                raise ValueError(
                    "interior_optimization_samples_per_segment must be >= 0")
            expected_sample_count = self.interior_sample_count + 2
            if ("samples_per_segment" in self.options and
                    int(self.options["samples_per_segment"]) !=
                    expected_sample_count):
                raise ValueError(
                    "samples_per_segment must equal interior optimization "
                    "samples plus the two endpoints")
            self.sample_count = expected_sample_count
        if self.sample_count < 2:
            raise ValueError("samples_per_segment must be >= 2")
        self.interior_sample_count = self.sample_count - 2
        initial_duration = float(self.options.get("initial_segment_duration", 1.0))
        self.times = np.full(self.piece_count, initial_duration, dtype=float)
        self.minimum_duration = float(self.options.get("minimum_segment_duration", 0.05))
        self.maximum_duration = float(self.options.get("maximum_segment_duration", 20.0))
        self.spatial_step = float(self.options.get("spatial_step", 0.02))
        self.time_step = float(self.options.get("time_step", 0.02))
        self.epsilon_c = float(self.options.get("epsilon_c", 1.0e-4))
        self.epsilon_t = float(self.options.get("epsilon_t", 1.0e-4))
        self.maximum_knot_deviation = float(
            self.options.get("maximum_knot_deviation", 0.05))
        if self.maximum_knot_deviation < 0.0:
            raise ValueError("maximum_knot_deviation must be nonnegative")
        self.inner_velocity_limit = self._vector_option(
            "velocity_max", [0.5 * math.pi] * 6)
        self.x = self._initial_spatial_variables(initial_duration)
        self._build_symbolic_problem()

    def _vector_option(self, name, default):
        value = np.asarray(self.options.get(name, default), dtype=float).reshape(-1)
        if value.shape != (6,) or not np.all(np.isfinite(value)):
            raise ValueError("{} must contain six finite values".format(name))
        return value

    def set_initial_state(self, segment_times, inner_velocities):
        """Warm-start this arm's existing variables without changing the cost.

        End positions/velocities and the number of optimization samples stay
        unchanged. No peer joint data is accepted here.
        """
        durations = np.asarray(segment_times, dtype=float)
        velocities = np.asarray(inner_velocities, dtype=float)
        if self.inner_count == 0 and velocities.size == 0:
            velocities = velocities.reshape(0, 6)
        if (durations.shape != (self.piece_count,) or
                not np.all(np.isfinite(durations)) or
                np.any(durations < self.minimum_duration) or
                np.any(durations > self.maximum_duration)):
            raise ValueError("initial segment times violate shape or duration bounds")
        if (velocities.shape != (self.inner_count, 6) or
                not np.all(np.isfinite(velocities)) or
                np.any(np.abs(velocities) > self.inner_velocity_limit)):
            raise ValueError("initial inner velocities violate shape or velocity bounds")
        self.times = durations.copy()
        self.x[6 * self.inner_count:self.active_spatial_dim] = velocities.reshape(-1)

    def _initial_spatial_variables(self, duration):
        result = np.zeros(self.spatial_dim, dtype=float)
        if self.inner_count == 0:
            return result
        positions = self.waypoints[1:-1].reshape(-1)
        result[: 6 * self.inner_count] = positions
        # Knot positions remain optimization variables.  Only their initial
        # velocities are zero; the paper spatial gradient updates both blocks.
        result[6 * self.inner_count : self.active_spatial_dim] = 0.0
        return result

    def _project_spatial(self, candidate):
        candidate = np.asarray(candidate, dtype=float).copy()
        if not self.inner_count:
            return candidate
        reference = self.waypoints[1:-1].reshape(-1)
        position_count = 6 * self.inner_count
        candidate[:position_count] = np.clip(
            candidate[:position_count],
            reference - self.maximum_knot_deviation,
            reference + self.maximum_knot_deviation)
        velocity_limit = np.tile(self.inner_velocity_limit, self.inner_count)
        candidate[position_count:self.active_spatial_dim] = np.clip(
            candidate[position_count:self.active_spatial_dim],
            -velocity_limit, velocity_limit)
        return candidate

    def _build_symbolic_problem(self):
        x = ca.SX.sym("spatial_{}".format(self.side), self.spatial_dim)
        times = ca.SX.sym("times_{}".format(self.side), self.piece_count)
        peer_positions = ca.SX.sym(
            "peer_positions_{}".format(self.side), 3, self.piece_count * self.sample_count)
        peer_orientations = ca.SX.sym(
            "peer_orientations_{}".format(self.side), 3, self.piece_count * self.sample_count)

        knots = [ca.DM(self.waypoints[0])]
        velocities = [ca.DM.zeros(6, 1)]
        if self.inner_count:
            inner_positions = ca.reshape(x[: 6 * self.inner_count], 6, self.inner_count)
            inner_velocities = ca.reshape(
                x[6 * self.inner_count : self.active_spatial_dim], 6, self.inner_count)
            for index in range(self.inner_count):
                knots.append(inner_positions[:, index])
                velocities.append(inner_velocities[:, index])
        knots.append(ca.DM(self.waypoints[-1]))
        velocities.append(ca.DM.zeros(6, 1))

        legacy_task_position = float(
            self.options.get("weight_task_position", 100.0))
        legacy_task_orientation = float(
            self.options.get("weight_task_orientation", 100.0))
        weights = {
            "path": float(self.options.get("weight_path", 1.0)),
            "velocity": float(self.options.get("weight_velocity", 1.0)),
            "joint": float(self.options.get("weight_joint_limit", 100.0)),
            "speed": float(self.options.get("weight_velocity_limit", 100.0)),
            "task_position_grasp": float(self.options.get(
                "weight_task_position_grasp", legacy_task_position)),
            "task_orientation_grasp": float(self.options.get(
                "weight_task_orientation_grasp", legacy_task_orientation)),
            "task_position_transport": float(self.options.get(
                "weight_task_position_transport", legacy_task_position)),
            "task_orientation_transport": float(self.options.get(
                "weight_task_orientation_transport", legacy_task_orientation)),
            "time": float(self.options.get("weight_time", 0.05)),
        }
        q_min = ca.DM(self._vector_option("joint_min", [-2.0 * math.pi] * 6))
        q_max = ca.DM(self._vector_option("joint_max", [2.0 * math.pi] * 6))
        v_max = ca.DM(self._vector_option("velocity_max", [0.5 * math.pi] * 6))
        distance_min = float(self.options.get("task_distance_min", 0.0))
        distance_max = float(self.options.get("task_distance_max", 1.0e6))
        orientation_cos_min = float(self.options.get("task_orientation_cos_min", -1.0))
        phase = str(self.options.get("phase", "move"))
        kinematics_profile = str(self.options.get(
            "kinematics_profile", "legacy_tool_origin"))
        if phase not in ("catch", "move"):
            raise ValueError("phase must be catch or move")

        task_position_weight = (
            weights["task_position_grasp"] if phase == "catch"
            else weights["task_position_transport"])
        task_orientation_weight = (
            weights["task_orientation_grasp"] if phase == "catch"
            else weights["task_orientation_transport"])

        path_cost = 0
        velocity_cost = 0
        joint_position_cost = 0
        joint_velocity_cost = 0
        task_position_cost = 0
        task_orientation_cost = 0
        coefficient_columns = []
        previous_path_sample = None
        for piece in range(self.piece_count):
            duration = times[piece]
            q0 = knots[piece]
            q1 = knots[piece + 1]
            v0 = velocities[piece]
            v1 = velocities[piece + 1]
            a0 = q0
            a1 = duration * v0
            a2 = 3.0 * (q1 - q0) - duration * (2.0 * v0 + v1)
            a3 = 2.0 * (q0 - q1) + duration * (v0 + v1)
            coefficient_columns.append(ca.horzcat(a0, a1, a2, a3))

            # Exact integral of ||dP_im(t)/dt||^2 for the cubic polynomial.
            # P(tau)=a0+a1*tau+a2*tau^2+a3*tau^3 and dt=T*d(tau).
            velocity_integral = (
                ca.sumsqr(a1)
                + 2.0 * ca.dot(a1, a2)
                + 2.0 * ca.dot(a1, a3)
                + (4.0 / 3.0) * ca.sumsqr(a2)
                + 3.0 * ca.dot(a2, a3)
                + (9.0 / 5.0) * ca.sumsqr(a3)
            ) / duration
            velocity_cost += weights["velocity"] * velocity_integral

            for sample in range(self.sample_count):
                tau = float(sample) / float(self.sample_count - 1)
                quadrature_weight = 0.5 if sample in (0, self.sample_count - 1) else 1.0
                quadrature_weight /= float(self.sample_count - 1)
                q = a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau
                qdot = (a1 + 2.0 * a2 * tau + 3.0 * a3 * tau * tau) / duration

                # J_pi is evaluated on the cubic polynomial samples, rather
                # than only on the original planning knot-to-knot endpoints.
                if previous_path_sample is not None and not (
                        piece > 0 and sample == 0):
                    path_cost += weights["path"] * ca.sumsqr(
                        q - previous_path_sample)
                previous_path_sample = q

                # Paper-form, nonnegative piecewise kinematic violations.
                # The signs in the manuscript must be the violating value
                # minus its bound; the reverse sign would reward violations.
                joint_upper = _paper_violation(
                    ca.power(q, 2) - ca.power(q_max, 2)) * ca.if_else(
                        q > q_max, 1.0, 0.0)
                joint_lower = _paper_violation(
                    ca.power(q, 2) - ca.power(q_min, 2)) * ca.if_else(
                        q < q_min, 1.0, 0.0)
                speed_violation = _paper_violation(
                    ca.power(qdot, 2) - ca.power(v_max, 2))
                node_scale = quadrature_weight * duration
                joint_position_cost += (
                    node_scale * weights["joint"]
                    * ca.sum1(joint_upper + joint_lower))
                joint_velocity_cost += (
                    node_scale * weights["speed"]
                    * ca.sum1(speed_violation))

                task_sample_active = phase == "move" or (
                    phase == "catch" and piece + 1 == self.piece_count and sample + 1 == self.sample_count)
                if task_sample_active:
                    position, axis = fk_expression(
                        q, self.side, kinematics_profile)
                    peer_index = piece * self.sample_count + sample
                    relative = peer_positions[:, peer_index] - position
                    distance_squared = ca.sumsqr(relative) + 1.0e-12
                    distance = ca.sqrt(distance_squared)
                    direction = relative / distance
                    cosine = ca.dot(axis, direction)
                    distance_penalty = (
                        _paper_violation(
                            distance_squared - distance_max * distance_max)
                        + _paper_violation(
                            distance_min * distance_min - distance_squared)
                    )
                    # Signed form of the manuscript's angular inequality:
                    # x_o dot p >= cos(theta_d)||p||. Squaring both sides
                    # would incorrectly accept an axis pointing backwards.
                    orientation_penalty = ca.sumsqr(_paper_violation(
                        orientation_cos_min * distance
                        - ca.dot(axis, relative)))
                    task_scale = 1.0 if phase == "catch" else quadrature_weight * duration
                    task_position_cost += (
                        task_scale * task_position_weight * distance_penalty)
                    task_orientation_cost += (
                        task_scale * task_orientation_weight
                        * orientation_penalty)

        time_cost = 0.5 * weights["time"] * ca.sum1(times)
        kinematic_cost = joint_position_cost + joint_velocity_cost
        task_cost = task_position_cost + task_orientation_cost
        total = path_cost + velocity_cost + kinematic_cost + task_cost + time_cost
        spatial_gradient = ca.gradient(total, x)
        time_gradient = ca.gradient(total, times)
        coefficients = ca.vertcat(*coefficient_columns)
        inputs = [x, times, peer_positions, peer_orientations]
        self._evaluate = ca.Function(
            "trajopt_evaluate_{}".format(self.side), inputs,
            [total, path_cost, velocity_cost,
             joint_position_cost, joint_velocity_cost,
             task_position_cost, task_orientation_cost, time_cost,
             spatial_gradient, time_gradient])
        self._coefficients = ca.Function(
            "trajopt_coefficients_{}".format(self.side), [x, times], [coefficients])
        q_symbol = ca.SX.sym("fk_q_{}".format(self.side), 6)
        fk_position, fk_axis = fk_expression(
            q_symbol, self.side, kinematics_profile)
        self._fk = ca.Function("trajopt_fk_{}".format(self.side), [q_symbol], [fk_position, fk_axis])

    def _peer_matrices(self, positions, orientations):
        expected = (self.piece_count, self.sample_count, 3)
        positions = np.asarray(positions, dtype=float)
        orientations = np.asarray(orientations, dtype=float)
        if positions.shape != expected or orientations.shape != expected:
            raise ValueError("peer Cartesian samples must have shape {}".format(expected))
        if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(orientations)):
            raise ValueError("peer Cartesian samples contain non-finite values")
        return (
            positions.reshape(-1, 3).T,
            orientations.reshape(-1, 3).T,
        )

    def evaluate(self, peer_positions, peer_orientations, x=None, times=None):
        peer_positions, peer_orientations = self._peer_matrices(peer_positions, peer_orientations)
        values = self._evaluate(
            self.x if x is None else x,
            self.times if times is None else times,
            peer_positions,
            peer_orientations,
        )
        costs = {
            "path": float(values[1]),
            "velocity": float(values[2]),
            "joint_position": float(values[3]),
            "joint_velocity": float(values[4]),
            "task_position": float(values[5]),
            "task_orientation": float(values[6]),
            "time": float(values[7]),
        }
        costs["kinematic"] = (
            costs["joint_position"] + costs["joint_velocity"])
        costs["task"] = (
            costs["task_position"] + costs["task_orientation"])
        return LocalEvaluation(
            total_cost=float(values[0]),
            costs=costs,
            spatial_gradient=np.asarray(values[8], dtype=float).reshape(-1),
            time_gradient=np.asarray(values[9], dtype=float).reshape(-1),
        )

    def update_spatial(self, evaluation, peer_positions, peer_orientations):
        result = {
            "method": "projected_gradient_descent_with_backtracking",
            "equation": "x_next=Pi_X(x-alpha*grad_x_J)",
            "attempted": bool(self.active_spatial_dim),
            "accepted": False,
            "configured_step": float(self.spatial_step),
            "accepted_step": 0.0,
            "line_search_trials": 0,
            "cost_before": float(evaluation.total_cost),
            "cost_after": float(evaluation.total_cost),
            "gradient_inf": 0.0,
            "maximum_variable_change": 0.0,
            "maximum_knot_position_change": 0.0,
        }
        if self.active_spatial_dim == 0:
            return result
        gradient = evaluation.spatial_gradient.copy()
        gradient[self.active_spatial_dim :] = 0.0
        result["gradient_inf"] = float(np.max(np.abs(
            gradient[:self.active_spatial_dim])))
        step = self.spatial_step
        initial_x = self.x.copy()
        for trial in range(12):
            candidate = self._project_spatial(self.x - step * gradient)
            candidate_evaluation = self.evaluate(peer_positions, peer_orientations, x=candidate)
            if candidate_evaluation.total_cost <= evaluation.total_cost:
                self.x = candidate
                result.update({
                    "accepted": True,
                    "accepted_step": float(step),
                    "line_search_trials": trial + 1,
                    "cost_after": float(candidate_evaluation.total_cost),
                    "maximum_variable_change": float(np.max(np.abs(
                        self.x[:self.active_spatial_dim]
                        - initial_x[:self.active_spatial_dim]))),
                    "maximum_knot_position_change": float(np.max(np.abs(
                        self.x[:6 * self.inner_count]
                        - initial_x[:6 * self.inner_count])))
                    if self.inner_count else 0.0,
                })
                return result
            step *= 0.5
        result["line_search_trials"] = 12
        return result

    def update_shared_time(self, averaged_gradient):
        averaged_gradient = np.asarray(averaged_gradient, dtype=float).reshape(-1)
        if averaged_gradient.shape != (self.piece_count,) or not np.all(np.isfinite(averaged_gradient)):
            raise ValueError("averaged time gradient has invalid shape")
        previous = self.times.copy()
        unconstrained = previous - self.time_step * averaged_gradient
        self.times = np.clip(
            unconstrained,
            self.minimum_duration,
            self.maximum_duration,
        )
        return {
            "method": "projected_shared_time_gradient_descent",
            "equation": "T_next=Pi_[Tmin,Tmax](T-beta*(grad_T_Jl+grad_T_Jr)/2)",
            "applied": True,
            "step": float(self.time_step),
            "gradient_inf": float(np.max(np.abs(averaged_gradient))),
            "maximum_time_change": float(np.max(np.abs(self.times - previous))),
            "clipped_segment_count": int(np.count_nonzero(
                np.abs(self.times - unconstrained) > 1.0e-12)),
        }

    def coefficients(self):
        raw = np.asarray(self._coefficients(self.x, self.times), dtype=float)
        return raw.reshape(self.piece_count, 6, 4).transpose(0, 2, 1)

    def joint_samples(self):
        coefficients = self.coefficients()
        samples = np.zeros((self.piece_count, self.sample_count, 6), dtype=float)
        for piece in range(self.piece_count):
            for sample in range(self.sample_count):
                tau = float(sample) / float(self.sample_count - 1)
                basis = np.array([1.0, tau, tau * tau, tau * tau * tau])
                samples[piece, sample] = basis.dot(coefficients[piece])
        return samples

    def cartesian_samples(self):
        joints = self.joint_samples()
        positions = np.zeros((self.piece_count, self.sample_count, 3), dtype=float)
        orientations = np.zeros_like(positions)
        for piece in range(self.piece_count):
            for sample in range(self.sample_count):
                position, orientation = self._fk(joints[piece, sample])
                positions[piece, sample] = np.asarray(position, dtype=float).reshape(3)
                orientations[piece, sample] = np.asarray(orientation, dtype=float).reshape(3)
        return positions, orientations

    def sample_uniform(self, sample_period):
        sample_period = float(sample_period)
        if sample_period <= 0.0:
            raise ValueError("sample_period must be positive")
        coefficients = self.coefficients()
        total = float(np.sum(self.times))
        global_times = np.arange(0.0, total, sample_period)
        if global_times.size == 0 or total - global_times[-1] > 1.0e-10:
            global_times = np.append(global_times, total)
        result = np.zeros((global_times.size, 6), dtype=float)
        boundaries = np.concatenate([[0.0], np.cumsum(self.times)])
        for index, global_time in enumerate(global_times):
            piece = min(int(np.searchsorted(boundaries, global_time, side="right") - 1), self.piece_count - 1)
            tau = (global_time - boundaries[piece]) / self.times[piece]
            tau = float(np.clip(tau, 0.0, 1.0))
            basis = np.array([1.0, tau, tau * tau, tau * tau * tau])
            result[index] = basis.dot(coefficients[piece])
        return global_times, result

    def convergence(self, evaluation, averaged_time_gradient):
        spatial_norm = 0.0
        if self.active_spatial_dim:
            spatial_norm = float(np.max(np.abs(evaluation.spatial_gradient[: self.active_spatial_dim])))
        time_norm = float(np.max(np.abs(averaged_time_gradient)))
        return spatial_norm < self.epsilon_c and time_norm < self.epsilon_t, spatial_norm, time_norm
