#!/usr/bin/env python3

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import numpy as np


def dh(theta, d, a, alpha):
    ct = ca.cos(theta)
    st = ca.sin(theta)
    calpha = math.cos(alpha)
    salpha = math.sin(alpha)
    return ca.vertcat(
        ca.horzcat(ct, -st * calpha, st * salpha, a * ct),
        ca.horzcat(st, ct * calpha, -ct * salpha, a * st),
        ca.horzcat(0, salpha, calpha, d),
        ca.horzcat(0, 0, 0, 1)
    )


def fk_transform_expression(q, side):
    alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
    if side == "left":
        d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.1434]
        a = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
        transform = ca.DM([
            [-1, 0, 0, 0], [0, -1, 0, 0],
            [0, 0, 1, 0], [0, 0, 0, 1]
        ])
    elif side == "right":
        d = [0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.1257]
        a = [0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0]
        transform = ca.DM([
            [1, 0, 0, 0], [0, 1, 0, 0.437],
            [0, 0, 1, 0], [0, 0, 0, 1]
        ])
    else:
        raise ValueError(side)
    for index in range(6):
        transform = ca.mtimes(
            transform, dh(q[index], d[index], a[index], alpha[index]))
    return transform


def fk_expression(q, side):
    transform = fk_transform_expression(q, side)
    return transform[:3, 3], transform[:3, 2]


def fk_function(side):
    q = ca.SX.sym("q_{}".format(side), 6)
    position, axis = fk_expression(q, side)
    return ca.Function("fk_{}".format(side), [q], [position, axis])


def frame_function(side):
    q = ca.SX.sym("frame_q_{}".format(side), 6)
    transform = fk_transform_expression(q, side)
    return ca.Function(
        "frame_{}".format(side), [q],
        [transform[:3, 3], transform[:3, :3]])


def differential_kinematics_function(side):
    q = ca.SX.sym("differential_q_{}".format(side), 6)
    qdot = ca.SX.sym("differential_qdot_{}".format(side), 6)
    transform = fk_transform_expression(q, side)
    position = transform[:3, 3]
    axis = transform[:3, 2]
    position_jacobian = ca.jacobian(position, q)
    axis_jacobian = ca.jacobian(axis, q)
    state_derivative = ca.vertcat(
        qdot,
        ca.mtimes(position_jacobian, qdot),
        ca.mtimes(axis_jacobian, qdot))
    return ca.Function(
        "differential_kinematics_{}".format(side),
        [q, qdot],
        [state_derivative, position_jacobian, axis_jacobian])


class ReferenceTrajectory:
    def __init__(self, path, source_dt, stride=1, sample_times=None):
        self.path = Path(path)
        self.source_dt = float(source_dt)
        raw = np.atleast_2d(np.loadtxt(str(self.path), dtype=float))
        self.raw_count = int(raw.shape[0])
        if sample_times is None:
            raw_times = np.arange(raw.shape[0], dtype=float) * self.source_dt
        else:
            raw_times = np.asarray(
                np.loadtxt(str(sample_times), dtype=float), dtype=float
            ).reshape(-1)
            if raw_times.shape[0] != raw.shape[0]:
                raise ValueError(
                    "reference/sample-time length mismatch for {}".format(
                        self.path))
            raw_times = raw_times - raw_times[0]
            if (not np.all(np.isfinite(raw_times)) or
                    np.any(np.diff(raw_times) <= 0.0)):
                raise ValueError(
                    "sample times must be finite and strictly increasing")
        self.stride = int(stride)
        if self.stride < 1:
            raise ValueError("reference stride must be positive")
        indices = list(range(0, raw.shape[0], self.stride))
        if indices[-1] != raw.shape[0] - 1:
            indices.append(raw.shape[0] - 1)
        selected = np.asarray(indices, dtype=int)
        self.q = raw[selected]
        self.times = raw_times[selected]
        if self.q.shape[1] != 6 or self.q.shape[0] < 2:
            raise ValueError("invalid reference {}".format(self.path))
        self.qdot = np.gradient(self.q, self.times, axis=0)
        self.duration = float(self.times[-1])

    def closest_time(self, q):
        difference = self.q - np.asarray(q, dtype=float).reshape(1, 6)
        index = int(np.argmin(np.linalg.norm(difference, axis=1)))
        return float(self.times[index])

    def sample(self, time_s):
        clipped = float(np.clip(time_s, 0.0, self.duration))
        upper = int(np.searchsorted(self.times, clipped, side="right"))
        upper = min(max(1, upper), self.q.shape[0] - 1)
        lower = upper - 1
        upper = min(lower + 1, self.q.shape[0] - 1)
        span = self.times[upper] - self.times[lower]
        alpha = 0.0 if span <= 0.0 else (
            clipped - self.times[lower]) / span
        q = (1.0 - alpha) * self.q[lower] + alpha * self.q[upper]
        qdot = (1.0 - alpha) * self.qdot[lower] + alpha * self.qdot[upper]
        return q, qdot

    def horizon(self, time_s, count, dt):
        q = []
        qdot = []
        for index in range(count):
            position, velocity = self.sample(time_s + index * dt)
            q.append(position)
            qdot.append(velocity)
        return np.asarray(q), np.asarray(qdot)


@dataclass
class LocalSolution:
    """Result of one local optimization; no peer joint state is retained."""

    controls: np.ndarray
    joint_prediction: np.ndarray
    position_prediction: np.ndarray
    orientation_prediction: np.ndarray
    success: bool
    solve_ms: float


@dataclass
class AICHorizonState:
    """Adaptive state frozen over one NMPC horizon; geometry is predicted."""

    active: bool
    object_length_m: float
    measured_force_n: float
    desired_force_n: float
    b_hat: float
    k_hat: float
    current_e_m: float
    max_displacement_m: float
    desired_displacement_m: float
    force_scale_n_m: float

    def vector(self):
        values = np.asarray([
            1.0 if self.active else 0.0,
            self.object_length_m,
            self.measured_force_n,
            self.desired_force_n,
            self.b_hat,
            self.k_hat,
            self.current_e_m,
            self.max_displacement_m,
            self.desired_displacement_m,
            self.force_scale_n_m,
        ], dtype=float)
        if not np.all(np.isfinite(values)):
            raise ValueError("AIC horizon state must be finite")
        if values[1] <= 0.0:
            raise ValueError("AIC object length must be positive")
        if values[7] < 0.0:
            raise ValueError("AIC maximum displacement must be nonnegative")
        if abs(values[8]) > values[7] + 1.0e-12:
            raise ValueError(
                "AIC desired displacement exceeds the configured limit")
        if values[9] <= 0.0:
            raise ValueError("AIC force scale must be positive")
        return values


def _finite_matrix(value, shape, label):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != shape or not np.all(np.isfinite(matrix)):
        raise ValueError("{} must have finite shape {}".format(label, shape))
    return matrix


WEIGHT_NAMES = ("Wd", "Wt", "Wo", "Wu", "Wq", "Wv", "WF")


def validate_weights(weights, label="weights"):
    result = {}
    for name in WEIGHT_NAMES:
        if name not in weights:
            raise ValueError("{} is missing {}".format(label, name))
        value = float(weights[name])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("{}.{} must be finite and nonnegative".format(
                label, name))
        result[name] = value
    return result


def validate_weight_error_scales(scales, label="weight_error_scales"):
    result = {}
    for name in WEIGHT_NAMES:
        if name not in scales:
            raise ValueError("{} is missing {}".format(label, name))
        value = float(scales[name])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{}.{} must be finite and positive".format(
                label, name))
        result[name] = value
    return result


def resolve_stage_weights(config, object_name, stage_id):
    """Resolve one explicit Stage 1--4 objective-weight profile."""
    stage_id = int(stage_id)
    if stage_id not in (1, 2, 3, 4):
        raise ValueError("stage_id must be one of 1, 2, 3, 4")
    result = dict(validate_weights(config["weights"], "weights"))
    profiles = config.get("stage_weights", {})
    for profile_name in ("default", object_name):
        profile = profiles.get(profile_name, {})
        override = profile.get(str(stage_id), profile.get(stage_id, {}))
        if override:
            unknown = set(override) - set(WEIGHT_NAMES)
            if unknown:
                raise ValueError(
                    "stage_weights.{}.{} has unknown keys {}".format(
                        profile_name, stage_id, sorted(unknown)))
            result.update({name: float(value) for name, value in override.items()})
    return validate_weights(
        result, "resolved {} Stage {} weights".format(object_name, stage_id))


def resolve_stage_weight_error_scales(config, object_name, stage_id):
    """Resolve characteristic error scales used to nondimensionalize costs."""
    stage_id = int(stage_id)
    if stage_id not in (1, 2, 3, 4):
        raise ValueError("stage_id must be one of 1, 2, 3, 4")
    result = {name: 1.0 for name in WEIGHT_NAMES}
    result.update(config.get("weight_error_scales", {}))
    profiles = config.get("stage_weight_error_scales", {})
    for profile_name in ("default", object_name):
        profile = profiles.get(profile_name, {})
        override = profile.get(str(stage_id), profile.get(stage_id, {}))
        if override:
            unknown = set(override) - set(WEIGHT_NAMES)
            if unknown:
                raise ValueError(
                    "stage_weight_error_scales.{}.{} has unknown keys {}".format(
                        profile_name, stage_id, sorted(unknown)))
            result.update({name: float(value) for name, value in override.items()})
    return validate_weight_error_scales(
        result, "resolved {} Stage {} weight error scales".format(
            object_name, stage_id))


class LocalDNMPC:
    """A single-arm nonlinear optimizer driven by peer Cartesian predictions."""

    def __init__(self, side, horizon, config, weights=None,
                 weight_error_scales=None, solver_label=None):
        if side not in ("left", "right"):
            raise ValueError(side)
        self.side = side
        self.horizon = int(horizon)
        self.config = config
        self.weights = validate_weights(
            config["weights"] if weights is None else weights,
            "LocalDNMPC weights")
        self.weight_error_scales = validate_weight_error_scales(
            ({name: 1.0 for name in WEIGHT_NAMES}
             if weight_error_scales is None else weight_error_scales),
            "LocalDNMPC weight error scales")
        self.force_stiffness_n_m = float(
            config.get("dnmpc_force_stiffness_n_m", 1.0))
        if (not math.isfinite(self.force_stiffness_n_m)
                or self.force_stiffness_n_m <= 0.0):
            raise ValueError("dnmpc_force_stiffness_n_m must be positive")
        self.solver_label = str(solver_label or "default")
        self.position_tracking_mode = config.get(
            "position_tracking_mode", "peer_distance")
        if self.position_tracking_mode not in ("reference", "peer_distance"):
            raise ValueError("Unknown position_tracking_mode")
        self.fk = fk_function(side)
        self.frame_fk = frame_function(side)
        self.differential_kinematics = differential_kinematics_function(side)
        self.warm = np.zeros((self.horizon, 6))
        self._build()

    def _build(self):
        n = self.horizon
        dt = float(self.config["Ts"])
        weights = self.weights
        scales = self.weight_error_scales
        control = ca.SX.sym("u_{}".format(self.side), 6, n)
        parameter_size = (
            6 + 6 * (n + 1) + 6 * n +
            3 + 3 * (n + 1) + 3 * (n + 1) + 10)
        parameter = ca.SX.sym("p_{}".format(self.side), parameter_size)
        offset = 0
        q = parameter[offset:offset + 6]
        offset += 6
        qref = ca.reshape(
            parameter[offset:offset + 6 * (n + 1)], 6, n + 1)
        offset += 6 * (n + 1)
        qdref = ca.reshape(parameter[offset:offset + 6 * n], 6, n)
        offset += 6 * n
        terminal_target = parameter[offset:offset + 3]
        offset += 3
        peer_position = ca.reshape(
            parameter[offset:offset + 3 * (n + 1)], 3, n + 1)
        offset += 3 * (n + 1)
        peer_orientation = ca.reshape(
            parameter[offset:offset + 3 * (n + 1)], 3, n + 1)
        offset += 3 * (n + 1)
        aic = parameter[offset:offset + 10]
        aic_active = aic[0]
        object_length = aic[1]
        measured_force = aic[2]
        desired_force = aic[3]
        b_hat = aic[4]
        k_hat = aic[5]
        current_e = aic[6]
        max_displacement = aic[7]
        desired_displacement = aic[8]
        force_scale = aic[9]

        initial_position, initial_axis = self.fk(q)
        initial_reference_position, _ = self.fk(qref[:, 0])
        state = ca.vertcat(q, initial_position, initial_axis)
        objective = 0
        predicted_joint = [q]
        predicted_state = [state]
        constraints = []
        for stage in range(n):
            q = state[:6]
            position = state[6:9]
            axis = state[9:12]
            peer_line = peer_position[:, stage] - position
            peer_distance = ca.sqrt(ca.sumsqr(peer_line) + 1.0e-12)
            peer_direction = peer_line / peer_distance
            reference_position, _ = self.fk(qref[:, stage])
            desired_peer_distance = ca.fmax(
                1.0e-6,
                object_length - desired_force / ca.fmax(force_scale, 1.0e-6))
            relative_position_error = (
                peer_line - peer_direction * desired_peer_distance)
            position_error = (
                position - reference_position
                if self.position_tracking_mode == "reference"
                else relative_position_error)
            # PaperElastic3 evaluates the corrected AIC law once per measured
            # control cycle.  Hold that Cartesian correction over this NMPC
            # horizon while updating its direction from the communicated peer
            # prediction.  This keeps the adaptive state causal and avoids a
            # second, inconsistent division by k_hat inside the optimizer.
            force_target = (
                initial_position
                + (reference_position - initial_reference_position)
                + peer_direction * desired_displacement)
            objective += dt * (
                weights["Wt"] * ca.sumsqr(
                    position_error / scales["Wt"])
                + weights["Wo"] * ca.sumsqr(
                    (axis + peer_orientation[:, stage]) / scales["Wo"])
                + weights["Wu"] * ca.sumsqr(
                    control[:, stage] / scales["Wu"])
                + weights["Wq"] * ca.sumsqr(
                    (q - qref[:, stage]) / scales["Wq"])
                + weights["Wv"] * ca.sumsqr(
                    (control[:, stage] - qdref[:, stage]) / scales["Wv"])
                + weights["WF"] * aic_active * ca.sumsqr(
                    (position - force_target) / scales["WF"]))
            state_derivative, _, _ = self.differential_kinematics(
                q, control[:, stage])
            state = state + dt * state_derivative
            q = state[:6]
            predicted_joint.append(q)
            predicted_state.append(state)
            constraints.append(q)

        terminal_position = state[6:9]
        objective += weights["Wd"] * ca.sumsqr(
            (terminal_position - terminal_target) / scales["Wd"])
        problem = {
            "x": ca.reshape(control, -1, 1),
            "p": parameter,
            "f": objective,
            "g": ca.vertcat(*constraints)
        }
        settings = self.config["solver"]
        options = {
            "ipopt.max_iter": int(settings["max_iter"]),
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.acceptable_tol": float(settings["acceptable_tol"]),
            "ipopt.acceptable_obj_change_tol": float(
                settings["acceptable_obj_change_tol"])
        }
        self.solver = ca.nlpsol(
            "solver_{}_{}".format(self.side, self.solver_label),
            "ipopt", problem, options)
        input_limit = np.asarray(
            self.config["input_max_rad_s"], dtype=float)
        self.lbx = np.tile(-input_limit, n)
        self.ubx = np.tile(input_limit, n)
        self.lbg = np.tile(
            np.asarray(self.config["joint_min"], dtype=float), n)
        self.ubg = np.tile(
            np.asarray(self.config["joint_max"], dtype=float), n)
        self.prediction = ca.Function(
            "prediction_{}".format(self.side),
            [problem["x"], parameter], [ca.horzcat(*predicted_joint)])
        self.state_prediction = ca.Function(
            "state_prediction_{}".format(self.side),
            [problem["x"], parameter], [ca.horzcat(*predicted_state)])

    def solve(self, q0, qref, qdref, peer_positions,
              peer_orientations, aic_state, terminal_target=None):
        n = self.horizon
        q0 = _finite_matrix(q0, (6,), "q0")
        qref = _finite_matrix(qref, (n + 1, 6), "qref")
        qdref = _finite_matrix(qdref, (n, 6), "qdref")
        peer_positions = _finite_matrix(
            peer_positions, (n + 1, 3), "peer positions")
        peer_orientations = _finite_matrix(
            peer_orientations, (n + 1, 3), "peer orientations")
        if terminal_target is None:
            terminal_target = self.pose(qref[-1])[0]
        terminal_target = _finite_matrix(
            terminal_target, (3,), "terminal target")
        if isinstance(aic_state, AICHorizonState):
            aic_values = aic_state.vector()
        else:
            aic_values = np.asarray(aic_state, dtype=float).reshape(-1)
            if aic_values.shape == (9,):
                # Preserve compatibility with callers using the earlier AIC
                # vector layout.  New callers should pass the force scale
                # explicitly so simulation stages can use their own model.
                aic_values = np.append(
                    aic_values, self.force_stiffness_n_m)
            aic_values = _finite_matrix(
                aic_values, (10,), "AIC horizon state")
        parameter = np.concatenate([
            q0.reshape(-1), qref.reshape(-1), qdref.reshape(-1),
            terminal_target.reshape(-1),
            peer_positions.reshape(-1),
            peer_orientations.reshape(-1),
            aic_values.reshape(-1)
        ])
        started = time.perf_counter()
        try:
            result = self.solver(
                x0=self.warm.reshape(-1),
                p=parameter,
                lbx=self.lbx,
                ubx=self.ubx,
                lbg=self.lbg,
                ubg=self.ubg)
            solve_ms = 1000.0 * (time.perf_counter() - started)
            controls = np.asarray(result["x"]).reshape(
                (6, n), order="F").T
            states = np.asarray(
                self.state_prediction(result["x"], parameter)).T
            success = bool(self.solver.stats().get("success", False))
            self.warm = np.vstack([controls[1:], controls[-1]])
            return LocalSolution(
                controls=controls,
                joint_prediction=states[:, :6],
                position_prediction=states[:, 6:9],
                orientation_prediction=states[:, 9:12],
                success=success,
                solve_ms=solve_ms)
        except Exception:
            solve_ms = 1000.0 * (time.perf_counter() - started)
            joint_prediction = np.vstack([
                q0,
                q0[None, :] + np.cumsum(self.warm, axis=0) *
                float(self.config["Ts"])
            ])
            positions, orientations = self.cartesian_horizon(joint_prediction)
            return LocalSolution(
                controls=self.warm.copy(),
                joint_prediction=joint_prediction,
                position_prediction=positions,
                orientation_prediction=orientations,
                success=False,
                solve_ms=solve_ms)

    def cartesian_horizon(self, joint_prediction):
        joint_prediction = _finite_matrix(
            joint_prediction, (self.horizon + 1, 6), "joint prediction")
        positions = []
        orientations = []
        for joint_state in joint_prediction:
            position, orientation = self.pose(joint_state)
            positions.append(position)
            orientations.append(orientation)
        return np.asarray(positions), np.asarray(orientations)

    def pose(self, q):
        position, axis = self.fk(np.asarray(q, dtype=float))
        return np.asarray(position).reshape(3), np.asarray(axis).reshape(3)

    def frame(self, q):
        position, rotation = self.frame_fk(np.asarray(q, dtype=float))
        return (
            np.asarray(position).reshape(3),
            np.asarray(rotation).reshape(3, 3))

    def jacobians(self, q):
        _, position_jacobian, axis_jacobian = self.differential_kinematics(
            np.asarray(q, dtype=float), np.zeros(6))
        return (
            np.asarray(position_jacobian).reshape(3, 6),
            np.asarray(axis_jacobian).reshape(3, 6))

    def state_derivative(self, q, qdot):
        derivative, _, _ = self.differential_kinematics(
            np.asarray(q, dtype=float), np.asarray(qdot, dtype=float))
        return np.asarray(derivative).reshape(12)


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))
