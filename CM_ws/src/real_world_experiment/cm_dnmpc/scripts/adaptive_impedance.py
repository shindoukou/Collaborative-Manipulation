#!/usr/bin/env python3
import math

import numpy as np


class PaperElastic3:
    # The offline AIC sweep uses the same 1e-6 threshold when classifying an
    # effective-damping singularity.  Reject a runtime configuration already
    # inside that unsafe neighbourhood instead of relying on the update-time
    # numerical fallback.
    MINIMUM_EFFECTIVE_DAMPING = 1.0e-6

    def __init__(self, dt, k0, wb, wk, b0, desired_force_n, object_length_m,
                 max_displacement_m, minimum_separation_m=1.0e-6):
        raw_values = {
            "dt": dt,
            "k0": k0,
            "wb": wb,
            "wk": wk,
            "b0": b0,
            "desired_force_n": desired_force_n,
            "object_length_m": object_length_m,
            "max_displacement_m": max_displacement_m,
            "minimum_separation_m": minimum_separation_m,
        }
        values = {}
        for name, value in raw_values.items():
            try:
                value = float(value)
            except (TypeError, ValueError, OverflowError):
                raise ValueError("{} must be a finite scalar".format(name))
            if not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
            values[name] = value

        if values["dt"] <= 0.0:
            raise ValueError("dt must be greater than zero")
        if values["k0"] <= 0.0:
            raise ValueError("k0 must be greater than zero")
        if values["wb"] < 0.0:
            raise ValueError("wb must be nonnegative")
        if values["wk"] <= 0.0:
            raise ValueError("wk must be greater than zero")
        if values["desired_force_n"] <= 0.0:
            raise ValueError("desired_force_n must be greater than zero")
        if values["object_length_m"] <= 0.0:
            raise ValueError("object_length_m must be greater than zero")
        if values["max_displacement_m"] < 0.0:
            raise ValueError("max_displacement_m must be nonnegative")
        if values["minimum_separation_m"] < 0.0:
            raise ValueError("minimum_separation_m must be nonnegative")
        initial_effective_damping = values["b0"] - values["wk"]
        if abs(initial_effective_damping) < self.MINIMUM_EFFECTIVE_DAMPING:
            raise ValueError(
                "initial abs(b0 - wk) must be at least {:.12g}".format(
                    self.MINIMUM_EFFECTIVE_DAMPING))

        self.dt = values["dt"]
        self.k0 = values["k0"]
        self.wb = values["wb"]
        self.wk = values["wk"]
        self.b_hat = values["b0"]
        self.desired_force = values["desired_force_n"]
        self.object_length = values["object_length_m"]
        self.max_displacement = values["max_displacement_m"]
        self.minimum_separation = values["minimum_separation_m"]
        self.previous_e = None
        self.previous_direction = None
        self.k_hat = self.k0

    def reset(self):
        self.previous_e = None
        self.previous_direction = None
        self.k_hat = self.k0

    def update(self, own_position, peer_position, measured_clamp_force_n):
        own = np.asarray(own_position, dtype=float).reshape(3)
        peer = np.asarray(peer_position, dtype=float).reshape(3)
        measured_force = float(measured_clamp_force_n)
        if not math.isfinite(measured_force):
            raise ValueError("measured_clamp_force_n must be finite")
        line = peer - own
        distance = float(np.linalg.norm(line))
        direction_valid = distance > self.minimum_separation
        if direction_valid:
            direction = line / distance
            self.previous_direction = direction.copy()
        elif self.previous_direction is None:
            direction = np.zeros(3)
        else:
            direction = self.previous_direction.copy()
        # The paper's signed deformation is negative in compression, which
        # is consistent with m*e_ddot + b_t*e_dot + k_t*e = -F_int and the
        # equilibrium e = -F_d/k_t.
        e = 0.5 * (distance - self.object_length)
        e_dot = 0.0 if self.previous_e is None else (e - self.previous_e) / self.dt
        # dot(b_hat) = -w_b * dot(e)^2.
        self.b_hat += -self.wb * e_dot * e_dot * self.dt
        if abs(e) > 1.0e-9:
            self.k_hat = -self.wk * e_dot / e
        else:
            self.k_hat = self.k0
        if not math.isfinite(self.k_hat):
            self.k_hat = self.k0
        delta_force = self.desired_force - measured_force

        # Substituting k_hat = -w_k*dot(e)/e into
        # b_hat*dot(e) + k_hat*e = Delta F gives the nonsingular equivalent
        # impedance law (b_hat - w_k)*dot(e) = Delta F.  Integrating this
        # rate for one control period implements the paper's continuous law
        # without dividing by e or by a k_hat that approaches zero at steady
        # state.
        effective_damping = self.b_hat - self.wk
        if abs(effective_damping) < 1.0e-9:
            effective_damping = math.copysign(
                1.0e-9,
                effective_damping if effective_damping != 0.0 else -1.0)
        desired_e_dot = delta_force / effective_damping
        deformation_step = desired_e_dot * self.dt
        desired_e = e + deformation_step
        deformation_error = deformation_step

        # e is negative in compression.  The line direction points inward,
        # hence the deformation-reference error maps with the opposite sign
        # to Cartesian displacement.
        displacement = float(np.clip(
            -deformation_error,
            -self.max_displacement,
            self.max_displacement))
        target = own + direction * displacement
        self.previous_e = e
        return target, {
            "e": e,
            "e_dot": e_dot,
            "b_hat": self.b_hat,
            "k_hat": self.k_hat,
            "measured_clamp_force_n": measured_force,
            "delta_force": delta_force,
            "desired_e": desired_e,
            "desired_e_dot": desired_e_dot,
            "deformation_step": deformation_step,
            "deformation_error": deformation_error,
            "effective_damping": effective_damping,
            "displacement": displacement,
            "distance_m": distance,
            "direction_valid": bool(direction_valid),
            "clamp_direction": direction.tolist()
        }
