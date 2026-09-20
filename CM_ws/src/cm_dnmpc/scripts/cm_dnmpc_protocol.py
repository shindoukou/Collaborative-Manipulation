#!/usr/bin/env python3
"""Typed Cartesian prediction protocol for paper-aligned state exchange."""

import time
from dataclasses import dataclass

import numpy as np


PROTOCOL = "CM_DNMPC_CARTESIAN_HORIZON_V1"


class ProtocolError(ValueError):
    pass


def _finite(value, shape, label):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ProtocolError(
            "{} must have finite shape {}".format(label, shape))
    return result


@dataclass
class CartesianHorizon:
    position: np.ndarray
    orientation: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)
        self.orientation = np.asarray(self.orientation, dtype=float)
        if (self.position.ndim != 2 or self.position.shape[1] != 3 or
                self.orientation.shape != self.position.shape or
                not np.all(np.isfinite(self.position)) or
                not np.all(np.isfinite(self.orientation))):
            raise ProtocolError(
                "Cartesian horizon needs matching finite Nx3 arrays")
        norms = np.linalg.norm(self.orientation, axis=1)
        if np.any(norms < 1.0e-9):
            raise ProtocolError("orientation vectors must be nonzero")
        self.orientation = self.orientation / norms[:, None]

    @property
    def count(self):
        return int(self.position.shape[0])

    def to_message(self):
        return {
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist(),
        }

    @classmethod
    def from_message(cls, value, count):
        if not isinstance(value, dict):
            raise ProtocolError("prediction must be an object")
        return cls(
            position=_finite(
                value.get("position"), (count, 3),
                "predicted position"),
            orientation=_finite(
                value.get("orientation"), (count, 3),
                "predicted orientation"))


@dataclass
class ExchangeState:
    side: str
    sequence: int
    wall_time: float
    reference_time: float
    trajectory_duration: float
    current_position: np.ndarray
    current_orientation: np.ndarray
    prediction: CartesianHorizon
    force_common: np.ndarray
    force_sensor_zeroed: np.ndarray
    stage_id: int = 0
    stage_ready: bool = False

    def __post_init__(self):
        if self.side not in ("left", "right"):
            raise ProtocolError("side must be left or right")
        try:
            self.sequence = int(self.sequence)
            self.stage_id = int(self.stage_id)
            self.wall_time = float(self.wall_time)
            self.reference_time = float(self.reference_time)
            self.trajectory_duration = float(self.trajectory_duration)
        except (TypeError, ValueError) as error:
            raise ProtocolError("invalid sequence or time field") from error
        scalars = np.asarray([
            self.wall_time, self.reference_time, self.trajectory_duration])
        if not np.all(np.isfinite(scalars)):
            raise ProtocolError("time fields must be finite")
        if self.stage_id not in (0, 1, 2, 3, 4):
            raise ProtocolError("stage_id must be in [0, 4]")
        self.stage_ready = bool(self.stage_ready)
        self.current_position = _finite(
            self.current_position, (3,), "current position")
        self.current_orientation = _finite(
            self.current_orientation, (3,), "current orientation")
        orientation_norm = np.linalg.norm(self.current_orientation)
        if orientation_norm < 1.0e-9:
            raise ProtocolError("current orientation must be nonzero")
        self.current_orientation = self.current_orientation / orientation_norm
        self.force_common = _finite(
            self.force_common, (3,), "common force")
        self.force_sensor_zeroed = _finite(
            self.force_sensor_zeroed, (3,), "zeroed sensor force")

    def to_message(self):
        return {
            "protocol": PROTOCOL,
            "side": self.side,
            "sequence": self.sequence,
            "wall_time": self.wall_time,
            "reference_time": self.reference_time,
            "trajectory_duration": self.trajectory_duration,
            "stage_id": self.stage_id,
            "stage_ready": self.stage_ready,
            "current": {
                "position": self.current_position.tolist(),
                "orientation": self.current_orientation.tolist(),
            },
            "prediction": self.prediction.to_message(),
            "force_common": self.force_common.tolist(),
            "force_sensor_zeroed": self.force_sensor_zeroed.tolist(),
        }

    @classmethod
    def from_message(cls, message, expected_side, horizon_count):
        if not isinstance(message, dict):
            raise ProtocolError("message must be an object")
        if message.get("protocol") != PROTOCOL:
            raise ProtocolError("protocol mismatch")
        if message.get("side") != expected_side:
            raise ProtocolError("peer side mismatch")
        current = message.get("current")
        if not isinstance(current, dict):
            raise ProtocolError("current state must be an object")
        return cls(
            side=message["side"],
            sequence=message.get("sequence"),
            wall_time=message.get("wall_time"),
            reference_time=message.get("reference_time"),
            trajectory_duration=message.get("trajectory_duration"),
            stage_id=message.get("stage_id", 0),
            stage_ready=message.get("stage_ready", False),
            current_position=current.get("position"),
            current_orientation=current.get("orientation"),
            prediction=CartesianHorizon.from_message(
                message.get("prediction"), horizon_count),
            force_common=message.get("force_common"),
            force_sensor_zeroed=message.get("force_sensor_zeroed"))


class PeerStateCache:
    def __init__(self):
        self.state = None
        self.received_at = None

    def update(self, state):
        if not isinstance(state, ExchangeState):
            raise TypeError("state must be ExchangeState")
        if self.state is not None and state.sequence < self.state.sequence:
            raise ProtocolError("peer sequence moved backward")
        self.state = state
        self.received_at = time.monotonic()

    def stale_s(self):
        if self.received_at is None:
            return float("inf")
        return max(0.0, time.monotonic() - self.received_at)
