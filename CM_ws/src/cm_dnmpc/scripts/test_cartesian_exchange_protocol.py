#!/usr/bin/env python3
import json

import numpy as np

from cm_dnmpc_protocol import (
    PROTOCOL,
    CartesianHorizon,
    ExchangeState,
    PeerStateCache,
    ProtocolError,
)


COUNT = 6
position = np.arange(COUNT * 3, dtype=float).reshape(COUNT, 3) * 0.01
orientation = np.tile([0.0, 0.0, 1.0], (COUNT, 1))
state = ExchangeState(
    side="left",
    sequence=7,
    wall_time=123.0,
    reference_time=0.4,
    trajectory_duration=8.0,
    current_position=position[0],
    current_orientation=orientation[0],
    prediction=CartesianHorizon(position, orientation),
    force_common=np.array([1.0, 2.0, 3.0]),
    force_sensor_zeroed=np.array([0.1, 0.2, 0.3]),
)
message = state.to_message()
encoded = json.dumps(message)
if "current_q" in encoded or "predicted_q" in encoded:
    raise RuntimeError("joint state leaked into Cartesian protocol")
parsed = ExchangeState.from_message(message, "left", COUNT)
if parsed.prediction.position.shape != (COUNT, 3):
    raise RuntimeError("position horizon shape")
if parsed.prediction.orientation.shape != (COUNT, 3):
    raise RuntimeError("orientation horizon shape")
cache = PeerStateCache()
cache.update(parsed)
if cache.state.sequence != 7 or cache.stale_s() < 0.0:
    raise RuntimeError("peer cache")
bad = dict(message)
bad["protocol"] = "wrong"
try:
    ExchangeState.from_message(bad, "left", COUNT)
except ProtocolError:
    pass
else:
    raise RuntimeError("protocol mismatch was accepted")
print("CARTESIAN_EXCHANGE_PROTOCOL_OK", {
    "protocol": PROTOCOL,
    "horizon_rows": COUNT,
    "joint_state_transmitted": False,
})
