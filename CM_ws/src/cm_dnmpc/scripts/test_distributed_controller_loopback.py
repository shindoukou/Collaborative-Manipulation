#!/usr/bin/env python3
"""Run both local optimizers as processes through the Cartesian TCP contract."""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from cm_dnmpc_common import (
    AICHorizonState,
    LocalDNMPC,
    ReferenceTrajectory,
    load_json,
)
from cm_dnmpc_protocol import CartesianHorizon, ExchangeState
from cm_dnmpc_transport import LocalDuplexPredictionSocket


ROOT = Path("/home/mas-2/3rd_party/CM_ws")
CONFIG = load_json(ROOT / "src/cm_dnmpc/config/controller.json")
HORIZON = int(CONFIG["horizon"])
COUNT = HORIZON + 1
DT = float(CONFIG["Ts"])
CYCLES = 10
LEFT_RECEIVE_PORT = 55997
RIGHT_RECEIVE_PORT = 55998


def selected_reference(side):
    package = ROOT / "src/cm_kirara_ball"
    run = Path((package / "generated/latest_run.txt").read_text(
        encoding="utf-8").strip())
    return ReferenceTrajectory(
        sorted(run.glob("move_{}_samples_*.txt".format(side)))[-1],
        CONFIG["source_dt"])


def worker(side):
    peer_side = "right" if side == "left" else "left"
    receive_port = (
        LEFT_RECEIVE_PORT if side == "left" else RIGHT_RECEIVE_PORT)
    send_port = (
        RIGHT_RECEIVE_PORT if side == "left" else LEFT_RECEIVE_PORT)
    channel = LocalDuplexPredictionSocket(
        "127.0.0.1", "127.0.0.1",
        receive_port, send_port, 1.0, 0.05)
    channel.wait_connected(lambda: False)
    reference = selected_reference(side)
    solver = LocalDNMPC(side, HORIZON, CONFIG)
    q = reference.horizon(0.0, COUNT, DT)[0][0].copy()
    initial_joint, _ = reference.horizon(0.0, COUNT, DT)
    initial_position, initial_orientation = solver.cartesian_horizon(
        initial_joint)
    prediction = CartesianHorizon(initial_position, initial_orientation)
    solve_times = []
    for cycle in range(CYCLES):
        reference_time = cycle * DT
        qref, qdref = reference.horizon(reference_time, COUNT, DT)
        position, orientation = solver.pose(q)
        outbound = ExchangeState(
            side=side,
            sequence=cycle,
            wall_time=time.time(),
            reference_time=reference_time,
            trajectory_duration=reference.duration,
            current_position=position,
            current_orientation=orientation,
            prediction=prediction,
            force_common=np.zeros(3),
            force_sensor_zeroed=np.zeros(3))
        incoming_message = channel.exchange(outbound.to_message())
        peer = ExchangeState.from_message(
            incoming_message, peer_side, COUNT)
        elastic = CONFIG["paper_elastic"]["ball"]["move"]
        current_e = 0.5 * (
            np.linalg.norm(peer.current_position - position)
            - elastic["object_length_m"])
        aic_state = AICHorizonState(
            active=True,
            object_length_m=elastic["object_length_m"],
            measured_force_n=elastic["desired_force_n"],
            desired_force_n=elastic["desired_force_n"],
            b_hat=elastic["b0"],
            k_hat=elastic["k0"],
            current_e_m=current_e,
            max_displacement_m=elastic["max_displacement_m"],
            desired_displacement_m=0.0,
            force_scale_n_m=CONFIG["stage4"]["ball"][
                "sim_stiffness_n_m"])
        solution = solver.solve(
            q, qref, qdref[:-1],
            peer.prediction.position,
            peer.prediction.orientation,
            aic_state)
        if not solution.success:
            raise RuntimeError("{} solve failed at {}".format(side, cycle))
        prediction = CartesianHorizon(
            solution.position_prediction,
            solution.orientation_prediction)
        q = q + DT * solution.controls[0]
        solve_times.append(solution.solve_ms)
    result = {
        "side": side,
        "cycles": len(solve_times),
        "mean_solve_ms": float(np.mean(solve_times)),
        "socket_messages": channel.messages,
        "bytes_sent": channel.bytes_sent,
        "bytes_received": channel.bytes_received,
        "joint_state_transmitted": False,
    }
    channel.close()
    print("WORKER_OK " + json.dumps(result, separators=(",", ":")))


def parse_worker_output(output):
    for line in reversed(output.splitlines()):
        if line.startswith("WORKER_OK "):
            return json.loads(line[len("WORKER_OK "):])
    raise RuntimeError("worker result missing:\n{}".format(output))


def parent():
    command = [sys.executable, str(Path(__file__).resolve()), "--worker"]
    left = subprocess.Popen(
        command + ["left"], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    time.sleep(0.2)
    right = subprocess.Popen(
        command + ["right"], stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True)
    try:
        right_output, _ = right.communicate(timeout=30.0)
        left_output, _ = left.communicate(timeout=30.0)
    except subprocess.TimeoutExpired:
        left.kill()
        right.kill()
        left.wait()
        right.wait()
        raise RuntimeError("distributed loopback timeout")
    if left.returncode != 0 or right.returncode != 0:
        raise RuntimeError(
            "worker failure left={} right={}\nLEFT\n{}\nRIGHT\n{}".format(
                left.returncode, right.returncode,
                left_output, right_output))
    output = {
        "left": parse_worker_output(left_output),
        "right": parse_worker_output(right_output),
    }
    print("DISTRIBUTED_CONTROLLER_LOOPBACK_OK " + json.dumps(
        output, separators=(",", ":")))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", choices=("left", "right"))
    args = parser.parse_args()
    if args.worker:
        worker(args.worker)
    else:
        parent()


if __name__ == "__main__":
    main()
