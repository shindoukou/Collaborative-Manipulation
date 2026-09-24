#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np

from cm_dnmpc_common import (
    AICHorizonState,
    LocalDNMPC,
    ReferenceTrajectory,
    load_json,
    resolve_stage_weight_error_scales,
    resolve_stage_weights,
)


ROOT = Path("/home/mas-2/3rd_party/CM_ws")
CONFIG = load_json(ROOT / "src/cm_dnmpc/config/controller.json")


def selected_run(object_name, phase):
    package = ROOT / "src" / ("cm_kirara_ball" if object_name == "ball" else "cm_kirara_box")
    selector = "latest_catch_run.txt" if object_name == "box" and phase == "catch" else "latest_run.txt"
    return Path((package / "generated" / selector).read_text(encoding="utf-8").strip())


def reference(object_name, phase, side):
    run = selected_run(object_name, phase)
    path = sorted(run.glob("{}_{}_samples_*.txt".format(phase, side)))[-1]
    return ReferenceTrajectory(path, CONFIG["source_dt"])


results = []
for object_name in ("ball", "box"):
    for phase in ("catch", "move"):
        left = reference(object_name, phase, "left")
        right = reference(object_name, phase, "right")
        stage_id = 1 if phase == "catch" else 3
        for side, own, peer in (("left", left, right), ("right", right, left)):
            solver = LocalDNMPC(
                side, CONFIG["horizon"], CONFIG,
                weights=resolve_stage_weights(CONFIG, object_name, stage_id),
                weight_error_scales=resolve_stage_weight_error_scales(
                    CONFIG, object_name, stage_id),
                solver_label="test_stage{}".format(stage_id))
            peer_side = "right" if side == "left" else "left"
            peer_model = LocalDNMPC(peer_side, CONFIG["horizon"], CONFIG)
            qref, qdref = own.horizon(0.0, CONFIG["horizon"] + 1, CONFIG["Ts"])
            peer_q, _ = peer.horizon(0.0, CONFIG["horizon"] + 1, CONFIG["Ts"])
            peer_position, peer_orientation = peer_model.cartesian_horizon(
                peer_q
            )
            elastic = CONFIG["paper_elastic"][object_name][phase]
            own_position = solver.pose(qref[0])[0]
            current_e = 0.5 * (
                np.linalg.norm(peer_position[0] - own_position)
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
                force_scale_n_m=CONFIG[
                    "stage2" if phase == "catch" else "stage4"
                ][object_name]["sim_stiffness_n_m"])
            solution = solver.solve(
                qref[0], qref, qdref[:-1],
                peer_position, peer_orientation, aic_state
            )
            if solution.controls.shape != (CONFIG["horizon"], 6):
                raise RuntimeError("control shape")
            if solution.joint_prediction.shape != (CONFIG["horizon"] + 1, 6):
                raise RuntimeError("prediction shape")
            if solution.position_prediction.shape != (
                    CONFIG["horizon"] + 1, 3):
                raise RuntimeError("position prediction shape")
            if solution.orientation_prediction.shape != (
                    CONFIG["horizon"] + 1, 3):
                raise RuntimeError("orientation prediction shape")
            if (not np.all(np.isfinite(solution.controls)) or
                    not np.all(np.isfinite(solution.joint_prediction)) or
                    not np.all(np.isfinite(solution.position_prediction)) or
                    not np.all(np.isfinite(solution.orientation_prediction))):
                raise RuntimeError("nonfinite result")
            if hasattr(solver, "peer_fk") or hasattr(solver, "peer_frame_fk"):
                raise RuntimeError("local solver owns a peer kinematic model")
            results.append({
                "object": object_name,
                "phase": phase,
                "side": side,
                "success": bool(solution.success),
                "solve_ms": float(solution.solve_ms)
            })

if not all(item["success"] for item in results):
    raise RuntimeError(json.dumps(results, ensure_ascii=False))
print("LOCAL_DNMPC_SOLVER_OK " + json.dumps(results, separators=(",", ":")))
