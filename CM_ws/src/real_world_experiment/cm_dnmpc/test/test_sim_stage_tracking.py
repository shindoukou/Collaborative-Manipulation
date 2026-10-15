#!/usr/bin/env python3
import json
import sys
import unittest
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
sys.path.insert(0, str(PACKAGE / "simulation_runtime"))
import controller_node as node
from controller_core import AICHorizonState, LocalDNMPC, resolve_stage_weights


def configuration(obj):
    path = (PACKAGE / "config/ball_sim.json" if obj == "ball" else
            PACKAGE.parent.parent / "simulation/cm_box/config/sim.json")
    return json.loads(path.read_text())


class SimulationStageTrackingTest(unittest.TestCase):
    def test_stage_policy_and_unchanged_config(self):
        for obj in ("ball", "box"):
            cfg = configuration(obj)
            original = json.loads(json.dumps(cfg))
            for stage in (1, 2, 3, 4):
                for side in ("left", "right"):
                    value = node.stage_solver_config(cfg, obj, stage, "sim", side)
                    self.assertEqual(value["position_tracking_mode"],
                                     "reference" if stage == 1 else "peer_distance")
            self.assertEqual(cfg, original)
            for stage in ("stage2", "stage4"):
                self.assertEqual(cfg[stage][obj]["force_tolerance_n"], 2.0)

    def test_stage1_does_not_prematurely_clamp(self):
        q0 = np.array([-1.256637061, -1.5707, 1.5707, -1.5707, 0, 0])
        qr, vr = np.tile(q0, (6, 1)), np.zeros((5, 6))
        orientation = np.tile([0, -1, 0], (6, 1))
        state = AICHorizonState(False, .18, 0, 16, 0, 500, 0, .02, 0, 1.)
        for obj in ("ball", "box"):
            cfg = configuration(obj)
            weights = resolve_stage_weights(cfg, obj, 1)
            weights.update(Wo=0, Wd=0)
            controls = []
            for peer_x in (.4, 1.):
                solver = LocalDNMPC("left", 5, node.stage_solver_config(
                    cfg, obj, 1, "sim", "left"), weights=weights)
                solution = solver.solve(q0, qr, vr, np.tile([peer_x, .3, .3], (6, 1)),
                                        orientation, state)
                self.assertTrue(solution.success)
                controls.append(solution.controls)
            np.testing.assert_allclose(controls[0], controls[1], atol=1e-7)

    def test_contact_stage_uses_future_peer_prediction(self):
        cfg = configuration("ball")
        weights = dict.fromkeys(("Wd", "Wt", "Wo", "Wu", "Wq", "Wv", "WF"), 0.)
        weights.update(Wt=100., Wu=.01)
        q0 = np.array([-.5, -1., 1.64, -.68, -.5, .36])
        qr, vr = np.tile(q0, (6, 1)), np.zeros((5, 6))
        state = AICHorizonState(True, .18, 16, 16, 0, 500, 0, .02, 0, 625.)
        controls = []
        for offset in (0., .03):
            solver = LocalDNMPC("left", 5, node.stage_solver_config(
                cfg, "ball", 3, "sim", "left"), weights=weights)
            peer = np.tile(solver.pose(q0)[0] + [0, .1544, 0], (6, 1))
            peer[1:, 0] += offset
            solution = solver.solve(q0, qr, vr, peer,
                                    np.tile([0, -1, 0], (6, 1)), state)
            self.assertTrue(solution.success)
            controls.append(solution.controls)
        self.assertGreater(float(np.max(np.abs(controls[0] - controls[1]))), 1e-4)


if __name__ == "__main__":
    unittest.main()
