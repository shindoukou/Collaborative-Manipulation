#!/usr/bin/env python3
"""Static tests for all real-object YAML profiles."""

import json
import unittest
from pathlib import Path

from cm_real_object_config import (
    OBJECT_IDS,
    apply_real_object_profile,
    format_side_value,
    joint_position_velocity_command,
    load_real_object_profile,
    profile_reference,
)


ROOT = Path(__file__).resolve().parent
PROFILE_ROOT = ROOT.parent / "config" / "real_objects"
if not PROFILE_ROOT.is_dir():
    PROFILE_ROOT = ROOT


class RealObjectProfilesTest(unittest.TestCase):
    def profiles(self):
        return [
            load_real_object_profile(
                PROFILE_ROOT / "{}.yaml".format(object_id), object_id)
            for object_id in OBJECT_IDS
        ]

    def test_all_profiles_are_valid_and_have_no_bundled_trajectory(self):
        for profile in self.profiles():
            self.assertFalse(profile["status"]["hardware_validated"])
            for phase in ("catch", "move"):
                for side in ("left", "right"):
                    self.assertIsNone(profile_reference(profile, phase, side))

    def test_each_object_has_a_distinct_four_stage_schedule(self):
        schedules = [
            json.dumps(profile["stage_weights"], sort_keys=True)
            for profile in self.profiles()
        ]
        self.assertEqual(len(schedules), len(set(schedules)))

    def test_overlay_is_scoped_to_selected_object(self):
        base = {
            "Ts": 1.0,
            "horizon": 1,
            "source_dt": 1.0,
            "reference_stride": {},
            "paper_elastic": {},
            "stage_weights": {},
            "stage2": {},
            "stage4": {},
        }
        profile = self.profiles()[0]
        config = apply_real_object_profile(base, profile)
        self.assertIn("ball", config["paper_elastic"])
        self.assertEqual(config["stage_weights"]["ball"], profile["stage_weights"])

    def test_real_command_contains_position_and_velocity(self):
        positions, velocities = joint_position_velocity_command(
            [1, 2, 3, 4, 5, 6], [0.5, -0.5, 1, -1, 2, -2], 0.02)
        self.assertEqual(velocities, [0.5, -0.5, 1.0, -1.0, 2.0, -2.0])
        self.assertAlmostEqual(positions[0], 1.01)
        self.assertAlmostEqual(positions[1], 1.99)

    def test_tcst_real_topics_are_selected_per_side(self):
        profile = self.profiles()[0]
        template = profile["real_io"]["command_topic"]
        self.assertEqual(
            format_side_value(template, "left"),
            "/left/scaled_pos_joint_traj_controller/command")
        self.assertEqual(
            format_side_value(template, "right"),
            "/right/scaled_pos_joint_traj_controller/command")


if __name__ == "__main__":
    unittest.main()
