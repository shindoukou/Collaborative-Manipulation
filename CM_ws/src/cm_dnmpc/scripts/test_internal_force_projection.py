#!/usr/bin/env python3
import math

import numpy as np

from internal_force_projection import ForceFrameCompensator, project_dual_arm_forces


def check_close(actual, expected, name, tolerance=1.0e-9):
    if not np.allclose(actual, expected, atol=tolerance, rtol=0.0):
        raise RuntimeError("{}: actual={} expected={}".format(name, actual, expected))


def main():
    state = project_dual_arm_forces(
        [0.0, 0.0, 0.0], [0.0, 0.2, 0.0],
        [3.0, 12.0, 4.0], [-5.0, -8.0, 6.0]
    )
    check_close(state["left_force_n"], 12.0, "left projection")
    check_close(state["right_force_n"], 8.0, "right projection")
    check_close(state["internal_force_n"], 10.0, "internal force")
    check_close(state["motion_force_n"], 4.0, "motion force")
    check_close(state["left_to_right"], [0.0, 1.0, 0.0], "clamp direction")

    rotated = ForceFrameCompensator(
        sensor_rpy_in_ee=[0.0, 0.0, math.pi / 2.0],
        tool_mass_kg=0.0
    )
    identity = np.eye(3)
    rotated.set_zero_orientation(identity)
    check_close(rotated.to_common([2.0, 0.0, 0.0], identity), [0.0, 2.0, 0.0],
                "sensor-to-common rotation")

    gravity = ForceFrameCompensator(
        sensor_rpy_in_ee=[0.0, 0.0, 0.0],
        tool_mass_kg=1.0,
        gravity_common_m_s2=[0.0, 0.0, -10.0]
    )
    gravity.set_zero_orientation(identity)
    rotation_y_90 = np.asarray([
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0]
    ])
    gravity_change_sensor = rotation_y_90.T.dot([0.0, 0.0, -10.0]) - np.asarray([0.0, 0.0, -10.0])
    check_close(gravity.to_common(gravity_change_sensor, rotation_y_90), np.zeros(3),
                "zero-orientation gravity compensation")

    guarded = project_dual_arm_forces(
        [1.0, 2.0, 3.0], [1.0, 2.0, 3.0],
        [0.0, 7.0, 0.0], [0.0, -9.0, 0.0],
        fallback_left_to_right=[0.0, 1.0, 0.0]
    )
    if guarded["direction_valid"]:
        raise RuntimeError("zero-separation guard incorrectly reported a valid direction")
    check_close(guarded["left_force_n"], 7.0, "guarded left projection")
    check_close(guarded["right_force_n"], 9.0, "guarded right projection")
    print("INTERNAL_FORCE_PROJECTION_OK", state)


if __name__ == "__main__":
    main()
