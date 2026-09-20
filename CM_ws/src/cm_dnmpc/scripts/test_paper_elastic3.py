#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np

from paper_elastic3 import PaperElastic3


def make_model(**overrides):
    values = {
        "dt": 0.02,
        "k0": 500.0,
        "wb": 10.0,
        "wk": 10.0,
        "b0": 0.0,
        "desired_force_n": 16.0,
        "object_length_m": 0.18,
        "max_displacement_m": 0.02,
    }
    values.update(overrides)
    return PaperElastic3(**values)


def expect_value_error(**overrides):
    try:
        make_model(**overrides)
    except ValueError:
        return
    raise RuntimeError("invalid PaperElastic3 configuration was accepted: {}"
                       .format(overrides))


def check_runtime_configuration_validation():
    expect_value_error(wk=0.0)
    expect_value_error(k0=0.0)
    expect_value_error(wb=-1.0)
    expect_value_error(desired_force_n=0.0)
    expect_value_error(object_length_m=0.0)
    expect_value_error(max_displacement_m=-0.001)
    expect_value_error(b0=10.0 - 0.5e-6)
    for name in (
            "dt", "k0", "wb", "wk", "b0", "desired_force_n",
            "object_length_m", "max_displacement_m",
            "minimum_separation_m"):
        expect_value_error(**{name: float("nan")})
        expect_value_error(**{name: float("inf")})

    script_path = Path(__file__).resolve()
    config_candidates = (
        script_path.with_name("controller.json"),
        script_path.parent.parent / "config" / "controller.json",
    )
    config_path = next((path for path in config_candidates if path.is_file()), None)
    if config_path is None:
        raise RuntimeError("controller.json was not found next to the test or in ../config")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    for object_name in ("ball", "box"):
        for phase in ("catch", "move"):
            PaperElastic3(
                dt=config["Ts"],
                **config["paper_elastic"][object_name][phase])

    # Runtime construction remains strict, while an out-of-tree offline sweep
    # can intentionally replace a public parameter after constructing the
    # valid nominal x1 controller in order to evaluate an x0 boundary case.
    sweep_model = make_model()
    sweep_model.wk = 0.0
    target, state = sweep_model.update(
        [0, 0, 0], [0, 0.18, 0], 10.0)
    if not np.all(np.isfinite(np.r_[target, state["desired_e_dot"]])):
        raise RuntimeError("direct offline x0 override produced nonfinite output")


def main():
    check_runtime_configuration_validation()
    model = make_model()
    target0, state0 = model.update([0, 0, 0], [0, 0.18, 0], 10.0)
    target1, state1 = model.update([0, 0, 0], [0, 0.17, 0], 20.0)
    values = np.r_[
        target0, target1,
        state0["e"], state0["delta_force"], state0["displacement"],
        state1["e"], state1["delta_force"], state1["displacement"]
    ]
    if not np.all(np.isfinite(values)):
        raise RuntimeError("nonfinite paper elastic output")
    if abs(state0["k_hat"] - 500.0) > 1.0e-9:
        raise RuntimeError("k0 branch failed")
    if state1["k_hat"] >= 0.0:
        raise RuntimeError("paper signed stiffness branch failed")
    if state0["delta_force"] != 6.0 or state1["delta_force"] != -4.0:
        raise RuntimeError("paper force-error sign was not used")
    if not (target0[1] > 0.0 and state0["displacement"] > 0.0):
        raise RuntimeError("insufficient force did not command inward motion")
    if not (target1[1] < 0.0 and state1["displacement"] < 0.0):
        raise RuntimeError("excess force did not command outward motion")
    expected_b_hat = -10.0 * 0.25 ** 2 * 0.02
    if abs(state1["b_hat"] - expected_b_hat) > 1.0e-12:
        raise RuntimeError("squared damping adaptation law was not used")
    _, zero_state = model.update([0, 0, 0], [0, 0, 0], 16.0)
    if zero_state["direction_valid"]:
        raise RuntimeError("zero-distance direction guard failed")
    print("PAPER_ELASTIC3_OK", target0.tolist(), target1.tolist(), state1)


if __name__ == "__main__":
    main()
