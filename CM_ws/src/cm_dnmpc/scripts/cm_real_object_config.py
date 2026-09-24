#!/usr/bin/env python3
"""Load and validate YAML profiles for CM real-object experiments."""

import argparse
import copy
import json
import math
from pathlib import Path

import yaml


OBJECT_IDS = ("ball", "box", "softbox", "basketball", "l", "cloth", "mango")
OBJECT_ARGUMENTS = OBJECT_IDS + ("L",)
WEIGHT_NAMES = ("Wd", "Wt", "Wo", "Wu", "Wq", "Wv", "WF")


def _finite_nonnegative(value, label, positive=False):
    result = float(value)
    if not math.isfinite(result) or result < 0.0 or (positive and result <= 0.0):
        relation = "positive" if positive else "finite and nonnegative"
        raise ValueError("{} must be {}".format(label, relation))
    return result


def _validate_weights(weights, label):
    if not isinstance(weights, dict):
        raise ValueError("{} must be a mapping".format(label))
    missing = set(WEIGHT_NAMES) - set(weights)
    unknown = set(weights) - set(WEIGHT_NAMES)
    if missing or unknown:
        raise ValueError(
            "{} weight keys mismatch: missing={} unknown={}".format(
                label, sorted(missing), sorted(unknown)))
    return {
        name: _finite_nonnegative(weights[name], "{}.{}".format(label, name))
        for name in WEIGHT_NAMES
    }


def _validate_elastic(values, label):
    required = (
        "k0", "wb", "wk", "b0", "desired_force_n",
        "object_length_m", "max_displacement_m")
    missing = set(required) - set(values)
    if missing:
        raise ValueError("{} is missing {}".format(label, sorted(missing)))
    result = {}
    for name in required:
        result[name] = _finite_nonnegative(
            values[name], "{}.{}".format(label, name),
            positive=name in (
                "k0", "desired_force_n", "object_length_m",
                "max_displacement_m"))
    return result


def _validate_transition(values, label):
    required = (
        "desired_force_n", "force_tolerance_n", "sim_stiffness_n_m",
        "endpoint_tolerance_m", "stable_cycles", "max_duration_s")
    missing = set(required) - set(values)
    if missing:
        raise ValueError("{} is missing {}".format(label, sorted(missing)))
    result = {}
    for name in required:
        if name == "stable_cycles":
            result[name] = int(values[name])
            if result[name] <= 0:
                raise ValueError("{}.stable_cycles must be positive".format(label))
        else:
            result[name] = _finite_nonnegative(
                values[name], "{}.{}".format(label, name), positive=True)
    return result


def load_real_object_profile(path, expected_object=None):
    path = Path(path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        raise ValueError("profile must contain a YAML mapping: {}".format(path))
    if int(profile.get("schema_version", 0)) != 1:
        raise ValueError("unsupported profile schema in {}".format(path))
    object_values = profile.get("object", {})
    object_id = str(object_values.get("id", "")).lower()
    if object_id not in OBJECT_IDS:
        raise ValueError("unsupported object id {!r} in {}".format(object_id, path))
    if expected_object and object_id != str(expected_object).lower():
        raise ValueError(
            "profile object {!r} does not match --object {!r}".format(
                object_id, expected_object))

    controller = profile.get("controller", {})
    controller["control_period_s"] = _finite_nonnegative(
        controller.get("control_period_s"), "controller.control_period_s",
        positive=True)
    controller["horizon"] = int(controller.get("horizon", 0))
    if controller["horizon"] <= 0:
        raise ValueError("controller.horizon must be positive")

    reference = profile.get("reference", {})
    reference["source_dt_s"] = _finite_nonnegative(
        reference.get("source_dt_s"), "reference.source_dt_s", positive=True)
    strides = reference.get("stride", {})
    for phase in ("catch", "move"):
        strides[phase] = int(strides.get(phase, 0))
        if strides[phase] <= 0:
            raise ValueError("reference.stride.{} must be positive".format(phase))
    paths = reference.setdefault("paths", {})
    for phase in ("catch", "move"):
        phase_paths = paths.setdefault(phase, {})
        for side in ("left", "right"):
            value = phase_paths.get(side)
            if value is not None and not str(value).strip():
                value = None
            phase_paths[side] = value

    elastic = profile.get("paper_elastic", {})
    for phase in ("catch", "move"):
        elastic[phase] = _validate_elastic(
            elastic.get(phase, {}), "paper_elastic.{}".format(phase))

    weights = profile.get("stage_weights", {})
    normalized_weights = {}
    for stage_id in (1, 2, 3, 4):
        stage_key = str(stage_id)
        normalized_weights[stage_key] = _validate_weights(
            weights.get(stage_key, weights.get(stage_id, {})),
            "stage_weights.{}".format(stage_key))

    transitions = profile.get("stage_transition", {})
    normalized_transitions = {}
    for stage_id in (2, 4):
        stage_key = str(stage_id)
        normalized_transitions[stage_key] = _validate_transition(
            transitions.get(stage_key, transitions.get(stage_id, {})),
            "stage_transition.{}".format(stage_key))

    real_io = profile.get("real_io", {})
    for name in ("joint_state_topic", "command_topic", "force_topic"):
        if name not in real_io or not str(real_io[name]).strip():
            raise ValueError("real_io.{} is required".format(name))
    if real_io.get("force_type") not in ("robotiq", "wrench"):
        raise ValueError("real_io.force_type must be robotiq or wrench")

    profile["object"]["id"] = object_id
    profile["controller"] = controller
    profile["reference"] = reference
    profile["paper_elastic"] = elastic
    profile["stage_weights"] = normalized_weights
    profile["stage_transition"] = normalized_transitions
    profile["_profile_path"] = str(path)
    return profile


def apply_real_object_profile(base_config, profile):
    """Return a controller config overlaid by one validated object profile."""
    config = copy.deepcopy(base_config)
    object_id = profile["object"]["id"]
    config["Ts"] = profile["controller"]["control_period_s"]
    config["horizon"] = profile["controller"]["horizon"]
    config["source_dt"] = profile["reference"]["source_dt_s"]
    config.setdefault("reference_stride", {})[object_id] = dict(
        profile["reference"]["stride"])
    config.setdefault("paper_elastic", {})[object_id] = copy.deepcopy(
        profile["paper_elastic"])
    config.setdefault("stage_weights", {})[object_id] = copy.deepcopy(
        profile["stage_weights"])
    config.setdefault("stage2", {})[object_id] = copy.deepcopy(
        profile["stage_transition"]["2"])
    config.setdefault("stage4", {})[object_id] = copy.deepcopy(
        profile["stage_transition"]["4"])
    return config


def profile_reference(profile, phase, side):
    value = profile["reference"]["paths"][phase][side]
    if value is None:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(profile["_profile_path"]).parent / path
    return str(path.resolve())


def format_side_value(template, side):
    return str(template).format(side=str(side))


def joint_position_velocity_command(q, velocity, dt):
    """Build one finite position/velocity command without hiding qdot."""
    q = [float(value) for value in q]
    velocity = [float(value) for value in velocity]
    dt = _finite_nonnegative(dt, "command dt", positive=True)
    if len(q) != 6 or len(velocity) != 6:
        raise ValueError("joint command requires six positions and velocities")
    if not all(math.isfinite(value) for value in q + velocity):
        raise ValueError("joint command values must be finite")
    return [position + dt * speed for position, speed in zip(q, velocity)], velocity


def default_profile_path(dnmpc_root, object_id):
    return Path(dnmpc_root) / "config" / "real_objects" / (
        "{}.yaml".format(str(object_id).lower()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile")
    parser.add_argument("--object")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    profile = load_real_object_profile(args.profile, args.object)
    summary = {
        "profile": profile["_profile_path"],
        "object": profile["object"]["id"],
        "hardware_validated": bool(profile.get("status", {}).get(
            "hardware_validated", False)),
        "references_ready": all(
            profile_reference(profile, phase, side)
            for phase in ("catch", "move")
            for side in ("left", "right")),
        "stage_weights": profile["stage_weights"],
    }
    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print("valid profile: {}".format(summary))


if __name__ == "__main__":
    main()
