#!/usr/bin/env python3
"""Build a fixed, collision-checked 20-trajectory RRT-only DNMPC pool."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from build_collision_free_dnmpc_pool import CollisionChecker, resample_rrt


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, required=True)
    parser.add_argument("--replacement-id", required=True)
    parser.add_argument("--replacement-left", type=Path, required=True)
    parser.add_argument("--replacement-right", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--package-dir", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ball-radius-m", type=float, default=0.09)
    parser.add_argument("--continuous-substeps", type=int, default=20)
    parser.add_argument("--velocity-limit-rad-s", type=float, default=3.141592653589793)
    parser.add_argument("--acceleration-limit-rad-s2", type=float, default=3.0)
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit("output already exists: {}".format(args.output))
    if (args.velocity_limit_rad_s <= 0.0 or
            args.acceleration_limit_rad_s2 <= 0.0):
        raise SystemExit("dynamic limits must be positive")
    args.output.mkdir(parents=True)
    source = json.loads(args.input_manifest.read_text(encoding="utf-8"))
    checker = CollisionChecker(
        args.urdf, args.package_dir, args.ball_radius_m)
    output_items = []
    audit = []
    hashes = set()

    for item in source["trajectories"]:
        trajectory_id = item["trajectory_id"]
        planned_left = Path(item["planned_left"])
        planned_right = Path(item["planned_right"])
        replaced = trajectory_id == args.replacement_id
        if replaced:
            planned_left = args.replacement_left
            planned_right = args.replacement_right

        left, right = resample_rrt(
            np.loadtxt(planned_left), np.loadtxt(planned_right),
            int(item["reference_rows"]))
        collision = checker.trajectory_collision(
            left, right, args.continuous_substeps)
        if collision is not None:
            raise RuntimeError(
                "{} is not collision-free: {}".format(
                    trajectory_id, collision))

        case_dir = args.output / "trajectories" / trajectory_id
        case_dir.mkdir(parents=True)
        left_path = case_dir / "move_left_rrtonly_reference.txt"
        right_path = case_dir / "move_right_rrtonly_reference.txt"
        times_path = case_dir / "move_sample_times.txt"
        np.savetxt(left_path, left, fmt="%.12f")
        np.savetxt(right_path, right, fmt="%.12f")
        nominal_times = np.linspace(
            0.0, float(item["duration_s"]), len(left))
        dual = np.hstack((left, right))
        velocity = np.gradient(dual, nominal_times, axis=0)
        acceleration = np.gradient(velocity, nominal_times, axis=0)
        velocity_ratio = (
            float(np.max(np.abs(velocity))) /
            args.velocity_limit_rad_s)
        acceleration_ratio = (
            float(np.max(np.abs(acceleration))) /
            args.acceleration_limit_rad_s2)
        time_scale = max(1.0, velocity_ratio, acceleration_ratio ** 0.5)
        sample_times = nominal_times * time_scale
        np.savetxt(times_path, sample_times, fmt="%.12f")
        pair_hash = (digest(left_path), digest(right_path))
        if pair_hash in hashes:
            raise RuntimeError("duplicate reference pair: " + trajectory_id)
        hashes.add(pair_hash)

        result = dict(item)
        result.update({
            "source": ("existing_rrt_replacement" if replaced
                       else "original_rrt_planning_run"),
            "reference_kind": "RRTONLY",
            "planned_left": str(planned_left.resolve()),
            "planned_right": str(planned_right.resolve()),
            "reference_left": str(left_path.resolve()),
            "reference_right": str(right_path.resolve()),
            "sample_times": str(times_path.resolve()),
            "reference_left_sha256": pair_hash[0],
            "reference_right_sha256": pair_hash[1],
            "sample_times_sha256": digest(times_path),
            "reference_rows": int(len(left)),
            "duration_s": float(sample_times[-1]),
            "source_duration_s": float(item["duration_s"]),
            "dynamic_time_scale": float(time_scale),
            "time_parameterization": (
                "minimum uniform dilation satisfying the declared joint "
                "velocity and acceleration limits"),
            "pre_dnmpc_collision_free": True,
            "continuous_collision_substeps": args.continuous_substeps,
        })
        output_items.append(result)
        audit.append({
            "trajectory_id": trajectory_id,
            "replacement": replaced,
            "planned_left": str(planned_left.resolve()),
            "planned_right": str(planned_right.resolve()),
            "collision_free": True,
        })

    manifest = dict(source)
    manifest.update({
        "schema": "CM_DNMPC_STRICT_RRTONLY_REFERENCE_POOL_V1",
        "source_manifest": str(args.input_manifest.resolve()),
        "source_manifest_sha256": digest(args.input_manifest),
        "selection_rule": (
            "Use the original native RRT paths. Replace only the one path "
            "invalidated by the updated obstacle geometry with an existing "
            "distinct native RRT output having the same endpoints."),
        "replacement_id": args.replacement_id,
        "replacement_left": str(args.replacement_left.resolve()),
        "replacement_right": str(args.replacement_right.resolve()),
        "collision_geometry": str(args.urdf.resolve()),
        "ball_radius_m": args.ball_radius_m,
        "continuous_substeps": args.continuous_substeps,
        "velocity_limit_rad_s": args.velocity_limit_rad_s,
        "acceleration_limit_rad_s2": args.acceleration_limit_rad_s2,
        "trajectory_count": len(output_items),
        "unique_reference_pair_count": len(hashes),
        "trajectories": output_items,
    })
    manifest_path = args.output / "trajectory_pool_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    (args.output / "pre_dnmpc_collision_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "trajectories": len(output_items),
        "unique_reference_pair_count": len(hashes),
        "replacement_id": args.replacement_id,
        "manifest": str(manifest_path.resolve()),
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
