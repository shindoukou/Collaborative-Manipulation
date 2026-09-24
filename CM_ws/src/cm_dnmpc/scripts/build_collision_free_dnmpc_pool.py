#!/usr/bin/env python3
"""Freeze a DNMPC pool whose references are feasible before control starts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import hppfcl
import numpy as np
import pinocchio as pin

from cm_dnmpc_common import fk_function
from validate_dnmpc_trace_collisions import EXPECTED_ADJACENT


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-manifest", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--package-dir", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ball-radius-m", type=float, default=0.09)
    parser.add_argument("--continuous-substeps", type=int, default=4)
    return parser.parse_args()


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def smoothstep5(value):
    return value ** 3 * (10.0 + value * (-15.0 + 6.0 * value))


def resample_rrt(left, right, rows):
    left = np.atleast_2d(left)[:, :6]
    right = np.atleast_2d(right)[:, :6]
    count = min(len(left), len(right))
    left = left[:count]
    right = right[:count]
    dual = np.hstack((left, right))
    segment_length = np.linalg.norm(np.diff(dual, axis=0), axis=1)
    if not np.any(segment_length > 0.0):
        return np.repeat(left[:1], rows, axis=0), np.repeat(
            right[:1], rows, axis=0)
    cumulative = np.r_[0.0, np.cumsum(segment_length)]
    samples = np.linspace(0.0, cumulative[-1], rows)
    dense = np.empty((rows, 12))
    for index, distance in enumerate(samples):
        segment = min(
            int(np.searchsorted(cumulative, distance, side="right") - 1),
            count - 2)
        span = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if span <= 0.0 else (
            distance - cumulative[segment]) / span
        alpha = smoothstep5(float(np.clip(ratio, 0.0, 1.0)))
        dense[index] = (
            (1.0 - alpha) * dual[segment] + alpha * dual[segment + 1])
    return dense[:, :6], dense[:, 6:]


class CollisionChecker:
    def __init__(self, urdf, package_dirs, ball_radius):
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.geometry_model = pin.buildGeomFromUrdf(
            self.model, str(urdf), pin.GeometryType.COLLISION,
            package_dirs=[str(path) for path in package_dirs])
        self.geometry_model.addAllCollisionPairs()
        self.geometry_data = pin.GeometryData(self.geometry_model)
        self.ball = hppfcl.Sphere(ball_radius)
        self.request = hppfcl.CollisionRequest()
        self.obstacle_ids = [
            index for index, item in enumerate(
                self.geometry_model.geometryObjects)
            if item.name.startswith("obs_")]
        self.fk = {side: fk_function(side) for side in ("left", "right")}

    def state_collision(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(
            self.model, self.data, self.geometry_model,
            self.geometry_data, q)
        for index, pair in enumerate(self.geometry_model.collisionPairs):
            first = self.geometry_model.geometryObjects[pair.first].name
            second = self.geometry_model.geometryObjects[pair.second].name
            if frozenset((first, second)) in EXPECTED_ADJACENT:
                continue
            self.geometry_data.collisionResults[index].clear()
            if pin.computeCollision(
                    self.geometry_model, self.geometry_data, index):
                distance = pin.computeDistance(
                    self.geometry_model, self.geometry_data, index)
                if distance.min_distance <= 0.0:
                    return "{} && {}".format(first, second)

        midpoint = 0.5 * (
            np.asarray(self.fk["left"](q[:6])[0]).reshape(3)
            + np.asarray(self.fk["right"](q[6:])[0]).reshape(3))
        ball_transform = hppfcl.Transform3f(np.eye(3), midpoint)
        for index in self.obstacle_ids:
            obstacle = self.geometry_model.geometryObjects[index]
            placement = self.geometry_data.oMg[index]
            result = hppfcl.CollisionResult()
            if hppfcl.collide(
                    self.ball, ball_transform, obstacle.geometry,
                    hppfcl.Transform3f(
                        placement.rotation, placement.translation),
                    self.request, result):
                return "carried_ball && {}".format(obstacle.name)
        return None

    def trajectory_collision(self, left, right, substeps):
        left = np.atleast_2d(left)[:, :6]
        right = np.atleast_2d(right)[:, :6]
        dual = np.hstack((left, right))
        for row in range(len(dual)):
            stop = 1 if row + 1 == len(dual) else substeps
            for substep in range(stop):
                alpha = substep / float(substeps)
                q = dual[row] if row + 1 == len(dual) else (
                    (1.0 - alpha) * dual[row] + alpha * dual[row + 1])
                pair = self.state_collision(q)
                if pair:
                    return {
                        "row": row,
                        "substep": substep,
                        "pair": pair,
                    }
        return None


def main():
    args = parse_args()
    if args.output.exists():
        raise SystemExit("output already exists: {}".format(args.output))
    args.output.mkdir(parents=True)
    source = json.loads(args.input_manifest.read_text(encoding="utf-8"))
    checker = CollisionChecker(
        args.urdf, args.package_dir, args.ball_radius_m)
    output_trajectories = []
    audit = []

    for item in source["trajectories"]:
        reference_left = Path(item["reference_left"])
        reference_right = Path(item["reference_right"])
        left = np.loadtxt(reference_left)
        right = np.loadtxt(reference_right)
        original_collision = checker.trajectory_collision(
            left, right, args.continuous_substeps)
        result = dict(item)
        result["original_reference_collision"] = original_collision
        result["reference_kind"] = "KIRARA"

        if original_collision is not None:
            planned_left = np.loadtxt(item["planned_left"])
            planned_right = np.loadtxt(item["planned_right"])
            left, right = resample_rrt(
                planned_left, planned_right,
                int(item["reference_rows"]))
            case_dir = args.output / "trajectories" / item["trajectory_id"]
            case_dir.mkdir(parents=True)
            reference_left = case_dir / "move_left_rrtonly_reference.txt"
            reference_right = case_dir / "move_right_rrtonly_reference.txt"
            sample_times = case_dir / "move_sample_times.txt"
            np.savetxt(reference_left, left, fmt="%.12f")
            np.savetxt(reference_right, right, fmt="%.12f")
            np.savetxt(
                sample_times,
                np.linspace(0.0, float(item["duration_s"]), len(left)),
                fmt="%.12f")
            result.update({
                "reference_left": str(reference_left.resolve()),
                "reference_right": str(reference_right.resolve()),
                "sample_times": str(sample_times.resolve()),
                "reference_kind": "RRTONLY",
                "source": "rrtonly_collision_fallback",
            })

        selected_collision = checker.trajectory_collision(
            left, right, args.continuous_substeps)
        if selected_collision is not None:
            raise RuntimeError(
                "selected reference still collides for {}: {}".format(
                    item["trajectory_id"], selected_collision))
        result.update({
            "reference_left_sha256": sha256(reference_left),
            "reference_right_sha256": sha256(reference_right),
            "sample_times_sha256": sha256(result["sample_times"]),
            "reference_rows": int(len(left)),
            "pre_dnmpc_collision_free": True,
        })
        output_trajectories.append(result)
        audit.append({
            "trajectory_id": item["trajectory_id"],
            "original_collision": original_collision,
            "selected_reference_kind": result["reference_kind"],
            "selected_collision_free": True,
        })

    manifest = dict(source)
    manifest.update({
        "schema": "CM_DNMPC_COLLISION_FREE_REFERENCE_POOL_V1",
        "source_manifest": str(args.input_manifest.resolve()),
        "source_manifest_sha256": sha256(args.input_manifest),
        "selection_rule": (
            "Keep KIRARA when collision-free; otherwise use the same "
            "native feasible RRT path with C2 scalar time interpolation."),
        "collision_geometry": str(args.urdf.resolve()),
        "continuous_substeps": args.continuous_substeps,
        "unique_reference_pair_count": len(output_trajectories),
        "trajectories": output_trajectories,
    })
    manifest_path = args.output / "trajectory_pool_manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    (args.output / "pre_dnmpc_collision_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "trajectories": len(output_trajectories),
        "kirara": sum(
            item["reference_kind"] == "KIRARA"
            for item in output_trajectories),
        "rrtonly": sum(
            item["reference_kind"] == "RRTONLY"
            for item in output_trajectories),
        "manifest": str(manifest_path.resolve()),
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
