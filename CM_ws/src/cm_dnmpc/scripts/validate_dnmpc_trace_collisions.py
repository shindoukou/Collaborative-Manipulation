#!/usr/bin/env python3
"""Post-validate DNMPC joint traces against the Gazebo collision geometry."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import hppfcl
import numpy as np
import pinocchio as pin


EXPECTED_ADJACENT = {
    frozenset(pair) for pair in (
        ("left_base_link_0", "left_shoulder_link_0"),
        ("left_shoulder_link_0", "left_upper_arm_link_0"),
        ("left_upper_arm_link_0", "left_forearm_link_0"),
        ("left_forearm_link_0", "left_wrist_1_link_0"),
        ("left_wrist_1_link_0", "left_wrist_2_link_0"),
        ("left_wrist_2_link_0", "left_wrist_3_link_0"),
        ("right_base_link_0", "right_shoulder_link_0"),
        ("right_shoulder_link_0", "right_upper_arm_link_0"),
        ("right_upper_arm_link_0", "right_forearm_link_0"),
        ("right_forearm_link_0", "right_wrist_1_link_0"),
        ("right_wrist_1_link_0", "right_wrist_2_link_0"),
        ("right_wrist_2_link_0", "right_wrist_3_link_0"),
    )
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace-root", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--package-dir", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ball-radius-m", type=float, default=0.09)
    return parser.parse_args()


def collision_name_pair(geometry_model, pair):
    return (
        geometry_model.geometryObjects[pair.first].name,
        geometry_model.geometryObjects[pair.second].name,
    )


def ignored_pair(first, second):
    return frozenset((first, second)) in EXPECTED_ADJACENT


def main():
    args = parse_args()
    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    geometry_model = pin.buildGeomFromUrdf(
        model, str(args.urdf), pin.GeometryType.COLLISION,
        package_dirs=[str(path) for path in args.package_dir])
    geometry_model.addAllCollisionPairs()
    geometry_data = pin.GeometryData(geometry_model)
    ball = hppfcl.Sphere(args.ball_radius_m)
    collision_request = hppfcl.CollisionRequest()
    obstacle_ids = [
        index for index, item in enumerate(geometry_model.geometryObjects)
        if item.name.startswith("obs_")]

    summaries = []
    for trace_path in sorted(args.trace_root.rglob("*_joint_trace.csv")):
        rows = list(csv.DictReader(trace_path.open(encoding="utf-8-sig")))
        first_collision = None
        collision_frames = 0
        minimum_checked_cycle = None
        for row in rows:
            q = np.array([
                float(row["{}_q{}".format(side, joint)])
                for side in ("left", "right")
                for joint in range(1, 7)
            ])
            pin.forwardKinematics(model, data, q)
            pin.updateGeometryPlacements(
                model, data, geometry_model, geometry_data, q)
            frame_hits = []
            for pair_index, pair in enumerate(geometry_model.collisionPairs):
                first, second = collision_name_pair(geometry_model, pair)
                if ignored_pair(first, second):
                    continue
                geometry_data.collisionResults[pair_index].clear()
                if pin.computeCollision(
                        geometry_model, geometry_data, pair_index):
                    distance = pin.computeDistance(
                        geometry_model, geometry_data, pair_index)
                    if distance.min_distance <= 0.0:
                        frame_hits.append("{} && {}".format(first, second))

            midpoint = np.array([
                float(row["ball_{}_m".format(axis)]) for axis in "xyz"])
            ball_transform = hppfcl.Transform3f(np.eye(3), midpoint)
            for obstacle_id in obstacle_ids:
                obstacle = geometry_model.geometryObjects[obstacle_id]
                placement = geometry_data.oMg[obstacle_id]
                result = hppfcl.CollisionResult()
                hit = hppfcl.collide(
                    ball, ball_transform, obstacle.geometry,
                    hppfcl.Transform3f(
                        placement.rotation, placement.translation),
                    collision_request, result)
                if hit:
                    frame_hits.append(
                        "carried_ball && {}".format(obstacle.name))

            if frame_hits:
                collision_frames += 1
                cycle = int(row["cycle"])
                if minimum_checked_cycle is None:
                    minimum_checked_cycle = cycle
                if first_collision is None:
                    first_collision = {
                        "cycle": cycle,
                        "sim_time_s": float(row["sim_time_s"]),
                        "pairs": sorted(set(frame_hits)),
                    }

        summaries.append({
            "trace": str(trace_path.resolve()),
            "collision_free": first_collision is None,
            "collision_frames": collision_frames,
            "first_collision": first_collision,
        })

    args.output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "CM_DNMPC_GAZEBO_GEOMETRY_COLLISION_AUDIT_V1",
        "urdf": str(args.urdf.resolve()),
        "ball_radius_m": args.ball_radius_m,
        "trace_count": len(summaries),
        "collision_free_count": sum(
            int(item["collision_free"]) for item in summaries),
        "collision_count": sum(
            int(not item["collision_free"]) for item in summaries),
        "results": summaries,
    }
    args.output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "trace_count": payload["trace_count"],
        "collision_free_count": payload["collision_free_count"],
        "collision_count": payload["collision_count"],
    }, ensure_ascii=False))


if __name__ == "__main__":
    main()
