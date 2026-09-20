#!/usr/bin/env python3
import argparse
import csv
import hashlib
import json
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path


WEIGHTS = ("Wt", "Wu", "WF")
PERTURBATION_KEYS = (
    "trajectory_id", "planner_seed", "seed", "reference_left_sha256",
    "reference_right_sha256", "sampled_object_stiffness_n_m",
    "sampled_object_damping_n_s_m", "sampled_force_delay_cycles",
    "sampled_peer_delay_cycles", "sampled_actuation_delay_cycles",
)
METRICS = (
    "terminal_target_error_mm", "q_rmse_rad", "relative_position_rmse_m",
    "orientation_rmse_deg", "velocity_rmse_rad_s", "force_rmse_n",
    "control_effort", "common_eval_cost", "mean_solve_ms", "p95_solve_ms",
    "force_min_n", "force_max_n",
)


def average(rows, key):
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    return statistics.fmean(values) if values else float("nan")


def percent_change(value, baseline):
    if baseline == 0:
        return 0.0 if value == 0 else float("nan")
    return 100.0 * (value / baseline - 1.0)


def load_collision_map(path):
    data = json.load(open(path, encoding="utf-8"))
    mapped = {}
    for row in data["results"]:
        match = re.search(r"[/\\]v(\d+)[/\\]([^/\\]+)_joint_trace\.csv$",
                          str(row["trace"]))
        if not match:
            raise RuntimeError("Cannot parse collision trace: %s" % row["trace"])
        mapped[(int(match.group(1)), match.group(2))] = bool(row["collision_free"])
    return mapped


def perturbation_signature(row):
    return tuple(row[key] for key in PERTURBATION_KEYS)


def stable_hash(value):
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True,
                         separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--base-weights-json", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base_weights = json.loads(args.base_weights_json)
    if float(base_weights.get("Wd", -1)) != 0.0:
        raise RuntimeError("Stage 3 report requires Wd=0")

    records = []
    signatures = defaultdict(dict)
    provenances = {}
    for weight in WEIGHTS:
        result = json.load(open(args.root / weight / "result.json", encoding="utf-8"))
        collision = load_collision_map(args.root / weight / "collision_audit.json")
        provenances[weight] = result["provenance"]
        grouped = defaultdict(list)
        for source in result["raw_runs"]:
            row = dict(source)
            index = int(row["value_index"])
            trajectory = row["trajectory_id"]
            row["collision_free"] = collision[(index, trajectory)]
            row["combined_success"] = bool(row["task_success"] and row["collision_free"])
            grouped[index].append(row)
            signatures[(weight, index)][trajectory] = perturbation_signature(row)
        if sorted(grouped) != [0, 1, 2]:
            raise RuntimeError("%s does not contain x0/x1/x2" % weight)
        for index, setting in enumerate(("x0", "x1", "x2")):
            rows = sorted(grouped[index], key=lambda item: item["trajectory_id"])
            reasons = Counter()
            for row in rows:
                if row.get("hard_failure_reasons"):
                    reasons.update(row["hard_failure_reasons"].split(";"))
                if not row["collision_free"]:
                    reasons["collision"] += 1
            record = {
                "weight": weight,
                "setting": setting,
                "weight_value": float(rows[0]["weight_value"]),
                "trials": len(rows),
                "controller_successes": sum(bool(row["task_success"]) for row in rows),
                "collision_free_trials": sum(bool(row["collision_free"]) for row in rows),
                "combined_successes": sum(bool(row["combined_success"]) for row in rows),
                "failure_reasons": dict(reasons),
            }
            for metric in METRICS:
                record[metric] = average(rows, metric)
            records.append(record)

    canonical = signatures[("Wt", 1)]
    mismatches = []
    for (weight, index), samples in signatures.items():
        for trajectory, signature in samples.items():
            if signature != canonical[trajectory]:
                mismatches.append({"weight": weight, "value_index": index,
                                   "trajectory_id": trajectory})
    manifest_samples = [dict(zip(PERTURBATION_KEYS, canonical[key]))
                        for key in sorted(canonical)]
    manifest = {
        "schema": "cm_dnmpc_stage3_wd0_fixed_20_perturbations_v1",
        "canonical_source": "Wt/x1",
        "keys": list(PERTURBATION_KEYS),
        "samples": manifest_samples,
        "comparison_setting_count": len(signatures),
        "mismatch_count": len(mismatches),
        "mismatches": mismatches,
        "sha256": stable_hash(manifest_samples),
    }
    (args.output / "fixed_perturbation_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    indexed = {(row["weight"], row["setting"]): row for row in records}
    for weight in WEIGHTS:
        baseline = indexed[(weight, "x1")]
        for setting in ("x0", "x1", "x2"):
            row = indexed[(weight, setting)]
            row["success_rate_pct"] = 100.0 * row["combined_successes"] / row["trials"]
            for metric in METRICS:
                row[metric + "_change_pct"] = percent_change(row[metric], baseline[metric])

    no_hidden_scale = all(
        all(abs(float(value) - 1.0) < 1e-12
            for value in provenance.get("weight_error_scales", {}).values())
        for provenance in provenances.values())
    summary = {
        "schema": "cm_dnmpc_stage3_wd0_wt_wu_wf_x0x1x2_v1",
        "root": str(args.root),
        "base_weights": base_weights,
        "no_hidden_weight_scale": no_hidden_scale,
        "drop_definition": {
            "slip_onset_force_n": 10.0,
            "multiplier": 1.5,
            "drop_threshold_n": 15.0,
            "comparison": "true_force < threshold",
        },
        "perturbation_manifest_sha256": manifest["sha256"],
        "perturbation_mismatch_count": len(mismatches),
        "records": records,
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    fields = ["weight", "setting", "weight_value", "trials",
              "controller_successes", "collision_free_trials",
              "combined_successes", "success_rate_pct", "failure_reasons"]
    for metric in METRICS:
        fields.extend((metric, metric + "_change_pct"))
    with open(args.output / "summary.csv", "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for record in records:
            output = dict(record)
            output["failure_reasons"] = json.dumps(output["failure_reasons"],
                                                    ensure_ascii=False)
            writer.writerow(output)

    lines = [
        "# Ball Stage 3：Wd=0 后 Wt/Wu/WF 裸权重边界重测报告",
        "",
        "## 结论与口径",
        "",
        "- Stage 3 固定 `Wd=0`，不再扫描 Wd。",
        "- x1 裸权重：`%s`。" % json.dumps(base_weights, ensure_ascii=False),
        "- 隐藏权重/误差放缩：`%s`。" % ("无" if no_hidden_scale else "存在，需复核"),
        "- 20条轨迹及全部扰动逐条不一致数：`%d`；清单SHA-256：`%s`。" %
          (len(mismatches), manifest["sha256"]),
        "- 掉落判据：任一周期真实夹持力低于 `1.5×10=15 N`。",
        "- FCL只用于结果复核，不进入DNMPC目标函数。",
        "",
        "## 完整结果",
        "",
        "|权重|设置|裸值|综合成功率|控制通过|无碰撞|终端误差mm|相对位置RMSE mm|方向RMSE deg|力RMSE N|速度RMSE rad/s|命令代价|固定Ei|失败原因|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in records:
        lines.append(
            "|{weight}|{setting}|{value:g}|{rate:.1f}%|{control}/20|{collision}/20|"
            "{terminal:.4f}|{relative:.4f}|{orientation:.4f}|{force:.4f}|{velocity:.5f}|"
            "{command:.5f}|{energy:.6f}|{reasons}|".format(
                weight=row["weight"], setting=row["setting"], value=row["weight_value"],
                rate=row["success_rate_pct"], control=row["controller_successes"],
                collision=row["collision_free_trials"],
                terminal=row["terminal_target_error_mm"],
                relative=1000.0 * row["relative_position_rmse_m"],
                orientation=row["orientation_rmse_deg"], force=row["force_rmse_n"],
                velocity=row["velocity_rmse_rad_s"], command=row["control_effort"],
                energy=row["common_eval_cost"],
                reasons=json.dumps(row["failure_reasons"], ensure_ascii=False)))
    lines.extend([
        "", "## 相对x1变化", "",
        "|权重|设置|成功率|终端误差|关节RMSE|相对位置|方向|速度|力|命令代价|固定Ei|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for row in records:
        lines.append(
            "|{weight}|{setting}|{rate:.1f}%|{terminal:+.2f}%|{q:+.2f}%|{relative:+.2f}%|"
            "{orientation:+.2f}%|{velocity:+.2f}%|{force:+.2f}%|{command:+.2f}%|"
            "{energy:+.2f}%|".format(
                weight=row["weight"], setting=row["setting"], rate=row["success_rate_pct"],
                terminal=row["terminal_target_error_mm_change_pct"],
                q=row["q_rmse_rad_change_pct"],
                relative=row["relative_position_rmse_m_change_pct"],
                orientation=row["orientation_rmse_deg_change_pct"],
                velocity=row["velocity_rmse_rad_s_change_pct"],
                force=row["force_rmse_n_change_pct"],
                command=row["control_effort_change_pct"],
                energy=row["common_eval_cost_change_pct"]))
    lines.extend([
        "", "## 可复现性文件", "",
        "- `summary.csv`、`summary.json`：完整汇总。",
        "- `fixed_perturbation_manifest.json`：固定20轨迹与扰动清单。",
        "- `Wt/Wu/WF/result.json`：逐次控制结果。",
        "- `traces_Wt/traces_Wu/traces_WF` 与各组 `collision_audit.json`：逐周期数据及碰撞复核。",
    ])
    (args.output / "Ball_Stage3_Wd0_Wt_Wu_WF边界重测中文报告.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
