#!/usr/bin/env python3
import argparse
import csv
import hashlib
import json
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path


WEIGHTS = ("Wd", "Wt", "Wo", "Wu", "Wq", "Wv", "WF")
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
INTENDED = {
    "Wd": "terminal_target_error_mm",
    "Wt": "relative_position_rmse_m",
    "Wo": "orientation_rmse_deg",
    "Wu": "control_effort",
    "Wq": "q_rmse_rad",
    "Wv": "velocity_rmse_rad_s",
    "WF": "force_rmse_n",
}
CN = {
    "terminal_target_error_mm": "终端位置误差(mm)",
    "q_rmse_rad": "关节RMSE(rad)",
    "relative_position_rmse_m": "相对位置RMSE(m)",
    "orientation_rmse_deg": "方向RMSE(deg)",
    "velocity_rmse_rad_s": "关节速度RMSE(rad/s)",
    "force_rmse_n": "力RMSE(N)",
    "control_effort": "命令代价",
    "common_eval_cost": "统一评价代价Ei",
    "mean_solve_ms": "平均求解时间(ms)",
    "p95_solve_ms": "P95求解时间(ms)",
    "force_min_n": "最低力(N)",
    "force_max_n": "峰值力(N)",
}


def mean(rows, key):
    values = [float(r[key]) for r in rows if r.get(key) is not None]
    return statistics.fmean(values) if values else float("nan")


def pct(value, base):
    if base == 0:
        return 0.0 if value == 0 else float("nan")
    return 100.0 * (value / base - 1.0)


def collision_map(path):
    data = json.load(open(path, encoding="utf-8"))
    mapped = {}
    for row in data["results"]:
        trace = Path(row["trace"])
        match = re.search(r"[/\\]v(\d+)[/\\]([^/\\]+)_joint_trace\.csv$", str(trace))
        if not match:
            raise RuntimeError("Cannot parse collision trace: %s" % trace)
        mapped[(int(match.group(1)), match.group(2))] = bool(row["collision_free"])
    return mapped, data


def signature(row):
    return tuple(row[key] for key in PERTURBATION_KEYS)


def json_hash(value):
    raw = json.dumps(value, sort_keys=True, ensure_ascii=False,
                     separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--base-weights-json", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base_weights = json.loads(args.base_weights_json)

    records = []
    signatures = defaultdict(dict)
    provenance = {}
    for weight in WEIGHTS:
        result = json.load(open(args.root / weight / "result.json", encoding="utf-8"))
        cmap, caudit = collision_map(args.root / weight / "collision_audit.json")
        provenance[weight] = result["provenance"]
        grouped = defaultdict(list)
        for row in result["raw_runs"]:
            value_index = int(row["value_index"])
            trajectory = row["trajectory_id"]
            collision_free = cmap[(value_index, trajectory)]
            row = dict(row)
            row["collision_free"] = collision_free
            row["combined_success"] = bool(row["task_success"] and collision_free)
            grouped[value_index].append(row)
            signatures[(weight, value_index)][trajectory] = signature(row)
        if sorted(grouped) != [0, 1, 2]:
            raise RuntimeError("%s does not contain x0/x1/x2" % weight)
        for value_index in (0, 1, 2):
            rows = sorted(grouped[value_index], key=lambda r: r["trajectory_id"])
            reason_counts = Counter()
            for row in rows:
                if row.get("hard_failure_reasons"):
                    reason_counts.update(row["hard_failure_reasons"].split(";"))
                if not row["collision_free"]:
                    reason_counts["collision"] += 1
            record = {
                "weight": weight,
                "setting": ("x0", "x1", "x2")[value_index],
                "weight_value": rows[0]["weight_value"],
                "trials": len(rows),
                "controller_successes": sum(bool(r["task_success"]) for r in rows),
                "collision_free_trials": sum(bool(r["collision_free"]) for r in rows),
                "combined_successes": sum(bool(r["combined_success"]) for r in rows),
                "failure_reasons": dict(reason_counts),
            }
            for metric in METRICS:
                record[metric] = mean(rows, metric)
            records.append(record)

    canonical = signatures[("Wd", 1)]
    mismatch_rows = []
    for key, by_trajectory in signatures.items():
        for trajectory, sig in by_trajectory.items():
            if sig != canonical[trajectory]:
                mismatch_rows.append({"weight": key[0], "value_index": key[1],
                                      "trajectory_id": trajectory})
    perturbation_manifest = {
        "schema": "cm_dnmpc_fixed_20_perturbations_v1",
        "canonical_source": "Wd/x1",
        "keys": list(PERTURBATION_KEYS),
        "samples": [dict(zip(PERTURBATION_KEYS, canonical[key]))
                    for key in sorted(canonical)],
        "comparison_setting_count": len(signatures),
        "mismatch_count": len(mismatch_rows),
        "mismatches": mismatch_rows,
    }
    perturbation_manifest["sha256"] = json_hash(perturbation_manifest["samples"])
    (args.output / "fixed_perturbation_manifest.json").write_text(
        json.dumps(perturbation_manifest, indent=2, ensure_ascii=False),
        encoding="utf-8")

    by_key = {(r["weight"], r["setting"]): r for r in records}
    for weight in WEIGHTS:
        baseline = by_key[(weight, "x1")]
        for setting in ("x0", "x1", "x2"):
            row = by_key[(weight, setting)]
            row["success_rate_pct"] = 100.0 * row["combined_successes"] / row["trials"]
            for metric in METRICS:
                row[metric + "_change_pct"] = pct(row[metric], baseline[metric])

    summary = {
        "schema": "cm_dnmpc_raw_x0x1x2_strict_summary_v1",
        "root": str(args.root),
        "base_weights": base_weights,
        "no_hidden_weight_scale": all(
            all(abs(float(v) - 1.0) < 1e-12
                for v in p.get("weight_error_scales", {}).values())
            for p in provenance.values()),
        "drop_definition": {
            "slip_onset_force_n": 10.0,
            "multiplier": 1.5,
            "drop_threshold_n": 15.0,
            "comparison": "true_force < threshold",
        },
        "perturbation_manifest_sha256": perturbation_manifest["sha256"],
        "perturbation_mismatch_count": len(mismatch_rows),
        "records": records,
    }
    (args.output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    csv_fields = ["weight", "setting", "weight_value", "trials",
                  "controller_successes", "collision_free_trials",
                  "combined_successes", "success_rate_pct", "failure_reasons"]
    for metric in METRICS:
        csv_fields.extend((metric, metric + "_change_pct"))
    with open(args.output / "summary.csv", "w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=csv_fields, extrasaction="ignore")
        writer.writeheader()
        for row in records:
            out = dict(row)
            out["failure_reasons"] = json.dumps(out["failure_reasons"], ensure_ascii=False)
            writer.writerow(out)

    lines = [
        "# CM_ws DNMPC 裸权重 x0/x1/x2 严格边界实验报告",
        "",
        "## 实验结论",
        "",
        "- 所有组使用相同20条原生可行RRT轨迹及相同扰动，不筛选样本。",
        "- x1权重为：`%s`。" % json.dumps(base_weights, ensure_ascii=False),
        "- 代价函数直接使用上述裸权重；隐藏误差/权重放缩：`%s`。" %
          ("无" if summary["no_hidden_weight_scale"] else "存在，需复核"),
        "- 掉落定义：测得滑动起始力为10 N，任一受检周期真实夹持力低于"
        " `1.5×10=15 N` 即判失败。",
        "- 扰动逐条一致性不匹配数：`%d`；扰动清单SHA-256：`%s`。" %
          (len(mismatch_rows), perturbation_manifest["sha256"]),
        "- 方向RMSE作为质量指标报告，不作为额外的人为5°失败阈值；逐周期硬失败"
        "包括碰撞、EE间距、速度/加速度、危险力、力变化率、命令跳变、求解失败、"
        "终端位置超限和掉落。",
        "",
        "## 成功率与主要指标",
        "",
        "|权重|设置|裸值|综合成功率|控制通过|无碰撞|终端误差 mm|相对位置 RMSE mm|方向 RMSE deg|力 RMSE N|速度 RMSE rad/s|Ei|失败原因|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in records:
        lines.append(
            "|{weight}|{setting}|{value:g}|{success:.1f}%|{control}/20|{collision}/20|"
            "{terminal:.4f}|{relative:.4f}|{orientation:.4f}|{force:.4f}|{velocity:.5f}|"
            "{ei:.6g}|{reasons}|".format(
                weight=row["weight"], setting=row["setting"],
                value=float(row["weight_value"]), success=row["success_rate_pct"],
                control=row["controller_successes"], collision=row["collision_free_trials"],
                terminal=row["terminal_target_error_mm"],
                relative=1000.0 * row["relative_position_rmse_m"],
                orientation=row["orientation_rmse_deg"], force=row["force_rmse_n"],
                velocity=row["velocity_rmse_rad_s"], ei=row["common_eval_cost"],
                reasons=json.dumps(row["failure_reasons"], ensure_ascii=False)))

    lines.extend(["", "## 相对 x1 的变化", "",
                  "负值表示该误差或代价降低，正值表示增加。成功率为绝对值。", "",
                  "|权重|设置|成功率|关节RMSE变化|相对位置变化|方向变化|速度变化|力变化|命令代价变化|Ei变化|",
                  "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"])
    for row in records:
        lines.append(
            "|{weight}|{setting}|{success:.1f}%|{q:+.2f}%|{rel:+.2f}%|{ori:+.2f}%|"
            "{vel:+.2f}%|{force:+.2f}%|{cmd:+.2f}%|{ei:+.2f}%|".format(
                weight=row["weight"], setting=row["setting"],
                success=row["success_rate_pct"],
                q=row["q_rmse_rad_change_pct"],
                rel=row["relative_position_rmse_m_change_pct"],
                ori=row["orientation_rmse_deg_change_pct"],
                vel=row["velocity_rmse_rad_s_change_pct"],
                force=row["force_rmse_n_change_pct"],
                cmd=row["control_effort_change_pct"],
                ei=row["common_eval_cost_change_pct"]))

    lines.extend(["", "## 单项权重解释", ""])
    for weight in WEIGHTS:
        x0 = by_key[(weight, "x0")]
        x1 = by_key[(weight, "x1")]
        x2 = by_key[(weight, "x2")]
        intended = INTENDED[weight]
        x0_intended = x0[intended + "_change_pct"]
        x2_intended = x2[intended + "_change_pct"]
        penalty_metrics = [m for m in ("terminal_target_error_mm", "q_rmse_rad",
                          "relative_position_rmse_m", "orientation_rmse_deg",
                          "velocity_rmse_rad_s", "force_rmse_n", "control_effort",
                          "common_eval_cost") if m != intended]
        x2_penalty = max([x2[m + "_change_pct"] for m in penalty_metrics] + [0.0])
        lines.append(
            "- **{w}**：x0时目标指标“{metric}”变化 `{x0:+.2f}%`，成功率"
            " `{s0:.1f}%`；x2时该指标变化 `{x2:+.2f}%`，其他指标最大恶化"
            " `{penalty:+.2f}%`，成功率 `{s2:.1f}%`。".format(
                w=weight, metric=CN[intended], x0=x0_intended,
                s0=x0["success_rate_pct"], x2=x2_intended,
                penalty=x2_penalty, s2=x2["success_rate_pct"]))

    lines.extend([
        "", "## 可复现性文件", "",
        "- `summary.csv`：论文制表用的全部均值及相对变化。",
        "- `summary.json`：机器可读汇总和判定口径。",
        "- `fixed_perturbation_manifest.json`：20条轨迹逐条种子、哈希、刚度、阻尼和时延。",
        "- 各权重目录中的 `result.json`：每次实验的原始统计。",
        "- `traces_*` 与 `collision_audit.json`：逐周期轨迹和FCL碰撞复核。",
    ])
    (args.output / "DNMPC裸权重x0_x1_x2严格边界实验中文报告.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
