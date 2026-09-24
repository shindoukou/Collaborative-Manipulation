#!/usr/bin/env python3
"""Summarize a 20-trajectory Ball Move trajectory-optimization sweep."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


GROUPS = (
    ("path", r"$w_{\mathrm{p}i}$"),
    ("velocity", r"$w_{\mathrm{u}i}$"),
    ("kinematic_limits", r"$(w_{\mathrm{q}i},w_{\mathrm{v}i})$"),
    ("task_transport",
     r"$(w_{\mathrm{p}i,\mathrm{t}},w_{\mathrm{o}i,\mathrm{t}})$"),
    ("time", r"$w_{\mathrm{T}i}$"),
)
METRICS = (
    ("path_length_sum_rad", "trajectory_length_change_percent"),
    ("joint_velocity_cost_sum", "joint_velocity_cost_change_percent"),
    ("execution_time_s", "execution_time_change_percent"),
    ("optimization_time_s", "optimization_time_change_percent"),
    ("fixed_x1_total_objective", "Ei_change_percent"),
)


def mean(values):
    return float(statistics.fmean(float(value) for value in values))


def std(values):
    values = [float(value) for value in values]
    return float(statistics.stdev(values)) if len(values) > 1 else 0.0


def percent(value, baseline):
    if abs(baseline) <= 1.0e-12:
        return 0.0 if abs(value) <= 1.0e-12 else math.copysign(math.inf, value)
    return 100.0 * (value / baseline - 1.0)


def wilson(successes, total, z=1.959963984540054):
    if total <= 0:
        return (0.0, 0.0)
    p = float(successes) / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    half = z * math.sqrt(
        p * (1.0 - p) / total + z * z / (4.0 * total * total)) / denominator
    return (100.0 * max(0.0, center - half),
            100.0 * min(1.0, center + half))


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def setting_label(scale):
    return "x{}".format(int(round(float(scale))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    run_manifest = json.loads(
        (args.input / "run_manifest.json").read_text(encoding="utf-8"))
    if run_manifest.get("completed") is not True:
        raise RuntimeError("trajectory sweep is not marked complete")
    runs = list(run_manifest.get("runs") or [])
    expected = int(run_manifest["sample_count"])
    if expected != 20:
        raise RuntimeError("formal report requires exactly 20 trajectories")
    if len(runs) != expected:
        raise RuntimeError("incomplete fixed-input sweep manifest")

    raw = []
    x1_weights = None
    for run in runs:
        sample_id = run["sample_id"]
        path = Path(run["summary"])
        summary = json.loads(path.read_text(encoding="utf-8"))
        if int(summary["interior_optimization_samples_per_segment"]) != 1:
            raise RuntimeError(sample_id + " is not an internal1 sweep")
        if x1_weights is None:
            x1_weights = summary["x1_weights"]
        elif summary["x1_weights"] != x1_weights:
            raise RuntimeError("x1 weights differ across samples")
        rows = list(summary["x0_x1_x2"])
        if len(rows) != 3 * len(GROUPS):
            raise RuntimeError(sample_id + " does not contain all x0/x1/x2 rows")
        for row in rows:
            raw.append({
                "sample_id": sample_id,
                "group": row["group"],
                "setting": setting_label(row["scale"]),
                "scale": float(row["scale"]),
                "successful": bool(row["successful"]),
                "optimization_completed": bool(row["optimization_completed"]),
                "native_invalid_samples": int(row["native_invalid_samples"]),
                "path_length_sum_rad": float(row["path_length_sum_rad"]),
                "joint_velocity_cost_sum": float(
                    row["joint_velocity_cost_sum"]),
                "execution_time_s": float(row["execution_time_s"]),
                "optimization_time_s": float(row["optimization_time_s"]),
                "fixed_x1_total_objective": float(
                    row["fixed_x1_total_objective"]),
            })

    summary_rows = []
    for group, _ in GROUPS:
        grouped = {
            setting: [row for row in raw
                      if row["group"] == group and row["setting"] == setting]
            for setting in ("x0", "x1", "x2")
        }
        if any(len(rows) != expected for rows in grouped.values()):
            raise RuntimeError("incomplete group: " + group)
        baseline = {
            source: mean(row[source] for row in grouped["x1"])
            for source, _ in METRICS
        }
        for setting in ("x0", "x1", "x2"):
            rows = grouped[setting]
            successes = sum(int(row["successful"]) for row in rows)
            low, high = wilson(successes, expected)
            out = {
                "group": group,
                "setting": setting,
                "runs": expected,
                "successes": successes,
                "success_rate_percent": 100.0 * successes / expected,
                "success_wilson95_low_percent": low,
                "success_wilson95_high_percent": high,
                "failed_optimization_count": sum(
                    int(not row["optimization_completed"]) for row in rows),
                "failed_native_gate_count": sum(
                    int(row["native_invalid_samples"] > 0) for row in rows),
                "native_invalid_samples_total": sum(
                    row["native_invalid_samples"] for row in rows),
            }
            for source, target in METRICS:
                values = [row[source] for row in rows]
                out[source + "_mean"] = mean(values)
                out[source + "_std"] = std(values)
                out[target] = percent(out[source + "_mean"], baseline[source])
            summary_rows.append(out)

    write_csv(args.output / "transport_trajopt_raw_20x15.csv", raw)
    write_csv(args.output / "transport_trajopt_x0_x1_x2_summary.csv", summary_rows)
    merged = {
        "schema": "CM_BALL_MOVE_TRAJOPT_INTERNAL1_FIXED20_SUMMARY_V1",
        "input_manifest": str((args.input / "run_manifest.json").resolve()),
        "sample_count": expected,
        "interior_optimization_samples_per_segment": 1,
        "x1_weights": x1_weights,
        "summary": summary_rows,
    }
    (args.output / "transport_trajopt_x0_x1_x2_summary.json").write_text(
        json.dumps(merged, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")

    row_lookup = {
        (row["group"], row["setting"]): row for row in summary_rows}
    x1_energy = mean(
        row["fixed_x1_total_objective"] for row in raw
        if row["group"] == GROUPS[0][0] and row["setting"] == "x1")
    x1_energy_std = std(
        row["fixed_x1_total_objective"] for row in raw
        if row["group"] == GROUPS[0][0] and row["setting"] == "x1")
    baseline_rows = [
        row for row in raw
        if row["group"] == GROUPS[0][0] and row["setting"] == "x1"]
    baseline_absolute = {
        key: (mean(row[key] for row in baseline_rows),
              std(row[key] for row in baseline_rows))
        for key in (
            "path_length_sum_rad", "joint_velocity_cost_sum",
            "execution_time_s", "optimization_time_s")
    }
    planning_manifest_path = Path(run_manifest["inputs"][0]["left"]).parent.parent / (
        "fixed20_input_manifest.json")
    planning_manifest = json.loads(
        planning_manifest_path.read_text(encoding="utf-8"))
    sources = {}
    for sample in planning_manifest.get("samples") or []:
        sources[sample["source"]] = sources.get(sample["source"], 0) + 1
    old_count = sources.get("existing_demonstration", 0)
    new_count = sources.get("new_planning_run", 0)
    report = [
        "# Ball Transporting 轨迹优化内部点1固定20轨迹平扫中文报告",
        "",
        "## 1. 实验口径",
        "",
        "- Grasping 本次不重跑，继续使用交接包中的正式 Grasping 结果。",
        "- Transporting 使用20条预先冻结且彼此不同的 Ball Move 原始规划轨迹；旧演示轨迹{}条，新规划轨迹{}条。".format(
            old_count, new_count),
        "- 每条原始轨迹对全部 x0/x1/x2 设置成对复用，不按任何优化或控制结果替换样本。",
        "- 每段优化采样点共3个，即端点2个、内部优化点1个。",
        "- 碰撞检查只作为优化后的二元可行性门槛，不进入目标函数或梯度。",
        "- 固定总能量 Ei：所有候选均使用同一组 x1 完整目标函数重新评价；报告同时给绝对值和相对变化。",
        "- 成功率为20条不同轨迹上的绝对成功比例，并给出 Wilson 95% 置信区间。",
        "",
        "## 2. Transporting x1 权重",
        "",
        "```text",
        ", ".join("{}={:g}".format(key, value)
                  for key, value in x1_weights.items()),
        "```",
        "",
        "x1 固定总能量 Ei = {:.6f} ± {:.6f}（20条轨迹均值±样本标准差）。".format(
            x1_energy, x1_energy_std),
        "",
        "交接包中既有 Grasping 内部点1严格重扫的单轨迹 x1 固定 Ei 为 220.0123；本报告不靠后处理缩放 Ei，二者只作量级核查，不合并统计。",
        "",
        "## 3. x1 绝对指标",
        "",
        "| 成功率 | 轨迹长度（rad） | 关节速度代价 | 执行时间（s） | 优化时间（s） | 固定 Ei |",
        "|---:|---:|---:|---:|---:|---:|",
        "| {:.1f}% | {:.5f} ± {:.5f} | {:.5f} ± {:.5f} | {:.5f} ± {:.5f} | {:.5f} ± {:.5f} | {:.5f} ± {:.5f} |".format(
            100.0 * sum(int(row["successful"]) for row in baseline_rows) / expected,
            baseline_absolute["path_length_sum_rad"][0],
            baseline_absolute["path_length_sum_rad"][1],
            baseline_absolute["joint_velocity_cost_sum"][0],
            baseline_absolute["joint_velocity_cost_sum"][1],
            baseline_absolute["execution_time_s"][0],
            baseline_absolute["execution_time_s"][1],
            baseline_absolute["optimization_time_s"][0],
            baseline_absolute["optimization_time_s"][1],
            x1_energy, x1_energy_std),
        "",
        "## 4. x0/x1/x2 完整结果",
        "",
        "变化率均相对于同一组 x1 的20条配对均值；正值表示指标或代价增加。失败轨迹仍保留其数值，但不能解释为有效收益。",
        "",
        "| 权重设置 | 成功率（95% CI） | 轨迹长度变化 | 关节速度代价变化 | 执行时间变化 | 优化时间变化 | 固定 Ei（绝对值，均值±标准差） | Ei 变化 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for group, label in GROUPS:
        for setting in ("x0", "x1", "x2"):
            row = row_lookup[(group, setting)]
            if setting == "x0":
                display = label[:-1] + "=0$" if label.endswith("$") else label + "=0"
            else:
                display = label[:-1] + r"\times" + setting[-1] + "$"
            report.append(
                "| {} | {:.1f}% [{:.1f}, {:.1f}] | {:+.2f}% | {:+.2f}% | "
                "{:+.2f}% | {:+.2f}% | {:.5f} ± {:.5f} | {:+.2f}% |".format(
                    display,
                    row["success_rate_percent"],
                    row["success_wilson95_low_percent"],
                    row["success_wilson95_high_percent"],
                    row["trajectory_length_change_percent"],
                    row["joint_velocity_cost_change_percent"],
                    row["execution_time_change_percent"],
                    row["optimization_time_change_percent"],
                    row["fixed_x1_total_objective_mean"],
                    row["fixed_x1_total_objective_std"],
                    row["Ei_change_percent"]))
        report.append("|  |  |  |  |  |  |  |  |")
    report.extend([
        "",
        "## 5. 参数选择判断",
        "",
    ])
    for group, label in GROUPS:
        x0 = row_lookup[(group, "x0")]
        x1 = row_lookup[(group, "x1")]
        x2 = row_lookup[(group, "x2")]
        reasons = []
        for setting, neighbor in (("x0", x0), ("x2", x2)):
            if neighbor["success_rate_percent"] < x1["success_rate_percent"]:
                reasons.append(
                    "{} 成功率 {:.1f}% 低于 x1 的 {:.1f}%".format(
                        setting, neighbor["success_rate_percent"],
                        x1["success_rate_percent"]))
            elif neighbor["Ei_change_percent"] > 1.0:
                reasons.append(
                    "{} 固定 Ei 增加 {:+.2f}%".format(
                        setting, neighbor["Ei_change_percent"]))
            else:
                reasons.append(
                    "{} 尚无充分劣化证据（成功率 {:.1f}%，Ei {:+.2f}%）".format(
                        setting, neighbor["success_rate_percent"],
                        neighbor["Ei_change_percent"]))
        report.append("- {}：{}；{}。".format(label, reasons[0], reasons[1]))
    report.extend([
        "",
        "选择规则是先比较成功率，再比较同一固定 x1 评价器下的完整 Ei；不会因为某个单项略有改善就把失败设置解释为更优。",
        "",
        "## 6. 数据完整性说明",
        "",
        "- `transport_trajopt_raw_20x15.csv`：20条轨迹乘5组权重乘3个设置的逐项结果。",
        "- `transport_trajopt_x0_x1_x2_summary.csv/json`：论文表汇总来源，含绝对 Ei、变化率和成功率置信区间。",
        "- 原始20条规划路径、每个优化候选、梯度迭代记录和原生验证结果均保留在实验目录。",
        "",
        "## 7. 旧演示轨迹排除说明",
        "",
        "旧演示轨迹在任何本轮轨迹优化之前，用当前 URDF/FCL 模型按 0.005 rad 最大关节步长密集验证时即出现 201 个碰撞采样点（left_upper_arm_link 与 obs_box_platform）。因此正式池使用20条当前规划器新生成且原生验证通过的轨迹；旧轨迹及其验证记录仍保留，未删除。",
        "",
        "## 8. 与后续 DNMPC 的关系",
        "",
        "只有本报告确定的 Transporting x1 输出会形成后续 DNMPC 的20轨迹参考池；DNMPC 不再回到旧的单轨迹重复实验口径。",
    ])
    (args.output / "Ball_Transport轨迹优化_内部点1_固定20轨迹_中文报告.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8")

    tex = [
        r"\begin{table}[h]",
        r"\color{blue}",
        r"\caption{Effects of the Trajectory Smoothing Weights in the Transporting Phase}",
        r"\centering",
        r"\scriptsize",
        r"\begin{tabular}{lrrrrrr}",
        r"\toprule",
        r"Weight setting & Success (\%) & Length (\%) & Velocity cost (\%) & Execution (\%) & Optimization (\%) & $E_i$ (\%) \\",
        r"\midrule",
    ]
    for group, label in GROUPS:
        for setting in ("x0", "x1", "x2"):
            row = row_lookup[(group, setting)]
            tex.append(
                "{} {} & {:.1f} & {:+.2f} & {:+.2f} & {:+.2f} & {:+.2f} & {:+.2f} \\\\".format(
                    label, setting,
                    row["success_rate_percent"],
                    row["trajectory_length_change_percent"],
                    row["joint_velocity_cost_change_percent"],
                    row["execution_time_change_percent"],
                    row["optimization_time_change_percent"],
                    row["Ei_change_percent"]))
        tex.append(r"\midrule")
    tex.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}"])
    (args.output / "transport_trajopt_x0_x1_x2_table.tex").write_text(
        "\n".join(tex) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
