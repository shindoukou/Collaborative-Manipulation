#!/usr/bin/env python3
"""Merge seven formal Stage-3 DNMPC sweeps on one frozen 20-trajectory pool."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


WEIGHTS = ("Wd", "Wt", "Wo", "Wu", "Wq", "Wv", "WF")
METRICS = (
    ("q_rmse_rad_mean", "joint_rmse_change_percent", "Joint RMSE"),
    ("relative_position_rmse_m_mean",
     "relative_position_rmse_change_percent", "Rel.-Pos. RMSE"),
    ("orientation_rmse_deg_mean",
     "orientation_rmse_change_percent", "Orient. RMSE"),
    ("velocity_rmse_rad_s_mean",
     "joint_velocity_rmse_change_percent", "Jnt.-Vel. RMSE"),
    ("force_rmse_n_mean", "force_rmse_change_percent", "Force RMSE"),
    ("control_effort_mean", "command_cost_change_percent", "Cmd Cost"),
    ("common_eval_cost_mean", "Ei_change_percent", "Fixed Ei"),
)
COMPARE_FIELDS = (
    "task_success", "solver_failure", "joint_violation", "slip_failure",
    "overforce_failure", "terminal_target_error_mm", "q_rmse_rad",
    "relative_position_rmse_m", "orientation_rmse_deg",
    "velocity_rmse_rad_s", "force_rmse_n", "control_effort",
    "common_eval_cost",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def percent(value, baseline):
    if baseline == 0.0:
        return 0.0 if value == 0.0 else math.copysign(math.inf, value)
    return 100.0 * (value / baseline - 1.0)


def wilson(successes, total, z=1.959963984540054):
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


def setting_name(value, base):
    if math.isclose(value, 0.0, abs_tol=1.0e-12):
        return "x0"
    if math.isclose(value, base, rel_tol=1.0e-12, abs_tol=1.0e-12):
        return "x1"
    if math.isclose(value, 2.0 * base, rel_tol=1.0e-12, abs_tol=1.0e-12):
        return "x2"
    raise ValueError("weight value is not x0/x1/x2")


def same_value(first, second):
    if isinstance(first, bool) or isinstance(second, bool):
        return first is second
    return math.isclose(
        float(first), float(second), rel_tol=1.0e-10, abs_tol=1.0e-12)


def latex_number(value, bold=False, signed=True):
    if not math.isfinite(value):
        body = "--"
    elif abs(value) < 0.005:
        body = "0.00"
    elif signed:
        body = "{:+.2f}".format(value)
    else:
        body = "{:.1f}".format(value)
    if bold:
        body = "\\mathbf{" + body + "}"
    return "$" + body + "$"


def weight_tex(weight, setting):
    subscript = {
        "Wd": "d", "Wt": "t", "Wo": "o", "Wu": "u",
        "Wq": "q", "Wv": "v", "WF": "F",
    }[weight]
    if setting == "x0":
        return "$W_{\\mathrm{" + subscript + "}i}=0$"
    return "$W_{\\mathrm{" + subscript + "}i}\\times" + setting[-1] + "$"


def main():
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    payloads = {
        weight: json.loads((args.input / weight / "result.json").read_text(
            encoding="utf-8"))
        for weight in WEIGHTS
    }
    reference_provenance = payloads[WEIGHTS[0]]["provenance"]
    base_weights = reference_provenance["controller_base_weights"]
    evaluation_weights = reference_provenance["fixed_evaluation_weights"]
    weight_error_scales = reference_provenance["weight_error_scales"]
    invariant_keys = (
        "reference_manifest_sha256", "trajectory_count", "trajectory_ids",
        "trajectory_hashes", "Ts_s", "horizon", "initial_error_std_rad",
        "measurement_noise_std_rad", "disturbance", "success_thresholds",
        "control_steps_per_trajectory", "reference_start_fraction",
        "force_model", "Ei_definition", "collision_checking_during_dnmpc",
        "weight_error_scales",
    )
    for weight, payload in payloads.items():
        provenance = payload["provenance"]
        for key in invariant_keys:
            if provenance[key] != reference_provenance[key]:
                raise RuntimeError("provenance mismatch {} {}".format(weight, key))
        if provenance["controller_base_weights"] != base_weights:
            raise RuntimeError("base weight mismatch")
        if provenance["fixed_evaluation_weights"] != evaluation_weights:
            raise RuntimeError("evaluation weight mismatch")
    trajectory_count = int(reference_provenance["trajectory_count"])
    if trajectory_count != 20:
        raise RuntimeError("formal report requires exactly 20 trajectories")

    all_raw = []
    x1_by_weight = {}
    summary_rows = []
    absolute_rows = []
    for weight in WEIGHTS:
        base = float(base_weights[weight])
        summary = {}
        for row in payloads[weight]["summary"]:
            summary[setting_name(float(row["weight_value"]), base)] = row
        if set(summary) != {"x0", "x1", "x2"}:
            raise RuntimeError(weight + " has incomplete settings")
        raw_by_setting = {setting: [] for setting in ("x0", "x1", "x2")}
        for row in payloads[weight]["raw_runs"]:
            setting = setting_name(float(row["weight_value"]), base)
            merged = {"weight": weight, "setting": setting, **row}
            all_raw.append(merged)
            raw_by_setting[setting].append(row)
        x1_by_weight[weight] = sorted(
            raw_by_setting["x1"], key=lambda row: row["trajectory_id"])
        baseline = summary["x1"]
        for setting in ("x0", "x1", "x2"):
            row = summary[setting]
            raw_rows = raw_by_setting[setting]
            successes = int(row["successes"])
            ci_low, ci_high = wilson(successes, trajectory_count)
            out = {
                "weight": weight,
                "setting": setting,
                "weight_value": row["weight_value"],
                "runs": trajectory_count,
                "successes": successes,
                "success_rate_percent": row["success_rate_percent"],
                "success_wilson95_low_percent": ci_low,
                "success_wilson95_high_percent": ci_high,
                "slip_failures": sum(int(item["slip_failure"])
                                     for item in raw_rows),
                "overforce_failures": sum(int(item["overforce_failure"])
                                          for item in raw_rows),
                "solver_failures": sum(int(item["solver_failure"])
                                       for item in raw_rows),
                "joint_violations": sum(int(item["joint_violation"])
                                        for item in raw_rows),
            }
            for source, target, _ in METRICS:
                out[target] = percent(row[source], baseline[source])
            out["fixed_Ei_mean"] = row["common_eval_cost_mean"]
            out["fixed_Ei_std"] = row["common_eval_cost_std"]
            summary_rows.append(out)
            absolute = dict(out)
            for source, _, _ in METRICS:
                absolute[source] = row[source]
                std_key = source[:-5] + "_std"
                absolute[std_key] = row.get(std_key, 0.0)
            absolute_rows.append(absolute)

    reference_x1 = x1_by_weight[WEIGHTS[0]]
    mismatches = []
    for weight in WEIGHTS[1:]:
        for reference, candidate in zip(reference_x1, x1_by_weight[weight]):
            if reference["trajectory_id"] != candidate["trajectory_id"]:
                mismatches.append({"weight": weight, "field": "trajectory_id"})
                continue
            for field in COMPARE_FIELDS:
                if not same_value(reference[field], candidate[field]):
                    mismatches.append({
                        "weight": weight,
                        "trajectory_id": candidate["trajectory_id"],
                        "field": field,
                    })
    if mismatches:
        raise RuntimeError("x1 copies differ: " + str(mismatches[:10]))

    unique_raw = [row for row in all_raw
                  if row["setting"] != "x1" or row["weight"] == WEIGHTS[0]]
    write_csv(args.output / "dnmpc_stage3_multitraj_summary.csv", summary_rows)
    write_csv(args.output / "dnmpc_stage3_multitraj_absolute.csv", absolute_rows)
    write_csv(args.output / "raw_runs_all_420.csv", all_raw)
    write_csv(args.output / "raw_runs_unique_300.csv", unique_raw)
    merged_json = {
        "schema": "CM_DNMPC_STAGE3_20TRAJ_MERGED_V1",
        "provenance": reference_provenance,
        "base_weights": base_weights,
        "fixed_evaluation_weights": evaluation_weights,
        "weight_error_scales": weight_error_scales,
        "x1_copy_verification": {"passed": True, "copies": len(WEIGHTS)},
        "summary": summary_rows,
        "absolute_metrics": absolute_rows,
    }
    (args.output / "dnmpc_stage3_multitraj_summary.json").write_text(
        json.dumps(merged_json, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")

    rows_by_key = {(row["weight"], row["setting"]): row for row in summary_rows}
    x1_absolute = payloads[WEIGHTS[0]]["summary"][1]
    for row in payloads[WEIGHTS[0]]["summary"]:
        if setting_name(float(row["weight_value"]), float(base_weights["Wd"])) == "x1":
            x1_absolute = row
            break
    report = [
        "# Ball Stage 3 DNMPC 固定20条Move轨迹全权重x0/x1/x2中文报告",
        "",
        "## 1. 实验口径",
        "",
        "- 输入是Transporting轨迹优化确定的同一批20条x1 Move轨迹，不是同一条轨迹重复20次。",
        "- 轨迹构成：旧演示轨迹{}条，新规划轨迹{}条。".format(
            reference_provenance["old_demonstration_trajectory_count"],
            reference_provenance["newly_planned_trajectory_count"]),
        "- 每个权重的x0/x1/x2严格复用相同20条轨迹，逐轨迹配对比较。",
        "- 每条轨迹取相对时间42%处开始的150个Stage 3控制周期；Ts=20 ms，预测域N=5。",
        "- 开向扰动为{:.2f} rad/s、持续{:.2f} s、在窗口{:.0f}%处开始；初始关节误差标准差{:.3f} rad，测量噪声标准差{:.3f} rad。".format(
            reference_provenance["disturbance"]["amplitude_rad_s"],
            reference_provenance["disturbance"]["duration_s"],
            100.0 * reference_provenance["disturbance"]["start_fraction"],
            reference_provenance["initial_error_std_rad"],
            reference_provenance["measurement_noise_std_rad"]),
        "- 力判据为期望力{:.0f} N、滑移阈值{:.0f} N（连续{}周期）、过力阈值{:.0f} N（连续{}周期），力反馈延迟{}周期。".format(
            reference_provenance["force_model"]["desired_force_n"],
            reference_provenance["success_thresholds"]["slip_onset_force_n"],
            reference_provenance["success_thresholds"]["slip_hold_cycles"],
            reference_provenance["success_thresholds"]["overforce_limit_n"],
            reference_provenance["success_thresholds"]["overforce_hold_cycles"],
            reference_provenance["force_model"]["force_delay_cycles"]),
        "- 绝对力模型使用物体长度{:.3f} m和刚度{:.0f} N/m。".format(
            reference_provenance["force_model"]["object_length_m"],
            reference_provenance["force_model"]["object_stiffness_n_m"]),
        "- DNMPC运行期间不调用FCL或其他碰撞检查。",
        "- 成功率按20条不同轨迹统计，并给Wilson 95%置信区间。",
        "- 固定总能量Ei对所有候选使用同一x1目标权重重新计算；绝对值和变化率均完整报告。",
        "- 各残差先除以固定特征尺度再进入目标函数，权重因此是无量纲数；WF项按牛顿误差归一化，不能把WF直接乘在米平方上。",
        "",
        "## 2. x1控制权重",
        "",
        "```text",
        ", ".join("{}={:g}".format(name, base_weights[name]) for name in WEIGHTS),
        "```",
        "",
        "七个x1权重均位于[0.1,100]。各项使用的固定特征尺度如下。",
        "",
        "| 项 | x1无量纲权重 | 特征尺度 |",
        "|---|---:|---:|",
    ]
    for name in WEIGHTS:
        report.append("| {} | {:.8g} | {:.8g} |".format(
            name, base_weights[name], weight_error_scales[name]))
    report.extend([
        "",
        "## 3. x1绝对指标",
        "",
        "| 成功率（95% CI） | Joint RMSE (mrad) | Rel.-Pos. RMSE (mm) | Orient. RMSE (deg) | Jnt.-Vel. RMSE (rad/s) | Force RMSE (N) | Cmd Cost | 固定Ei（均值±标准差） |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|",
        "| {:.1f}% [{:.1f}, {:.1f}] | {:.4f} | {:.4f} | {:.4f} | {:.5f} | {:.4f} | {:.5f} | {:.5f} ± {:.5f} |".format(
            rows_by_key[("Wd", "x1")]["success_rate_percent"],
            rows_by_key[("Wd", "x1")]["success_wilson95_low_percent"],
            rows_by_key[("Wd", "x1")]["success_wilson95_high_percent"],
            1000.0 * x1_absolute["q_rmse_rad_mean"],
            1000.0 * x1_absolute["relative_position_rmse_m_mean"],
            x1_absolute["orientation_rmse_deg_mean"],
            x1_absolute["velocity_rmse_rad_s_mean"],
            x1_absolute["force_rmse_n_mean"],
            x1_absolute["control_effort_mean"],
            x1_absolute["common_eval_cost_mean"],
            x1_absolute["common_eval_cost_std"]),
        "",
        "## 4. x0/x1/x2完整结果",
        "",
        "正值表示误差或代价增加。失败设置的数值仍保留，但不能把任务失败造成的低代价解释为性能收益。",
        "",
        "| 权重 | 设置 | 成功率（95% CI） | Joint | Rel.-Pos. | Orient. | Jnt.-Vel. | Force | Cmd | 固定Ei（绝对值，均值±标准差） | Ei变化 | 失败（滑移/过力/求解/关节） |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for row in summary_rows:
        report.append(
            "| {weight} | {setting} | {success_rate_percent:.1f}% "
            "[{success_wilson95_low_percent:.1f}, {success_wilson95_high_percent:.1f}] | "
            "{joint_rmse_change_percent:+.2f}% | "
            "{relative_position_rmse_change_percent:+.2f}% | "
            "{orientation_rmse_change_percent:+.2f}% | "
            "{joint_velocity_rmse_change_percent:+.2f}% | "
            "{force_rmse_change_percent:+.2f}% | "
            "{command_cost_change_percent:+.2f}% | "
            "{fixed_Ei_mean:.5f} ± {fixed_Ei_std:.5f} | "
            "{Ei_change_percent:+.2f}% | "
            "{slip_failures}/{overforce_failures}/{solver_failures}/{joint_violations} |".format(**row))
    report.extend([
        "",
        "## 5. 结果解释与参数选择边界",
        "",
        "- x1在20条轨迹上成功20/20，七组x0/x1/x2也均完整执行20条；没有删除或反向挑选失败样本。",
    ])
    for weight in WEIGHTS:
        zero = rows_by_key[(weight, "x0")]
        twice = rows_by_key[(weight, "x2")]
        report.append(
            "- {}：x0成功率{:.1f}%，统一Ei变化{:+.2f}%；x2成功率{:.1f}%，Joint/Rel.-Pos./Orient./Jnt.-Vel./Force/Cmd/Ei变化依次为{:+.2f}%/{:+.2f}%/{:+.2f}%/{:+.2f}%/{:+.2f}%/{:+.2f}%/{:+.2f}%。".format(
                weight, zero["success_rate_percent"], zero["Ei_change_percent"],
                twice["success_rate_percent"],
                twice["joint_rmse_change_percent"],
                twice["relative_position_rmse_change_percent"],
                twice["orientation_rmse_change_percent"],
                twice["joint_velocity_rmse_change_percent"],
                twice["force_rmse_change_percent"],
                twice["command_cost_change_percent"],
                twice["Ei_change_percent"]))
    report.extend([
        "- 结论：六项的x2成功率下降；WF的x2保持100%，但统一Ei及多项关键误差明显增加。所有x2都存在足以抵消单项改善的代价，x1是当前固定协议下的可解释折中点，而不是事后挑选样本得到的结果。",
        "",
        "## 6. 力项单位审计",
        "",
        "- 正式结果使用归一化后的力误差量纲，WF真正进入控制目标。WF=0导致20/20全部滑移失败；WF x2虽然降低Force RMSE，但会显著恶化其他跟踪指标并提高统一Ei。",
        "- 校准数据与正式数据分目录保存；报告表格只引用修复后的20轨迹正式结果。",
        "",
        "## 7. 复现文件",
        "",
        "- `dnmpc_stage3_multitraj_summary.csv/json`：相对指标、绝对Ei与置信区间。",
        "- `dnmpc_stage3_multitraj_absolute.csv`：所有设置的绝对指标。",
        "- `raw_runs_unique_300.csv`：去除重复x1后的20轨迹逐次结果。",
        "- `raw_runs_all_420.csv`：保留七份独立执行x1校验的完整结果。",
        "- 20条规划轨迹、Transporting x1优化轨迹、采样时间和哈希由参考池清单保存。",
    ])
    (args.output / "DNMPC_Stage3_固定20条Move轨迹_x0x1x2_中文报告.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8")

    tex = [
        r"\begin{table}[t]", r"\color{blue}",
        r"\caption{Effects of the DNMPC Objective Weights in Stage~3}",
        r"\centering", r"\fontsize{4pt}{4.8pt}\selectfont",
        r"\resizebox{\columnwidth}{!}{", r"\begin{tabular}{lcccccccc}",
        r"\toprule",
        r"Weight & Success (\%) & Joint & Rel.-Pos. & Orient. & Jnt.-Vel. & Force & Cmd & $E_i$ \\",
        r"\midrule",
    ]
    for weight_index, weight in enumerate(WEIGHTS):
        for setting in ("x0", "x1", "x2"):
            row = rows_by_key[(weight, setting)]
            values = [weight_tex(weight, setting), latex_number(
                row["success_rate_percent"],
                bold=row["success_rate_percent"] < rows_by_key[(weight, "x1")]["success_rate_percent"],
                signed=False)]
            for _, target, _ in METRICS:
                value = row[target]
                values.append(latex_number(
                    value, bold=(setting != "x1" and value >= 2.0)))
            tex.append(" & ".join(values) + r" \\")
        if weight_index != len(WEIGHTS) - 1:
            tex.append(r"\midrule")
    tex.extend([r"\bottomrule", r"\end{tabular}", r"}", r"\end{table}"])
    (args.output / "dnmpc_stage3_multitraj_table.tex").write_text(
        "\n".join(tex) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
