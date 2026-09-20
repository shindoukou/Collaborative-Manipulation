#!/usr/bin/env bash
set -euo pipefail

WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909/profile_probe_v14_low_wv_low_wf
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
MANIFEST=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/04_dnmpc_reference_pool_final_internal1_margin0p15_wv6_20260828/trajectory_pool_manifest.json

source "${WS}/devel/setup.bash"
mkdir -p "${ROOT}"

run_profile() {
    local name=$1
    local weights=$2
    local wd=$3
    python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
        --config "${CONFIG}" \
        --reference-manifest "${MANIFEST}" \
        --output "${ROOT}/${name}" \
        --weight Wd --values "${wd}" \
        --base-weights-json "${weights}" \
        --evaluation-weights-json "${weights}" \
        --steps 150 --start-fraction 0.42 \
        --run-full-reference --settle-cycles 100 \
        --initial-error-std-rad 0.003 \
        --measurement-noise-std-rad 0.002 \
        --acceleration-limit-rad-s2 3 \
        --desired-force-n 16 \
        --object-stiffness-n-m 625 \
        --object-stiffness-min-n-m 500 \
        --object-stiffness-max-n-m 750 \
        --object-damping-n-s-m 0 \
        --object-damping-min-n-s-m 0 \
        --object-damping-max-n-s-m 8 \
        --force-equilibrium-mode runtime_absolute \
        --object-length-m 0.18 \
        --force-delay-cycles 0 \
        --force-delay-min-cycles 0 \
        --force-delay-max-cycles 1 \
        --force-noise-std-n 0.15 \
        --bhat-left 0 --bhat-right 0 \
        --wb-left 200 --wb-right 200 \
        --wk-left 10 --wk-right 10 --k0 500 \
        --peer-delay-min-cycles 0 \
        --peer-delay-max-cycles 1 \
        --actuation-delay-min-cycles 0 \
        --actuation-delay-max-cycles 0 \
        --period-jitter-std-fraction 0.08 \
        --period-jitter-limit-fraction 0.20 \
        --disturbance-start-fraction 0.45 \
        --disturbance-duration-s 0.50 \
        --disturbance-amplitude-rad-s 0.35 \
        --orientation-disturbance-amplitude-rad-s 0.10 \
        --smooth-disturbance \
        --slip-onset-force-n 10 \
        --overforce-limit-n 25 \
        --slip-hold-cycles 5 \
        --overforce-hold-cycles 1 \
        --grace-cycles 10 \
        --terminal-target-error-mm 10 \
        --orientation-rmse-deg 5 \
        --hard-distance-min-m 0.10 \
        --hard-distance-max-m 0.18 \
        --hard-joint-velocity-scale 1 \
        --hard-joint-acceleration-rad-s2 25 \
        --hard-command-jump-rad-s 1.2 \
        --hard-force-slew-n-s 350 \
        --deadline-ms 20 \
        --deadline-miss-hold-cycles 0 \
        --jobs 6 >"${ROOT}/${name}.log" 2>&1
}

names=(z1 z2 z3 z4)
weights=(
    '{"Wd":1000,"Wt":1000,"Wo":1000,"Wu":0.1,"Wq":1000,"Wv":1,"WF":0.1}'
    '{"Wd":1000,"Wt":1000,"Wo":1000,"Wu":0.1,"Wq":1000,"Wv":2,"WF":0.1}'
    '{"Wd":1000,"Wt":1000,"Wo":1000,"Wu":0.1,"Wq":1000,"Wv":5,"WF":0.1}'
    '{"Wd":1000,"Wt":1000,"Wo":1000,"Wu":0.1,"Wq":1000,"Wv":10,"WF":0.1}'
)
wds=(1000 1000 1000 1000)

for ((batch=0; batch<4; batch+=2)); do
    pids=()
    for ((offset=0; offset<2; offset++)); do
        index=$((batch+offset))
        run_profile "${names[index]}" "${weights[index]}" "${wds[index]}" &
        pids+=("$!")
    done
    for pid in "${pids[@]}"; do
        wait "${pid}"
    done
done

python3 - "${ROOT}" <<'PY'
import json, pathlib, sys
root = pathlib.Path(sys.argv[1])
for p in sorted(root.glob('z*/result.json')):
    s=json.load(p.open())['summary'][0]
    print(p.parent.name, 'success', s['successes'], '/', s['runs'],
          'terminal_mm', s['terminal_target_error_mm_mean'],
          'force_rmse', s['force_rmse_n_mean'],
          'orientation_deg', s['orientation_rmse_deg_mean'],
          'failures', s['hard_failure_reason_counts'])
PY
