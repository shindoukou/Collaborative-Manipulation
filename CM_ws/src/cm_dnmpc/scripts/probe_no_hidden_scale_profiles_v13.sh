#!/usr/bin/env bash
set -euo pipefail

WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json

source "${WS}/devel/setup.bash"

run_profile() {
    local name=$1
    local weights=$2
    local output=${ROOT}/peer_wt_force_margin_profile_${name}_v13
    python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
        --config "${CONFIG}" \
        --reference-manifest "${MANIFEST}" \
        --output "${output}" \
        --weight Wu --values 0.1 \
        --base-weights-json "${weights}" \
        --evaluation-weights-json "${weights}" \
        --steps 150 --start-fraction 0.0 \
        --run-full-reference --settle-cycles 0 \
        --initial-error-std-rad 0.001 \
        --measurement-noise-std-rad 0.0005 \
        --acceleration-limit-rad-s2 0 \
        --desired-force-n 20 \
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
        --period-jitter-std-fraction 0.03 \
        --period-jitter-limit-fraction 0.10 \
        --disturbance-start-fraction 0.45 \
        --disturbance-duration-s 0.50 \
        --disturbance-amplitude-rad-s 0.05 \
        --orientation-disturbance-amplitude-rad-s 0.03 \
        --smooth-disturbance \
        --slip-onset-force-n 15 \
        --overforce-limit-n 25 \
        --slip-hold-cycles 1 \
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
        --jobs 4 >"${output}.log" 2>&1
}

names=(aa1 aa2 aa3 aa4 aa5 aa6 aa7 aa8)
profiles=(
    '{"Wd":10,"Wt":75,"Wo":200,"Wu":0.1,"Wq":250,"Wv":60,"WF":500}'
    '{"Wd":10,"Wt":75,"Wo":200,"Wu":0.1,"Wq":100,"Wv":60,"WF":500}'
    '{"Wd":10,"Wt":75,"Wo":200,"Wu":0.1,"Wq":50,"Wv":60,"WF":500}'
    '{"Wd":25,"Wt":75,"Wo":200,"Wu":0.1,"Wq":250,"Wv":60,"WF":500}'
    '{"Wd":50,"Wt":75,"Wo":200,"Wu":0.1,"Wq":250,"Wv":60,"WF":500}'
    '{"Wd":10,"Wt":50,"Wo":200,"Wu":0.1,"Wq":250,"Wv":60,"WF":500}'
    '{"Wd":10,"Wt":100,"Wo":200,"Wu":0.1,"Wq":250,"Wv":60,"WF":500}'
    '{"Wd":25,"Wt":50,"Wo":200,"Wu":0.1,"Wq":250,"Wv":60,"WF":500}'
)

for ((batch=0; batch<${#names[@]}; batch+=4)); do
    pids=()
    for ((offset=0; offset<4 && batch+offset<${#names[@]}; offset++)); do
        index=$((batch+offset))
        run_profile "${names[index]}" "${profiles[index]}" &
        pids+=("$!")
    done
    for pid in "${pids[@]}"; do
        wait "${pid}"
    done
done

python3 - "${ROOT}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
for name in ('aa1', 'aa2', 'aa3', 'aa4', 'aa5', 'aa6', 'aa7', 'aa8'):
    summary = json.load(open(root / ('peer_wt_force_margin_profile_%s_v13' % name) / 'result.json'))['summary'][0]
    print(name,
          'success=%d/%d' % (summary['successes'], summary['runs']),
          'terminal=%.2fmm' % summary['terminal_target_error_mm_mean'],
          'orientation=%.2fdeg' % summary['orientation_rmse_deg_mean'],
          'relative=%.2fmm' % (1000.0 * summary['relative_position_rmse_m_mean']),
          'force_min=%.2fN' % summary['force_min_n_mean'],
          'force_max=%.2fN' % summary['force_max_n_mean'],
          'failures=%s' % summary['hard_failure_reason_counts'])
PY
