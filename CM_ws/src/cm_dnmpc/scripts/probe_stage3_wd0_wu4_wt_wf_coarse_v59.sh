#!/usr/bin/env bash
set -euo pipefail
WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/56_stage3_wd0_wt_wu_wf_retune_20260910
SOURCE_ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${SOURCE_ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
BASE='{"Wd":0,"Wt":1000,"Wo":200,"Wu":4,"Wq":750,"Wv":300,"WF":1000}'
source "${WS}/devel/setup.bash"
mkdir -p "${ROOT}"

run_probe() {
  local weight=$1 values=$2 out=$3
  python3 -u "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
    --config "${CONFIG}" --reference-manifest "${MANIFEST}" --output "${out}" \
    --weight "${weight}" --values "${values}" \
    --base-weights-json "${BASE}" --evaluation-weights-json "${BASE}" \
    --steps 150 --start-fraction 0 --run-full-reference --settle-cycles 0 \
    --initial-error-std-rad 0.001 --measurement-noise-std-rad 0.0005 \
    --acceleration-limit-rad-s2 0 --desired-force-n 20 \
    --object-stiffness-n-m 625 --object-stiffness-min-n-m 500 \
    --object-stiffness-max-n-m 750 --object-damping-n-s-m 0 \
    --object-damping-min-n-s-m 0 --object-damping-max-n-s-m 8 \
    --force-equilibrium-mode runtime_absolute --object-length-m 0.18 \
    --force-delay-cycles 0 --force-delay-min-cycles 0 --force-delay-max-cycles 1 \
    --force-noise-std-n 0.15 --bhat-left 0 --bhat-right 0 \
    --wb-left 20 --wb-right 20 --wk-left 1 --wk-right 1 --k0 500 \
    --max-aic-displacement-m 0.070 --peer-delay-min-cycles 0 \
    --peer-delay-max-cycles 1 --actuation-delay-min-cycles 0 \
    --actuation-delay-max-cycles 0 --period-jitter-std-fraction 0.03 \
    --period-jitter-limit-fraction 0.10 --disturbance-start-fraction 0.45 \
    --disturbance-duration-s 0.50 --disturbance-amplitude-rad-s 0.05 \
    --orientation-disturbance-amplitude-rad-s 0.03 --smooth-disturbance \
    --slip-onset-force-n 10 --drop-force-multiplier 1.5 \
    --overforce-limit-n 25 --slip-hold-cycles 1 --overforce-hold-cycles 1 \
    --grace-cycles 0 --terminal-target-error-mm 10 --orientation-rmse-deg 30 \
    --hard-distance-min-m 0.10 --hard-distance-max-m 0.18 \
    --hard-joint-velocity-scale 1 --hard-joint-acceleration-rad-s2 25 \
    --hard-command-jump-rad-s 1.2 --hard-force-slew-n-s 350 \
    --deadline-ms 20 --deadline-miss-hold-cycles 0 --jobs 4 \
    >"${out}.log" 2>&1
}

WT_OUT=${ROOT}/probe_wt_coarse_v59
WF_OUT=${ROOT}/probe_wf_coarse_v59
run_probe Wt "0,500,1000,2000,3000,4000,6000,8000,9999" "${WT_OUT}" & p1=$!
run_probe WF "0,500,1000,2000,3000,4000,6000,8000,9999" "${WF_OUT}" & p2=$!
wait "${p1}"
wait "${p2}"

python3 - "${WT_OUT}" "${WF_OUT}" <<'PY'
import json, pathlib, sys
for raw in sys.argv[1:]:
 d=json.load(open(pathlib.Path(raw)/'result.json'))
 print('\n'+d['provenance']['weight'])
 for s in d['summary']:
  print('value=%g'%s['weight_value'],'success=%d/20'%s['successes'],
        'failures='+s['hard_failure_reason_counts'],
        'Ei=%.6f'%s['common_eval_cost_mean'],
        'terminal=%.3fmm'%s['terminal_target_error_mm_mean'],
        'rel=%.3fmm'%(1000*s['relative_position_rmse_m_mean']),
        'force=%.3fN'%s['force_rmse_n_mean'])
PY
