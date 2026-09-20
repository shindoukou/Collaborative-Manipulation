#!/usr/bin/env bash
set -euo pipefail
WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
BASE='{"Wd":2,"Wt":1000,"Wo":200,"Wu":0.1,"Wq":750,"Wv":300,"WF":1000}'
source "${WS}/devel/setup.bash"

common=(
  --config "${CONFIG}" --reference-manifest "${MANIFEST}"
  --base-weights-json "${BASE}" --evaluation-weights-json "${BASE}"
  --steps 150 --start-fraction 0 --run-full-reference --settle-cycles 0
  --initial-error-std-rad 0.001 --measurement-noise-std-rad 0.0005
  --acceleration-limit-rad-s2 0 --desired-force-n 20
  --object-damping-n-s-m 0 --object-damping-min-n-s-m 0
  --object-damping-max-n-s-m 8 --force-equilibrium-mode runtime_absolute
  --object-length-m 0.18 --force-noise-std-n 0.15
  --bhat-left 0 --bhat-right 0 --wb-left 20 --wb-right 20
  --wk-left 1 --wk-right 1 --k0 500 --max-aic-displacement-m 0.070
  --peer-delay-min-cycles 0 --peer-delay-max-cycles 1
  --actuation-delay-min-cycles 0 --actuation-delay-max-cycles 0
  --period-jitter-std-fraction 0.03 --period-jitter-limit-fraction 0.10
  --disturbance-start-fraction 0.45 --disturbance-duration-s 0.50
  --disturbance-amplitude-rad-s 0.05
  --orientation-disturbance-amplitude-rad-s 0.03 --smooth-disturbance
  --slip-onset-force-n 10 --drop-force-multiplier 1.5
  --overforce-limit-n 25 --slip-hold-cycles 1 --overforce-hold-cycles 1
  --grace-cycles 0 --terminal-target-error-mm 10 --orientation-rmse-deg 30
  --hard-distance-min-m 0.10 --hard-distance-max-m 0.18
  --hard-joint-velocity-scale 1 --hard-joint-acceleration-rad-s2 25
  --hard-command-jump-rad-s 1.2 --hard-force-slew-n-s 350
  --deadline-ms 20 --deadline-miss-hold-cycles 0 --jobs 4
)

run_wu() {
  local out=${ROOT}/probe_wu_raw_boundary_v51
  python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
    "${common[@]}" --output "${out}" --weight Wu \
    --values "0,1,5,10,25,50,100,200,500,1000" \
    --object-stiffness-n-m 625 --object-stiffness-min-n-m 500 \
    --object-stiffness-max-n-m 750 --force-delay-cycles 0 \
    --force-delay-min-cycles 0 --force-delay-max-cycles 1 \
    >"${out}.log" 2>&1
}

run_wf_profile() {
  local name=$1 nominal=$2 kmin=$3 kmax=$4 dmin=$5 dmax=$6 out
  out=${ROOT}/probe_wf_physical_${name}_v51
  python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
    "${common[@]}" --output "${out}" --weight WF --values "1000,2000" \
    --object-stiffness-n-m "${nominal}" --object-stiffness-min-n-m "${kmin}" \
    --object-stiffness-max-n-m "${kmax}" --force-delay-cycles "${dmin}" \
    --force-delay-min-cycles "${dmin}" --force-delay-max-cycles "${dmax}" \
    >"${out}.log" 2>&1
}

run_wu & p0=$!
run_wf_profile k500_750_d01 625 500 750 0 1 & p1=$!
run_wf_profile k750_1250_d01 1000 750 1250 0 1 & p2=$!
run_wf_profile k1000_1500_d01 1250 1000 1500 0 1 & p3=$!
run_wf_profile k750_1250_d12 1000 750 1250 1 2 & p4=$!
wait "${p0}"; wait "${p1}"; wait "${p2}"; wait "${p3}"; wait "${p4}"

python3 - "${ROOT}" <<'PY'
import json, pathlib, sys
r=pathlib.Path(sys.argv[1])
d=json.load(open(r/'probe_wu_raw_boundary_v51'/'result.json'))
print('WU')
for s in d['summary']:
 print(s['weight_value'],s['successes'],s['hard_failure_reason_counts'],
       'cmd',round(s['control_effort_mean'],5),'Ei',round(s['common_eval_cost_mean'],5),
       'terminal',round(s['terminal_target_error_mm_mean'],3))
print('WF')
for name in ('k500_750_d01','k750_1250_d01','k1000_1500_d01','k750_1250_d12'):
 d=json.load(open(r/('probe_wf_physical_'+name+'_v51')/'result.json'))
 print(name,[(s['weight_value'],s['successes'],s['hard_failure_reason_counts'],
              round(s['force_rmse_n_mean'],3),round(s['common_eval_cost_mean'],3),
              round(s['force_max_n_mean'],3)) for s in d['summary']])
PY
