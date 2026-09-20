#!/usr/bin/env bash
set -euo pipefail
WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
URDF=${WS}/src/cm_kirara_ball/generated/gazebo_newbase_expanded_for_validation.urdf
AUDITOR=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/52_dnmpc_physical_scale_boundary_20260908/formal_final_physical_weights_force5_20260909/provenance/validate_dnmpc_trace_collisions.py
OUT=${ROOT}/validate_wu12_wf_x0x1x2_v55
TRACE=${ROOT}/traces_validate_wu12_wf_x0x1x2_v55
BASE='{"Wd":2,"Wt":1000,"Wo":200,"Wu":12,"Wq":750,"Wv":300,"WF":1000}'
source "${WS}/devel/setup.bash"

python3 -u "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
  --config "${CONFIG}" --reference-manifest "${MANIFEST}" \
  --output "${OUT}" --trace-output "${TRACE}" \
  --weight WF --values "0,1000,2000" \
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
  >"${OUT}.log" 2>&1

python3 "${AUDITOR}" --trace-root "${TRACE}" --urdf "${URDF}" \
  --package-dir "${WS}/src/cm_kirara_ball" \
  --output "${OUT}/collision_audit.json" --ball-radius-m 0.09 \
  >"${OUT}/collision_audit.log" 2>&1

python3 - "${OUT}" <<'PY'
import json, pathlib, re, sys
p=pathlib.Path(sys.argv[1]); d=json.load(open(p/'result.json'))
c=json.load(open(p/'collision_audit.json'))
cf={i:0 for i in range(3)}
for row in c['results']:
 m=re.search(r'/v(\d+)/',row['trace']); cf[int(m.group(1))]+=int(row['collision_free'])
for s in d['summary']:
 print('WF=%g'%s['weight_value'],'control=%d/20'%s['successes'],
       'collision=%d/20'%cf[int(s['setting_label'][1:])],
       'failures='+s['hard_failure_reason_counts'],
       'forceRMSE=%.3f'%s['force_rmse_n_mean'],
       'terminal=%.3fmm'%s['terminal_target_error_mm_mean'],
       'Ei=%.4f'%s['common_eval_cost_mean'])
PY
