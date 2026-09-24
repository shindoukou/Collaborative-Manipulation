#!/usr/bin/env bash
set -euo pipefail
WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
source "${WS}/devel/setup.bash"

run_profile() {
 local name=$1 wt=$2 wq=$3 output weights
 output=${ROOT}/high_wq_low_wt_${name}_v35
 weights=$(printf '{"Wd":2,"Wt":%s,"Wo":200,"Wu":0.1,"Wq":%s,"Wv":100,"WF":1000}' "${wt}" "${wq}")
 python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
  --config "${CONFIG}" --reference-manifest "${MANIFEST}" --output "${output}" \
  --weight Wu --values 0.1 --base-weights-json "${weights}" \
  --evaluation-weights-json "${weights}" --steps 150 --start-fraction 0 \
  --run-full-reference --settle-cycles 0 --initial-error-std-rad 0.001 \
  --measurement-noise-std-rad 0.0005 --acceleration-limit-rad-s2 0 \
  --desired-force-n 20 --object-stiffness-n-m 625 \
  --object-stiffness-min-n-m 500 --object-stiffness-max-n-m 750 \
  --object-damping-n-s-m 0 --object-damping-min-n-s-m 0 \
  --object-damping-max-n-s-m 8 --force-equilibrium-mode runtime_absolute \
  --object-length-m 0.18 --force-delay-cycles 0 --force-delay-min-cycles 0 \
  --force-delay-max-cycles 1 --force-noise-std-n 0.15 \
  --bhat-left 0 --bhat-right 0 --wb-left 20 --wb-right 20 \
  --wk-left 1 --wk-right 1 --k0 500 --max-aic-displacement-m 0.050 \
  --peer-delay-min-cycles 0 --peer-delay-max-cycles 1 \
  --actuation-delay-min-cycles 0 --actuation-delay-max-cycles 0 \
  --period-jitter-std-fraction 0.03 --period-jitter-limit-fraction 0.10 \
  --disturbance-start-fraction 0.45 --disturbance-duration-s 0.50 \
  --disturbance-amplitude-rad-s 0.05 \
  --orientation-disturbance-amplitude-rad-s 0.03 --smooth-disturbance \
  --slip-onset-force-n 10 --drop-force-multiplier 1.5 \
  --overforce-limit-n 25 --slip-hold-cycles 1 --overforce-hold-cycles 1 \
  --grace-cycles 0 --terminal-target-error-mm 10 --orientation-rmse-deg 5 \
  --hard-distance-min-m 0.10 --hard-distance-max-m 0.18 \
  --hard-joint-velocity-scale 1 --hard-joint-acceleration-rad-s2 25 \
  --hard-command-jump-rad-s 1.2 --hard-force-slew-n-s 350 \
  --deadline-ms 20 --deadline-miss-hold-cycles 0 --jobs 4 >"${output}.log" 2>&1
}

names=(q600_t20 q700_t20 q800_t20 q900_t20 q1000_t20 q700_t40 q800_t40 q900_t40)
wqs=(600 700 800 900 1000 700 800 900)
wts=(20 20 20 20 20 40 40 40)
for ((batch=0;batch<8;batch+=4)); do
 pids=()
 for ((o=0;o<4;o++)); do i=$((batch+o)); run_profile "${names[i]}" "${wts[i]}" "${wqs[i]}" & pids+=("$!"); done
 for pid in "${pids[@]}"; do wait "${pid}"; done
done

python3 - "${ROOT}" <<'PY'
import json,pathlib,sys
r=pathlib.Path(sys.argv[1])
for n in ('q600_t20','q700_t20','q800_t20','q900_t20','q1000_t20','q700_t40','q800_t40','q900_t40'):
 s=json.load(open(r/('high_wq_low_wt_%s_v35'%n)/'result.json'))['summary'][0]
 print(n,'success=%d/%d'%(s['successes'],s['runs']),
       'terminal=%.2fmm'%s['terminal_target_error_mm_mean'],
       'force_min=%.2fN'%s['force_min_n_mean'],
       'force_max=%.2fN'%s['force_max_n_mean'],
       'failures=%s'%s['hard_failure_reason_counts'])
PY
