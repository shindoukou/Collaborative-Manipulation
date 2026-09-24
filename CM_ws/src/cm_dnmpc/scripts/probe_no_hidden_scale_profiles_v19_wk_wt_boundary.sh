#!/usr/bin/env bash
set -euo pipefail

WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
source "${WS}/devel/setup.bash"

run_profile() {
    local name=$1 wt=$2 wk=$3
    local output=${ROOT}/aic_wk_wt_boundary_${name}_v19
    local weights
    weights=$(printf '{"Wd":10,"Wt":%s,"Wo":200,"Wu":0.1,"Wq":500,"Wv":60,"WF":500}' "${wt}")
    python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
      --config "${CONFIG}" --reference-manifest "${MANIFEST}" \
      --output "${output}" --weight Wu --values 0.1 \
      --base-weights-json "${weights}" --evaluation-weights-json "${weights}" \
      --steps 150 --start-fraction 0 --run-full-reference --settle-cycles 0 \
      --initial-error-std-rad 0.001 --measurement-noise-std-rad 0.0005 \
      --acceleration-limit-rad-s2 0 --desired-force-n 20 \
      --object-stiffness-n-m 625 --object-stiffness-min-n-m 500 \
      --object-stiffness-max-n-m 750 --object-damping-n-s-m 0 \
      --object-damping-min-n-s-m 0 --object-damping-max-n-s-m 8 \
      --force-equilibrium-mode runtime_absolute --object-length-m 0.18 \
      --force-delay-cycles 0 --force-delay-min-cycles 0 \
      --force-delay-max-cycles 1 --force-noise-std-n 0.15 \
      --bhat-left 0 --bhat-right 0 --wb-left 20 --wb-right 20 \
      --wk-left "${wk}" --wk-right "${wk}" --k0 500 \
      --peer-delay-min-cycles 0 --peer-delay-max-cycles 1 \
      --actuation-delay-min-cycles 0 --actuation-delay-max-cycles 0 \
      --period-jitter-std-fraction 0.03 --period-jitter-limit-fraction 0.10 \
      --disturbance-start-fraction 0.45 --disturbance-duration-s 0.50 \
      --disturbance-amplitude-rad-s 0.05 \
      --orientation-disturbance-amplitude-rad-s 0.03 --smooth-disturbance \
      --slip-onset-force-n 10 --drop-force-multiplier 1.5 \
      --overforce-limit-n 25 --slip-hold-cycles 1 \
      --overforce-hold-cycles 1 --grace-cycles 0 \
      --terminal-target-error-mm 10 --orientation-rmse-deg 5 \
      --hard-distance-min-m 0.10 --hard-distance-max-m 0.18 \
      --hard-joint-velocity-scale 1 --hard-joint-acceleration-rad-s2 25 \
      --hard-command-jump-rad-s 1.2 --hard-force-slew-n-s 350 \
      --deadline-ms 20 --deadline-miss-hold-cycles 0 --jobs 4 \
      >"${output}.log" 2>&1
}

names=(t140_k1 t140_k2 t160_k1 t160_k2 t180_k1 t180_k2 t200_k1 t200_k2)
wts=(140 140 160 160 180 180 200 200)
wks=(1 2 1 2 1 2 1 2)
for ((batch=0; batch<${#names[@]}; batch+=4)); do
  pids=()
  for ((offset=0; offset<4 && batch+offset<${#names[@]}; offset++)); do
    i=$((batch+offset)); run_profile "${names[i]}" "${wts[i]}" "${wks[i]}" & pids+=("$!")
  done
  for pid in "${pids[@]}"; do wait "${pid}"; done
done

python3 - "${ROOT}" <<'PY'
import json, pathlib, sys
root=pathlib.Path(sys.argv[1])
for name in ('t140_k1','t140_k2','t160_k1','t160_k2','t180_k1','t180_k2','t200_k1','t200_k2'):
 s=json.load(open(root/('aic_wk_wt_boundary_%s_v19'%name)/'result.json'))['summary'][0]
 print(name,'success=%d/%d'%(s['successes'],s['runs']),
       'terminal=%.2fmm'%s['terminal_target_error_mm_mean'],
       'force_min=%.2fN'%s['force_min_n_mean'],
       'force_max=%.2fN'%s['force_max_n_mean'],
       'failures=%s'%s['hard_failure_reason_counts'])
PY
