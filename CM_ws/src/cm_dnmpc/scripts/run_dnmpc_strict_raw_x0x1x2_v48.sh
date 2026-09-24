#!/usr/bin/env bash
set -euo pipefail

WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
FORMAL=${ROOT}/formal_raw_x0x1x2_20traj_v48
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
URDF=${WS}/src/cm_kirara_ball/generated/gazebo_newbase_expanded_for_validation.urdf
AUDITOR=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/52_dnmpc_physical_scale_boundary_20260908/formal_final_physical_weights_force5_20260909/provenance/validate_dnmpc_trace_collisions.py
BASE_WEIGHTS='{"Wd":2,"Wt":1000,"Wo":200,"Wu":0.1,"Wq":750,"Wv":300,"WF":1000}'

source "${WS}/devel/setup.bash"
mkdir -p "${FORMAL}"

run_weight() {
  local weight=$1 values=$2
  python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
    --config "${CONFIG}" --reference-manifest "${MANIFEST}" \
    --output "${FORMAL}/${weight}" --trace-output "${FORMAL}/traces_${weight}" \
    --weight "${weight}" --values "${values}" \
    --base-weights-json "${BASE_WEIGHTS}" \
    --evaluation-weights-json "${BASE_WEIGHTS}" \
    --steps 150 --start-fraction 0 --run-full-reference --settle-cycles 0 \
    --initial-error-std-rad 0.001 --measurement-noise-std-rad 0.0005 \
    --acceleration-limit-rad-s2 0 \
    --desired-force-n 20 --object-stiffness-n-m 625 \
    --object-stiffness-min-n-m 500 --object-stiffness-max-n-m 750 \
    --object-damping-n-s-m 0 --object-damping-min-n-s-m 0 \
    --object-damping-max-n-s-m 8 --force-equilibrium-mode runtime_absolute \
    --object-length-m 0.18 --force-delay-cycles 0 \
    --force-delay-min-cycles 0 --force-delay-max-cycles 1 \
    --force-noise-std-n 0.15 \
    --bhat-left 0 --bhat-right 0 --wb-left 20 --wb-right 20 \
    --wk-left 1 --wk-right 1 --k0 500 --max-aic-displacement-m 0.070 \
    --peer-delay-min-cycles 0 --peer-delay-max-cycles 1 \
    --actuation-delay-min-cycles 0 --actuation-delay-max-cycles 0 \
    --period-jitter-std-fraction 0.03 --period-jitter-limit-fraction 0.10 \
    --disturbance-start-fraction 0.45 --disturbance-duration-s 0.50 \
    --disturbance-amplitude-rad-s 0.05 \
    --orientation-disturbance-amplitude-rad-s 0.03 --smooth-disturbance \
    --slip-onset-force-n 10 --drop-force-multiplier 1.5 \
    --overforce-limit-n 25 --slip-hold-cycles 1 --overforce-hold-cycles 1 \
    --grace-cycles 0 --terminal-target-error-mm 10 --orientation-rmse-deg 30 \
    --hard-distance-min-m 0.10 --hard-distance-max-m 0.18 \
    --hard-joint-velocity-scale 1 --hard-joint-acceleration-rad-s2 25 \
    --hard-command-jump-rad-s 1.2 --hard-force-slew-n-s 350 \
    --deadline-ms 20 --deadline-miss-hold-cycles 0 --jobs 4 \
    >"${FORMAL}/${weight}.log" 2>&1
}

weights=(Wd Wt Wo Wu Wq Wv WF)
values=("0,2,4" "0,1000,2000" "0,200,400" "0,0.1,0.2" \
        "0,750,1500" "0,300,600" "0,1000,2000")

for ((batch=0; batch<${#weights[@]}; batch+=2)); do
  pids=()
  for ((offset=0; offset<2 && batch+offset<${#weights[@]}; offset++)); do
    i=$((batch+offset))
    run_weight "${weights[i]}" "${values[i]}" & pids+=("$!")
  done
  for pid in "${pids[@]}"; do wait "${pid}"; done
done

for ((batch=0; batch<${#weights[@]}; batch+=2)); do
  pids=()
  for ((offset=0; offset<2 && batch+offset<${#weights[@]}; offset++)); do
    i=$((batch+offset)); weight=${weights[i]}
    python3 "${AUDITOR}" \
      --trace-root "${FORMAL}/traces_${weight}" --urdf "${URDF}" \
      --package-dir "${WS}/src/cm_kirara_ball" \
      --output "${FORMAL}/${weight}/collision_audit.json" --ball-radius-m 0.09 \
      >"${FORMAL}/${weight}/collision_audit.log" 2>&1 &
    pids+=("$!")
  done
  for pid in "${pids[@]}"; do wait "${pid}"; done
done

python3 "${WS}/src/cm_dnmpc/scripts/summarize_dnmpc_raw_x0x1x2_v48.py" \
  --root "${FORMAL}" --base-weights-json "${BASE_WEIGHTS}" \
  --output "${FORMAL}/report_cn"

echo "${FORMAL}"
