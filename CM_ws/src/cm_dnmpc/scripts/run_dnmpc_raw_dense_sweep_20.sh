#!/usr/bin/env bash
set -euo pipefail

WS=/home/mas-2/3rd_party/CM_ws
EXPERIMENT_BASE=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/54_dnmpc_raw_weight_boundary_20260909
FORMAL=${EXPERIMENT_BASE}/formal_raw_dense_20traj_v1
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
MANIFEST=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/04_dnmpc_reference_pool_final_internal1_margin0p15_wv6_20260828/trajectory_pool_manifest.json
URDF=${WS}/src/cm_kirara_ball/generated/gazebo_newbase_expanded_for_validation.urdf
BASE_WEIGHTS='{"Wd":0.5625,"Wt":100,"Wo":0.5625,"Wu":0.0005625,"Wq":0.5625,"Wv":0.005625,"WF":0.000225}'

source "${WS}/devel/setup.bash"
mkdir -p "${FORMAL}"

run_weight() {
    local weight=$1
    local values=$2
    python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
        --config "${CONFIG}" \
        --reference-manifest "${MANIFEST}" \
        --output "${FORMAL}/${weight}" \
        --trace-output "${FORMAL}/traces_${weight}" \
        --weight "${weight}" --values "${values}" \
        --base-weights-json "${BASE_WEIGHTS}" \
        --evaluation-weights-json "${BASE_WEIGHTS}" \
        --steps 150 --start-fraction 0.42 \
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
        --jobs 6
}

weights=(Wd Wt Wo Wu Wq Wv WF)
values=(
    "0,0.140625,0.28125,0.421875,0.5625,0.703125,0.84375,1.125"
    "0,25,50,75,100,125,150,200"
    "0,0.140625,0.28125,0.421875,0.5625,0.703125,0.84375,1.125"
    "0,0.000140625,0.00028125,0.000421875,0.0005625,0.000703125,0.00084375,0.001125"
    "0,0.140625,0.28125,0.421875,0.5625,0.703125,0.84375,1.125"
    "0,0.00140625,0.0028125,0.00421875,0.005625,0.00703125,0.0084375,0.01125"
    "0,0.00005625,0.0001125,0.00016875,0.000225,0.00028125,0.0003375,0.00045"
)

for ((batch=0; batch<${#weights[@]}; batch+=2)); do
    pids=()
    for ((offset=0; offset<2 && batch+offset<${#weights[@]}; offset++)); do
        index=$((batch+offset))
        run_weight "${weights[index]}" "${values[index]}" \
            >"${FORMAL}/${weights[index]}.log" 2>&1 &
        pids+=("$!")
    done
    for pid in "${pids[@]}"; do
        wait "${pid}"
    done
done

for weight in "${weights[@]}"; do
    python3 "${WS}/src/cm_dnmpc/scripts/validate_dnmpc_trace_collisions.py" \
        --trace-root "${FORMAL}/traces_${weight}" \
        --urdf "${URDF}" \
        --package-dir "${WS}/src" \
        --output "${FORMAL}/${weight}/collision_audit.json" \
        >"${FORMAL}/${weight}/collision.log" 2>&1 &
done
wait

python3 "${WS}/src/cm_dnmpc/scripts/summarize_dnmpc_boundary_with_collisions.py" \
    --root "${FORMAL}" \
    --weights "${weights[@]}" \
    --base-weights-json "${BASE_WEIGHTS}" \
    --terminal-target-error-mm 10 \
    --orientation-rmse-deg 5 \
    --output "${FORMAL}/report_cn"

printf '%s\n' "${FORMAL}"
