#!/usr/bin/env bash
set -euo pipefail
WS=/home/mas-2/3rd_party/CM_ws
ROOT=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/55_dnmpc_strict_raw_0p1_1000_20260909
MANIFEST=${ROOT}/reference_pool_rrtonly_collision_checked_dynamics_v3/trajectory_pool_manifest.json
CONFIG=${WS}/src/cm_dnmpc/config/controller_raw_no_normalization.json
URDF=${WS}/src/cm_kirara_ball/generated/gazebo_newbase_expanded_for_validation.urdf
AUDITOR=/home/mas-2/3rd_party/CM_experiments/ball_move_transport_dnmpc_20traj_20260827/52_dnmpc_physical_scale_boundary_20260908/formal_final_physical_weights_force5_20260909/provenance/validate_dnmpc_trace_collisions.py
source "${WS}/devel/setup.bash"

run_candidate() {
  local wv=$1 output weights
  output=${ROOT}/relative_wt_x1_candidate_q750_v${wv}_d070_v46
  weights=$(printf '{"Wd":2,"Wt":1000,"Wo":200,"Wu":0.1,"Wq":750,"Wv":%s,"WF":1000}' "${wv}")
  python3 "${WS}/src/cm_dnmpc/scripts/cm_dnmpc_stage3_multi_reference_sweep.py" \
    --config "${CONFIG}" --reference-manifest "${MANIFEST}" \
    --output "${output}" --trace-output "${output}/traces" \
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
    --wk-left 1 --wk-right 1 --k0 500 --max-aic-displacement-m 0.070 \
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
    --deadline-ms 20 --deadline-miss-hold-cycles 0 --jobs 1 \
    >"${output}.log" 2>&1
  python3 "${AUDITOR}" --trace-root "${output}/traces" --urdf "${URDF}" \
    --package-dir "${WS}/src/cm_kirara_ball" \
    --output "${output}/collision_audit.json" --ball-radius-m 0.09 \
    >"${output}/collision_audit.log" 2>&1
}

run_candidate 200 & p1=$!
run_candidate 300 & p2=$!
run_candidate 400 & p3=$!
run_candidate 450 & p4=$!
wait "${p1}"; wait "${p2}"; wait "${p3}"; wait "${p4}"

python3 - "${ROOT}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
for wv in (200, 300, 400, 450):
    output = root / ('relative_wt_x1_candidate_q750_v%d_d070_v46' % wv)
    summary = json.load(open(output / 'result.json'))['summary'][0]
    collision = json.load(open(output / 'collision_audit.json'))
    print('Wv=%d' % wv, 'control=%d/%d' % (summary['successes'], summary['runs']),
          'collision_free=%d/%d' % (collision['collision_free_count'], collision['trace_count']),
          'terminal=%.2fmm' % summary['terminal_target_error_mm_mean'],
          'force=[%.2f,%.2f]N' % (summary['force_min_n_mean'], summary['force_max_n_mean']),
          'failures=%s' % summary['hard_failure_reason_counts'])
PY
