#!/usr/bin/env bash
set -euo pipefail

SOURCE=/home/mas-2/3rd_party/CM_ws/src/cm_dnmpc/scripts/probe_no_hidden_scale_profiles_v29_low_wt_path_timing.sh
TEMP=$(mktemp)
trap 'rm -f "${TEMP}"' EXIT
sed 's/_v29/_v31/g' "${SOURCE}" >"${TEMP}"
bash "${TEMP}"
