import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

WS = Path('/home/mas-2/3rd_party/CM_ws')
ROOT = WS / 'src/cm_kirara_ball/generated/grasp_weight_rescan_20260914'
sys.path.insert(0, str(WS / 'src/cm_trajectory_optimization/scripts'))
from cm_trajopt_native_validator import validate_sampled_pair


def main():
    ROOT.mkdir(exist_ok=True)
    out = ROOT / 'input_audit'
    out.mkdir(exist_ok=True)
    candidates = []
    gen = WS / 'src/cm_kirara_ball/generated'
    for tag in ['trajopt_grasp_fixed20_inputs_20260827', 'trajopt_grasp_fixed20_aligned_inputs_20260827']:
        for path in sorted((gen / tag).glob('sample_*/catch_left_planned_path.txt')):
            candidates.append((tag + '_' + path.parent.name, path))
    for sub in ['socket_custom_20260819/catch', 'socket_custom_20260819/final_catch', 'socket_latest', 'distributed_socket_validation_20260810/catch_full']:
        p = gen / sub / 'catch_left_planned_path.txt'
        if p.exists():
            candidates.append((sub.replace('/', '_'), p))
    rows = []
    for label, left in candidates:
        right = left.with_name('catch_right_planned_path.txt')
        ql, qr = [np.atleast_2d(np.loadtxt(p)) for p in (left, right)]
        q = np.hstack((ql, qr))
        dense = []
        for a, b in zip(q[:-1], q[1:]):
            n = max(1, math.ceil(np.max(np.abs(b-a)) / .005))
            dense.extend(a + np.arange(n)[:, None] / n * (b-a))
        dense.append(q[-1])
        dense = np.asarray(dense)
        d = out / label
        d.mkdir(exist_ok=True)
        np.savetxt(d / 'left.txt', dense[:, :6])
        np.savetxt(d / 'right.txt', dense[:, 6:])
        validation = validate_sampled_pair(WS / 'devel/lib/cm_kirara_ball/CM_SOCKET_BALL_VALIDATOR', 'catch', WS / 'src/cm_kirara_ball/config/catch_template.yaml', d / 'left.txt', d / 'right.txt')
        row = dict(label=label, left=str(left), right=str(right), knots=len(q), validation=validation,
                   hash=hashlib.sha256(q.tobytes()).hexdigest(), length=float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()))
        rows.append(row)
        (out / 'audit.json').write_text(json.dumps(rows, indent=2))
        print(label, len(q), validation['valid'], validation['reason_counts'], flush=True)


if __name__ == '__main__':
    main()
