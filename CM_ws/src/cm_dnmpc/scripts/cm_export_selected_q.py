#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import numpy as np


ROOT = Path("/home/mas-2/3rd_party/CM_ws/src")


def selected_run(package, object_name, phase):
    selector = "latest_catch_run.txt" if object_name == "box" and phase == "catch" else "latest_run.txt"
    return Path((package / "generated" / selector).read_text(encoding="utf-8").strip())


def load_reference(package, object_name, phase, side):
    run = selected_run(package, object_name, phase)
    matches = sorted(run.glob("{}_{}_samples_*.txt".format(phase, side)))
    if not matches:
        raise IOError("missing {} {} reference in {}".format(phase, side, run))
    values = np.atleast_2d(np.loadtxt(str(matches[-1]), dtype=float))
    if values.shape[1] != 6 or not np.all(np.isfinite(values)):
        raise ValueError("invalid reference {}".format(matches[-1]))
    return run, matches[-1], values


parser = argparse.ArgumentParser()
parser.add_argument("--output", default="/home/mas-2/3rd_party/CM_ws/reference_q.json")
args = parser.parse_args()
result = {}
for object_name, package_name in (("ball", "cm_kirara_ball"), ("box", "cm_kirara_box")):
    package = ROOT / package_name
    result[object_name] = {}
    for phase in ("catch", "move"):
        result[object_name][phase] = {}
        for side in ("left", "right"):
            run, source, values = load_reference(package, object_name, phase, side)
            result[object_name][phase][side] = {
                "run_dir": str(run),
                "source": str(source),
                "points": int(values.shape[0]),
                "initial_q": values[0].tolist(),
                "terminal_q": values[-1].tolist()
            }
output = Path(args.output)
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(str(output))
