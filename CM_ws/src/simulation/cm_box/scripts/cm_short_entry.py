
import json
import sys
from pathlib import Path

import rospkg


def configure_sim_defaults(side, argv, packages=None):
    """Use the verified Box-only simulation profile unless explicitly overridden."""
    def supplied(option):
        return any(value == option or value.startswith(option + "=")
                   for value in argv[1:])

    if supplied("--config"):
        return
    packages = packages or rospkg.RosPack()
    profile = Path(packages.get_path("cm_box")) / "config" / "sim.json"
    with profile.open() as stream:
        values = json.load(stream)
    reference = values["simulation_reference"]
    directory = (Path(packages.get_path(reference["package"])) /
                 reference["directory"])
    additions = ["--config", str(profile)]
    for phase in ("catch", "move"):
        option = "--{}-reference".format(phase)
        if supplied(option):
            continue
        path = directory / phase / "{}_{}_smoothed_path.txt".format(phase, side)
        if not path.is_file():
            raise IOError("Box simulation reference missing: {}".format(path))
        additions.extend((option, str(path)))
    argv.extend(additions)


def run_dnmpc(object_name, side, runtime, object_profile=False):
    if object_name == "box" and runtime == "sim":
        configure_sim_defaults(side, sys.argv)
    scripts = Path(rospkg.RosPack().get_path("cm_dnmpc")) / "scripts"
    sys.path.insert(0, str(scripts))
    if runtime == "sim":
        simulation = scripts.parent / "simulation_runtime"
        if not (simulation / "controller_node.py").is_file():
            raise IOError("Unified simulation runtime is missing: {}".format(simulation))
        sys.path.insert(0, str(simulation))
    from controller_node import main
    main(
        forced_side=side,
        forced_object=object_name,
        forced_runtime=runtime,
        auto_real_profile=bool(object_profile),
        forced_phase="all")


