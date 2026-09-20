"""Shared implementation behind the short cm_box commands."""

import json
import runpy
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
    from cm_dnmpc_arm_node import main
    main(
        forced_side=side,
        forced_object=object_name,
        forced_runtime=runtime,
        auto_real_profile=bool(object_profile),
        forced_phase="all")


def run_legacy(package, script):
    scripts = Path(rospkg.RosPack().get_path(package)) / "scripts"
    path = scripts / script
    if not path.is_file():
        raise IOError("legacy entry does not exist: {}".format(path))
    sys.path.insert(0, str(scripts))
    runpy.run_path(str(path), run_name="__main__")
