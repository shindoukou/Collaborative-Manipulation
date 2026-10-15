
import sys
from pathlib import Path

import rospkg


def run_dnmpc(object_name, side, runtime, object_profile=False):
    dnmpc = Path(rospkg.RosPack().get_path("cm_dnmpc"))
    scripts = dnmpc / "scripts"
    has_explicit_config = any(
        argument == "--config" or argument.startswith("--config=")
        for argument in sys.argv[1:])
    if object_name == "ball" and not has_explicit_config:
        config_name = (
            "ball_sim.json"
            if runtime == "sim"
            else "ball_real.json")
        sys.argv.extend(("--config", str(dnmpc / "config" / config_name)))
    if object_name == "ball" and runtime == "sim":
        ball = Path(rospkg.RosPack().get_path("cm_trajectory_generation"))
        catch_reference_root = ball / "generated" / "ball" / "simulation_reference"
        move_reference_root = catch_reference_root
        reference_options = {
            "--catch-reference": (
                catch_reference_root / "catch" /
                "catch_{}_smoothed_path.txt".format(side)),
            "--move-reference": (
                move_reference_root / "move" /
                "move_{}_smoothed_path.txt".format(side)),
        }
        for option, path in reference_options.items():
            has_explicit_reference = any(
                argument == option or argument.startswith(option + "=")
                for argument in sys.argv[1:])
            if not has_explicit_reference:
                if not path.is_file():
                    raise IOError(
                        "verified Ball simulation reference does not exist: "
                        "{}".format(path))
                sys.argv.extend((option, str(path)))
    sys.path.insert(0, str(scripts))
    if runtime == "sim":
        simulation = dnmpc / "simulation_runtime"
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


