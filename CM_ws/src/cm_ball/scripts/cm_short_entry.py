"""Shared implementation behind the short cm_ball commands."""

import runpy
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
            "controller_ball_sim_verified_20260831.json"
            if runtime == "sim"
            else "controller_ball_real_paper_20260911.json")
        sys.argv.extend(("--config", str(dnmpc / "config" / config_name)))
    if object_name == "ball" and runtime == "sim":
        ball = Path(rospkg.RosPack().get_path("cm_kirara_ball"))
        catch_reference_root = (
            ball / "generated" /
            "trajopt_weight_sweep_grasp_single_knot_strict_repro_20260826_195000" /
            "selected_x1_controller")
        move_reference_root = (
            ball / "generated" /
            "move_canonical_single_insert_20260826_010707")
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
