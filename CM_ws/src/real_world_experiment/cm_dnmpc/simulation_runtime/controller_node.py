#!/usr/bin/env python3

import argparse
import datetime
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from controller_core import (
    AICHorizonState,
    LocalDNMPC,
    ReferenceTrajectory,
    load_json,
    resolve_stage_weight_error_scales,
    resolve_stage_weights,
)
from state_protocol import (
    PROTOCOL,
    CartesianHorizon,
    ExchangeState,
    PeerStateCache,
    ProtocolError,
)
from socket_transport import LocalDuplexPredictionSocket, PredictionSocket
from object_profiles import (
    OBJECT_ARGUMENTS,
    canonical_object_id,
    apply_real_object_profile,
    default_profile_path,
    format_side_value,
    joint_position_velocity_command,
    load_real_object_profile,
    profile_reference,
)
from internal_force_projection import (
    ForceFrameCompensator,
    project_dual_arm_forces,
)
from adaptive_impedance import PaperElastic3

try:
    from robotiq_ft_sensor.msg import ft_sensor
except ImportError:
    ft_sensor = None


class JointBuffer:
    def __init__(self, names):
        self.names = list(names)
        self.value = None
        self.lock = threading.Lock()
        self.event = threading.Event()

    def callback(self, message):
        mapping = {
            name: value
            for name, value in zip(message.name, message.position)
        }
        if not all(name in mapping for name in self.names):
            return
        with self.lock:
            self.value = np.asarray(
                [mapping[name] for name in self.names], dtype=float)
        self.event.set()

    def get(self):
        with self.lock:
            return None if self.value is None else self.value.copy()


class ForceBuffer:
    def __init__(self):
        self.value = np.zeros(3)
        self.zero = np.zeros(3)
        self.samples = []
        self.lock = threading.Lock()
        self.event = threading.Event()

    def callback_custom(self, message):
        self._set([message.Fx, message.Fy, message.Fz])

    def callback_wrench(self, message):
        self._set([
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
        ])

    def _set(self, value):
        vector = np.asarray(value, dtype=float)
        if not np.all(np.isfinite(vector)):
            return
        with self.lock:
            self.value = vector
            if len(self.samples) < 25:
                self.samples.append(vector.copy())
        self.event.set()

    def calibrate(self, timeout_s):
        deadline = time.monotonic() + float(timeout_s)
        while (len(self.samples) < 25 and time.monotonic() < deadline and
               not rospy.is_shutdown()):
            self.event.wait(0.05)
            self.event.clear()
        with self.lock:
            if len(self.samples) >= 10:
                self.zero = np.mean(
                    np.asarray(self.samples[5:], dtype=float), axis=0)
            return self.zero.copy(), len(self.samples)

    def get(self):
        with self.lock:
            return self.value.copy() - self.zero


def parse_args(side=None, forced_object=None, forced_runtime=None,
               forced_phase=None):
    parser = argparse.ArgumentParser()
    if side is None:
        parser.add_argument("--side", choices=("left", "right"), required=True)
    else:
        parser.set_defaults(side=side)
    if forced_object is None:
        parser.add_argument(
            "--object", type=canonical_object_id,
            choices=OBJECT_ARGUMENTS, required=True)
    else:
        parser.set_defaults(object=forced_object)
    if forced_runtime is None:
        parser.add_argument(
            "--runtime", choices=("sim", "real"), required=True)
    else:
        parser.set_defaults(runtime=forced_runtime)
    if forced_phase is None:
        parser.add_argument(
            "--phase", choices=("all", "catch", "move"), default="all",
            help="all runs Stage1->Stage2->Stage3->Stage4 in one process")
    else:
        parser.set_defaults(phase=forced_phase)
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--peer-host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=55005)
    parser.add_argument("--receive-port", type=int)
    parser.add_argument("--send-port", type=int)
    parser.add_argument("--reference")
    parser.add_argument("--catch-reference")
    parser.add_argument("--move-reference")
    parser.add_argument("--transition-delay", type=float, default=1.0)
    parser.add_argument("--max-cycles", type=int, default=0)
    parser.add_argument("--config")
    parser.add_argument("--object-profile")
    parser.add_argument("--joint-state-topic")
    parser.add_argument("--command-topic")
    parser.add_argument("--force-topic")
    parser.add_argument(
        "--force-type", choices=("robotiq", "wrench"))
    parser.add_argument("--joint-timeout", type=float, default=10.0)
    parser.add_argument("--force-zero-timeout", type=float, default=5.0)
    parser.add_argument("--command-enabled", action="store_true")
    parser.add_argument(
        "--dry-run", dest="command_enabled", action="store_false")
    parser.set_defaults(command_enabled=False)
    return parser.parse_args(rospy.myargv()[1:])


def apply_private_parameters(args):
    """Apply launch-file private parameters after rospy initialization."""
    args.command_enabled = bool(rospy.get_param(
        "~command_enabled", args.command_enabled))
    args.force_topic = rospy.get_param("~force_topic", args.force_topic)
    args.command_topic = rospy.get_param("~command_topic", args.command_topic)
    args.joint_state_topic = rospy.get_param(
        "~joint_state_topic", args.joint_state_topic)
    args.force_type = rospy.get_param("~force_type", args.force_type)


def package_root(object_name, profile=None):
    if profile is not None:
        package = profile["reference"].get("package")
        if package:
            return Path(rospkg.RosPack().get_path(package))
        return Path(rospkg.RosPack().get_path("cm_dnmpc"))
    if object_name not in ("ball", "box"):
        raise ValueError(
            "object {!r} requires --object-profile".format(object_name))
    package = "cm_trajectory_generation"
    return Path(rospkg.RosPack().get_path(package))


def resolve_run(root, object_name, phase):
    selector = (
        "latest_catch_run.txt"
        if object_name == "box" and phase == "catch"
        else "latest_run.txt")
    path = Path((root / "generated" / object_name / selector).read_text(
        encoding="utf-8").strip())
    if not path.is_dir():
        raise IOError("selected run does not exist: {}".format(path))
    return path


def resolve_reference(root, object_name, phase, side, explicit):
    if explicit:
        return Path(explicit)
    controller_selector = root / "generated" / object_name / "latest_controller_run.txt"
    if controller_selector.is_file():
        controller_run = Path(controller_selector.read_text(
            encoding="utf-8").strip())
        candidates = (
            controller_run / phase /
            "{}_{}_smoothed_path.txt".format(phase, side),
            controller_run / phase /
            "{}_{}_planned_path.txt".format(phase, side),
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate
    run = resolve_run(root, object_name, phase)
    matches = sorted(
        run.glob("{}_{}_samples_*.txt".format(phase, side)))
    if not matches:
        raise IOError(
            "no {} {} reference in {}".format(phase, side, run))
    return matches[-1]


def publish_command(publisher, names, q, velocity, dt):
    message = JointTrajectory()
    message.header.stamp = rospy.Time.now()
    message.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions, point.velocities = joint_position_velocity_command(
        q, velocity, dt)
    point.time_from_start = rospy.Duration.from_sec(float(dt))
    message.points = [point]
    publisher.publish(message)


def publish_prediction_command(publisher, names, joint_prediction,
                               controls, dt, max_points, start_step=1,
                               time_scale=1.0):
    joint_prediction = np.asarray(joint_prediction, dtype=float)
    controls = np.asarray(controls, dtype=float)
    if (joint_prediction.shape != (controls.shape[0] + 1, 6) or
            controls.shape[1] != 6):
        raise ValueError("invalid prediction command shape")
    time_scale = float(time_scale)
    if not np.isfinite(time_scale) or time_scale < 1.0:
        raise ValueError("simulation command time_scale must be finite and >= 1")
    message = JointTrajectory()
    message.header.stamp = rospy.Time.now()
    message.joint_names = list(names)
    start_step = int(start_step)
    if start_step < 1 or start_step > controls.shape[0]:
        raise ValueError("start_step must select a prediction step")
    command_count = min(
        int(max_points), controls.shape[0] - start_step + 1)
    if command_count < 1:
        raise ValueError("max_points must be positive")
    for offset in range(command_count):
        prediction_index = start_step + offset
        # Keep position and velocity consistent when lengthening the execution
        # window; the NMPC prediction and reference trajectory remain unchanged.
        velocity = controls[prediction_index - 1] / time_scale
        point = JointTrajectoryPoint()
        point.positions = joint_prediction[prediction_index].tolist()
        point.velocities = velocity.tolist()
        point.time_from_start = rospy.Duration.from_sec(
            float(prediction_index) * float(dt) * time_scale)
        message.points.append(point)
    publisher.publish(message)


def simulation_command_time_scale(config, object_name, stage_id):
    settings = config.get("sim_stage_commands", {}).get(object_name, {})
    value = float(settings.get(str(int(stage_id)), {}).get("time_scale", 1.0))
    if not np.isfinite(value) or value < 1.0:
        raise ValueError("simulation command time_scale must be finite and >= 1")
    return value


def simulated_clamp_force(distance_m, object_length_m, stiffness_n_m):
    return max(
        0.0,
        (float(object_length_m) - float(distance_m)) *
        float(stiffness_n_m))


def stage2_sample_is_stable(endpoint_error_m, left_force_n, right_force_n,
                            settings):
    desired = float(settings["desired_force_n"])
    tolerance = float(settings["force_tolerance_n"])
    return bool(
        float(endpoint_error_m) <=
        float(settings["endpoint_tolerance_m"]) and
        abs(float(left_force_n) - desired) <= tolerance and
        abs(float(right_force_n) - desired) <= tolerance)


def stage_solver_config(config, object_name, stage_id, runtime, side=None):
    """Use reference tracking before contact and peer tracking after approach."""
    config = dict(config)
    config["position_tracking_mode"] = (
        "reference" if int(stage_id) == 1 else "peer_distance")
    if runtime != "sim" or int(stage_id) != 2:
        return config
    settings = config.get("stage2", {}).get(object_name) or {}
    limit = settings.get("input_max_rad_s")
    if side in ("left", "right"):
        limit = settings.get("input_max_rad_s_" + side, limit)
    if limit is None:
        return config
    limit = np.asarray(limit, dtype=float)
    base = np.asarray(config["input_max_rad_s"], dtype=float)
    if (limit.shape != (6,) or base.shape != (6,) or
            not np.all(np.isfinite(limit)) or
            not np.all(np.isfinite(base)) or
            np.any(limit <= 0.0) or np.any(limit > base)):
        raise ValueError(
            "Stage 2 input_max_rad_s must contain six finite positive "
            "limits no larger than the global limits")
    resolved = dict(config)
    resolved["input_max_rad_s"] = limit.tolist()
    return resolved


def make_elastic(config, object_name, phase, dt):
    return PaperElastic3(
        dt=dt, **config["paper_elastic"][object_name][phase])


def make_exchange_state(side, sequence, reference_time, duration,
                        current_position, current_orientation, prediction,
                        force_common, force_sensor_zeroed,
                        stage_id=0, stage_ready=False):
    return ExchangeState(
        side=side,
        sequence=sequence,
        wall_time=time.time(),
        reference_time=reference_time,
        trajectory_duration=duration,
        current_position=current_position,
        current_orientation=current_orientation,
        prediction=prediction,
        force_common=force_common,
        force_sensor_zeroed=force_sensor_zeroed,
        stage_id=stage_id,
        stage_ready=stage_ready)


def parse_peer(message, peer_side, horizon_count):
    if message is None:
        return None
    return ExchangeState.from_message(
        message, expected_side=peer_side, horizon_count=horizon_count)


def main(forced_side=None, forced_object=None, forced_runtime=None,
         auto_real_profile=False, forced_phase=None):
    args = parse_args(
        forced_side, forced_object, forced_runtime, forced_phase)
    args.object = canonical_object_id(args.object)
    side = args.side
    dnmpc_root = Path(rospkg.RosPack().get_path("cm_dnmpc"))
    profile = None
    profile_path = args.object_profile
    if auto_real_profile and not profile_path:
        profile_path = default_profile_path(dnmpc_root, args.object)
    if profile_path:
        profile = load_real_object_profile(profile_path, args.object)
    rospy.init_node(
        "cm_{}_{}_{}_controller".format(
            args.object, args.runtime, side),
        anonymous=False)
    apply_private_parameters(args)
    if profile is not None:
        io_values = profile["real_io"]
        args.joint_state_topic = (
            args.joint_state_topic or
            format_side_value(io_values["joint_state_topic"], side))
        args.command_topic = (
            args.command_topic or
            format_side_value(io_values["command_topic"], side))
        args.force_topic = (
            args.force_topic or
            format_side_value(io_values["force_topic"], side))
        args.force_type = args.force_type or io_values["force_type"]
        rospy.loginfo(
            "CM real-object profile=%s object=%s hardware_validated=%s",
            profile["_profile_path"], args.object,
            bool(profile.get("status", {}).get("hardware_validated", False)))
    args.joint_state_topic = args.joint_state_topic or "/joint_states"
    args.force_type = args.force_type or "robotiq"
    root = package_root(args.object, profile)
    config_path = (
        Path(args.config)
        if args.config
        else dnmpc_root / "config" / "controller.json")
    config = load_json(config_path)
    if profile is not None:
        config = apply_real_object_profile(config, profile)
    dt = float(config["Ts"])
    horizon = int(config["horizon"])
    horizon_count = horizon + 1
    if args.phase == "all" and args.reference:
        raise ValueError(
            "--reference is only for legacy single-phase runs; use "
            "--catch-reference and --move-reference with --phase all")
    requested_phases = (
        ("catch", "move") if args.phase == "all" else (args.phase,))
    explicit_references = {
        "catch": args.catch_reference,
        "move": args.move_reference,
    }
    if profile is not None:
        for phase in ("catch", "move"):
            if not explicit_references[phase]:
                explicit_references[phase] = profile_reference(
                    profile, phase, side)
    if args.phase in ("catch", "move") and args.reference:
        explicit_references[args.phase] = args.reference
    reference_paths = {}
    references = {}
    reference_strides = config.get("reference_stride", {}).get(
        args.object, {})
    for phase in requested_phases:
        if profile is not None and not explicit_references[phase]:
            raise ValueError(
                "{} profile does not bundle trajectories; provide "
                "--{}-reference for the {} arm".format(
                    args.object, phase, side))
        reference_paths[phase] = resolve_reference(
            root, args.object, phase, side, explicit_references[phase])
        stride = int(reference_strides.get(phase, 1))
        references[phase] = ReferenceTrajectory(
            reference_paths[phase], config["source_dt"], stride=stride)
        rospy.loginfo(
            "CM DNMPC %s %s reference=%s raw_points=%d points=%d "
            "stride=%d duration=%.3f s",
            side, phase, reference_paths[phase],
            references[phase].raw_count,
            references[phase].q.shape[0], references[phase].stride,
            references[phase].duration)
    active_phase = requested_phases[0]
    own_reference_path = reference_paths[active_phase]
    own_reference = references[active_phase]
    peer_side = "right" if side == "left" else "left"

    names = ["{}_{}_joint".format(side, name) for name in (
        "shoulder_pan", "shoulder_lift", "elbow",
        "wrist_1", "wrist_2", "wrist_3")]
    joints = JointBuffer(names)
    rospy.Subscriber(
        args.joint_state_topic, JointState, joints.callback, queue_size=1)
    force = ForceBuffer()
    force_topic = (
        args.force_topic or
        "/{}_ft300/robotiq_ft_sensor".format(side))
    if force_topic:
        if args.force_type == "robotiq" and ft_sensor is not None:
            rospy.Subscriber(
                force_topic, ft_sensor, force.callback_custom, queue_size=1)
        else:
            rospy.Subscriber(
                force_topic, WrenchStamped,
                force.callback_wrench, queue_size=1)
    if not joints.event.wait(float(args.joint_timeout)):
        raise RuntimeError("joint_states timeout")

    if args.runtime == "real":
        if args.force_type == "robotiq" and ft_sensor is None:
            raise RuntimeError(
                "real controller requires robotiq_ft_sensor messages")
        zero, zero_count = force.calibrate(float(args.force_zero_timeout))
        if zero_count < 10:
            raise RuntimeError(
                "real FT300 initialization failed on {}: only {} samples".format(
                    force_topic, zero_count))
        rospy.loginfo(
            "CM %s REAL %s FT300 zero=%s samples=%d",
            args.object, side, zero.tolist(), zero_count)
    else:
        zero = np.zeros(3)
        zero_count = 0
        rospy.loginfo(
            "CM %s SIM %s: FT300 zero calibration disabled",
            args.object, side)

    q = joints.get()
    local_start_time = own_reference.closest_time(q)
    initial_stage_id = 1 if active_phase == "catch" else 3
    solver_cache = {}

    def solver_for_stage(requested_stage_id):
        requested_stage_id = int(requested_stage_id)
        if requested_stage_id not in solver_cache:
            stage_weights = resolve_stage_weights(
                config, args.object, requested_stage_id)
            stage_weight_error_scales = resolve_stage_weight_error_scales(
                config, args.object, requested_stage_id)
            solver_cache[requested_stage_id] = LocalDNMPC(
                side, horizon, stage_solver_config(
                    config, args.object, requested_stage_id, args.runtime, side),
                weights=stage_weights,
                weight_error_scales=stage_weight_error_scales,
                solver_label="stage{}".format(requested_stage_id))
        return solver_cache[requested_stage_id]

    solver = solver_for_stage(initial_stage_id)
    projection_values = config["force_projection"]
    side_projection = projection_values[side]
    force_compensator = ForceFrameCompensator(
        sensor_rpy_in_ee=side_projection["sensor_rpy_in_ee"],
        tool_mass_kg=side_projection["tool_mass_kg"],
        gravity_common_m_s2=projection_values["gravity_common_m_s2"])
    own_position, own_rotation = solver.frame(q)
    own_orientation = own_rotation[:, 2]
    force_compensator.set_zero_orientation(own_rotation)
    elastic_values = config["paper_elastic"][args.object][active_phase]
    elastic = make_elastic(config, args.object, active_phase, dt)

    socket_values = config["socket"]
    if args.runtime == "sim":
        default_receive_port = 55005 if side == "left" else 55006
        default_send_port = 55006 if side == "left" else 55005
        receive_port = args.receive_port or default_receive_port
        send_port = args.send_port or default_send_port
        channel = LocalDuplexPredictionSocket(
            bind_host="127.0.0.1",
            peer_host="127.0.0.1",
            receive_port=receive_port,
            send_port=send_port,
            timeout_s=max(0.5, float(socket_values["timeout_s"])),
            reconnect_s=socket_values["reconnect_s"])
        rospy.loginfo(
            "CM DNMPC %s protocol=%s local-duplex TCP "
            "receive=127.0.0.1:%d send=127.0.0.1:%d",
            side, PROTOCOL, receive_port, send_port)
    else:
        role = "server" if side == "left" else "client"
        channel = PredictionSocket(
            role=role,
            bind_host=args.bind_host,
            peer_host=args.peer_host,
            port=args.port,
            timeout_s=socket_values["timeout_s"],
            reconnect_s=socket_values["reconnect_s"])
        rospy.loginfo(
            "CM DNMPC %s protocol=%s waiting for TCP %s:%d as %s",
            side, PROTOCOL, args.peer_host, args.port, role)
    if not channel.wait_connected(rospy.is_shutdown):
        return

    command_topic = (
        args.command_topic or
        "/{}_trajectory_controller/command".format(side))
    publisher = rospy.Publisher(
        command_topic, JointTrajectory, queue_size=1)
    command_enabled = bool(args.command_enabled)

    initial_joint_prediction = own_reference.horizon(
        local_start_time, horizon_count, dt)[0]
    initial_joint_prediction[0] = q
    initial_positions, initial_orientations = solver.cartesian_horizon(
        initial_joint_prediction)
    last_prediction = CartesianHorizon(
        initial_positions, initial_orientations)
    initial_force_sensor = force.get()
    initial_force_common = force_compensator.to_common(
        initial_force_sensor, own_rotation)
    initial_state = make_exchange_state(
        side=side,
        sequence=-1,
        reference_time=local_start_time,
        duration=own_reference.duration,
        current_position=own_position,
        current_orientation=own_orientation,
        prediction=last_prediction,
        force_common=initial_force_common,
        force_sensor_zeroed=initial_force_sensor)
    initial_message = channel.exchange(initial_state.to_message())
    try:
        initial_peer = parse_peer(
            initial_message, peer_side, horizon_count)
    except ProtocolError as error:
        raise RuntimeError(
            "initial Cartesian state exchange failed: {}".format(error))
    if initial_peer is None:
        raise RuntimeError("initial Cartesian state exchange timed out")
    if args.runtime == "sim":
        channel.set_timeout(socket_values["timeout_s"])

    peer_cache = PeerStateCache()
    peer_cache.update(initial_peer)
    reference_start_time = min(local_start_time, initial_peer.reference_time)
    total_duration = max(
        own_reference.duration, initial_peer.trajectory_duration)
    stage2_values = config.get("stage2", {}).get(args.object)
    stage4_values = config.get("stage4", {}).get(args.object)
    stage2_required = bool("catch" in requested_phases and stage2_values)
    stage4_required = bool("move" in requested_phases and stage4_values)
    if "catch" in requested_phases and not stage2_required:
        raise RuntimeError("Stage 2 settings are required for {}".format(
            args.object))
    if "move" in requested_phases and not stage4_required:
        raise RuntimeError("Stage 4 settings are required for {}".format(
            args.object))
    stage4_terminal_inward_shift_m = float(
        stage4_values.get("terminal_inward_shift_m", 0.0)
        if stage4_values else 0.0)
    if (not math.isfinite(stage4_terminal_inward_shift_m) or
            stage4_terminal_inward_shift_m < 0.0):
        raise RuntimeError(
            "Stage 4 terminal_inward_shift_m must be finite and nonnegative")
    if (stage4_values and stage4_terminal_inward_shift_m >
            float(stage4_values["endpoint_tolerance_m"])):
        raise RuntimeError(
            "Stage 4 terminal_inward_shift_m exceeds endpoint tolerance")
    stage_id = initial_stage_id
    stage_started_at = time.monotonic()
    reference_delay_s = 0.0
    stage2_started_at = None
    stage2_elapsed = 0.0
    stage2_stable_cycles = 0
    stage2_success = False
    stage2_timed_out = False
    stage4_started_at = None
    stage4_elapsed = 0.0
    stage4_stable_cycles = 0
    stage4_success = False
    stage4_timed_out = False
    stage4_terminal_target = None
    stage2_joint_target = (
        references["move"].q[0].copy()
        if "move" in requested_phases else own_reference.q[-1].copy())
    endpoint_position = solver.pose(own_reference.q[-1])[0]
    last_left_to_right = None
    sequence = 0
    log_root = Path("/home/mas-2/3rd_party/CM_ws/logs") / args.runtime
    log_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    log_path = log_root / "{}_{}_{}_{}_{}.jsonl".format(
        args.object, args.runtime, args.phase, side, stamp)

    started = time.monotonic()
    rate = rospy.Rate(1.0 / dt)
    with log_path.open("w", encoding="utf-8") as log:
        while not rospy.is_shutdown():
            q = joints.get()
            if q is None:
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
                continue
            now = time.monotonic()
            elapsed = now - started
            active_elapsed = max(
                0.0, now - stage_started_at - reference_delay_s)
            reference_time = min(
                reference_start_time + active_elapsed,
                total_duration)
            qref, qdref_full = own_reference.horizon(
                reference_time, horizon_count, dt)
            if (stage2_required and stage_id == 1 and
                    reference_time >= total_duration):
                stage_id = 2
                solver = solver_for_stage(stage_id)
                stage2_started_at = time.monotonic()
                endpoint_position = solver.pose(stage2_joint_target)[0]
                elastic.reset()
                rospy.loginfo(
                    "CM DNMPC %s Stage 1 completed; entering Stage 2 "
                    "desired_force=%.3f N tolerance=%.3f N",
                    side, stage2_values["desired_force_n"],
                    stage2_values["force_tolerance_n"])
            if stage_id == 3 and reference_time >= total_duration:
                stage_id = 4
                solver = solver_for_stage(stage_id)
                stage4_started_at = time.monotonic()
                stage4_terminal_target = None
                rospy.loginfo(
                    "CM DNMPC %s Stage 3 completed; entering Stage 4 "
                    "desired_force=%.3f N tolerance=%.3f N",
                    side, stage4_values["desired_force_n"],
                    stage4_values["force_tolerance_n"])
            if stage_id == 2:
                stage2_elapsed = time.monotonic() - stage2_started_at
                qref = np.repeat(
                    stage2_joint_target[None, :], horizon_count, axis=0)
                qdref_full = np.zeros((horizon_count, 6))
            elif stage_id == 4:
                stage4_elapsed = time.monotonic() - stage4_started_at
                qref = np.repeat(
                    own_reference.q[-1][None, :], horizon_count, axis=0)
                qdref_full = np.zeros((horizon_count, 6))
            own_position, own_rotation = solver.frame(q)
            own_orientation = own_rotation[:, 2]
            force_sensor_value = force.get()
            force_common_value = force_compensator.to_common(
                force_sensor_value, own_rotation)
            outbound_stage_ready = bool(
                (stage_id == 2 and stage2_success) or
                (stage_id == 4 and stage4_success))
            outbound_state = make_exchange_state(
                side=side,
                sequence=sequence,
                reference_time=reference_time,
                duration=own_reference.duration,
                current_position=own_position,
                current_orientation=own_orientation,
                prediction=last_prediction,
                force_common=force_common_value,
                force_sensor_zeroed=force_sensor_value,
                stage_id=stage_id,
                stage_ready=outbound_stage_ready)
            incoming_message = channel.exchange(outbound_state.to_message())
            if incoming_message is not None:
                try:
                    peer_cache.update(parse_peer(
                        incoming_message, peer_side, horizon_count))
                except ProtocolError as error:
                    rospy.logwarn_throttle(
                        1.0, "DNMPC rejected peer state: %s", error)
            peer = peer_cache.state
            stale_s = peer_cache.stale_s()

            if side == "left":
                force_state = project_dual_arm_forces(
                    own_position, peer.current_position,
                    force_common_value, peer.force_common,
                    minimum_separation_m=projection_values[
                        "minimum_separation_m"],
                    use_absolute=projection_values["use_absolute"],
                    fallback_left_to_right=last_left_to_right)
                own_clamp_force = force_state["left_force_n"]
            else:
                force_state = project_dual_arm_forces(
                    peer.current_position, own_position,
                    peer.force_common, force_common_value,
                    minimum_separation_m=projection_values[
                        "minimum_separation_m"],
                    use_absolute=projection_values["use_absolute"],
                    fallback_left_to_right=last_left_to_right)
                own_clamp_force = force_state["right_force_n"]
            if force_state["direction_valid"]:
                last_left_to_right = force_state["left_to_right"].copy()

            solver_terminal_target = endpoint_position
            if (stage_id == 4 and args.runtime == "sim" and
                    stage4_terminal_inward_shift_m > 0.0):
                if stage4_terminal_target is None:
                    inward_direction = force_state["left_to_right"].copy()
                    if side == "right":
                        inward_direction *= -1.0
                    stage4_terminal_target = (
                        endpoint_position + inward_direction *
                        stage4_terminal_inward_shift_m)
                solver_terminal_target = stage4_terminal_target

            force_scale_n_m = solver.force_stiffness_n_m
            if stage_id in (2, 3, 4) and args.runtime == "sim":
                force_settings = (
                    stage2_values if stage_id == 2 else stage4_values)
                force_scale_n_m = force_settings["sim_stiffness_n_m"]
                synthetic_force = simulated_clamp_force(
                    force_state["distance_m"],
                    elastic_values["object_length_m"],
                    force_settings["sim_stiffness_n_m"])
                force_state["left_force_n"] = synthetic_force
                force_state["right_force_n"] = synthetic_force
                force_state["internal_force_n"] = synthetic_force
                force_state["motion_force_n"] = 0.0
                own_clamp_force = synthetic_force

            if stage_id in (2, 3, 4):
                _, elastic_state = elastic.update(
                    own_position, peer.current_position, own_clamp_force)
                elastic_state["active"] = True
            else:
                elastic_state = {
                    "active": False,
                    "stage": stage_id,
                    "distance_m": force_state["distance_m"],
                }
            aic_state = AICHorizonState(
                active=stage_id in (2, 3, 4),
                object_length_m=elastic_values["object_length_m"],
                measured_force_n=own_clamp_force,
                desired_force_n=elastic_values["desired_force_n"],
                b_hat=(elastic_state["b_hat"]
                       if elastic_state["active"] else elastic.b_hat),
                k_hat=(elastic_state["k_hat"]
                       if elastic_state["active"] else elastic.k_hat),
                current_e_m=(elastic_state["e"]
                             if elastic_state["active"] else
                             0.5 * (force_state["distance_m"]
                                    - elastic_values["object_length_m"])),
                max_displacement_m=elastic_values["max_displacement_m"],
                desired_displacement_m=(
                    elastic_state["displacement"]
                    if elastic_state["active"] else 0.0),
                force_scale_n_m=force_scale_n_m)
            solution = solver.solve(
                q,
                qref,
                qdref_full[:horizon],
                peer.prediction.position,
                peer.prediction.orientation,
                aic_state,
                terminal_target=solver_terminal_target)
            last_prediction = CartesianHorizon(
                solution.position_prediction,
                solution.orientation_prediction)
            safe_peer = stale_s <= float(socket_values["stale_limit_s"])
            command_sent = bool(
                command_enabled and safe_peer and solution.success)
            if command_sent:
                if args.runtime == "sim":
                    publish_prediction_command(
                        publisher, names, solution.joint_prediction,
                        solution.controls, dt,
                        config.get("sim_command_points", 2),
                        config.get("sim_command_start_step", 1),
                        simulation_command_time_scale(config, args.object, stage_id))
                else:
                    publish_command(
                        publisher, names, q, solution.controls[0], dt)

            endpoint_error = float(np.linalg.norm(
                own_position - endpoint_position))
            stage2_sample_stable = False
            if stage_id == 2:
                stage2_sample_stable = stage2_sample_is_stable(
                    endpoint_error,
                    force_state["left_force_n"],
                    force_state["right_force_n"],
                    stage2_values)
                if stage2_sample_stable:
                    stage2_stable_cycles += 1
                else:
                    stage2_stable_cycles = 0
                stage2_success = (
                    stage2_stable_cycles >=
                    int(stage2_values["stable_cycles"]))
                stage2_timed_out = (
                    stage2_elapsed >=
                    float(stage2_values["max_duration_s"]))
            stage4_sample_stable = False
            if stage_id == 4:
                stage4_sample_stable = stage2_sample_is_stable(
                    endpoint_error,
                    force_state["left_force_n"],
                    force_state["right_force_n"],
                    stage4_values)
                if stage4_sample_stable:
                    stage4_stable_cycles += 1
                else:
                    stage4_stable_cycles = 0
                stage4_success = (
                    stage4_stable_cycles >=
                    int(stage4_values["stable_cycles"]))
                stage4_timed_out = (
                    stage4_elapsed >=
                    float(stage4_values["max_duration_s"]))

            record = {
                "controller_revision": "CM_UNIFIED_SIM_STAGE_TRACKING_20261015",
                "position_tracking_mode": solver.position_tracking_mode,
                "force_cost_residual_units": "m",
                "force_model_stiffness_n_m": force_scale_n_m,
                "protocol": PROTOCOL,
                "sequence": sequence,
                "ros_time": rospy.Time.now().to_sec(),
                "reference_time": reference_time,
                "reference_path": str(own_reference_path),
                "trajectory_duration": own_reference.duration,
                "reference_stride": own_reference.stride,
                "reference_raw_count": own_reference.raw_count,
                "effective_trajectory_duration": own_reference.duration,
                "active_phase": active_phase,
                "stage_id": stage_id,
                "active_weights": dict(solver.weights),
                "stage2_elapsed_s": stage2_elapsed,
                "stage2_endpoint_error_m": endpoint_error,
                "stage2_sample_stable": stage2_sample_stable,
                "stage2_stable_cycles": stage2_stable_cycles,
                "stage2_success": stage2_success,
                "stage2_timed_out": stage2_timed_out,
                "stage4_elapsed_s": stage4_elapsed,
                "stage4_endpoint_error_m": endpoint_error,
                "stage4_sample_stable": stage4_sample_stable,
                "stage4_stable_cycles": stage4_stable_cycles,
                "stage4_success": stage4_success,
                "stage4_timed_out": stage4_timed_out,
                "stage4_terminal_inward_shift_m": (
                    stage4_terminal_inward_shift_m),
                "trajectory_endpoint_position": endpoint_position.tolist(),
                "solver_terminal_target": solver_terminal_target.tolist(),
                "solved": solution.success,
                "solve_ms": solution.solve_ms,
                "peer_stale_s": stale_s,
                "peer_sequence": peer.sequence,
                "outbound_stage_ready": outbound_stage_ready,
                "peer_stage_id": peer.stage_id,
                "peer_stage_ready": peer.stage_ready,
                "command_enabled": command_enabled,
                "command_sent": command_sent,
                "q": q.tolist(),
                "u": solution.controls[0].tolist(),
                "current_position": own_position.tolist(),
                "current_orientation": own_orientation.tolist(),
                "peer_current_position": peer.current_position.tolist(),
                "peer_current_orientation": peer.current_orientation.tolist(),
                "prediction_horizon": horizon,
                "sim_command_points": config.get("sim_command_points", 2),
                "sim_command_start_step": config.get(
                    "sim_command_start_step", 1),
                "sim_command_time_scale": (
                    simulation_command_time_scale(config, args.object, stage_id)
                    if args.runtime == "sim" else 1.0),
                "exchanged_joint_state": False,
                "force_sensor_zeroed": force_sensor_value.tolist(),
                "force_common": force_common_value.tolist(),
                "peer_force_common": peer.force_common.tolist(),
                "clamp_force_n": own_clamp_force,
                "left_clamp_force_n": force_state["left_force_n"],
                "right_clamp_force_n": force_state["right_force_n"],
                "internal_force_n": force_state["internal_force_n"],
                "motion_force_n": force_state["motion_force_n"],
                "clamp_distance_m": force_state["distance_m"],
                "clamp_direction_valid": force_state["direction_valid"],
                "left_to_right": force_state["left_to_right"].tolist(),
                "elastic": elastic_state,
                "socket_messages": channel.messages,
                "socket_bytes_sent": channel.bytes_sent,
                "socket_bytes_received": channel.bytes_received,
            }
            log.write(json.dumps(record, separators=(",", ":")) + "\n")
            if sequence % 25 == 0:
                log.flush()
            sequence += 1
            if args.max_cycles > 0 and sequence >= args.max_cycles:
                break
            if stage_id == 2:
                if stage2_timed_out:
                    break
                if (stage2_success and outbound_stage_ready and
                        peer.stage_id == 2 and peer.stage_ready):
                    rospy.loginfo(
                        "CM DNMPC %s Stage 2 grasp completed in %.3f s "
                        "after %d stable cycles",
                        side, stage2_elapsed, stage2_stable_cycles)
                    if "move" not in requested_phases:
                        break
                    active_phase = "move"
                    own_reference_path = reference_paths[active_phase]
                    own_reference = references[active_phase]
                    elastic_values = config["paper_elastic"][
                        args.object][active_phase]
                    elastic = make_elastic(
                        config, args.object, active_phase, dt)
                    stage_id = 3
                    solver = solver_for_stage(stage_id)
                    stage_started_at = time.monotonic()
                    reference_delay_s = max(0.0, args.transition_delay)
                    reference_start_time = 0.0
                    total_duration = own_reference.duration
                    endpoint_position = solver.pose(
                        own_reference.q[-1])[0]
                    stage4_terminal_target = None
                    rospy.loginfo(
                        "CM DNMPC %s entering Stage 3 after %.3f s hold; "
                        "Move reference=%s raw_points=%d points=%d "
                        "stride=%d duration=%.3f s",
                        side, reference_delay_s, own_reference_path,
                        own_reference.raw_count, own_reference.q.shape[0],
                        own_reference.stride, own_reference.duration)
            if stage_id == 4 and stage4_timed_out:
                break
            if (stage_id == 4 and stage4_success and
                    outbound_stage_ready and peer.stage_id == 4 and
                    peer.stage_ready):
                break
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
    channel.close()
    if stage2_required and stage2_timed_out:
            rospy.logerr(
                "CM DNMPC %s Stage 2 timed out after %.3f s; "
                "Catch is not valid",
                side, stage2_elapsed)
    if stage4_required:
        if stage4_success:
            rospy.loginfo(
                "CM DNMPC %s Stage 4 completed in %.3f s after %d "
                "stable cycles",
                side, stage4_elapsed, stage4_stable_cycles)
        elif stage4_timed_out:
            rospy.logerr(
                "CM DNMPC %s Stage 4 timed out after %.3f s; "
                "Move is not valid",
                side, stage4_elapsed)
    rospy.loginfo(
        "CM DNMPC %s completed protocol=%s log=%s",
        side, PROTOCOL, log_path)


if __name__ == "__main__":
    main()
