#!/usr/bin/env python3
import argparse
import copy
import datetime
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml


TARGET_OBJECTS = ('ball', 'box', 'basketball', 'foam_frame', 'plush_toy',
                  'l_shaped_wire_mesh', 'clothes_filled_bag')


def planning_spec(package, target_object=None):
    target = target_object or 'ball'
    if target not in TARGET_OBJECTS:
        raise ValueError('unsupported target object: {}'.format(target))
    profile_path = package.parent / 'cm_dnmpc/config/objects' / (target + '.yaml')
    profile = yaml.safe_load(profile_path.read_text(encoding='utf-8'))
    planning = profile.get('planning')
    if not planning:
        raise ValueError('No confirmed real planning margins for {}'.format(target))
    family = planning['family']
    spec = json.loads((package / 'config' / family / 'real_planning.json').read_text())
    size = float(profile['object']['grasping_size_m'])
    rg = float(planning['catch_margin_m'])
    rt = float(planning['move_margin_m'])
    if not (all(math.isfinite(v) for v in (size, rg, rt)) and size > 0 and rg > 0 and 0 < rt < size / 2):
        raise ValueError('invalid real target size or safety margins')
    if not math.isclose(rt, float(planning['move_margin_base_m']) +
                        float(planning['move_margin_increment_m']), abs_tol=1e-12):
        raise ValueError('transport margin must equal base plus increment exactly once')
    spec.update(target_object=target, family=family, grasping_size_m=size,
                catch_margin_m=rg, move_margin_m=rt,
                margin_source=planning['margin_source'],
                object_profile=str(profile_path))
    return spec


def real_parameters(package, phase, base_config, start_pair=None, target_object=None):
    spec = planning_spec(package, target_object)
    config = copy.deepcopy(base_config)
    size = float(spec['grasping_size_m'])
    margin = float(spec[phase + '_margin_m'])
    # Existing native checkers use inclusive comparisons. nextafter preserves
    # the strict boundary in the manuscript without adding a physical margin.
    low, high = ((float(np.nextafter(size, math.inf)), size + 2 * margin)
                 if phase == 'catch' else
                 (size - 2 * margin, float(np.nextafter(size, -math.inf))))
    goal = config.setdefault('goal_generator', {})
    goal.update(spec[phase])
    goal.update(distance_min=low, distance_max=high,
                fallback_to_joint_final=False, allow_reference_collisions=False)
    config.setdefault('planning', {}).update(lowbound=low, upperbound=high)
    if 'palnning' in config:
        config['palnning'].update(lowbound=low, upperbound=high)
    cost = config.setdefault('cost', {})
    cost.update(cos=math.cos(spec[phase]['theta_left']),
                cos_2=math.cos(spec[phase]['theta_right']))
    if 'box_goal_generator' in config:
        box = config['box_goal_generator']
        box.update(distance_min=low, distance_max=high,
                   theta_left=spec[phase]['theta_left'],
                   theta_right=spec[phase]['theta_right'],
                   allow_reference_collisions=False)
        box.setdefault('dimensions', {})[box.get('long_axis', 'y')] = size
    if 'move_grasp_distance_dh' in config:
        config['move_grasp_distance_dh'].update(
            min_m=low, max_m=high, physical_box_length_m=size)
    if start_pair is not None:
        pair = np.asarray(start_pair, dtype=float).reshape(-1)
        if pair.size != 12 or not np.all(np.isfinite(pair)):
            raise ValueError('start pair must contain 12 finite joint angles')
        config['Joint_init'] = {
            'joint_{}'.format(i + 1): float(q) for i, q in enumerate(pair)}
    return config, spec


def relocate(value, workspace):
    if isinstance(value, dict):
        return {key: relocate(item, workspace) for key, item in value.items()}
    if isinstance(value, list):
        return [relocate(item, workspace) for item in value]
    if isinstance(value, str):
        return value.replace('/home/mas-2/3rd_party/CM_ws/', workspace.as_posix() + '/')
    return value


def prepare(workspace, object_name, phase, output, scene_config=None, start_pair=None,
            target_object=None):
    if target_object and target_object != object_name and scene_config is None:
        raise ValueError('Non-default targets require --scene-config with their measured geometry')
    package = workspace / 'src/real_world_experiment/cm_trajectory_generation'
    filename = (phase + '_template.yaml' if object_name == 'ball'
                else phase + '_kirara_box.yaml')
    source = scene_config or package / 'config' / object_name / filename
    base = yaml.safe_load(source.read_text())
    config, spec = real_parameters(package, phase, base, start_pair, target_object or object_name)
    if spec['family'] != object_name:
        raise ValueError('Target profile and planner family do not match')
    config = relocate(config, workspace)
    # Every generated artifact stays in the new real run, not staging/sim paths.
    for section in ('kirara_path', 'planned_path'):
        config[section] = {side: str(output / (phase + '_' + side + '_' + section + '.txt'))
                           for side in ('left', 'right')}
    for section in ('goal_generator', 'box_goal_generator'):
        if section in config:
            config[section]['output_file'] = str(output / (section + '_q.yaml'))
            config[section]['debug_file'] = str(output / (section + '.log'))
    # Copy the planning URDF to relocate mesh references without editing its source.
    model_source = Path(config['file_path']['pinocchio_file'])
    model = model_source.read_text()
    model = relocate(model, workspace)
    (output / 'scene.urdf').write_text(model, encoding='utf-8')
    config['file_path']['pinocchio_file'] = str(output / 'scene.urdf')
    config_path = output / (phase + '_real.yaml')
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    options = json.loads((package / 'config' / object_name / 'move_selected_fcl_options.json').read_text())
    options.update(spec['smoothing_weights'])
    options['paper_source'] = 'main20261009.tex / tab:weight_parameters_soomth'
    if options.get('spatial_step', 0) <= 0:
        raise ValueError('real smoothing requires a positive spatial update step')
    options_path = output / 'smoothing_real.json'
    options_path.write_text(json.dumps(options, indent=2) + '\n', encoding='utf-8')
    manifest = {'runtime': 'real', 'hardware_validated': False,
                'object': spec['target_object'], 'planning_family': object_name,
                'phase': phase, 'paper': spec['manuscript'],
                'grasping_size_m': spec['grasping_size_m'],
                'catch_margin_m': spec['catch_margin_m'],
                'move_margin_m': spec['move_margin_m'],
                'margin_source': spec['margin_source'],
                'object_profile': spec['object_profile'],
                'scene_source': str(source),
                'scene_calibration_verified': False,
                'simulation_configuration_changed': False,
                'start_source': 'explicit joint measurement' if start_pair is not None else 'scene configuration',
                'config': str(config_path), 'smoothing_options': str(options_path)}
    (output / 'real_parameters.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return config_path, options_path


def main(forced_object=None):
    parser = argparse.ArgumentParser(description=__doc__)
    if forced_object:
        parser.set_defaults(object=forced_object)
    else:
        parser.add_argument('--object', choices=('ball', 'box'), required=True)
    parser.add_argument('--phase', choices=('catch', 'move'), required=True)
    parser.add_argument('--target-object', choices=TARGET_OBJECTS,
                        help='Real object profile; defaults to the Ball/Box entry family. Other targets require --scene-config.')
    parser.add_argument('--workspace', type=Path,
                        default=Path(os.environ.get('CM_WORKSPACE', '/home/mas-2/3rd_party/CM_ws')))
    parser.add_argument('--scene-config', type=Path,
                        help='YAML with the measured real scene/target/start; defaults are not a new calibration')
    parser.add_argument('--start-pair', type=Path,
                        help='12 joint angles: left six then right six; required for Move from Stage2')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--goal-port', type=int, default=55301)
    parser.add_argument('--plan-port', type=int, default=55305)
    parser.add_argument('--optimization-port', type=int, default=55841)
    parser.add_argument('--max-iterations', type=int, default=20000)
    parser.add_argument('--timeout', type=float, default=900.)
    parser.add_argument('--optimization-iterations', type=int, default=80)
    parser.add_argument('--output-dt', type=float, default=0.01)
    parser.add_argument('--smoothing-overrides', type=Path)
    args = parser.parse_args()
    if args.optimization_iterations <= 0 or not math.isfinite(args.output_dt) or args.output_dt <= 0:
        parser.error('optimization iterations and output interval must be positive')
    if args.phase == 'move' and not args.start_pair:
        parser.error('Move requires --start-pair from the measured Stage2 endpoint')
    workspace = args.workspace.resolve()
    package = workspace / 'src/real_world_experiment/cm_trajectory_generation'
    output = args.output_dir or package / 'generated' / args.object / (
        'real_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + args.phase)
    output = output.resolve()
    if output.exists():
        parser.error('output directory already exists; use a new run directory')
    pair = np.loadtxt(str(args.start_pair)) if args.start_pair else None
    output.mkdir(parents=True)
    config, options = prepare(workspace, args.object, args.phase, output, args.scene_config,
                              pair, args.target_object)
    if args.smoothing_overrides:
        values = json.loads(options.read_text())
        values.update(json.loads(args.smoothing_overrides.read_text()))
        if not math.isfinite(float(values.get('spatial_step', 0))) or float(values.get('spatial_step', 0)) <= 0:
            parser.error('real smoothing requires a positive spatial_step')
        options.write_text(json.dumps(values, indent=2) + '\n', encoding='utf-8')
    if args.prepare_only:
        print('Real parameters prepared (no planning or robot commands): {}'.format(output))
        return
    common = workspace / 'devel/lib/cm_trajectory_generation'
    own = workspace / 'devel/lib/cm_trajectory_generation'
    validator = own / ('CM_SOCKET_' + args.object.upper() + '_VALIDATOR')
    goal_executable = own / ('CM_SOCKET_BALL_GOAL' if args.object == 'ball' else 'CM_KIRARA_BOX_GOAL')
    goal_prefix = output / 'goals' / args.phase
    commands = [[common / 'run_distributed_goal_pair.py', '--phase', args.phase,
                 '--config', config, '--output-dir', output / 'goals',
                 '--output-prefix', goal_prefix, '--port', args.goal_port,
                 '--timeout', args.timeout, '--goal-node', common / 'cm_distributed_goal_node.py',
                 '--goal-executable', goal_executable, '--validator', validator]]
    if pair is not None:
        commands[0] += ['--reference-pair', args.start_pair.resolve()]
    commands += [[common / 'run_socket_planning_pair.py', '--phase', args.phase,
                  '--config', config, '--output-dir', output / 'path',
                  '--left-endpoints', str(goal_prefix) + '_left.txt',
                  '--right-endpoints', str(goal_prefix) + '_right.txt',
                  '--port', args.plan_port, '--max-iterations', args.max_iterations,
                  '--timeout', args.timeout, '--planner-node', common / 'cm_socket_planner_node.py',
                  '--validator', validator],
                 [workspace / 'devel/lib/cm_trajectory_smoothing/run_distributed_trajopt_pair.py',
                  '--object', args.object, '--phase', args.phase,
                  '--left-path', output / 'path' / (args.phase + '_left_planned_path.txt'),
                  '--right-path', output / 'path' / (args.phase + '_right_planned_path.txt'),
                  '--output-dir', output / 'smoothed', '--options-json', options,
                  '--constraint-config', config, '--port', args.optimization_port,
                  '--max-iterations', args.optimization_iterations, '--output-dt', args.output_dt]]
    for command in commands:
        subprocess.run([str(item) for item in command], check=True)
    print('Real offline run complete; not hardware-validated: {}'.format(output))


if __name__ == '__main__':
    main()
