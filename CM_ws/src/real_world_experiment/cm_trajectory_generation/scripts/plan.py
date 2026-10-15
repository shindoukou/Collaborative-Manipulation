#!/usr/bin/env python3

import argparse
import datetime
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import yaml

OBJECTS = ('ball', 'box', 'basketball', 'foam_frame', 'plush_toy',
           'l_shaped_wire_mesh', 'clothes_filled_bag')
SIM_OBJECTS = {'simball': 'ball', 'simbox': 'box'}


def select_object(value):
    if value in [str(i) for i in range(1, 8)]:
        return OBJECTS[int(value) - 1]
    if value not in OBJECTS and value not in SIM_OBJECTS:
        raise ValueError('Choose simball/simbox, a real-world object name, or a number from 1 to 7')
    return value


def workspace_path():
    if os.environ.get('CM_WORKSPACE'):
        candidates = [Path(os.environ['CM_WORKSPACE']).expanduser().resolve()]
    else:
        candidates = list(Path(__file__).resolve().parents)
    for candidate in candidates:
        if (candidate / 'src/real_world_experiment/cm_trajectory_generation').is_dir():
            return candidate
    raise ValueError('Set CM_WORKSPACE to the built source workspace')


def path_value(workspace, value):
    if not value:
        raise ValueError('Required path is empty; edit config/offline.yaml')
    path = Path(value).expanduser()
    return (path if path.is_absolute() else workspace / path).resolve()


def input_file(workspace, value):
    path = path_value(workspace, value)
    if not path.is_file():
        raise ValueError('Missing input: {}'.format(path))
    return path


def positive(value, name, integer=False):
    number = float(value)
    if not math.isfinite(number) or number <= 0 or (integer and int(number) != number):
        raise ValueError('{} must be positive{}'.format(name, ' integer' if integer else ''))
    return int(number) if integer else number


def check_settings(workspace, config, name):
    settings = config['objects'][name]
    runtime, family = settings['runtime'], settings['family']
    if runtime not in ('sim', 'real') or family not in ('ball', 'box'):
        raise ValueError('runtime must be sim/real; family must be ball/box')
    expected_runtime = 'sim' if name in SIM_OBJECTS else 'real'
    expected_family = SIM_OBJECTS.get(name, 'ball' if name in ('ball', 'basketball') else 'box')
    if runtime != expected_runtime or family != expected_family:
        raise ValueError('{} requires runtime={} and family={}; simulation uses simball/simbox'.format(
            name, expected_runtime, expected_family))
    phases = settings['phases']
    if not isinstance(phases, list) or not phases or len(set(phases)) != len(phases):
        raise ValueError('phases must be a nonempty list without duplicates')
    if any(phase not in ('catch', 'move') for phase in phases):
        raise ValueError('phases may contain only catch and move')
    if runtime == 'sim' and phases != ['catch', 'move']:
        raise ValueError('The simulation planner generates both phases; use [catch, move]')
    positive(config['planning']['max_iterations'], 'planning.max_iterations', True)
    positive(config['planning']['timeout_s'], 'planning.timeout_s')
    positive(config['smoothing']['max_iterations'], 'smoothing.max_iterations', True)
    positive(config['smoothing']['output_dt'], 'smoothing.output_dt')
    for phase in phases:
        scene_value = settings[runtime + '_scenes'].get(phase)
        if not scene_value:
            raise ValueError('Empty objects.{}.{}_scenes.{} in config/offline.yaml'.format(name, runtime, phase))
        source = input_file(workspace, scene_value)
        scene = yaml.safe_load(source.read_text(encoding='utf-8'))
        model = scene['file_path']['pinocchio_file'].replace(
            '/home/mas-2/3rd_party/CM_ws/', workspace.as_posix() + '/')
        input_file(workspace, model)
        options = settings.get('sim_options', {}).get(phase) if runtime == 'sim' else None
        if options:
            json.loads(input_file(workspace, options).read_text())
    if runtime == 'real':
        from real_planning import planning_spec
        package = workspace / 'src/real_world_experiment/cm_trajectory_generation'
        planning_spec(package, name)
        if 'move' in phases:
            import numpy as np
            if not settings.get('stage2_joints'):
                raise ValueError('Empty objects.{}.stage2_joints: supply 12 measured Stage2 angles'.format(name))
            pair = np.loadtxt(str(input_file(workspace, settings['stage2_joints'])))
            if pair.size != 12 or not np.all(np.isfinite(pair)):
                raise ValueError('stage2_joints must contain 12 finite measured joint angles')
    return settings


def relocated(value, workspace):
    if isinstance(value, dict):
        return {key: relocated(item, workspace) for key, item in value.items()}
    if isinstance(value, list):
        return [relocated(item, workspace) for item in value]
    if isinstance(value, str):
        return value.replace('/home/mas-2/3rd_party/CM_ws/', workspace.as_posix() + '/')
    return value


def snapshot_scene(source, workspace, output, phase):
    scene = relocated(yaml.safe_load(source.read_text(encoding='utf-8')), workspace)
    model = input_file(workspace, scene['file_path']['pinocchio_file'])
    model_copy = output / (phase + '_scene.urdf')
    model_copy.write_text(relocated(model.read_text(encoding='utf-8'), workspace), encoding='utf-8')
    scene['file_path']['pinocchio_file'] = str(model_copy)
    for section in ('planned_path', 'kirara_path'):
        scene[section] = {side: str(output / (phase + '_' + side + '_' + section + '.txt'))
                          for side in ('left', 'right')}
    for section in ('goal_generator', 'box_goal_generator'):
        if section in scene:
            scene[section]['output_file'] = str(output / (phase + '_' + section + '.yaml'))
            scene[section]['debug_file'] = str(output / (phase + '_' + section + '.log'))
    destination = output / (phase + '_scene.yaml')
    destination.write_text(yaml.safe_dump(scene, sort_keys=False), encoding='utf-8')
    return destination


def run_pipeline(workspace, config, name, settings, output, execute):
    family, runtime = settings['family'], settings['runtime']
    binary = workspace / 'devel/lib'
    plan = config['planning']
    smooth = config['smoothing']
    overrides = settings.get('smoothing_overrides', {})
    if runtime == 'real':
        for phase in settings['phases']:
            command = [binary / 'cm_trajectory_generation/real_planning.py',
                       '--workspace', workspace, '--object', family, '--target-object', name,
                       '--phase', phase, '--scene-config',
                       input_file(workspace, settings['real_scenes'][phase]),
                       '--output-dir', output / phase,
                       '--max-iterations', plan['max_iterations'], '--timeout', plan['timeout_s'],
                       '--optimization-iterations', smooth['max_iterations'],
                       '--output-dt', smooth['output_dt']]
            if phase == 'move':
                command += ['--start-pair', input_file(workspace, settings['stage2_joints'])]
            if overrides.get(phase):
                options = output / (phase + '_overrides.json')
                options.write_text(json.dumps(overrides[phase], indent=2), encoding='utf-8')
                command += ['--smoothing-overrides', options]
            execute(command)
        return
    scenes = {phase: snapshot_scene(input_file(workspace, settings['sim_scenes'][phase]),
                                     workspace, output, phase) for phase in ('catch', 'move')}
    execute([binary / 'cm_trajectory_generation' / ('generate_socket_' + family + '_plan.py'),
             '--workspace', workspace, '--output-dir', output / 'paths',
             '--catch-config', scenes['catch'], '--move-config', scenes['move'],
             '--max-iterations', plan['max_iterations'], '--timeout', plan['timeout_s'],
             '--no-update-latest'])
    for phase in ('catch', 'move'):
        options = {}
        base = settings.get('sim_options', {}).get(phase)
        if base:
            options.update(json.loads(input_file(workspace, base).read_text()))
        options.update(overrides.get(phase, {}))
        options_path = output / (phase + '_options.json')
        options_path.write_text(json.dumps(options, indent=2), encoding='utf-8')
        execute([binary / 'cm_trajectory_smoothing/run_distributed_trajopt_pair.py',
                 '--object', family, '--phase', phase,
                 '--left-path', output / 'paths' / phase / (phase + '_left_planned_path.txt'),
                 '--right-path', output / 'paths' / phase / (phase + '_right_planned_path.txt'),
                 '--constraint-config', scenes[phase], '--options-json', options_path,
                 '--output-dir', output / 'smoothed' / phase,
                 '--max-iterations', smooth['max_iterations'], '--output-dt', smooth['output_dt']])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('object', nargs='?', help='simball/simbox, real-world object name or 1-7; omit for menu')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--check', action='store_true', help='validate inputs without planning')
    args = parser.parse_args()
    try:
        value = args.object
        if value is None:
            print('Simulation: simball / simbox')
            print('Real-world experiments:')
            for number, name in enumerate(OBJECTS, 1):
                print('{}  {}'.format(number, name))
            value = input('Object [simball/simbox, 1-7 or name]: ').strip()
        name = select_object(value)
        workspace = workspace_path()
        source = args.config or workspace / 'src/real_world_experiment/cm_trajectory_generation/config/offline.yaml'
        config = yaml.safe_load(source.read_text(encoding='utf-8'))
        settings = check_settings(workspace, config, name)
        print('Object: {} | runtime: {} | phases: {}'.format(name, settings['runtime'], settings['phases']))
        if args.check:
            print('Inputs checked; no planning or hardware commands executed.')
            return
        output = path_value(workspace, config['output_root']) / (
            name + '_' + settings['runtime'] + '_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        output.mkdir(parents=True, exist_ok=False)
        (output / 'settings.yaml').write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
        env = os.environ.copy()
        env['CM_WORKSPACE'] = str(workspace)
        commands = []
        def execute(command):
            command = [str(item) for item in command]
            commands.append(command)
            (output / 'commands.json').write_text(json.dumps(commands, indent=2), encoding='utf-8')
            print('Running: {}'.format(Path(command[0]).name), flush=True)
            subprocess.run(command, check=True, env=env)
        status = {'object': name, 'runtime': settings['runtime'], 'status': 'running'}
        try:
            run_pipeline(workspace, config, name, settings, output, execute)
            status['status'] = 'completed'
        except BaseException as exc:
            status.update(status='failed', error=str(exc))
            raise
        finally:
            (output / 'status.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
            print('Results: {}'.format(output), flush=True)
    except (ValueError, KeyError, OSError, EOFError, yaml.YAMLError, subprocess.CalledProcessError) as exc:
        parser.exit(2, 'Offline planning stopped: {}\n'.format(exc))


if __name__ == '__main__':
    main()
