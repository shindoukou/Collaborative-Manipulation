#!/usr/bin/env python3
"""Verify explicit DNMPC Stage 1--4 weight resolution."""

from pathlib import Path

from cm_dnmpc_common import (
    load_json,
    resolve_stage_weight_error_scales,
    resolve_stage_weights,
)


ROOT = Path('/home/mas-2/3rd_party/CM_ws')
config = load_json(ROOT / 'src/cm_dnmpc/config/controller.json')
profiles = {}
for object_name in ('ball', 'box'):
    profiles[object_name] = {}
    for stage_id in (1, 2, 3, 4):
        weights = resolve_stage_weights(config, object_name, stage_id)
        scales = resolve_stage_weight_error_scales(
            config, object_name, stage_id)
        assert set(weights) == {'Wd', 'Wt', 'Wo', 'Wu', 'Wq', 'Wv', 'WF'}
        assert set(scales) == {'Wd', 'Wt', 'Wo', 'Wu', 'Wq', 'Wv', 'WF'}
        assert all(value > 0.0 for value in scales.values())
        profiles[object_name][stage_id] = weights

assert profiles['ball'] == {
    1: {'Wd': 0.0, 'Wt': 0.0, 'Wo': 0.0, 'Wu': 0.01,
        'Wq': 10.0, 'Wv': 5.0, 'WF': 0.0},
    2: {'Wd': 0.1, 'Wt': 0.5, 'Wo': 1.0, 'Wu': 0.01,
        'Wq': 0.0, 'Wv': 0.0, 'WF': 1.0},
    3: {'Wd': 0.0, 'Wt': 40.0, 'Wo': 2.0, 'Wu': 0.02,
        'Wq': 7.5, 'Wv': 3.0, 'WF': 100.0},
    4: {'Wd': 10.0, 'Wt': 1.5, 'Wo': 1.0, 'Wu': 0.01,
        'Wq': 0.0, 'Wv': 0.0, 'WF': 1.0},
}
for stage_id in (1, 2, 3, 4):
    assert resolve_stage_weight_error_scales(config, 'ball', stage_id) == {
        'Wd': 1.0, 'Wt': 1.0, 'Wo': 1.0, 'Wu': 1.0,
        'Wq': 1.0, 'Wv': 1.0, 'WF': 1.0,
    }
assert resolve_stage_weight_error_scales(
    config, 'box', 3)['Wt'] == 1.0

override = dict(config)
override['stage_weights'] = {
    'default': {'3': {'Wt': 123.0}},
    'ball': {'3': {'WF': 0.25}},
}
resolved = resolve_stage_weights(override, 'ball', 3)
assert resolved['Wt'] == 123.0
assert resolved['WF'] == 0.25
assert resolved['Wq'] == config['weights']['Wq']

print('DNMPC_STAGE_WEIGHTS_OK {}'.format(profiles))
