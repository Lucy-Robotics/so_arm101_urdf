# Copyright 2025-2026 Sentience Robotics Team
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Unit tests for scripts/apply_lerobot_calibration.py."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def _load_module():
    path = ROOT / 'scripts' / 'apply_lerobot_calibration.py'
    spec = importlib.util.spec_from_file_location('apply_lerobot_calibration', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def _yaml_g(value: float) -> float:
    """Match ``apply_updates`` ``:g`` formatting after a YAML round-trip."""
    return float(f'{value:g}')


CONFIG = """\
actuators:
  # A comment that must survive the rewrite.
  - id: rotation
    urdf_joint: Rotation
    physical_pin: 1
    offset_rad: 3.490659
    direction: 1
    scale: 1
    servo_min_rad: 1.396263
    servo_max_rad: 5.585054
    servo_default_rad: 3.490659
    enabled: true
  - id: jaw
    urdf_joint: Jaw
    physical_pin: 6
    offset_rad: 2.399828
    direction: -1
    scale: 1
    servo_min_rad: 2.007129
    servo_max_rad: 4.363323
    servo_default_rad: 2.399828
    enabled: true

sensors:
  - id: rotation_encoder
    associated_actuator: rotation
    min_value: 1.396263
    max_value: 5.585054
    enabled: false
"""

CALIBRATION = {
    'shoulder_pan': {
        'id': 1, 'drive_mode': 0, 'homing_offset': -1710,
        'range_min': 768, 'range_max': 3273,
    },
    'gripper': {
        'id': 6, 'drive_mode': 0, 'homing_offset': 1534,
        'range_min': 2046, 'range_max': 3277,
    },
}


def _apply(text=CONFIG, calibration=None, tmp_path=None):
    path = tmp_path / 'active.yaml'
    path.write_text(text, encoding='utf-8')
    return mod.apply_to_file(path, calibration or CALIBRATION)


def test_ticks_to_radians_matches_the_firmware_scale():
    """0-4096 ticks span 0-2π rad, so the centre is π."""
    assert mod.ticks_to_radians(0) == 0.0
    assert mod.ticks_to_radians(4096) == 6.283185
    assert mod.ticks_to_radians(2048) == 3.141593
    assert mod.ticks_to_radians(768) == 1.178097


def test_windows_and_centre_are_written(tmp_path):
    """Ranges convert to radians and joint zero lands on the homed centre."""
    out, _ = _apply(tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}

    rotation = actuators['rotation']
    assert rotation['servo_min_rad'] == _yaml_g(mod.ticks_to_radians(768))
    assert rotation['servo_max_rad'] == _yaml_g(mod.ticks_to_radians(3273))
    assert rotation['offset_rad'] == _yaml_g(mod.ticks_to_radians(2048))
    assert rotation['servo_default_rad'] == _yaml_g(mod.ticks_to_radians(2048))

    # Not the midpoint of the window: the gripper's travel is far from symmetric.
    jaw = actuators['jaw']
    assert jaw['servo_min_rad'] == _yaml_g(mod.ticks_to_radians(2046))
    assert jaw['offset_rad'] == _yaml_g(mod.ticks_to_radians(2048))


def test_matching_is_by_servo_id_not_name(tmp_path):
    """Joint names in the calibration carry no relation to urdf_joint; the bus id does."""
    out, _ = _apply(tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    # 'gripper' (id 6) reached 'jaw' (physical_pin 6), not the alphabetically near
    # 'rotation'.
    assert actuators['jaw']['servo_max_rad'] == _yaml_g(mod.ticks_to_radians(3277))
    assert actuators['rotation']['servo_max_rad'] == _yaml_g(mod.ticks_to_radians(3273))


def test_untouched_fields_and_comments_survive(tmp_path):
    """The file is edited as text, so nothing outside the named keys moves."""
    out, _ = _apply(tmp_path=tmp_path)
    assert '# A comment that must survive the rewrite.' in out
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['jaw']['direction'] == -1
    assert actuators['rotation']['urdf_joint'] == 'Rotation'
    assert actuators['jaw']['enabled'] is True


def test_sensor_bounds_follow_their_actuator(tmp_path):
    """Encoder bounds stay in the same space as the actuator they report on."""
    out, _ = _apply(tmp_path=tmp_path)
    sensor = yaml.safe_load(out)['sensors'][0]
    assert sensor['min_value'] == _yaml_g(mod.ticks_to_radians(768))
    assert sensor['max_value'] == _yaml_g(mod.ticks_to_radians(3273))
    assert sensor['enabled'] is False


def test_applying_twice_changes_nothing(tmp_path):
    """Re-running on an already-calibrated config is a no-op."""
    once, _ = _apply(tmp_path=tmp_path)
    twice, _ = _apply(text=once, tmp_path=tmp_path)
    assert once == twice


def test_unmatched_record_refuses_to_write(tmp_path):
    """A half-applied calibration drives joints against someone else's window."""
    calibration = dict(CALIBRATION, elbow_flex={
        'id': 3, 'drive_mode': 0, 'homing_offset': 0,
        'range_min': 866, 'range_max': 3083,
    })
    with pytest.raises(mod.CalibrationError, match='matches no actuator'):
        _apply(calibration=calibration, tmp_path=tmp_path)


def test_drive_mode_is_reported_not_applied(tmp_path):
    """Leave direction alone: an inversion is surfaced for a human, not guessed."""
    calibration = json.loads(json.dumps(CALIBRATION))
    calibration['gripper']['drive_mode'] = 1
    out, warnings = _apply(calibration=calibration, tmp_path=tmp_path)
    assert any('drive_mode=1' in w for w in warnings)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['jaw']['direction'] == -1


def test_actuator_without_a_record_is_left_alone(tmp_path):
    """Only calibrated joints move; the rest keep their values and are reported."""
    out, warnings = _apply(calibration={'gripper': CALIBRATION['gripper']},
                           tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['rotation']['servo_min_rad'] == 1.396263
    assert any('rotation' in w for w in warnings)


def test_inverted_range_is_rejected(tmp_path):
    """range_min >= range_max is a broken calibration, not a window."""
    path = tmp_path / 'cal.json'
    path.write_text(json.dumps({'gripper': {
        'id': 6, 'drive_mode': 0, 'homing_offset': 0,
        'range_min': 3277, 'range_max': 2046,
    }}), encoding='utf-8')
    with pytest.raises(mod.CalibrationError, match='range_min'):
        mod.load_calibration(path)


def test_missing_key_is_rejected(tmp_path):
    """A record without a range says nothing about the joint."""
    path = tmp_path / 'cal.json'
    path.write_text(json.dumps({'gripper': {'id': 6}}), encoding='utf-8')
    with pytest.raises(mod.CalibrationError, match='missing'):
        mod.load_calibration(path)


REAL_CALIBRATION = {
    'shoulder_pan': {'id': 1, 'drive_mode': 0, 'homing_offset': -1981,
                     'range_min': 709, 'range_max': 3229},
    'shoulder_lift': {'id': 2, 'drive_mode': 0, 'homing_offset': -1635,
                      'range_min': 838, 'range_max': 3210},
    'elbow_flex': {'id': 3, 'drive_mode': 0, 'homing_offset': 1105,
                   'range_min': 845, 'range_max': 3085},
    'wrist_flex': {'id': 4, 'drive_mode': 0, 'homing_offset': 1690,
                   'range_min': 798, 'range_max': 3183},
    'wrist_roll': {'id': 5, 'drive_mode': 0, 'homing_offset': 1819,
                   'range_min': 0, 'range_max': 4095},
    'gripper': {'id': 6, 'drive_mode': 0, 'homing_offset': 1382,
                'range_min': 2033, 'range_max': 3525},
}


def test_real_config_round_trips():
    """The committed config is what this tool produces from the recorded ticks."""
    active = ROOT / 'config/hardware/active.yaml'
    out, warnings = mod.apply_to_file(active, REAL_CALIBRATION)
    assert out == active.read_text(encoding='utf-8')
    assert not warnings
    default = ROOT / 'config/hardware/configs/default.yaml'
    out_default, warnings_default = mod.apply_to_file(default, REAL_CALIBRATION)
    assert out_default == default.read_text(encoding='utf-8')
    assert not warnings_default
    assert active.read_text(encoding='utf-8') == default.read_text(encoding='utf-8')


def test_main_is_self_sufficient_for_the_real_package(tmp_path, capsys):
    """One invocation covers active.yaml and the preset behind it."""
    cal_path = tmp_path / 'cal.json'
    cal_path.write_text(json.dumps(REAL_CALIBRATION), encoding='utf-8')
    rc = mod.main([str(cal_path), '--dry-run'])
    out = capsys.readouterr().out
    assert rc == 0
    assert 'active.yaml: no change' in out
    # The xacro is generated from active.yaml, so this tool must not touch it.
    assert 'so_arm101_ros2_control.xacro' not in out
