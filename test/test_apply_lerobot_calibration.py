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
    offset: 3.490659
    direction: 1
    scale: 1
    limit_min: 1.396263
    limit_max: 5.585054
    default: 3.490659
    enabled: true
  - id: jaw
    urdf_joint: Jaw
    physical_pin: 6
    offset: 2.399828
    direction: -1
    scale: 1
    limit_min: 2.007129
    limit_max: 4.363323
    default: 2.399828
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
    new_text, urdf_limits, warnings = mod.apply_to_file(
        path, calibration or CALIBRATION
    )
    return new_text, warnings, urdf_limits


def test_ticks_to_radians_matches_the_firmware_scale():
    """0-4096 ticks span 0-2π rad, so the centre is π."""
    assert mod.ticks_to_radians(0) == 0.0
    assert mod.ticks_to_radians(4096) == 6.283185
    assert mod.ticks_to_radians(2048) == 3.141593
    assert mod.ticks_to_radians(768) == 1.178097


def test_windows_and_centre_are_written(tmp_path):
    """Ranges convert to radians and joint zero lands on the homed centre."""
    out, _, urdf_limits = _apply(tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}

    rotation = actuators['rotation']
    assert rotation['limit_min'] == _yaml_g(mod.ticks_to_radians(768))
    assert rotation['limit_max'] == _yaml_g(mod.ticks_to_radians(3273))
    assert rotation['offset'] == _yaml_g(mod.ticks_to_radians(2048))
    assert rotation['default'] == _yaml_g(mod.ticks_to_radians(2048))

    # Not the midpoint of the window: the gripper's travel is far from symmetric.
    jaw = actuators['jaw']
    assert jaw['limit_min'] == _yaml_g(mod.ticks_to_radians(2046))
    assert jaw['offset'] == _yaml_g(mod.ticks_to_radians(2048))

    # URDF limits are joint-space: (servo - offset) * direction * scale.
    centre = mod.ticks_to_radians(2048)
    assert urdf_limits['Rotation'] == mod.servo_window_to_joint_limits(
        mod.ticks_to_radians(768), mod.ticks_to_radians(3273), centre, 1, 1
    )
    assert urdf_limits['Jaw'] == mod.servo_window_to_joint_limits(
        mod.ticks_to_radians(2046), mod.ticks_to_radians(3277), centre, -1, 1
    )


def test_matching_is_by_servo_id_not_name(tmp_path):
    """Joint names in the calibration carry no relation to urdf_joint; the bus id does."""
    out, _, _ = _apply(tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    # 'gripper' (id 6) reached 'jaw' (physical_pin 6), not the alphabetically near
    # 'rotation'.
    assert actuators['jaw']['limit_max'] == _yaml_g(mod.ticks_to_radians(3277))
    assert actuators['rotation']['limit_max'] == _yaml_g(mod.ticks_to_radians(3273))


def test_untouched_fields_and_comments_survive(tmp_path):
    """The file is edited as text, so nothing outside the named keys moves."""
    out, _, _ = _apply(tmp_path=tmp_path)
    assert '# A comment that must survive the rewrite.' in out
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['jaw']['direction'] == -1
    assert actuators['rotation']['urdf_joint'] == 'Rotation'
    assert actuators['jaw']['enabled'] is True


def test_sensor_bounds_follow_their_actuator(tmp_path):
    """Encoder bounds stay in the same space as the actuator they report on."""
    out, _, _ = _apply(tmp_path=tmp_path)
    sensor = yaml.safe_load(out)['sensors'][0]
    assert sensor['min_value'] == _yaml_g(mod.ticks_to_radians(768))
    assert sensor['max_value'] == _yaml_g(mod.ticks_to_radians(3273))
    assert sensor['enabled'] is False


def test_applying_twice_changes_nothing(tmp_path):
    """Re-running on an already-calibrated config is a no-op."""
    once, _, _ = _apply(tmp_path=tmp_path)
    twice, _, _ = _apply(text=once, tmp_path=tmp_path)
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
    out, warnings, _ = _apply(calibration=calibration, tmp_path=tmp_path)
    assert any('drive_mode=1' in w for w in warnings)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['jaw']['direction'] == -1


def test_actuator_without_a_record_is_left_alone(tmp_path):
    """Only calibrated joints move; the rest keep their values and are reported."""
    out, warnings, _ = _apply(calibration={'gripper': CALIBRATION['gripper']},
                              tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['rotation']['limit_min'] == 1.396263
    assert any('rotation' in w for w in warnings)


def test_apply_urdf_limits_rewrites_lower_upper(tmp_path):
    """Joint-space windows land on the matching <limit> without touching other attrs."""
    urdf = (
        '<robot>\n'
        '  <joint name="Rotation" type="revolute">\n'
        '    <limit effort="10" velocity="10" lower="-1.9" upper="1.9"/>\n'
        '  </joint>\n'
        '  <joint name="Jaw" type="revolute">\n'
        '    <limit effort="10" velocity="10" lower="-0.1" upper="1.7"/>\n'
        '  </joint>\n'
        '</robot>\n'
    )
    out = mod.apply_urdf_limits(urdf, {'Rotation': (-1.5, 1.2), 'Jaw': (-0.2, 1.8)})
    assert 'lower="-1.5"' in out and 'upper="1.2"' in out
    assert 'lower="-0.2"' in out and 'upper="1.8"' in out
    assert 'effort="10"' in out


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
    'shoulder_pan': {'id': 1, 'drive_mode': 0, 'homing_offset': -1695,
                     'range_min': 867, 'range_max': 3274},
    'shoulder_lift': {'id': 2, 'drive_mode': 0, 'homing_offset': -1031,
                      'range_min': 752, 'range_max': 3242},
    'elbow_flex': {'id': 3, 'drive_mode': 0, 'homing_offset': 1631,
                   'range_min': 835, 'range_max': 3059},
    'wrist_flex': {'id': 4, 'drive_mode': 0, 'homing_offset': -2012,
                   'range_min': 877, 'range_max': 3226},
    'wrist_roll': {'id': 5, 'drive_mode': 0, 'homing_offset': 1899,
                   'range_min': 0, 'range_max': 4095},
    'gripper': {'id': 6, 'drive_mode': 0, 'homing_offset': 1536,
                'range_min': 2046, 'range_max': 3515},
}


def test_real_config_round_trips():
    """The committed config is what this tool produces from the recorded ticks."""
    active = ROOT / 'config/hardware/active.yaml'
    out, urdf_limits, warnings = mod.apply_to_file(active, REAL_CALIBRATION)
    assert out == active.read_text(encoding='utf-8')
    assert not warnings
    assert set(urdf_limits) == {
        'Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw',
    }
    default = ROOT / 'config/hardware/configs/default.yaml'
    out_default, _, warnings_default = mod.apply_to_file(default, REAL_CALIBRATION)
    assert out_default == default.read_text(encoding='utf-8')
    assert not warnings_default
    assert active.read_text(encoding='utf-8') == default.read_text(encoding='utf-8')


def test_main_is_self_sufficient_for_the_real_package(tmp_path, capsys):
    """One invocation covers active.yaml, its preset, and the URDF limits."""
    cal_path = tmp_path / 'cal.json'
    cal_path.write_text(json.dumps(REAL_CALIBRATION), encoding='utf-8')
    rc = mod.main([str(cal_path), '--dry-run'])
    out = capsys.readouterr().out
    assert rc == 0
    assert 'active.yaml:' in out
    assert 'robot_description.urdf.xacro:' in out
    # Generated ros2_control xacro is not patched here (regenerate step owns it).
    assert 'so_arm101_ros2_control.xacro' not in out
