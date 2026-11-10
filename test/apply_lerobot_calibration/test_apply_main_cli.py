# Copyright 2025-2026 Lucy Robotics Team
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

"""The CLI entry point, and a round trip against the real stock config."""

from __future__ import annotations

import json

from conftest import ROOT
from conftest import try_load_module
from conftest import yaml_g
import pytest
import yaml

mod, _skip_reason = try_load_module()
pytestmark = pytest.mark.skipif(mod is None, reason=_skip_reason or '')

# Fixture windows must differ from stock default.yaml so apply() is not a no-op.
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


def test_real_config_round_trips(tmp_path):
    """
    Applying the recorded ticks is idempotent on a copy of the stock config.

    Use ``configs/default.yaml`` as the template. A dirty ``active.yaml`` from a
    desk calibration must not fail this check; stock default is not required to
    already equal one arm's LeRobot window.
    """
    default = ROOT / 'config/hardware/configs/default.yaml'
    stock = default.read_text(encoding='utf-8')

    copy = tmp_path / 'active.yaml'
    copy.write_text(stock, encoding='utf-8')
    out, urdf_limits, warnings = mod.apply_to_file(copy, REAL_CALIBRATION)
    assert not warnings
    assert set(urdf_limits) == {
        'Rotation', 'Pitch', 'Elbow', 'Wrist_Pitch', 'Wrist_Roll', 'Jaw',
    }
    assert out != stock
    copy.write_text(out, encoding='utf-8')
    out2, _, warnings2 = mod.apply_to_file(copy, REAL_CALIBRATION)
    assert out2 == out
    assert not warnings2

    # Spot-check Rotation window from REAL_CALIBRATION shoulder_pan ticks.
    data = yaml.safe_load(out)
    rotation = next(a for a in data['actuators'] if a['id'] == 'rotation')
    assert rotation['limit_min'] == yaml_g(mod.ticks_to_radians(867))
    assert rotation['limit_max'] == yaml_g(mod.ticks_to_radians(3274))
    assert rotation['offset'] == yaml_g(mod.ticks_to_radians(2048))


def test_main_is_self_sufficient_for_the_real_package(tmp_path, capsys):
    """Default path creates my_custom_so101_config and leaves default.yaml alone."""
    cal_path = tmp_path / 'cal.json'
    cal_path.write_text(json.dumps(REAL_CALIBRATION), encoding='utf-8')
    rc = mod.main([str(cal_path), '--dry-run'])
    out = capsys.readouterr().out
    assert rc == 0
    assert 'my_custom_so101_config.yaml' in out
    assert 'active.yaml' in out
    assert 'active_meta.yaml' in out
    assert 'robot_description.urdf.xacro:' in out
    assert 'configs/default.yaml' not in out or 'template:' in out
    # Generated ros2_control xacro is not patched here (regenerate step owns it).
    assert 'so_arm101_ros2_control.xacro' not in out


def test_main_refuses_stock_preset_name(tmp_path, capsys):
    cal = tmp_path / 'cal.json'
    cal.write_text(json.dumps(REAL_CALIBRATION), encoding='utf-8')
    rc = mod.main([str(cal), '--preset-name', 'default', '--dry-run'])
    err = capsys.readouterr().err
    assert rc == 1
    assert 'default' in err
