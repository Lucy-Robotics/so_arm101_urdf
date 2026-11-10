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

"""Applying a calibration's windows and centre onto the hardware config."""

from __future__ import annotations

import json

from conftest import apply
from conftest import CALIBRATION
from conftest import try_load_module
from conftest import yaml_g
import pytest
import yaml

mod, _skip_reason = try_load_module()
pytestmark = pytest.mark.skipif(mod is None, reason=_skip_reason or '')


def test_windows_and_centre_are_written(tmp_path):
    """Ranges convert to radians and joint zero lands on the homed centre."""
    out, _, urdf_limits = apply(mod, tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}

    rotation = actuators['rotation']
    assert rotation['limit_min'] == yaml_g(mod.ticks_to_radians(768))
    assert rotation['limit_max'] == yaml_g(mod.ticks_to_radians(3273))
    assert rotation['offset'] == yaml_g(mod.ticks_to_radians(2048))
    assert rotation['default'] == yaml_g(mod.ticks_to_radians(2048))

    # Not the midpoint of the window: the gripper's travel is far from symmetric.
    jaw = actuators['jaw']
    assert jaw['limit_min'] == yaml_g(mod.ticks_to_radians(2046))
    assert jaw['offset'] == yaml_g(mod.ticks_to_radians(2048))

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
    out, _, _ = apply(mod, tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    # 'gripper' (id 6) reached 'jaw' (physical_pin 6), not the alphabetically near
    # 'rotation'.
    assert actuators['jaw']['limit_max'] == yaml_g(mod.ticks_to_radians(3277))
    assert actuators['rotation']['limit_max'] == yaml_g(mod.ticks_to_radians(3273))


def test_untouched_fields_and_comments_survive(tmp_path):
    """The file is edited as text, so nothing outside the named keys moves."""
    out, _, _ = apply(mod, tmp_path=tmp_path)
    assert '# A comment that must survive the rewrite.' in out
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['jaw']['direction'] == -1
    assert actuators['rotation']['urdf_joint'] == 'Rotation'
    assert actuators['jaw']['enabled'] is True


def test_sensor_bounds_follow_their_actuator(tmp_path):
    """Encoder bounds stay in the same space as the actuator they report on."""
    out, _, _ = apply(mod, tmp_path=tmp_path)
    sensor = yaml.safe_load(out)['sensors'][0]
    assert sensor['min_value'] == yaml_g(mod.ticks_to_radians(768))
    assert sensor['max_value'] == yaml_g(mod.ticks_to_radians(3273))
    assert sensor['enabled'] is False


def test_applying_twice_changes_nothing(tmp_path):
    """Re-running on an already-calibrated config is a no-op."""
    once, _, _ = apply(mod, tmp_path=tmp_path)
    twice, _, _ = apply(mod, text=once, tmp_path=tmp_path)
    assert once == twice


def test_unmatched_record_refuses_to_write(tmp_path):
    """A half-applied calibration drives joints against someone else's window."""
    calibration = dict(CALIBRATION, elbow_flex={
        'id': 3, 'drive_mode': 0, 'homing_offset': 0,
        'range_min': 866, 'range_max': 3083,
    })
    with pytest.raises(mod.CalibrationError, match='matches no actuator'):
        apply(mod, calibration=calibration, tmp_path=tmp_path)


def test_drive_mode_is_reported_not_applied(tmp_path):
    """Leave direction alone: an inversion is surfaced for a human, not guessed."""
    calibration = json.loads(json.dumps(CALIBRATION))
    calibration['gripper']['drive_mode'] = 1
    out, warnings, _ = apply(mod, calibration=calibration, tmp_path=tmp_path)
    assert any('drive_mode=1' in w for w in warnings)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['jaw']['direction'] == -1


def test_actuator_without_a_record_is_left_alone(tmp_path):
    """Only calibrated joints move; the rest keep their values and are reported."""
    out, warnings, _ = apply(mod, calibration={'gripper': CALIBRATION['gripper']},
                             tmp_path=tmp_path)
    actuators = {a['id']: a for a in yaml.safe_load(out)['actuators']}
    assert actuators['rotation']['limit_min'] == 1.396263
    assert any('rotation' in w for w in warnings)
