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

"""Rejecting a malformed or ambiguous calibration record before it is applied."""

from __future__ import annotations

import json

from conftest import CALIBRATION
from conftest import mod
import pytest


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


def test_unknown_drive_mode_is_refused(tmp_path):
    calibration = json.loads(json.dumps(CALIBRATION))
    calibration['gripper']['drive_mode'] = 2
    path = tmp_path / 'cal.json'
    path.write_text(json.dumps(calibration), encoding='utf-8')
    with pytest.raises(mod.CalibrationError, match='unknown drive_mode'):
        mod.load_calibration(path)
