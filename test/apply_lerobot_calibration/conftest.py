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

"""Shared fixtures for the apply_lerobot_calibration tests."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load_module():
    path = ROOT / 'scripts' / 'apply_lerobot_calibration.py'
    spec = importlib.util.spec_from_file_location('apply_lerobot_calibration', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    mod = _load_module()
except ModuleNotFoundError as e:
    # lucy_config_generator ships in lucy_ros_packages, a sibling repo this package's own
    # standalone CI never checks out, so it is only reachable from a sourced lucy_ws workspace.
    pytest.skip(f"{e} (needs a sourced lucy_ws workspace)", allow_module_level=True)


def yaml_g(value: float) -> float:
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


def apply(text=CONFIG, calibration=None, tmp_path=None):
    path = tmp_path / 'active.yaml'
    path.write_text(text, encoding='utf-8')
    new_text, urdf_limits, warnings = mod.apply_to_file(
        path, calibration or CALIBRATION
    )
    return new_text, warnings, urdf_limits
