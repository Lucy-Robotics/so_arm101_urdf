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

"""Ticks-to-radians, URDF limit rewriting, and the persisted calibration block."""

from __future__ import annotations

from conftest import apply
from conftest import CALIBRATION
from conftest import try_load_module
import pytest
import yaml

mod, _skip_reason = try_load_module()
pytestmark = pytest.mark.skipif(mod is None, reason=_skip_reason or '')


def test_ticks_to_radians_matches_the_firmware_scale():
    """0-4096 ticks span 0-2π rad, so the centre is π."""
    assert mod.ticks_to_radians(0) == 0.0
    assert mod.ticks_to_radians(4096) == 6.283185
    assert mod.ticks_to_radians(2048) == 3.141593
    assert mod.ticks_to_radians(768) == 1.178097


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


def test_set_calibration_block_records_the_offsets(tmp_path):
    out, _, _ = apply(mod, tmp_path=tmp_path)
    config = yaml.safe_load(out)
    block = yaml.safe_load(mod.set_calibration_block(
        out,
        calibrated_at='T',
        homing_offsets=mod.homing_offsets_by_actuator(CALIBRATION, config),
    ))['calibration']
    assert block['homing_offsets']['jaw'] == 1534
    assert block['adapter_serial'] == ''
