#!/usr/bin/env python3
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
r"""
Apply a LeRobot follower calibration to the Lucy hardware config + URDF limits.

LeRobot's half-turn homing makes the pose you hold during calibration read as
tick 2048 (π rad on Lucy's 0–2π ↔ 0–4096 map). That pose is the URDF default
(joint 0). Lucy therefore stores:

* ``offset`` / ``default`` = π (servo-frame centre)
* ``limit_min`` / ``limit_max`` = travel window in servo-frame radians
* URDF ``lower`` / ``upper`` = the same window mapped to joint space:
  ``joint = (servo - offset) * direction * scale``

The hardware interface maps joint commands → servo with the inverse, and
publishes joint_states in joint space so the 3D view matches the real arm.

Usage (from ``lucy_ws`` root)::

    src/so_arm101_urdf/scripts/apply_lerobot_calibration.py \\
        ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<arm>.json

Writes ``config/hardware/configs/my_custom_so101_config.yaml`` (stock
``default.yaml`` is left untouched), mirrors it to ``active.yaml``, and points
``active_meta.yaml`` at that preset. Override the name with ``--preset-name``.

    ... cal.json --dry-run
    ... cal.json --config path/to/other.yaml
    ... cal.json --no-urdf
"""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from lucy_config_generator import calibration as _calibration
except ImportError:  # Unsourced workspace: the package is a sibling under src/.
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[2] / 'lucy_ros_packages/lucy_config_generator')
    )
    from lucy_config_generator import calibration as _calibration

from apply_lerobot_calibration_apply import run  # noqa: E402 (needs the path fix above)
from apply_lerobot_calibration_cli import build_parser  # noqa: E402
from apply_lerobot_calibration_io import now as _now  # noqa: E402

CalibrationError = _calibration.CalibrationError
apply_to_file = _calibration.apply_to_file
apply_updates = _calibration.apply_updates
apply_urdf_limits = _calibration.apply_urdf_limits
homing_offsets_by_actuator = _calibration.homing_offsets_by_actuator
load_calibration = _calibration.load_calibration
plan = _calibration.plan
sensor_updates = _calibration.sensor_updates
servo_window_to_joint_limits = _calibration.servo_window_to_joint_limits
set_calibration_block = _calibration.set_calibration_block
ticks_to_radians = _calibration.ticks_to_radians

CENTRE_TICKS = _calibration.CENTRE_TICKS
TICKS_PER_TURN = _calibration.TICKS_PER_TURN

DEFAULT_URDF_REL = Path('description/robot_description/urdf/robot_description.urdf.xacro')
DEFAULT_PRESET_NAME = 'my_custom_so101_config'
STOCK_PRESET_NAME = 'default'


def main(argv: list[str] | None = None) -> int:
    """Apply the calibration named on the command line, or report why it cannot be."""
    parser = build_parser(DEFAULT_PRESET_NAME, STOCK_PRESET_NAME, DEFAULT_URDF_REL)
    args = parser.parse_args(argv)

    package_root = Path(__file__).resolve().parents[1]
    urdf_path = None if args.no_urdf else (args.urdf or package_root / DEFAULT_URDF_REL)
    return run(args, package_root, _now(), STOCK_PRESET_NAME, urdf_path)


if __name__ == "__main__":
    sys.exit(main())
