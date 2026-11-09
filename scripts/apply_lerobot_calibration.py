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

import argparse
from datetime import datetime
from datetime import timezone
from pathlib import Path
import sys

import yaml

try:
    from lucy_config_generator import calibration as _calibration
except ImportError:  # Unsourced workspace: the package is a sibling under src/.
    sys.path.insert(
        0, str(Path(__file__).resolve().parents[2] / 'lucy_ros_packages/lucy_config_generator')
    )
    from lucy_config_generator import calibration as _calibration

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


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='microseconds').replace('+00:00', 'Z')


def template_config(package_root: Path) -> Path:
    """Stock template to read (never written by the default apply path)."""
    stock = package_root / "config/hardware/configs" / f"{STOCK_PRESET_NAME}.yaml"
    if stock.is_file():
        return stock
    return package_root / "config/hardware/active.yaml"


def preset_path(package_root: Path, preset_name: str) -> Path:
    return package_root / "config/hardware/configs" / f"{preset_name}.yaml"


def write_active_meta(package_root: Path, preset_name: str) -> None:
    meta = package_root / 'config/hardware/active_meta.yaml'
    payload = {}
    if meta.is_file():
        payload = yaml.safe_load(meta.read_text(encoding='utf-8')) or {}
    payload.update({'name': preset_name, 'activated_at': _now()})
    meta.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    """Apply the calibration named on the command line, or report why it cannot be."""
    parser = argparse.ArgumentParser(
        description=(
            "Apply LeRobot calibration to a new Lucy hardware preset "
            f"(default name: {DEFAULT_PRESET_NAME}; never overwrites "
            f"configs/{STOCK_PRESET_NAME}.yaml)."
        ),
    )
    parser.add_argument("calibration", type=Path, help="lerobot-calibrate JSON")
    parser.add_argument(
        "--preset-name",
        default=DEFAULT_PRESET_NAME,
        help=(
            f"new preset basename under config/hardware/configs/ "
            f"(default: {DEFAULT_PRESET_NAME}; refused if '{STOCK_PRESET_NAME}')"
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        action="append",
        dest="configs",
        help=(
            "explicit hardware YAML to patch in place (repeatable; skips "
            "creating a new preset / updating active_meta)"
        ),
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=None,
        help=f"URDF xacro to patch joint limits (default: {DEFAULT_URDF_REL})",
    )
    parser.add_argument(
        "--no-urdf",
        action="store_true",
        help="do not rewrite URDF joint lower/upper from the calibration window",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report what would change without writing",
    )
    args = parser.parse_args(argv)

    package_root = Path(__file__).resolve().parents[1]
    urdf_path = None if args.no_urdf else (args.urdf or package_root / DEFAULT_URDF_REL)
    preset_name = str(args.preset_name).strip()
    use_new_preset = not args.configs

    try:
        if use_new_preset:
            if not preset_name or "/" in preset_name or preset_name == STOCK_PRESET_NAME:
                raise CalibrationError(
                    f"invalid --preset-name {preset_name!r}: use a basename other "
                    f"than {STOCK_PRESET_NAME!r}"
                )
            source = template_config(package_root)
            if not source.is_file():
                raise CalibrationError(f"template config not found: {source}")
            dest_preset = preset_path(package_root, preset_name)
            active = package_root / "config/hardware/active.yaml"
            write_paths = [dest_preset, active]
        else:
            write_paths = list(args.configs)
            source = write_paths[0]

        calibration = load_calibration(args.calibration)
        new_text, urdf_limits, warnings = apply_to_file(source, calibration)
        if use_new_preset:
            new_text = set_calibration_block(
                new_text,
                calibrated_at=_now(),
                homing_offsets=homing_offsets_by_actuator(
                    calibration, yaml.safe_load(new_text) or {}
                ),
            )
        new_urdf = None
        if urdf_path is not None:
            if not urdf_path.is_file():
                raise CalibrationError(f"URDF not found: {urdf_path}")
            new_urdf = apply_urdf_limits(urdf_path.read_text(encoding="utf-8"), urdf_limits)
    except CalibrationError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    for message in dict.fromkeys(warnings):
        print(f"warning: {message}", file=sys.stderr)

    if use_new_preset:
        print(f"template: {source}")

    for path in write_paths:
        if path.name == f"{STOCK_PRESET_NAME}.yaml" and path.parent.name == "configs":
            print(
                f"error: refusing to overwrite stock preset {path}",
                file=sys.stderr,
            )
            return 1
        exists = path.is_file()
        unchanged = exists and new_text == path.read_text(encoding="utf-8")
        if args.dry_run:
            if not exists:
                print(f"{path}: would be created")
            else:
                print(f"{path}: {'no change' if unchanged else 'would be updated'}")
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        if unchanged:
            print(f"{path}: already up to date")
        else:
            path.write_text(new_text, encoding="utf-8")
            print(f"{path}: {'created' if not exists else 'updated'}")

    if use_new_preset:
        meta = package_root / "config/hardware/active_meta.yaml"
        if args.dry_run:
            print(f"{meta}: would set name={preset_name!r}")
        else:
            write_active_meta(package_root, preset_name)
            print(f"{meta}: name={preset_name!r}")

    if urdf_path is not None and new_urdf is not None:
        unchanged = new_urdf == urdf_path.read_text(encoding="utf-8")
        if args.dry_run:
            print(f"{urdf_path}: {'no change' if unchanged else 'would be updated'}")
        elif unchanged:
            print(f"{urdf_path}: already up to date")
        else:
            urdf_path.write_text(new_urdf, encoding="utf-8")
            print(f"{urdf_path}: updated")

    if not args.dry_run:
        print(
            "\nRegenerate the package configs so the new windows reach the "
            "ros2_control and gazebo xacros (see docs/HARDWARE_SETUP.md), then "
            "restart the stack with real:=true."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
