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

"""Reading a calibration, patching the hardware config and URDF, and reporting it."""

from __future__ import annotations

from pathlib import Path
import sys

from apply_lerobot_calibration_io import preset_path
from apply_lerobot_calibration_io import template_config
from apply_lerobot_calibration_io import write_active_meta
from lucy_config_generator import calibration as _calibration
import yaml

CalibrationError = _calibration.CalibrationError
apply_to_file = _calibration.apply_to_file
apply_urdf_limits = _calibration.apply_urdf_limits
homing_offsets_by_actuator = _calibration.homing_offsets_by_actuator
load_calibration = _calibration.load_calibration
set_calibration_block = _calibration.set_calibration_block


def run(
    args, package_root: Path, now: str, stock_preset_name: str, urdf_path: Path | None
) -> int:
    """Apply the calibration described by ``args``, or report why it cannot be."""
    preset_name = str(args.preset_name).strip()
    use_new_preset = not args.configs

    try:
        if use_new_preset:
            if not preset_name or "/" in preset_name or preset_name == stock_preset_name:
                raise CalibrationError(
                    f"invalid --preset-name {preset_name!r}: use a basename other "
                    f"than {stock_preset_name!r}"
                )
            source = template_config(package_root, stock_preset_name)
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
                calibrated_at=now,
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
        if path.name == f"{stock_preset_name}.yaml" and path.parent.name == "configs":
            print(f"error: refusing to overwrite stock preset {path}", file=sys.stderr)
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
            "restart the stack with mode:=real bridge:=linux."
        )
    return 0
