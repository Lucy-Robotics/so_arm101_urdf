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

"""The command-line surface for applying a LeRobot calibration."""

from __future__ import annotations

import argparse
from pathlib import Path


def build_parser(
    default_preset_name: str, stock_preset_name: str, default_urdf_rel: Path
) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Apply LeRobot calibration to a new Lucy hardware preset "
            f"(default name: {default_preset_name}; never overwrites "
            f"configs/{stock_preset_name}.yaml)."
        ),
    )
    parser.add_argument("calibration", type=Path, help="lerobot-calibrate JSON")
    parser.add_argument(
        "--preset-name",
        default=default_preset_name,
        help=(
            f"new preset basename under config/hardware/configs/ "
            f"(default: {default_preset_name}; refused if '{stock_preset_name}')"
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
        help=f"URDF xacro to patch joint limits (default: {default_urdf_rel})",
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
    return parser
