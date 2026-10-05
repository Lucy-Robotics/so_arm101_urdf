#!/usr/bin/env python3
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
"""
Apply a LeRobot follower calibration to the Lucy hardware config.

Usage::
    cd src/so_arm101_urdf
    scripts/apply_lerobot_calibration.py \
        ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/<arm>.json

    scripts/apply_lerobot_calibration.py cal.json --dry-run
    scripts/apply_lerobot_calibration.py cal.json --config path/to/other.yaml
"""

from __future__ import annotations

import math
import argparse
import json
import re
import sys
from pathlib import Path

import yaml

# Firmware / YAML map 0–2π rad linearly onto the STS3215 0–4096 tick span.
TICKS_PER_TURN = 4096
# Calibration centres every joint here, which is what makes it joint zero.
CENTRE_TICKS = TICKS_PER_TURN // 2

REQUIRED_RECORD_KEYS = ("id", "range_min", "range_max")


class CalibrationError(RuntimeError):
    """A calibration that cannot be applied to this config."""


def ticks_to_radians(ticks: float) -> float:
    """Convert raw encoder ticks to radians stored in the hardware YAML."""
    return round(ticks * (2.0 * math.pi) / TICKS_PER_TURN, 6)


def load_calibration(path: Path) -> dict[str, dict]:
    """Read a lerobot-calibrate JSON, rejecting records missing required keys."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        raise CalibrationError(f"cannot read calibration {path}: {e}") from e
    if not isinstance(data, dict) or not data:
        raise CalibrationError(f"{path}: expected a non-empty object of motor records")

    for motor, record in data.items():
        if not isinstance(record, dict):
            raise CalibrationError(f"{path}: {motor!r} is not an object")
        missing = [k for k in REQUIRED_RECORD_KEYS if k not in record]
        if missing:
            raise CalibrationError(f"{path}: {motor!r} is missing {missing}")
        if record["range_min"] >= record["range_max"]:
            raise CalibrationError(
                f"{path}: {motor!r} has range_min {record['range_min']} "
                f">= range_max {record['range_max']}"
            )
    return data


def plan(calibration: dict[str, dict], config: dict) -> tuple[dict, list[str]]:
    """
    Pair calibration records with actuators by servo id, and derive their values.

    Returns the per-actuator-id field updates plus any warnings worth printing.
    Raises when a record matches no actuator or two actuators share its id.
    """
    actuators = config.get("actuators") or []
    warnings: list[str] = []
    updates: dict[str, dict[str, float]] = {}

    by_pin: dict[int, list[dict]] = {}
    for actuator in actuators:
        pin = actuator.get("physical_pin")
        if pin is not None:
            by_pin.setdefault(int(pin), []).append(actuator)

    centre = ticks_to_radians(CENTRE_TICKS)
    for motor, record in sorted(calibration.items(), key=lambda kv: kv[1]["id"]):
        servo_id = int(record["id"])
        matched = by_pin.get(servo_id, [])
        if not matched:
            raise CalibrationError(
                f"{motor!r} (servo id {servo_id}) matches no actuator: no "
                f"physical_pin {servo_id} in the config"
            )
        if len(matched) > 1:
            ids = [str(a["id"]) for a in matched]
            raise CalibrationError(
                f"{motor!r} (servo id {servo_id}) matches several actuators "
                f"({', '.join(ids)}); physical_pin must be unique"
            )
        actuator = matched[0]
        actuator_id = str(actuator["id"])
        if record.get("drive_mode"):
            warnings.append(
                f"{motor}: drive_mode={record['drive_mode']} means LeRobot drives "
                f"this joint inverted. direction is left at its current value; "
                f"check {actuator_id} against the 3D view."
            )
        fields = {
            "offset_rad": centre,
            "servo_min_rad": ticks_to_radians(record["range_min"]),
            "servo_max_rad": ticks_to_radians(record["range_max"]),
            "servo_default_rad": centre,
        }
        updates[actuator_id] = fields

    unmatched = [str(a["id"]) for a in actuators if str(a["id"]) not in updates]
    if unmatched:
        warnings.append(
            f"no calibration record for: {', '.join(sorted(unmatched))} "
            f"(left untouched)"
        )
    return updates, warnings


def sensor_updates(config: dict, updates: dict[str, dict]) -> dict[str, dict]:
    """Mirror each actuator's new window onto the sensors that report on it."""
    out: dict[str, dict[str, float]] = {}
    for sensor in config.get("sensors") or []:
        actuator = str(sensor.get("associated_actuator", ""))
        if actuator in updates:
            out[str(sensor["id"])] = {
                "min_value": updates[actuator]["servo_min_rad"],
                "max_value": updates[actuator]["servo_max_rad"],
            }
    return out


def _block_span(text: str, entry_id: str) -> tuple[int, int]:
    """Character span of the `- id: <entry_id>` list entry, up to the next one."""
    opener = re.search(rf"(?m)^\s*-\s+id:\s+{re.escape(entry_id)}\s*$", text)
    if opener is None:
        raise CalibrationError(f'no "- id: {entry_id}" entry found')
    start = opener.start()
    following = re.search(r"(?m)^\s*-\s+id:\s+\S+\s*$", text[opener.end():])
    end = opener.end() + following.start() if following else len(text)
    return start, end


def apply_updates(text: str, updates: dict[str, dict]) -> str:
    """Rewrite the named keys inside each entry, leaving comments and order alone."""
    for entry_id, fields in updates.items():
        start, end = _block_span(text, entry_id)
        block = text[start:end]
        for key, value in fields.items():
            block, count = re.subn(rf"(?m)^(\s*{key}:).*$", rf"\g<1> {value:g}", block)
            if count != 1:
                raise CalibrationError(
                    f'{entry_id}: expected one "{key}:" line, found {count}'
                )
        text = text[:start] + block + text[end:]
    return text


def default_targets(package_root: Path) -> list[Path]:
    """Return the active config, plus the preset behind it, so re-activation keeps this."""
    targets = [package_root / "config/hardware/active.yaml"]
    meta = package_root / "config/hardware/active_meta.yaml"
    try:
        name = (yaml.safe_load(meta.read_text(encoding="utf-8")) or {}).get("name")
    except (OSError, yaml.YAMLError):
        return targets
    if isinstance(name, str) and name.strip():
        preset = package_root / "config/hardware/configs" / f"{name.strip()}.yaml"
        if preset.exists():
            targets.append(preset)
    return targets


def apply_to_file(path: Path, calibration: dict[str, dict]) -> tuple[str, list[str]]:
    """Return the rewritten text for one config, plus its warnings."""
    text = path.read_text(encoding="utf-8")
    config = yaml.safe_load(text) or {}
    updates, warnings = plan(calibration, config)
    updates.update(sensor_updates(config, updates))
    return apply_updates(text, updates), warnings


def main(argv: list[str] | None = None) -> int:
    """Apply the calibration named on the command line, or report why it cannot be."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("calibration", type=Path, help="lerobot-calibrate JSON")
    parser.add_argument(
        "--config",
        type=Path,
        action="append",
        dest="configs",
        help="hardware YAML to patch (repeatable; default: active.yaml + its preset)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report what would change without writing",
    )
    args = parser.parse_args(argv)

    package_root = Path(__file__).resolve().parents[1]
    targets = args.configs or default_targets(package_root)

    try:
        calibration = load_calibration(args.calibration)
        results = [(path, *apply_to_file(path, calibration)) for path in targets]
    except CalibrationError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    for message in dict.fromkeys(w for _, _, warns in results for w in warns):
        print(f"warning: {message}", file=sys.stderr)

    for path, new_text, _ in results:
        unchanged = new_text == path.read_text(encoding="utf-8")
        if args.dry_run:
            print(f"{path}: {'no change' if unchanged else 'would be updated'}")
            continue
        if unchanged:
            print(f"{path}: already up to date")
        else:
            path.write_text(new_text, encoding="utf-8")
            print(f"{path}: updated")

    if not args.dry_run:
        print(
            "\nRegenerate the package configs so the new windows reach the "
            "ros2_control and gazebo xacros (see DEVELOPER.md), then restart "
            "the stack."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
