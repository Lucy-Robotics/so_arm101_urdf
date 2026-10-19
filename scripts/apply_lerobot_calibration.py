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

import math
import argparse
import json
import re
import sys
from datetime import datetime
from datetime import timezone
from pathlib import Path

import yaml

# Firmware / YAML map 0–2π rad linearly onto the STS3215 0–4096 tick span.
TICKS_PER_TURN = 4096
# LeRobot half-turn homing centres Present_Position here (= URDF joint 0).
CENTRE_TICKS = TICKS_PER_TURN // 2

REQUIRED_RECORD_KEYS = ("id", "range_min", "range_max")
DEFAULT_URDF_REL = Path("description/robot_description/urdf/robot_description.urdf.xacro")
DEFAULT_PRESET_NAME = "my_custom_so101_config"
STOCK_PRESET_NAME = "default"


class CalibrationError(RuntimeError):
    """A calibration that cannot be applied to this config."""


def ticks_to_radians(ticks: float) -> float:
    """Convert raw encoder ticks to radians stored in the hardware YAML."""
    return round(ticks * (2.0 * math.pi) / TICKS_PER_TURN, 6)


def servo_window_to_joint_limits(
    limit_min: float,
    limit_max: float,
    offset: float,
    direction: float,
    scale: float,
) -> tuple[float, float]:
    """Map a servo-frame travel window to URDF joint lower/upper (sorted)."""
    denom = direction * scale
    if denom == 0:
        raise CalibrationError("direction*scale must be non-zero")
    a = (limit_min - offset) * denom
    b = (limit_max - offset) * denom
    lo, hi = (a, b) if a <= b else (b, a)
    return round(lo, 6), round(hi, 6)


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


def plan(
    calibration: dict[str, dict], config: dict
) -> tuple[dict, dict[str, tuple[float, float]], list[str]]:
    """
    Pair calibration records with actuators by servo id, and derive their values.

    Returns per-actuator YAML field updates, per-``urdf_joint`` limit pairs, and
    warnings. Raises when a record matches no actuator or two actuators share
    its id.
    """
    actuators = config.get("actuators") or []
    warnings: list[str] = []
    updates: dict[str, dict[str, float]] = {}
    urdf_limits: dict[str, tuple[float, float]] = {}

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
        limit_min = ticks_to_radians(record["range_min"])
        limit_max = ticks_to_radians(record["range_max"])
        fields = {
            "offset": centre,
            "limit_min": limit_min,
            "limit_max": limit_max,
            "default": centre,
        }
        updates[actuator_id] = fields

        urdf_joint = str(actuator.get("urdf_joint") or "").strip()
        if urdf_joint:
            direction = float(actuator.get("direction", 1))
            scale = float(actuator.get("scale", 1))
            urdf_limits[urdf_joint] = servo_window_to_joint_limits(
                limit_min, limit_max, centre, direction, scale
            )

    unmatched = [str(a["id"]) for a in actuators if str(a["id"]) not in updates]
    if unmatched:
        warnings.append(
            f"no calibration record for: {', '.join(sorted(unmatched))} "
            f"(left untouched)"
        )
    return updates, urdf_limits, warnings


def sensor_updates(config: dict, updates: dict[str, dict]) -> dict[str, dict]:
    """Mirror each actuator's new window onto the sensors that report on it."""
    out: dict[str, dict[str, float]] = {}
    for sensor in config.get("sensors") or []:
        actuator = str(sensor.get("associated_actuator", ""))
        if actuator in updates:
            out[str(sensor["id"])] = {
                "min_value": updates[actuator]["limit_min"],
                "max_value": updates[actuator]["limit_max"],
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


def apply_urdf_limits(text: str, limits: dict[str, tuple[float, float]]) -> str:
    """Rewrite ``<limit ... lower= upper=>`` on named joints; keep other attrs."""
    for joint, (lo, hi) in limits.items():
        pattern = (
            rf'(<joint\s+name="{re.escape(joint)}"[^>]*>.*?'
            rf'<limit\b)([^>]*)(/>)'
        )

        def _rewrite(match: re.Match[str], *, lower: float = lo, upper: float = hi) -> str:
            attrs = match.group(2)
            attrs, n_lo = re.subn(r'\blower="[^"]*"', f'lower="{lower:g}"', attrs)
            attrs, n_hi = re.subn(r'\bupper="[^"]*"', f'upper="{upper:g}"', attrs)
            if n_lo != 1 or n_hi != 1:
                raise CalibrationError(
                    f'joint {joint!r}: expected one lower and one upper on <limit>, '
                    f"found lower={n_lo} upper={n_hi}"
                )
            return f"{match.group(1)}{attrs}{match.group(3)}"

        text, count = re.subn(pattern, _rewrite, text, count=1, flags=re.S)
        if count != 1:
            raise CalibrationError(f"joint {joint!r}: no <limit> found in URDF xacro")
    return text


def template_config(package_root: Path) -> Path:
    """Stock template to read (never written by the default apply path)."""
    stock = package_root / "config/hardware/configs" / f"{STOCK_PRESET_NAME}.yaml"
    if stock.is_file():
        return stock
    return package_root / "config/hardware/active.yaml"


def preset_path(package_root: Path, preset_name: str) -> Path:
    return package_root / "config/hardware/configs" / f"{preset_name}.yaml"


def write_active_meta(package_root: Path, preset_name: str) -> None:
    meta = package_root / "config/hardware/active_meta.yaml"
    payload = {
        "name": preset_name,
        "activated_at": datetime.now(timezone.utc)
        .isoformat(timespec="microseconds")
        .replace("+00:00", "Z"),
    }
    meta.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def apply_to_file(
    path: Path, calibration: dict[str, dict]
) -> tuple[str, dict[str, tuple[float, float]], list[str]]:
    """Return rewritten YAML text, URDF limit map, and warnings."""
    text = path.read_text(encoding="utf-8")
    config = yaml.safe_load(text) or {}
    updates, urdf_limits, warnings = plan(calibration, config)
    updates.update(sensor_updates(config, updates))
    return apply_updates(text, updates), urdf_limits, warnings


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
            "ros2_control and gazebo xacros (see REAL_HARDWARE_SETUP.md), then "
            "restart the stack with real:=true."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
