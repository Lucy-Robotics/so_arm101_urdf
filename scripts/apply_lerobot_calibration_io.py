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

"""Where a preset's files live, and the small metadata file that points at one."""

from __future__ import annotations

from datetime import datetime
from datetime import timezone
from pathlib import Path

import yaml


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='microseconds').replace('+00:00', 'Z')


def template_config(package_root: Path, stock_preset_name: str) -> Path:
    """Stock template to read (never written by the default apply path)."""
    stock = package_root / "config/hardware/configs" / f"{stock_preset_name}.yaml"
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
    payload.update({'name': preset_name, 'activated_at': now()})
    meta.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
