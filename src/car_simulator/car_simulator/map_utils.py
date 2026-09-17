"""Load/save the ground-truth cone map used by the simulator.

File format (YAML):

    cones:
      - {x: 1.0, y: 0.5}
      - {x: 1.0, y: -0.5}
      ...

Swapping tracks is just a matter of pointing the `map_file` parameter of the
simulator nodes at a different YAML file.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Tuple

import yaml

ConePoint = Tuple[float, float]


def load_cone_map(path: str) -> List[ConePoint]:
    """Read a cone map YAML file and return a list of (x, y) tuples.

    Returns an empty list if the file is missing or malformed instead of
    raising, so a simulator node can keep running while a map is fixed.
    """
    p = Path(path)
    if not p.exists():
        return []

    try:
        with p.open('r') as f:
            data = yaml.safe_load(f) or {}
    except yaml.YAMLError:
        return []

    cones = data.get('cones', []) if isinstance(data, dict) else []
    points: List[ConePoint] = []
    for cone in cones:
        try:
            x = float(cone['x'])
            y = float(cone['y'])
        except (KeyError, TypeError, ValueError):
            continue
        points.append((x, y))
    return points


def save_cone_map(path: str, points: List[ConePoint]) -> None:
    """Write a list of (x, y) points to a cone map YAML file."""
    data = {'cones': [{'x': float(x), 'y': float(y)} for x, y in points]}
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def get_mtime(path: str) -> float:
    """Return the file modification time, or 0.0 if the file does not exist."""
    try:
        return os.path.getmtime(path)
    except OSError:
        return 0.0
