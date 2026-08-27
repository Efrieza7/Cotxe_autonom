"""Utility helpers for the fsd path-planning bridge."""

from __future__ import annotations

from typing import List, Sequence

import numpy as np


def build_unknown_cone_observations(
    cons_data: Sequence[float],
    cone_types_count: int,
    unknown_index: int,
    min_cone_count: int = 1,
) -> List[np.ndarray]:
    """Convert ConsMap interleaved data [x, y, count, ...] into planner cone arrays."""
    valid_points = []
    usable_len = len(cons_data) - (len(cons_data) % 3)

    for i in range(0, usable_len, 3):
        try:
            x = float(cons_data[i])
            y = float(cons_data[i + 1])
            count = int(cons_data[i + 2])
        except (TypeError, ValueError):
            continue

        if not np.isfinite(x) or not np.isfinite(y):
            continue

        if count < int(min_cone_count):
            continue

        valid_points.append([x, y])

    cone_observations = [np.zeros((0, 2), dtype=np.float64) for _ in range(cone_types_count)]
    cone_observations[unknown_index] = (
        np.asarray(valid_points, dtype=np.float64)
        if valid_points
        else np.zeros((0, 2), dtype=np.float64)
    )
    return cone_observations


def extract_xy_path(planner_result) -> np.ndarray | None:
    """Extract Nx2 [x,y] points from fsd planner outputs."""
    if planner_result is None:
        return None

    array = np.asarray(planner_result)


    # fsd_path_planning returns [s, x, y, curvature].
    # Keep a strict branch for Nx2 arrays so the intent is explicit.
    if array.shape[1] >= 3:
        path_xy = array[:, 1:3]
    elif array.shape[1] == 2:
        path_xy = array[:, :2]
    else:
        return None

    if not np.all(np.isfinite(path_xy)):
        return None

    return path_xy
