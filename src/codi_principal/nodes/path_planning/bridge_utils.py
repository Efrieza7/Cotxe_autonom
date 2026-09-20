"""Utility helpers for the fsd path-planning bridge."""

from __future__ import annotations

from typing import List, Sequence

import numpy as np


def build_cone_observations(
    cons_data: Sequence[float],
    cone_types_count: int,
    unknown_index: int,
    min_cone_count: int = 1,
    vehicle_position: Sequence[float] | None = None,
    vehicle_direction: Sequence[float] | None = None,
    left_index: int | None = None,
    right_index: int | None = None,
    lateral_deadband: float = 1e-6,
    heading_min_norm: float = 1e-9,
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

    if not valid_points:
        return cone_observations

    valid_points_array = np.asarray(valid_points, dtype=np.float64)
    can_split_sides = (
        vehicle_position is not None
        and vehicle_direction is not None
        and left_index is not None
        and right_index is not None
    )
    if not can_split_sides:
        cone_observations[unknown_index] = valid_points_array
        return cone_observations

    vehicle_position_array = np.asarray(vehicle_position, dtype=np.float64)
    vehicle_direction_array = np.asarray(vehicle_direction, dtype=np.float64)
    cone_indices = {unknown_index, left_index, right_index}

    if (
        len(cone_indices) != 3
        or min(cone_indices) < 0
        or max(cone_indices) >= cone_types_count
    ):
        cone_observations[unknown_index] = valid_points_array
        return cone_observations

    if vehicle_position_array.shape != (2,) or vehicle_direction_array.shape != (2,):
        cone_observations[unknown_index] = valid_points_array
        return cone_observations
    if not (
        np.all(np.isfinite(vehicle_position_array))
        and np.all(np.isfinite(vehicle_direction_array))
    ):
        cone_observations[unknown_index] = valid_points_array
        return cone_observations

    direction_norm = np.linalg.norm(vehicle_direction_array)
    if not np.isfinite(direction_norm) or direction_norm <= heading_min_norm:
        cone_observations[unknown_index] = valid_points_array
        return cone_observations

    direction_unit = vehicle_direction_array / direction_norm
    relative_points = valid_points_array - vehicle_position_array
    lateral_offsets = (
        direction_unit[0] * relative_points[:, 1]
        - direction_unit[1] * relative_points[:, 0]
    )

    left_mask = lateral_offsets > lateral_deadband
    right_mask = lateral_offsets < -lateral_deadband
    unknown_mask = ~(left_mask | right_mask)

    cone_observations[left_index] = valid_points_array[left_mask]
    cone_observations[right_index] = valid_points_array[right_mask]
    cone_observations[unknown_index] = valid_points_array[unknown_mask]
    return cone_observations


def build_unknown_cone_observations(
    cons_data: Sequence[float],
    cone_types_count: int,
    unknown_index: int,
    min_cone_count: int = 1,
) -> List[np.ndarray]:
    """Convert ConsMap data into planner arrays with all cones marked unknown."""
    return build_cone_observations(
        cons_data=cons_data,
        cone_types_count=cone_types_count,
        unknown_index=unknown_index,
        min_cone_count=min_cone_count,
    )


def extract_xy_path(planner_result) -> np.ndarray | None:
    """Extract Nx2 [x,y] points from fsd planner outputs."""
    if planner_result is None:
        return None

    array = np.asarray(planner_result)
    if array.ndim == 1:
        if array.shape[0] >= 3:
            array = array.reshape(1, -1)
        else:
            return None
    if array.ndim != 2:
        return None


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
