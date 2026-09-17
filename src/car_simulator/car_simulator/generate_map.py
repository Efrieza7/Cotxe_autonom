"""Generate a cone map YAML file (an oval track) so users have a template
for creating their own maps.

Usage:
    ros2 run car_simulator generate_map --output my_track.yaml \
        --length 10 --width 6 --track-width 3 --spacing 2
"""

from __future__ import annotations

import argparse
import math
from typing import List, Tuple

from .map_utils import save_cone_map


def generate_oval_track(
    length: float, width: float, track_width: float, spacing: float
) -> List[Tuple[float, float]]:
    """Build two boundary lines of cones (left/right) around a rounded rectangle."""
    half_l = length / 2.0
    half_w = width / 2.0
    half_track = track_width / 2.0

    centerline: List[Tuple[float, float]] = []
    steps = max(4, int((2 * (length + width)) / max(spacing, 0.1)))
    for i in range(steps):
        t = (i / steps) * 2.0 * math.pi
        cx = half_l * math.cos(t)
        cy = half_w * math.sin(t)
        centerline.append((cx, cy))

    cones: List[Tuple[float, float]] = []
    n = len(centerline)
    for i in range(n):
        x0, y0 = centerline[i]
        x1, y1 = centerline[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / norm, dx / norm
        cones.append((x0 + nx * half_track, y0 + ny * half_track))
        cones.append((x0 - nx * half_track, y0 - ny * half_track))

    return cones


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate a sample cone map YAML file.')
    parser.add_argument('--output', default='sample_track.yaml')
    parser.add_argument('--length', type=float, default=10.0)
    parser.add_argument('--width', type=float, default=6.0)
    parser.add_argument('--track-width', type=float, default=3.0)
    parser.add_argument('--spacing', type=float, default=2.0)
    args = parser.parse_args()

    cones = generate_oval_track(args.length, args.width, args.track_width, args.spacing)
    save_cone_map(args.output, cones)
    print(f'Wrote {len(cones)} cones to {args.output}')


if __name__ == '__main__':
    main()
