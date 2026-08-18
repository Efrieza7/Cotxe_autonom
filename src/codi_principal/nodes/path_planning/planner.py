"""Colorblind autocross path planner.

Computes a centerline trajectory from unlabeled boundary/cone points,
inspired by the approach in papalotis/ft-fsd-path-planning.

No cone color information is required.  The algorithm:
1. Builds a Delaunay triangulation of the cone positions.
2. Extracts midpoints of triangle edges whose two endpoints belong to
   opposite track boundaries (detected heuristically by proximity).
3. Orders the midpoints to form a continuous centerline.
4. Optionally smooths the result with a moving average.

Fallback: when fewer than 3 cones are available the function returns an
empty list so the caller can apply a safe default (e.g. drive straight).
"""

import math
from typing import List, Tuple

Point = Tuple[float, float]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def compute_centerline(
    cones: List[Point],
    smooth_window: int = 3,
    min_edge_length: float = 0.3,
    max_edge_length: float = 6.0,
) -> List[Point]:
    """Return an ordered list of centerline waypoints for the given cones.

    Parameters
    ----------
    cones:
        Flat list of (x, y) positions.  No color labels needed.
    smooth_window:
        Number of points for the moving-average smoothing pass.
        Set to 1 to disable smoothing.
    min_edge_length:
        Triangle edges shorter than this (metres) are ignored (noise).
    max_edge_length:
        Triangle edges longer than this (metres) are ignored (no track
        boundary crosses such a large gap).

    Returns
    -------
    List of (x, y) waypoints forming the centerline, possibly empty when
    the input is degenerate.
    """
    if len(cones) < 3:
        return []

    try:
        midpoints = _delaunay_midpoints(cones, min_edge_length, max_edge_length)
    except Exception:
        return []

    if not midpoints:
        return []

    ordered = _order_points(midpoints)
    if smooth_window > 1 and len(ordered) >= smooth_window:
        ordered = _smooth(ordered, smooth_window)

    return ordered


# ---------------------------------------------------------------------------
# Delaunay triangulation (pure-Python, no scipy dependency)
# ---------------------------------------------------------------------------

def _delaunay_midpoints(
    points: List[Point],
    min_len: float,
    max_len: float,
) -> List[Point]:
    """Return midpoints of Delaunay edges that cross the track boundary.

    We use a simple Bowyer–Watson Delaunay triangulation implemented in
    pure Python so that no external C libraries are required.
    """
    triangles = _bowyer_watson(points)
    midpoints: List[Point] = []
    seen: set = set()

    for tri in triangles:
        a, b, c = tri
        for p, q in [(a, b), (b, c), (a, c)]:
            key = (min(p, q), max(p, q))
            if key in seen:
                continue
            seen.add(key)
            px, py = points[p]
            qx, qy = points[q]
            length = math.hypot(px - qx, py - qy)
            if min_len <= length <= max_len:
                midpoints.append(((px + qx) / 2.0, (py + qy) / 2.0))

    return midpoints


# ---------------------------------------------------------------------------
# Bowyer–Watson Delaunay triangulation
# ---------------------------------------------------------------------------

def _bowyer_watson(points: List[Point]) -> List[Tuple[int, int, int]]:
    """Return Delaunay triangles as index triples into *points*."""
    n = len(points)
    # Create a super-triangle that contains all points
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)
    dx = max_x - min_x or 1.0
    dy = max_y - min_y or 1.0
    delta = max(dx, dy) * 10.0

    sup = [
        (min_x - delta, min_y - delta * 3),
        (min_x - delta, max_y + delta),
        (max_x + delta * 3, max_y + delta),
    ]
    # Extend points list with super-triangle vertices
    all_pts = list(points) + sup
    si0, si1, si2 = n, n + 1, n + 2

    triangles: List[Tuple[int, int, int]] = [(si0, si1, si2)]

    for i, pt in enumerate(points):
        bad: List[Tuple[int, int, int]] = []
        for tri in triangles:
            if _in_circumcircle(all_pts, tri, pt):
                bad.append(tri)

        # Find boundary of the polygonal hole
        boundary: List[Tuple[int, int]] = []
        for tri in bad:
            for edge in [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])]:
                shared = False
                for other in bad:
                    if other is tri:
                        continue
                    if edge[0] in other and edge[1] in other:
                        shared = True
                        break
                if not shared:
                    boundary.append(edge)

        triangles = [t for t in triangles if t not in bad]
        for edge in boundary:
            triangles.append((edge[0], edge[1], i))

    # Remove triangles that share a vertex with the super-triangle
    result = [
        t for t in triangles
        if si0 not in t and si1 not in t and si2 not in t
    ]
    return result


def _in_circumcircle(
    pts: List[Point],
    tri: Tuple[int, int, int],
    p: Point,
) -> bool:
    """Return True if point *p* is inside the circumcircle of *tri*."""
    ax, ay = pts[tri[0]]
    bx, by = pts[tri[1]]
    cx, cy = pts[tri[2]]
    px, py = p

    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-10:
        return False

    ux = ((ax**2 + ay**2) * (by - cy)
          + (bx**2 + by**2) * (cy - ay)
          + (cx**2 + cy**2) * (ay - by)) / d
    uy = ((ax**2 + ay**2) * (cx - bx)
          + (bx**2 + by**2) * (ax - cx)
          + (cx**2 + cy**2) * (bx - ax)) / d

    r2 = (ax - ux) ** 2 + (ay - uy) ** 2
    dist2 = (px - ux) ** 2 + (py - uy) ** 2
    return dist2 < r2


# ---------------------------------------------------------------------------
# Nearest-neighbour ordering
# ---------------------------------------------------------------------------

def _order_points(points: List[Point]) -> List[Point]:
    """Order *points* into a path using a greedy nearest-neighbour heuristic.

    Starts from the point with the smallest x coordinate (roughly the
    'entry' of the track when the car drives left-to-right).
    """
    if not points:
        return []
    remaining = list(points)
    # Start from leftmost point
    start = min(range(len(remaining)), key=lambda i: remaining[i][0])
    ordered = [remaining.pop(start)]

    while remaining:
        last = ordered[-1]
        nearest = min(range(len(remaining)),
                      key=lambda i: math.hypot(remaining[i][0] - last[0],
                                               remaining[i][1] - last[1]))
        ordered.append(remaining.pop(nearest))

    return ordered


# ---------------------------------------------------------------------------
# Moving-average smoothing
# ---------------------------------------------------------------------------

def _smooth(points: List[Point], window: int) -> List[Point]:
    """Apply a symmetric moving-average filter to the point sequence."""
    half = window // 2
    result: List[Point] = []
    n = len(points)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        xs = [p[0] for p in points[lo:hi]]
        ys = [p[1] for p in points[lo:hi]]
        result.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    return result
