"""Colorblind autocross path planner.

Computes a centerline trajectory from unlabeled boundary/cone points,
inspired by the approach in papalotis/ft-fsd-path-planning.

No cone color information is required.  The algorithm:
1. Groups cones into two sides using a geometric heuristic (perpendicular
   deviation from the estimated track axis).
2. Builds a Delaunay triangulation of all cone positions.
3. Extracts midpoints of Delaunay edges that connect cones from opposite
   sides (cross-boundary edges), filtered by length.
4. Orders the midpoints using a direction-aware nearest-neighbour walk
   that avoids sharp back-tracking.
5. Smooths the result with a corner-preserving weighted moving average.

Fallback: when fewer than 3 cones are available or the midpoint set is
empty, a minimal straight-ahead path is synthesised from the mean cone
position so the caller always receives actionable waypoints.
"""

import math
from typing import List, Optional, Tuple

Point = Tuple[float, float]

# Minimum dot-product with the current heading for a candidate next point
# to be accepted in the direction-aware walk (prevents U-turns).
_MIN_DOT = -0.3


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
    List of (x, y) waypoints forming the centerline, never empty: when
    the input is degenerate a short straight-ahead fallback path is
    returned instead.
    """
    if len(cones) < 3:
        return _fallback_path(cones)

    try:
        sides = _assign_sides(cones)
        midpoints = _delaunay_midpoints(cones, sides, min_edge_length, max_edge_length)
    except Exception:
        midpoints = []

    if not midpoints:
        return _fallback_path(cones)

    ordered = _order_points_directed(midpoints)
    if smooth_window > 1 and len(ordered) >= smooth_window:
        ordered = _smooth_weighted(ordered, smooth_window)

    return ordered


# ---------------------------------------------------------------------------
# Fallback path generator
# ---------------------------------------------------------------------------

def _fallback_path(cones: List[Point]) -> List[Point]:
    """Return a minimal straight-ahead path usable when cones are sparse.

    If cones are available their centroid is used as the starting point;
    otherwise the origin is used.  Two waypoints are returned so downstream
    controllers always have a valid heading.
    """
    if cones:
        cx = sum(p[0] for p in cones) / len(cones)
        cy = sum(p[1] for p in cones) / len(cones)
    else:
        cx, cy = 0.0, 0.0
    # Two waypoints pointing in the +x direction (straight ahead).
    return [(cx, cy), (cx + 1.0, cy)]


# ---------------------------------------------------------------------------
# Side-grouping heuristic
# ---------------------------------------------------------------------------

def _assign_sides(cones: List[Point]) -> List[int]:
    """Assign each cone to side 0 or 1 using a perpendicular-deviation heuristic.

    The method estimates the track axis by fitting a line through the cone
    cloud (via principal component analysis on the 2-D point set), then
    labels each cone by the sign of its signed distance from that axis.

    Returns
    -------
    List of ints (0 or 1) with the same length as *cones*.
    """
    n = len(cones)
    cx = sum(p[0] for p in cones) / n
    cy = sum(p[1] for p in cones) / n

    # 2×2 covariance matrix entries
    sxx = sum((p[0] - cx) ** 2 for p in cones)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in cones)
    syy = sum((p[1] - cy) ** 2 for p in cones)

    # Largest eigenvector of [[sxx, sxy],[sxy, syy]] → track axis direction
    # Power-iteration style: start with a guess and refine once.
    if abs(sxy) < 1e-10:
        # Axis-aligned: pick the dimension with more spread
        ax, ay = (1.0, 0.0) if sxx >= syy else (0.0, 1.0)
    else:
        # Dominant eigenvector of 2×2 symmetric matrix
        diff = sxx - syy
        lam = (sxx + syy + math.sqrt(diff ** 2 + 4 * sxy ** 2)) / 2.0
        ax, ay = lam - syy, sxy  # unnormalised
        norm = math.hypot(ax, ay) or 1.0
        ax, ay = ax / norm, ay / norm

    # Perpendicular to the axis (rotated 90°)
    nx, ny = -ay, ax  # normal vector

    sides: List[int] = []
    for p in cones:
        dot = (p[0] - cx) * nx + (p[1] - cy) * ny
        sides.append(0 if dot >= 0 else 1)
    return sides


# ---------------------------------------------------------------------------
# Delaunay triangulation (pure-Python, no scipy dependency)
# ---------------------------------------------------------------------------

def _delaunay_midpoints(
    points: List[Point],
    sides: List[int],
    min_len: float,
    max_len: float,
) -> List[Point]:
    """Return midpoints of Delaunay edges that cross the track boundary.

    Only edges connecting cones from *opposite* sides are considered, so
    the midpoints sit on the centerline rather than along one boundary.
    Side labels come from :func:`_assign_sides`.

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
            # Only keep cross-boundary edges
            if sides[p] == sides[q]:
                continue
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
# Direction-aware nearest-neighbour ordering
# ---------------------------------------------------------------------------

def _order_points_directed(points: List[Point]) -> List[Point]:
    """Order *points* into a path using a direction-aware nearest-neighbour walk.

    Unlike a pure greedy nearest-neighbour search this variant rejects
    candidate points that would require a sharp reversal of heading.  This
    prevents the path from folding back on itself at hairpins or when
    midpoints cluster near the apex of a tight turn.

    Starts from the point with the smallest x coordinate (approximately
    the track entry when the car drives in the +x direction).
    """
    if not points:
        return []
    remaining = list(points)
    start = min(range(len(remaining)), key=lambda i: remaining[i][0])
    ordered = [remaining.pop(start)]

    # Initial heading: +x direction
    hdx, hdy = 1.0, 0.0

    while remaining:
        last = ordered[-1]
        best_idx: Optional[int] = None
        best_dist = math.inf

        for i, pt in enumerate(remaining):
            dx = pt[0] - last[0]
            dy = pt[1] - last[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-9:
                continue
            # Normalised dot product with current heading
            dot = (dx * hdx + dy * hdy) / dist
            if dot < _MIN_DOT:
                continue  # would require a near-reversal
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_idx is None:
            # All remaining candidates would require a reversal;
            # fall back to pure nearest-neighbour for the rest.
            best_idx = min(range(len(remaining)),
                           key=lambda i: math.hypot(remaining[i][0] - last[0],
                                                    remaining[i][1] - last[1]))

        nxt = remaining.pop(best_idx)
        dx = nxt[0] - ordered[-1][0]
        dy = nxt[1] - ordered[-1][1]
        norm = math.hypot(dx, dy) or 1.0
        hdx, hdy = dx / norm, dy / norm
        ordered.append(nxt)

    return ordered


# Keep the old name as an alias for backward compatibility and tests.
_order_points = _order_points_directed


# ---------------------------------------------------------------------------
# Corner-preserving weighted moving-average smoothing
# ---------------------------------------------------------------------------

def _smooth_weighted(points: List[Point], window: int) -> List[Point]:
    """Apply a corner-preserving weighted moving-average filter.

    Weights decrease toward the edges of the window so interior points
    (away from corners) are smoothed more strongly than endpoints.  This
    avoids the excessive corner-cutting produced by a flat moving average.
    """
    half = window // 2
    result: List[Point] = []
    n = len(points)
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        # Triangular weights peaking at the centre of the window
        weights = [1.0 + half - abs(j - i) for j in range(lo, hi)]
        total = sum(weights)
        xs = [points[j][0] * w for j, w in zip(range(lo, hi), weights)]
        ys = [points[j][1] * w for j, w in zip(range(lo, hi), weights)]
        result.append((sum(xs) / total, sum(ys) / total))
    return result


# Keep the old name as an alias for backward compatibility and tests.
def _smooth(points: List[Point], window: int) -> List[Point]:
    """Alias for :func:`_smooth_weighted` (kept for backward compatibility)."""
    return _smooth_weighted(points, window)
