"""Tests for the colorblind autocross path planner.

Covers straight track, slalom, hairpin, and degenerate/sparse inputs.
"""

import math
import sys
import os

# Allow importing planner without a full ROS install
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..', 'nodes', 'path_planning'),
)

from planner import (  # noqa: E402
    compute_centerline,
    _order_points,
    _smooth,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_straight_cones(n: int = 6, width: float = 2.0, spacing: float = 1.5):
    """Two parallel rows of cones forming a straight track."""
    left = [(i * spacing, -width / 2) for i in range(n)]
    right = [(i * spacing, width / 2) for i in range(n)]
    return left + right


def _make_slalom_cones(n: int = 7, spacing: float = 1.5, offset: float = 1.5):
    """Alternating left/right cones forming a slalom."""
    cones = []
    for i in range(n):
        sign = 1 if i % 2 == 0 else -1
        cones.append((i * spacing, sign * offset))
    return cones


def _make_hairpin_cones(radius: float = 3.0, n_outer: int = 8, n_inner: int = 5):
    """Hairpin: semicircular outer wall + inner wall."""
    outer = [
        (radius * math.cos(t), radius * math.sin(t))
        for t in [math.pi * i / (n_outer - 1) for i in range(n_outer)]
    ]
    r_inner = radius / 2.0
    inner = [
        (r_inner * math.cos(t), r_inner * math.sin(t))
        for t in [math.pi * i / (n_inner - 1) for i in range(n_inner)]
    ]
    return outer + inner


# ---------------------------------------------------------------------------
# Degenerate / sparse inputs
# ---------------------------------------------------------------------------

def test_empty_cones():
    assert compute_centerline([]) == []


def test_one_cone():
    assert compute_centerline([(0.0, 0.0)]) == []


def test_two_cones():
    assert compute_centerline([(0.0, -1.0), (0.0, 1.0)]) == []


# ---------------------------------------------------------------------------
# Straight track
# ---------------------------------------------------------------------------

def test_straight_returns_waypoints():
    cones = _make_straight_cones()
    wp = compute_centerline(cones, smooth_window=1)
    assert len(wp) >= 2, "Expected at least 2 waypoints on straight track"


def test_straight_centerline_near_y0():
    """Waypoints should be centred between the two rows (y ≈ 0)."""
    cones = _make_straight_cones(n=8, width=2.0)
    wp = compute_centerline(cones, smooth_window=1)
    assert wp, "No waypoints returned"
    ys = [p[1] for p in wp]
    for y in ys:
        assert abs(y) < 1.5, f"Waypoint y={y:.3f} is too far from centreline"


def test_straight_x_range():
    """Waypoints should span the x range of the cones."""
    n, spacing = 6, 1.5
    cones = _make_straight_cones(n=n, spacing=spacing)
    wp = compute_centerline(cones, smooth_window=1)
    assert wp, "No waypoints returned"
    xs = [p[0] for p in wp]
    # At least some waypoints should be spread out
    assert max(xs) - min(xs) > spacing, "Waypoints do not span track length"


# ---------------------------------------------------------------------------
# Slalom
# ---------------------------------------------------------------------------

def test_slalom_returns_waypoints():
    cones = _make_slalom_cones()
    wp = compute_centerline(cones, smooth_window=1)
    assert len(wp) >= 2, "Expected waypoints for slalom track"


# ---------------------------------------------------------------------------
# Hairpin
# ---------------------------------------------------------------------------

def test_hairpin_returns_waypoints():
    cones = _make_hairpin_cones()
    wp = compute_centerline(cones, smooth_window=1)
    assert len(wp) >= 2, "Expected waypoints for hairpin track"


# ---------------------------------------------------------------------------
# Smoothing
# ---------------------------------------------------------------------------

def test_smoothing_reduces_jitter():
    """Smoothed path should have lower variance than unsmoothed."""
    cones = _make_straight_cones(n=10, width=2.0)
    raw = compute_centerline(cones, smooth_window=1)
    smoothed = compute_centerline(cones, smooth_window=5)
    if len(raw) < 3 or len(smoothed) < 3:
        return  # not enough points to compare
    raw_ys = [p[1] for p in raw]
    sm_ys = [p[1] for p in smoothed]
    raw_var = sum((y - sum(raw_ys) / len(raw_ys)) ** 2 for y in raw_ys)
    sm_var = sum((y - sum(sm_ys) / len(sm_ys)) ** 2 for y in sm_ys)
    assert sm_var <= raw_var + 1e-6, "Smoothed path should not be more jittery"


# ---------------------------------------------------------------------------
# _order_points
# ---------------------------------------------------------------------------

def test_order_points_empty():
    assert _order_points([]) == []


def test_order_points_contiguous():
    pts = [(float(i), 0.0) for i in range(5)]
    import random
    shuffled = pts[:]
    random.shuffle(shuffled)
    ordered = _order_points(shuffled)
    assert len(ordered) == 5
    # Should be monotonically increasing in x (greedy NN from leftmost)
    xs = [p[0] for p in ordered]
    assert xs == sorted(xs), "Expected ordered by x for collinear points"


# ---------------------------------------------------------------------------
# _smooth
# ---------------------------------------------------------------------------

def test_smooth_preserves_length():
    pts = [(float(i), float(i)) for i in range(10)]
    sm = _smooth(pts, window=3)
    assert len(sm) == len(pts)


def test_smooth_window_1_is_identity():
    pts = [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    sm = _smooth(pts, window=1)
    for (x1, y1), (x2, y2) in zip(pts, sm):
        assert abs(x1 - x2) < 1e-9
        assert abs(y1 - y2) < 1e-9
