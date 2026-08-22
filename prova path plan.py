import math
from typing import List, Optional, Tuple
import matplotlib.pyplot as plt


Point = Tuple[float, float]

# Mínim producte escalar per acceptar un punt
_MIN_DOT = -0.3

# Multiplicador per decidir quan el path s'ha acabat
MAX_STEP_FACTOR = 3.0


# ===========================================================================
# PATH PLANNER
# ===========================================================================

def compute_centerline(
    cones: List[Point],
    smooth_window: int = 3,
    min_edge_length: float = 0.3,
    max_edge_length: float = 6.0,
) -> List[Point]:

    if len(cones) < 3:
        return _fallback_path(cones)

    try:
        sides = _assign_sides(cones)

        midpoints = _delaunay_midpoints(
            cones,
            sides,
            min_edge_length,
            max_edge_length
        )

    except Exception as e:
        print("Error:", e)
        midpoints = []

    if not midpoints:
        return _fallback_path(cones)

    ordered = _order_points_directed(midpoints)

    if smooth_window > 1 and len(ordered) >= smooth_window:
        ordered = _smooth_weighted(
            ordered,
            smooth_window
        )

    return ordered


# ===========================================================================
# FALLBACK
# ===========================================================================

def _fallback_path(cones: List[Point]) -> List[Point]:

    if cones:

        cx = sum(p[0] for p in cones) / len(cones)
        cy = sum(p[1] for p in cones) / len(cones)

    else:

        cx = 0.0
        cy = 0.0

    return [
        (cx, cy),
        (cx + 1.0, cy)
    ]


# ===========================================================================
# CLASSIFICACIÓ GEOMÈTRICA DELS CONS
# ===========================================================================

def _assign_sides(
    cones: List[Point]
) -> List[int]:

    n = len(cones)

    cx = sum(p[0] for p in cones) / n
    cy = sum(p[1] for p in cones) / n

    # Matriu de covariància

    sxx = sum(
        (p[0] - cx) ** 2
        for p in cones
    )

    sxy = sum(
        (p[0] - cx) * (p[1] - cy)
        for p in cones
    )

    syy = sum(
        (p[1] - cy) ** 2
        for p in cones
    )

    # Calcular eix principal

    if abs(sxy) < 1e-10:

        if sxx >= syy:
            ax, ay = 1.0, 0.0
        else:
            ax, ay = 0.0, 1.0

    else:

        diff = sxx - syy

        lam = (
            sxx
            + syy
            + math.sqrt(
                diff ** 2
                + 4 * sxy ** 2
            )
        ) / 2.0

        ax = lam - syy
        ay = sxy

        norm = math.hypot(ax, ay)

        if norm == 0:
            norm = 1.0

        ax /= norm
        ay /= norm

    # Vector perpendicular

    nx = -ay
    ny = ax

    sides = []

    for p in cones:

        dot = (
            (p[0] - cx) * nx
            + (p[1] - cy) * ny
        )

        if dot >= 0:
            sides.append(0)
        else:
            sides.append(1)

    return sides


# ===========================================================================
# MIDPOINTS DELAUNAY
# ===========================================================================

def _delaunay_midpoints(
    points: List[Point],
    sides: List[int],
    min_len: float,
    max_len: float,
) -> List[Point]:

    triangles = _bowyer_watson(points)

    midpoints = []

    seen = set()

    for tri in triangles:

        a, b, c = tri

        for p, q in [
            (a, b),
            (b, c),
            (a, c)
        ]:

            key = (
                min(p, q),
                max(p, q)
            )

            if key in seen:
                continue

            seen.add(key)

            # Només acceptar arestes entre costats diferents

            if sides[p] == sides[q]:
                continue

            px, py = points[p]
            qx, qy = points[q]

            length = math.hypot(
                px - qx,
                py - qy
            )

            if min_len <= length <= max_len:

                midpoint = (
                    (px + qx) / 2.0,
                    (py + qy) / 2.0
                )

                midpoints.append(midpoint)

    return midpoints


# ===========================================================================
# DELAUNAY - BOWYER WATSON
# ===========================================================================

def _bowyer_watson(
    points: List[Point]
) -> List[Tuple[int, int, int]]:

    n = len(points)

    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)

    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)

    dx = max_x - min_x
    dy = max_y - min_y

    if dx == 0:
        dx = 1.0

    if dy == 0:
        dy = 1.0

    delta = max(dx, dy) * 10.0

    sup = [

        (
            min_x - delta,
            min_y - delta * 3
        ),

        (
            min_x - delta,
            max_y + delta
        ),

        (
            max_x + delta * 3,
            max_y + delta
        )
    ]

    all_pts = list(points) + sup

    si0 = n
    si1 = n + 1
    si2 = n + 2

    triangles = [
        (si0, si1, si2)
    ]

    for i, pt in enumerate(points):

        bad = []

        for tri in triangles:

            if _in_circumcircle(
                all_pts,
                tri,
                pt
            ):

                bad.append(tri)

        # Trobar les arestes del forat

        edge_count = {}

        for tri in bad:

            edges = [

                (tri[0], tri[1]),
                (tri[1], tri[2]),
                (tri[2], tri[0])

            ]

            for a, b in edges:

                edge = (
                    min(a, b),
                    max(a, b)
                )

                edge_count[edge] = (
                    edge_count.get(edge, 0)
                    + 1
                )

        boundary = [

            edge
            for edge, count in edge_count.items()
            if count == 1

        ]

        triangles = [

            t
            for t in triangles
            if t not in bad

        ]

        for edge in boundary:

            triangles.append(
                (
                    edge[0],
                    edge[1],
                    i
                )
            )

    result = [

        t
        for t in triangles

        if (
            si0 not in t
            and si1 not in t
            and si2 not in t
        )

    ]

    return result


# ===========================================================================
# CIRCUMCIRCLE
# ===========================================================================

def _in_circumcircle(
    pts: List[Point],
    tri: Tuple[int, int, int],
    p: Point,
) -> bool:

    ax, ay = pts[tri[0]]
    bx, by = pts[tri[1]]
    cx, cy = pts[tri[2]]

    px, py = p

    d = 2.0 * (

        ax * (by - cy)
        + bx * (cy - ay)
        + cx * (ay - by)

    )

    if abs(d) < 1e-10:
        return False

    ux = (

        (ax ** 2 + ay ** 2)
        * (by - cy)

        + (bx ** 2 + by ** 2)
        * (cy - ay)

        + (cx ** 2 + cy ** 2)
        * (ay - by)

    ) / d

    uy = (

        (ax ** 2 + ay ** 2)
        * (cx - bx)

        + (bx ** 2 + by ** 2)
        * (ax - cx)

        + (cx ** 2 + cy ** 2)
        * (bx - ax)

    ) / d

    r2 = (
        (ax - ux) ** 2
        + (ay - uy) ** 2
    )

    dist2 = (
        (px - ux) ** 2
        + (py - uy) ** 2
    )

    return dist2 < r2


# ===========================================================================
# ORDENACIÓ DEL PATH
# ===========================================================================

def _order_points_directed(
    points: List[Point]
) -> List[Point]:

    if not points:
        return []

    remaining = list(points)

    # Començar pel punt més a l'esquerra

    start = min(
        range(len(remaining)),
        key=lambda i: remaining[i][0]
    )

    ordered = [
        remaining.pop(start)
    ]

    # Calcular la distància típica entre midpoints

    nearest_distances = []

    for i, p in enumerate(points):

        distances = []

        for j, q in enumerate(points):

            if i == j:
                continue

            d = math.hypot(
                q[0] - p[0],
                q[1] - p[1]
            )

            if d > 1e-9:
                distances.append(d)

        if distances:

            nearest_distances.append(
                min(distances)
            )

    if nearest_distances:

        nearest_distances.sort()

        typical_step = nearest_distances[
            len(nearest_distances) // 2
        ]

    else:

        typical_step = 1.0

    max_step = (
        typical_step
        * MAX_STEP_FACTOR
    )

    # Direcció inicial

    hdx = 1.0
    hdy = 0.0

    while remaining:

        last = ordered[-1]

        best_idx = None
        best_dist = math.inf

        for i, pt in enumerate(remaining):

            dx = pt[0] - last[0]
            dy = pt[1] - last[1]

            dist = math.hypot(dx, dy)

            if dist < 1e-9:
                continue

            # Si el punt està massa lluny,
            # no continuar el path

            if dist > max_step:
                continue

            # Comprovar direcció

            dot = (
                dx * hdx
                + dy * hdy
            ) / dist

            if dot < _MIN_DOT:
                continue

            if dist < best_dist:

                best_dist = dist
                best_idx = i

        # Si no existeix cap punt vàlid:
        # el path acaba aquí

        if best_idx is None:
            break

        nxt = remaining.pop(best_idx)

        dx = nxt[0] - last[0]
        dy = nxt[1] - last[1]

        norm = math.hypot(dx, dy)

        if norm == 0:
            norm = 1.0

        hdx = dx / norm
        hdy = dy / norm

        ordered.append(nxt)

    return ordered


# Compatibilitat

_order_points = _order_points_directed


# ===========================================================================
# SMOOTHING
# ===========================================================================

def _smooth_weighted(
    points: List[Point],
    window: int
) -> List[Point]:

    half = window // 2

    result = []

    n = len(points)

    for i in range(n):

        lo = max(
            0,
            i - half
        )

        hi = min(
            n,
            i + half + 1
        )

        total_weight = 0.0

        sx = 0.0
        sy = 0.0

        for j in range(lo, hi):

            distance = abs(j - i)

            weight = 1.0 / (
                1.0 + distance
            )

            sx += (
                points[j][0]
                * weight
            )

            sy += (
                points[j][1]
                * weight
            )

            total_weight += weight

        result.append(
            (
                sx / total_weight,
                sy / total_weight
            )
        )

    return result


# ===========================================================================
# PROVA
# ===========================================================================

if __name__ == "__main__":

    # ---------------------------------------------------------------
    # EDITA NOMÉS AQUESTA PART PER PROVAR DIFERENTS CIRCUITS
    # ---------------------------------------------------------------

    cones = [(0,0), (5,19), (13,37), (23,55), (34,71), (50,86), (69,101), (90,113), (113,124), (138,133), (164,142), (193,147), (220,150), (251,147), (280,142), (306,133), (331,124), (354,113), (375,101), (394,86), (410,71), (423,55), (433,37), (438,19), (440,0), (438,-19), (433,-37), (423,-55), (410,-71), (394,-86), (375,-101), (354,-113), (331,-124), (306,-133), (280,-142), (251,-147), (220,-150), (193,-147), (164,-142), (138,-133), (113,-124), (90,-113), (69,-101), (50,-86), (34,-71), (23,-55), (13,-37), (5,-19),

(30,0), (35,18), (43,36), (53,52), (64,67), (80,81), (99,94), (120,105), (140,115), (164,122), (187,129), (214,133), (240,135), (271,133), (300,129), (324,122), (350,115), (370,105), (391,94), (410,81), (426,67), (437,52), (447,36), (452,18), (460,0), (452,-18), (447,-36), (437,-52), (426,-67), (410,-81), (391,-94), (370,-105), (350,-115), (324,-122), (300,-129), (271,-133), (240,-135), (214,-133), (187,-129), (164,-122), (140,-115), (120,-105), (99,-94), (80,-81), (64,-67), (53,-52), (43,-36), (35,-18)]


    # ---------------------------------------------------------------
    # CALCULAR PATHPLAN
    # ---------------------------------------------------------------

    pathplan = compute_centerline(
        cones,
        smooth_window=3,
        min_edge_length=0.3,
        max_edge_length=6.0
    )


    # ---------------------------------------------------------------
    # CALCULAR ELEMENTS DE DEBUG
    # ---------------------------------------------------------------

    sides = _assign_sides(cones)

    triangles = _bowyer_watson(cones)

    midpoints = _delaunay_midpoints(
        cones,
        sides,
        0.3,
        6.0
    )


    # ---------------------------------------------------------------
    # IMPRIMIR RESULTATS
    # ---------------------------------------------------------------

    print("\nPATHPLAN\n")

    for i, point in enumerate(pathplan):

        print(
            f"{i}: "
            f"({point[0]:.3f}, "
            f"{point[1]:.3f})"
        )

    print(
        "\nNombre de triangles:",
        len(triangles)
    )

    print(
        "Nombre de midpoints:",
        len(midpoints)
    )

    print(
        "Nombre de punts del path:",
        len(pathplan)
    )


    # ===========================================================================
    # VISUALITZACIÓ
    # ===========================================================================

    plt.figure(
        figsize=(12, 8)
    )


    # ---------------------------------------------------------------
    # TRIANGULACIÓ DELAUNAY
    # ---------------------------------------------------------------

    drawn_edges = set()

    for tri in triangles:

        a, b, c = tri

        for p, q in [

            (a, b),
            (b, c),
            (c, a)

        ]:

            edge = (
                min(p, q),
                max(p, q)
            )

            if edge in drawn_edges:
                continue

            drawn_edges.add(edge)

            plt.plot(

                [
                    cones[p][0],
                    cones[q][0]
                ],

                [
                    cones[p][1],
                    cones[q][1]
                ],

                color="gray",
                linewidth=0.8,
                alpha=0.6,
                zorder=1
            )


    # ---------------------------------------------------------------
    # CONS
    # ---------------------------------------------------------------

    plt.scatter(

        [p[0] for p in cones],
        [p[1] for p in cones],

        color="yellow",
        edgecolors="black",

        s=70,

        label="Cons",

        zorder=3
    )


    # ---------------------------------------------------------------
    # MIDPOINTS
    # ---------------------------------------------------------------

    if midpoints:

        plt.scatter(

            [p[0] for p in midpoints],
            [p[1] for p in midpoints],

            color="magenta",

            marker="x",

            s=60,

            label="Midpoints",

            zorder=4
        )


    # ---------------------------------------------------------------
    # PATHPLAN
    # ---------------------------------------------------------------

    if pathplan:

        plt.plot(

            [p[0] for p in pathplan],
            [p[1] for p in pathplan],

            color="red",

            marker="o",

            markersize=4,

            linewidth=2,

            label="Pathplan",

            zorder=5
        )


        # Inici

        plt.scatter(

            pathplan[0][0],
            pathplan[0][1],

            color="green",

            marker="s",

            s=100,

            label="Inici",

            zorder=6
        )


        # Final

        plt.scatter(

            pathplan[-1][0],
            pathplan[-1][1],

            color="black",

            marker="X",

            s=100,

            label="Final",

            zorder=6
        )


    # ---------------------------------------------------------------
    # CONFIGURACIÓ
    # ---------------------------------------------------------------

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.title(
        "Cons + Delaunay + Midpoints + Pathplan"
    )

    plt.axis("equal")

    plt.grid(True)

    plt.legend()

    plt.show()