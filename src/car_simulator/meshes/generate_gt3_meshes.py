#!/usr/bin/env python3
"""Generate the GT3-style body meshes used by urdf/car.urdf.

    python3 generate_gt3_meshes.py      # writes gt3_body.stl and gt3_cabin.stl here

Frame: base_link = front (steering) axle on the ground, x forward, z up, metres.
The body is 0.33 m long and 0.13 m wide:
    FRONT_OVERHANG (ahead of the front axle) + WHEELBASE + REAR_OVERHANG = 0.33 m.
Each part is a loft of rounded (superellipse) cross-sections along x.
"""

from pathlib import Path

import numpy as np

WHEELBASE = 0.18
FRONT_OVERHANG = 0.07
REAR_OVERHANG = 0.08
X_FRONT = FRONT_OVERHANG
X_REAR = -(WHEELBASE + REAR_OVERHANG)

# Main body: (x, bottom z, top z, half width). Fenders bulge over both axles,
# low nose at the front, short rear deck under the wing.
BODY = [
    (X_REAR,          0.020, 0.052, 0.058),
    (X_REAR + 0.010,  0.015, 0.060, 0.063),
    (-0.23,           0.013, 0.064, 0.065),
    (-0.18,           0.012, 0.068, 0.065),   # rear axle: rear fender
    (-0.13,           0.012, 0.062, 0.064),
    (-0.09,           0.012, 0.058, 0.060),   # doors: narrower waist
    (-0.05,           0.012, 0.059, 0.062),
    (-0.01,           0.012, 0.062, 0.065),
    (0.010,           0.012, 0.061, 0.065),   # front axle: front fender
    (0.035,           0.012, 0.050, 0.063),
    (0.055,           0.013, 0.038, 0.060),
    (X_FRONT - 0.005, 0.014, 0.030, 0.056),
    (X_FRONT,         0.016, 0.024, 0.050),   # nose
]

# Greenhouse: fastback from the rear deck up to the roof, then the windscreen.
CABIN = [
    (-0.205, 0.050, 0.056, 0.036),
    (-0.170, 0.050, 0.078, 0.042),
    (-0.130, 0.050, 0.095, 0.044),
    (-0.100, 0.050, 0.099, 0.044),
    (-0.070, 0.050, 0.097, 0.045),
    (-0.045, 0.050, 0.082, 0.047),
    (-0.020, 0.050, 0.064, 0.047),
    (-0.005, 0.050, 0.056, 0.044),
]

N_AROUND = 48
EXPONENT = 5.0  # superellipse exponent: higher = boxier cross-section


def section(bottom, top, half_width):
    """Closed ring of (y, z) points around a rounded cross-section."""
    zc, b = (top + bottom) / 2.0, (top - bottom) / 2.0
    t = np.linspace(0.0, 2.0 * np.pi, N_AROUND, endpoint=False)
    c, s = np.cos(t), np.sin(t)
    y = half_width * np.sign(c) * np.abs(c) ** (2.0 / EXPONENT)
    z = zc + b * np.sign(s) * np.abs(s) ** (2.0 / EXPONENT)
    return np.column_stack([y, z])


def loft(stations, samples_per_span=6):
    """Triangles of a closed loft through the stations (interpolated smoothly)."""
    table = np.array(stations, dtype=float)
    xs = np.linspace(table[0, 0], table[-1, 0], (len(table) - 1) * samples_per_span + 1)
    cols = [np.interp(xs, table[:, 0], table[:, i]) for i in (1, 2, 3)]
    rings = [
        np.column_stack([np.full(N_AROUND, x), section(bt, tp, hw)])
        for x, bt, tp, hw in zip(xs, *cols)
    ]

    tris = []
    for r0, r1 in zip(rings, rings[1:]):
        for i in range(N_AROUND):
            j = (i + 1) % N_AROUND
            tris.append((r0[i], r1[i], r1[j]))
            tris.append((r0[i], r1[j], r0[j]))
    for ring, flip in ((rings[0], True), (rings[-1], False)):  # end caps
        centre = ring.mean(axis=0)
        for i in range(N_AROUND):
            a, b = ring[i], ring[(i + 1) % N_AROUND]
            tris.append((centre, b, a) if flip else (centre, a, b))

    tris = np.array(tris)
    # orient all faces outwards (positive enclosed volume)
    volume = np.einsum('ij,ij->i', tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6.0
    if volume < 0.0:
        tris = tris[:, ::-1]
    return tris


def write_binary_stl(path, tris):
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    record = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    record['n'] = normals
    record['v'] = tris
    with open(path, 'wb') as f:
        f.write(b'GT3 body for car_simulator'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(record.tobytes())


def main():
    here = Path(__file__).resolve().parent
    write_binary_stl(here / 'gt3_body.stl', loft(BODY))
    write_binary_stl(here / 'gt3_cabin.stl', loft(CABIN))
    print(f'Wrote gt3_body.stl and gt3_cabin.stl in {here} '
          f'(body {X_FRONT - X_REAR:.2f} m long, x {X_REAR:.2f}..{X_FRONT:.2f})')


if __name__ == '__main__':
    main()
