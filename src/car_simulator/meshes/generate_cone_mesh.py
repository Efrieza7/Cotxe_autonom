#!/usr/bin/env python3
"""Generate the traffic cone mesh used by cone_map_publisher_node.

    python3 generate_cone_mesh.py      # writes cone.stl here

Base on the ground (z = 0), axis along z, metres. Keep BASE_DIAMETER and
HEIGHT in sync with cone_base_radius / cone_height of lidar_simulator_node.
"""

from pathlib import Path

import numpy as np

from generate_gt3_meshes import write_binary_stl

BASE_DIAMETER = 0.08
HEIGHT = 0.12
TOP_DIAMETER = 0.008   # small flat tip, like a real cone
N = 48


def cone_triangles():
    t = np.linspace(0.0, 2.0 * np.pi, N, endpoint=False)
    rb, rt = BASE_DIAMETER / 2.0, TOP_DIAMETER / 2.0
    base = np.column_stack([rb * np.cos(t), rb * np.sin(t), np.zeros(N)])
    top = np.column_stack([rt * np.cos(t), rt * np.sin(t), np.full(N, HEIGHT)])
    tris = []
    for i in range(N):
        j = (i + 1) % N
        tris.append((base[i], base[j], top[j]))      # side
        tris.append((base[i], top[j], top[i]))
        tris.append(((0, 0, 0), base[j], base[i]))   # bottom cap
        tris.append(((0, 0, HEIGHT), top[i], top[j]))  # top cap
    return np.array(tris, dtype=float)


def main():
    path = Path(__file__).resolve().parent / 'cone.stl'
    write_binary_stl(path, cone_triangles())
    print(f'Wrote {path} ({BASE_DIAMETER * 100:.0f} cm base, {HEIGHT * 100:.0f} cm high)')


if __name__ == '__main__':
    main()
