#!/usr/bin/env python3
"""
gen_maps.py - Generates Nav2 occupancy-grid maps for the Override field:

  maps/override_field_map.pgm(.yaml)   walls + goals + loaders as obstacles
  maps/override_keepout.pgm(.yaml)     keepout zones around every goal

Resolution: 0.01 m/px, 366x366 px (field 3.6576 m, origin at bottom-left).
Run:  python3 tools/gen_maps.py
"""

import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS = os.path.join(ROOT, "maps")

RES = 0.01
FIELD_HALF = 1.8288
SIZE = int(FIELD_HALF * 2 / RES) + 1          # 367 px

GOALS = [
    (0.0, 0.0), (-0.6, 1.2), (-1.2, 0.6), (1.2, -0.6), (0.6, -1.2),
    (-1.2, -0.6), (-0.6, -1.2), (0.6, 1.2), (1.2, 0.6),
]
LOADERS = [(-1.74, 1.49), (-1.74, -1.49), (1.74, 1.49), (1.74, -1.49)]


def cell(x, y):
    """meters (center origin) -> pixel (row, col)"""
    col = int((x + FIELD_HALF) / RES + 0.5)
    row = int((y + FIELD_HALF) / RES + 0.5)
    return row, col


def make_map(goal_radius, loader_radius):
    grid = [[254] * SIZE for _ in range(SIZE)]

    def fill_circle(cx, cy, r):
        r0, c0 = cell(cx, cy)
        dr = int(r / RES) + 1
        for r in range(max(0, r0 - dr), min(SIZE, r0 + dr + 1)):
            for c in range(max(0, c0 - dr), min(SIZE, c0 + dr + 1)):
                if math.hypot((r - r0) * RES, (c - c0) * RES) <= r:
                    grid[r][c] = 0

    # perimeter walls
    wall = int(0.02 / RES) + 1
    for r in range(SIZE):
        for c in range(SIZE):
            x = c * RES - FIELD_HALF
            y = r * RES - FIELD_HALF
            if abs(x) > FIELD_HALF - 0.025 or abs(y) > FIELD_HALF - 0.025:
                grid[r][c] = 0

    for gx, gy in GOALS:
        fill_circle(gx, gy, goal_radius)
    for lx, ly in LOADERS:
        fill_circle(lx, ly, loader_radius)

    return grid


def write_pgm(path, grid):
    with open(path, "w") as f:
        f.write("P2\n%d %d\n255\n" % (SIZE, SIZE))
        for row in grid:
            f.write(" ".join(str(v) for v in row) + "\n")


def write_yaml(path, image, origin):
    with open(path, "w") as f:
        f.write(f"image: {image}\n"
                f"mode: trinary\n"
                f"resolution: {RES}\n"
                f"origin: [{origin[0]}, {origin[1]}, 0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n")


def main():
    os.makedirs(MAPS, exist_ok=True)
    origin = (-FIELD_HALF, -FIELD_HALF)

    nav = make_map(goal_radius=0.13, loader_radius=0.15)
    write_pgm(os.path.join(MAPS, "override_field_map.pgm"), nav)
    write_yaml(os.path.join(MAPS, "override_field_map.yaml"),
               "override_field_map.pgm", origin)

    keep = make_map(goal_radius=0.28, loader_radius=0.18)
    write_pgm(os.path.join(MAPS, "override_keepout.pgm"), keep)
    write_yaml(os.path.join(MAPS, "override_keepout.yaml"),
               "override_keepout.pgm", origin)

    print("maps generated: override_field_map, override_keepout")


if __name__ == "__main__":
    main()
