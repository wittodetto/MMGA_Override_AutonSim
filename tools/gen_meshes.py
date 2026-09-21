#!/usr/bin/env python3
"""
gen_meshes.py - Deterministic STL mesh generator for the V5RC Override simulation.

Generates ASCII STL meshes for the Override game elements:
  * Pin        - two halves (bottom: hex flange + taper, top: straight hex prism)
  * Cup        - hourglass solid + a transparent "window" half-shell
  * Goal       - tapered octagonal prism (3 heights: alliance / neutral / center)

All dimensions are authored in METERS (Gazebo convention), derived from the
VEX V5RC Override Game Manual v1.1 Appendix A figures (A5/A6/A7):

  Pin  : total 0.165 m; bottom hex 35.6 mm flat-to-flat; top hex 80.3 mm;
         taper 74.5 mm; flange 16.2 mm; top section 74.3 mm.
  Cup  : top OD 80.2 mm; waist 59 mm @ 55 mm above base; height 164.5 mm.
  Goal : base octagon 142.5 mm (across flats); top 88.8 mm; base layer 10 mm;
         heights: alliance 82.5 / neutral 146.5 / center 222.7 mm.

Run:  python3 tools/gen_meshes.py
Idempotent: regenerates the same meshes every time.
"""

import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(ROOT, "models")
PIN_MODELS = ["pin-ry", "pin-by", "pin-yy", "pin-rb"]  # all share the same geometry


# ---------------------------------------------------------------------------
# STL writer
# ---------------------------------------------------------------------------
def write_stl(path, triangles):
    """triangles: list of ((v0, v1, v2), (nx, ny, nz))"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("solid mesh\n")
        for (v0, v1, v2), n in triangles:
            f.write("  facet normal %.6g %.6g %.6g\n" % n)
            f.write("    outer loop\n")
            for v in (v0, v1, v2):
                f.write("      vertex %.6g %.6g %.6g\n" % v)
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write("endsolid mesh\n")


def _norm(v0, v1, v2):
    """unit normal of a triangle (right-hand rule)"""
    ax, ay, az = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    bx, by, bz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    nx, ny, nz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
    L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return (nx / L, ny / L, nz / L)


def _fan_triangles(ring0, ring1, closed_loops=True):
    """Build quads between two vertex rings (each a closed loop) -> triangles.
    ring0/ring1: lists of (x, y, z). ring1 may have one extra vertex at the end
    to close the fan (used by the half-shell)."""
    n = min(len(ring0), len(ring1))
    tris = []
    for i in range(n - 1):
        v00, v01 = ring0[i], ring0[i + 1]
        v10, v11 = ring1[i], ring1[i + 1]
        tris.append(((v00, v01, v11), _norm(v00, v01, v11)))
        tris.append(((v00, v11, v10), _norm(v00, v11, v10)))
    if closed_loops:
        v00, v01 = ring0[-1], ring0[0]
        v10, v11 = ring1[-1], ring1[0]
        tris.append(((v00, v01, v11), _norm(v00, v01, v11)))
        tris.append(((v00, v11, v10), _norm(v00, v11, v10)))
    return tris


def _cap(ring, outward):
    """triangulate a planar polygon ring (fan from centroid)."""
    cx = sum(v[0] for v in ring) / len(ring)
    cy = sum(v[1] for v in ring) / len(ring)
    cz = sum(v[2] for v in ring) / len(ring)
    tris = []
    for i in range(len(ring) - 1):
        a, b = ring[i], ring[i + 1]
        n = _norm(a, b, (cx, cy, cz))
        if not outward:
            n = (-n[0], -n[1], -n[2])
        tris.append(((a, b, (cx, cy, cz)), n))
    return tris


def hex_ring(z, flat_to_flat, phase=0.0):
    """vertices of a regular hexagon (flat-to-flat) at height z, meters."""
    r = (flat_to_flat / 2.0) / math.cos(math.radians(30))
    pts = []
    for k in range(6):
        a = math.radians(60 * k) + phase
        pts.append((r * math.cos(a), r * math.sin(a), z))
    pts.append(pts[0])  # close loop for the fan code
    return pts


def oct_ring(z, across_flats):
    """vertices of a regular octagon (across-flats) at height z, meters."""
    r = (across_flats / 2.0) / math.cos(math.radians(22.5))
    pts = []
    for k in range(8):
        a = math.radians(22.5 + 45 * k)
        pts.append((r * math.cos(a), r * math.sin(a), z))
    pts.append(pts[0])
    return pts


def circle_ring(z, radius, n, a0=0.0, a1=2 * math.pi):
    """circle vertices from angle a0..a1 (for lathe / half shells)."""
    pts = []
    steps = max(2, int(round(n * (a1 - a0) / (2 * math.pi))))
    for k in range(steps + 1):
        a = a0 + (a1 - a0) * k / steps
        pts.append((radius * math.cos(a), radius * math.sin(a), z))
    return pts


# ---------------------------------------------------------------------------
# Pin meshes
# ---------------------------------------------------------------------------
def pin_bottom_mesh():
    """hex flange (16.2 mm) + taper (35.6 -> 80.3 mm flat-to-flat over 74.5 mm)."""
    tris = []
    n_taper_slices = 6
    rings = [hex_ring(0.0, 0.0356), hex_ring(0.0162, 0.0356)]
    for i in range(1, n_taper_slices + 1):
        z = 0.0162 + 0.0745 * i / n_taper_slices
        f = 0.0356 + (0.0803 - 0.0356) * i / n_taper_slices
        rings.append(hex_ring(z, f))
    # rings: [0]=flange bottom, [1]=flange top, [2..7]=taper slices
    for i in range(len(rings) - 2):
        tris += _fan_triangles(rings[i], rings[i + 1])
    tris += _cap(hex_ring(0.0, 0.0356), outward=False)
    tris += _cap(hex_ring(0.0907, 0.0803), outward=True)
    return tris


def pin_top_mesh():
    """straight hex prism 80.3 mm flat-to-flat, 74.3 mm tall."""
    tris = []
    tris += _fan_triangles(hex_ring(0.0, 0.0803), hex_ring(0.0743, 0.0803))
    tris += _cap(hex_ring(0.0, 0.0803), outward=False)
    tris += _cap(hex_ring(0.0743, 0.0803), outward=True)
    return tris


# ---------------------------------------------------------------------------
# Cup meshes
# ---------------------------------------------------------------------------
def _cup_radius(z):
    """hourglass profile (meters): bottom 40 -> waist 29.5 @55mm -> top 40.1."""
    h = 0.1645
    waist_z = 0.055
    if z <= waist_z:
        t = z / waist_z
        return 0.040 + (0.0295 - 0.040) * t
    t = (z - waist_z) / (h - waist_z)
    return 0.0295 + (0.0401 - 0.0295) * t


def cup_mesh():
    tris = []
    n_slices, n_circ = 20, 24
    rings = []
    for i in range(n_slices + 1):
        z = 0.1645 * i / n_slices
        rings.append(circle_ring(z, _cup_radius(z), n_circ))
    for i in range(n_slices):
        tris += _fan_triangles(rings[i], rings[i + 1])
    # bottom cap (solid base); top is open (no cap)
    tris += _cap(circle_ring(0.0, _cup_radius(0.0), n_circ), outward=False)
    return tris


def cup_window_mesh():
    """transparent half-shell: front 180 deg of the hourglass, slightly larger
    radius so it never z-fights with the opaque body."""
    tris = []
    n_slices, n_circ = 20, 18
    rings = []
    for i in range(n_slices + 1):
        z = 0.1645 * i / n_slices
        r = _cup_radius(z) + 0.0005
        rings.append(circle_ring(z, r, n_circ, a0=-math.pi / 2, a1=math.pi / 2))
    for i in range(n_slices):
        tris += _fan_triangles(rings[i], rings[i + 1])
    # close the two flat cut faces (theta = +/- 90 deg) and the bottom edge
    for side in (-1, 1):
        ring = []
        for i in range(n_slices + 1):
            z = 0.1645 * i / n_slices
            a = side * math.pi / 2
            ring.append(((_cup_radius(z) + 0.0005) * math.cos(a),
                         (_cup_radius(z) + 0.0005) * math.sin(a), z))
        ring.append(ring[-1])
        tris += _fan_triangles(ring[:-1], ring[1:], closed_loops=False)
    return tris


# ---------------------------------------------------------------------------
# Goal meshes (tapered octagonal prism, optional top receptacle rim)
# ---------------------------------------------------------------------------
def goal_mesh(height):
    """octagon base 142.5 mm across-flats -> top 88.8 mm, with 10 mm base layer."""
    tris = []
    body_h = height - 0.010
    n_slices = 5
    rings = [oct_ring(0.0, 0.1425), oct_ring(0.010, 0.1425)]
    for i in range(1, n_slices + 1):
        z = 0.010 + body_h * i / n_slices
        f = 0.1425 + (0.0888 - 0.1425) * i / n_slices
        rings.append(oct_ring(z, f))
    tris += _fan_triangles(rings[0], rings[1])
    for i in range(1, n_slices + 1):
        tris += _fan_triangles(rings[i], rings[i + 1])
    tris += _cap(oct_ring(0.0, 0.1425), outward=False)
    tris += _cap(oct_ring(height, 0.0888), outward=True)
    return tris


def main():
    # shared element mesh dir (source of truth for pins + cup)
    shared = os.path.join(MODELS, "override-elements", "meshes")
    write_stl(os.path.join(shared, "pin-bottom.stl"), pin_bottom_mesh())
    write_stl(os.path.join(shared, "pin-top.stl"), pin_top_mesh())
    write_stl(os.path.join(shared, "cup.stl"), cup_mesh())
    write_stl(os.path.join(shared, "cup-window.stl"), cup_window_mesh())

    # cup model meshes (model.sdf references model://cup/meshes/...)
    cup_dir = os.path.join(MODELS, "cup", "meshes")
    os.makedirs(cup_dir, exist_ok=True)
    write_stl(os.path.join(cup_dir, "cup.stl"), cup_mesh())
    write_stl(os.path.join(cup_dir, "cup-window.stl"), cup_window_mesh())

    # goal model meshes, one per model dir referenced by the worlds
    for name, h in (("goal-alliance-red", 0.0825),
                    ("goal-alliance-blue", 0.0825),
                    ("goal-neutral", 0.1465),
                    ("goal-center", 0.2227)):
        write_stl(os.path.join(MODELS, name, "meshes", "goal.stl"), goal_mesh(h))

    # pins share geometry; copy the meshes into each pin model dir
    for p in PIN_MODELS:
        d = os.path.join(MODELS, p, "meshes")
        os.makedirs(d, exist_ok=True)
        for f in ("pin-bottom.stl", "pin-top.stl"):
            src = os.path.join(shared, f)
            dst = os.path.join(d, f)
            if not os.path.exists(dst) or os.path.getsize(src) != os.path.getsize(dst):
                with open(src, "rb") as fin, open(dst, "wb") as fout:
                    fout.write(fin.read())

    # report
    total = 0
    for root, _dirs, files in os.walk(MODELS):
        for f in files:
            if f.endswith(".stl"):
                p = os.path.join(root, f)
                total += os.path.getsize(p)
    print("meshes generated. total STL bytes: %d" % total)


if __name__ == "__main__":
    sys.exit(main())
