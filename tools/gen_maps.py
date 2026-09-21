#!/usr/bin/env python3
"""
gen_maps.py - Generates standard, detailed Nav2 maps for the Override field.

Outputs (all regenerable, deterministic):
  maps/override_field_map.pgm(.yaml)   navigation map (0=occupied, 254=free)
  maps/override_keepout.pgm(.yaml)     keepout mask  (0=keepout,  254=free)
  maps/override_field_detail.png       high-detail visual preview (not Nav2)

Conventions (nav2 map_server):
  * PGM is P5 binary, grayscale, maxval 255.
  * The image TOP row is the highest y (origin = bottom-left corner).
  * yaml: trinary mode, negate 0, occupied 0.65 / free 0.196.
  * resolution 0.01 m/px, map half-size 1.83 m (>= field half 1.8288 m),
    origin [-1.83, -1.83] so the field center (0,0) is the exact center pixel.

Geometry matched to the world models (override.sdf / models/*):
  * 4 perimeter walls (inner faces at |x|,|y| = 1.8288 m) -> 0.03 m band
  * 9 goals: regular octagons, 142.5 mm across-flats (inradius 71.25 mm)
  * 4 wall Toggles: 0.66 m along the wall, 0.12 m deep into the field
  * 4 Loaders: 0.34 x 0.24 m platforms
  (Field tape lines are visual only -> free space in the nav map.)

Run:  python3 tools/gen_maps.py
"""

import math
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS = os.path.join(ROOT, "maps")

RES = 0.01               # m/px
HALF = 1.83              # map half-size (m), slightly larger than 1.8288 field
SIZE = int(HALF * 2 / RES)          # 366 px
ORIGIN = (-HALF, -HALF)

OCC = 0
FREE = 254
KEEP = 0
FREE_KEEP = 254

WALL_BAND = 0.03         # thickness of the wall ring drawn on the map (m)

GOALS = {
    0: (0.0, 0.0, 'center'), 1: (-0.6, 1.2, 'neutral'), 2: (-1.2, 0.6, 'neutral'),
    3: (-1.2, -0.6, 'red'), 4: (-0.6, -1.2, 'red'), 5: (0.6, 1.2, 'blue'),
    6: (1.2, 0.6, 'blue'), 7: (1.2, -0.6, 'neutral'), 8: (0.6, -1.2, 'neutral'),
}
GOAL_INRADIUS = 0.1425 / 2.0        # 142.5 mm across-flats

LOADERS = {0: (-1.74, 1.49), 1: (-1.74, -1.49), 2: (1.74, 1.49), 3: (1.74, -1.49)}
LOADER_HX, LOADER_HY = 0.34 / 2.0, 0.24 / 2.0

TOGGLE_HALF = 0.33       # 0.66 m along the wall
TOGGLE_DEPTH = 0.12      # prism extends this far into the field


def world_to_pixel(x, y):
    """meters (field center origin) -> (row, col); row 0 = TOP of image."""
    col = int((x - ORIGIN[0]) / RES)
    row = int((ORIGIN[1] + SIZE * RES - y) / RES)
    return row, col


def pixel_to_world(row, col):
    x = ORIGIN[0] + (col + 0.5) * RES
    y = ORIGIN[1] + (SIZE - 1 - row + 0.5) * RES
    return x, y


def inside_octagon(dx, dy, inradius):
    """regular octagon (flat-to-flat 2*inradius), sides at 0/45/90/135 deg."""
    if abs(dx) > inradius or abs(dy) > inradius:
        return False
    return abs(dx) + abs(dy) <= inradius * math.sqrt(2.0)


def new_grid():
    return [[FREE] * SIZE for _ in range(SIZE)]


def fill_octagon(grid, cx, cy, inradius, value):
    r0, c0 = world_to_pixel(cx, cy)
    dr = int(inradius / RES) + 2
    for r in range(max(0, r0 - dr), min(SIZE, r0 + dr + 1)):
        for c in range(max(0, c0 - dr), min(SIZE, c0 + dr + 1)):
            x, y = pixel_to_world(r, c)
            if inside_octagon(x - cx, y - cy, inradius):
                grid[r][c] = value


def fill_rect(grid, x0, y0, x1, y1, value):
    r0, c0 = world_to_pixel(x0, y1)   # top-left
    r1, c1 = world_to_pixel(x1, y0)   # bottom-right
    for r in range(max(0, r0), min(SIZE, r1 + 1)):
        for c in range(max(0, c0), min(SIZE, c1 + 1)):
            grid[r][c] = value


def draw_walls(grid, value):
    band = int(WALL_BAND / RES)
    for r in range(SIZE):
        for c in range(SIZE):
            if r < band or r >= SIZE - band or c < band or c >= SIZE - band:
                grid[r][c] = value


def draw_toggles(grid, value):
    # wall-mounted prism: 0.66 m along the wall, 0.12 m into the field
    inner = 1.8288                      # wall inner face
    for tid, (tx, ty) in ((0, (0.0, 1.8288)), (1, (1.8288, 0.0)),
                          (2, (0.0, -1.8288)), (3, (-1.8288, 0.0))):
        if tid in (0, 2):               # north / south wall
            y0, y1 = (inner - TOGGLE_DEPTH, inner) if ty > 0 else (-inner, -inner + TOGGLE_DEPTH)
            fill_rect(grid, tx - TOGGLE_HALF, min(y0, y1), tx + TOGGLE_HALF, max(y0, y1), value)
        else:                           # east / west wall
            x0, x1 = (inner - TOGGLE_DEPTH, inner) if tx > 0 else (-inner, -inner + TOGGLE_DEPTH)
            fill_rect(grid, min(x0, x1), ty - TOGGLE_HALF, max(x0, x1), ty + TOGGLE_HALF, value)


def draw_loaders(grid, value):
    for lid, (lx, ly) in LOADERS.items():
        fill_rect(grid, lx - LOADER_HX, ly - LOADER_HY, lx + LOADER_HX, ly + LOADER_HY, value)


def write_pgm(path, grid):
    with open(path, "wb") as f:
        f.write(("P5\n%d %d\n255\n" % (SIZE, SIZE)).encode("ascii"))
        for row in grid:
            f.write(bytes(row))


def write_yaml(path, image, mode):
    with open(path, "w") as f:
        f.write("# Nav2 map_server configuration - %s\n" % image)
        f.write("image: %s\n" % image)
        f.write("mode: %s\n" % mode)
        f.write("resolution: %s\n" % RES)
        f.write("origin: [%s, %s, 0]\n" % ORIGIN)
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")


# ---------------------------------------------------------------------------
# detailed visual preview (for humans / RViz backdrop, not for Nav2)
# ---------------------------------------------------------------------------
def render_detail_png(path):
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        return
    S, M = 1600, 130
    scale = (S - 2 * M) / (2 * HALF)

    def P(x, y):
        return (M + (x + HALF) * scale, S - M - (y + HALF) * scale)

    def rect(x0, y0, x1, y1, **kw):
        d.rectangle([P(x0, y1), P(x1, y0)], **kw)

    img = Image.new("RGB", (S, S), "#0c1118")
    d = ImageDraw.Draw(img)

    # floor + walls
    rect(-1.8288, -1.8288, 1.8288, 1.8288, fill="#1d2633", outline="#3d4c63", width=4)
    d.rectangle([P(-1.8288, 1.8288), P(1.8288, -1.8288)], outline="#5b6b80", width=8)

    # alliance station platforms (outside the walls)
    rect(-2.25, -1.05, -1.83, 1.05, fill="#3a1f22", outline="#e74c3c", width=3)
    rect(1.83, -1.05, 2.25, 1.05, fill="#1c2b3d", outline="#3498db", width=3)

    # quadrant divider lines (corner -> midfield diamond)
    for sx, sy in ((1, -1), (1, 1), (-1, 1), (-1, -1)):
        d.line([P(0.6 * sx, 0.6 * sy * -1), P(1.8288 * sx, 1.8288 * sy * -1)],
               fill="#aebed2", width=3)
    # midfield diamond |x|+|y| <= 0.6
    d.polygon([P(0.6, 0), P(0, 0.6), P(-0.6, 0), P(0, -0.6)],
              outline="#dfe8f2", width=4)

    # goals (octagons)
    def octagon(cx, cy, r):
        return [P(cx + r * math.cos(math.radians(22.5 + 45 * k)),
                  cy + r * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)]

    colors = {'center': "#f4d03f", 'neutral': "#9aa7b8", 'red': "#e74c3c", 'blue': "#3498db"}
    r_goal = GOAL_INRADIUS / math.cos(math.radians(22.5))
    for gid, (gx, gy, kind) in GOALS.items():
        r = r_goal if kind != 'center' else r_goal * 1.2
        d.polygon(octagon(gx, gy, r), fill=colors[kind], outline="#ffffff", width=3)

    # toggles (wall prisms)
    for tid, (tx, ty) in ((0, (0.0, 1.8288)), (1, (1.8288, 0.0)),
                          (2, (0.0, -1.8288)), (3, (-1.8288, 0.0))):
        c = P(tx, ty)
        s = TOGGLE_HALF * scale
        depth = TOGGLE_DEPTH * scale
        if tid == 0:
            tri = [(c[0] - s, c[1]), (c[0] + s, c[1]), (c[0], c[1] - depth)]
        elif tid == 1:
            tri = [(c[0], c[1] - s), (c[0], c[1] + s), (c[0] - depth, c[1])]
        elif tid == 2:
            tri = [(c[0] - s, c[1]), (c[0] + s, c[1]), (c[0], c[1] + depth)]
        else:
            tri = [(c[0], c[1] - s), (c[0], c[1] + s), (c[0] + depth, c[1])]
        d.polygon(tri, fill="#e67e22", outline="#fff", width=2)

    # loaders
    for lid, (lx, ly) in LOADERS.items():
        rect(lx - LOADER_HX, ly - LOADER_HY, lx + LOADER_HX, ly + LOADER_HY,
             fill="#46576a", outline="#9fb0c0", width=3)

    # labels
    try:
        f_title = ImageFont.truetype("/System/Library/Fonts/PingFang.ttc", 30)
        f_small = ImageFont.truetype("/System/Library/Fonts/PingFang.ttc", 20)
        f_id = ImageFont.truetype("/System/Library/Fonts/PingFang.ttc", 18)
    except Exception:
        f_title = f_small = f_id = ImageFont.load_default()

    d.text((M, 16), "VEX V5RC Override 2026-27 - field detail map (12' x 12')",
           fill="#dfe6ee", font=f_title)
    d.text((M, S - M + 10),
           "9 goals (octagonal) | 4 wall toggles | 4 loaders | midfield |x|+|y|<=0.6m | "
           "red: x<0  blue: x>0", fill="#c6d2e0", font=f_small)
    for gid, (gx, gy, kind) in GOALS.items():
        c = P(gx, gy)
        d.text((c[0] - 6, c[1] - 10), str(gid), fill="#ffffff", font=f_id)

    legend = [("center", "#f4d03f", "center goal (222.7mm)"),
              ("neutral", "#9aa7b8", "short neutral (146.5mm)"),
              ("red", "#e74c3c", "red alliance (82.5mm)"),
              ("blue", "#3498db", "blue alliance (82.5mm)"),
              ("tri", "#e67e22", "toggle"), ("rect", "#46576a", "loader")]
    lx, ly = S - M - 520, 24
    for shape, col, name in legend:
        if shape == "tri":
            d.polygon([(lx + 12, ly + 2), (lx, ly + 22), (lx + 24, ly + 22)], fill=col, outline="#fff")
        elif shape == "rect":
            d.rectangle([lx, ly + 2, lx + 24, ly + 22], fill=col, outline="#fff")
        else:
            d.ellipse([lx, ly + 2, lx + 24, ly + 22], fill=col, outline="#fff")
        d.text((lx + 32, ly), name, fill="#dfe6ee", font=f_small)
        ly += 32

    img.save(path)
    print("detail preview saved:", path)


def main():
    os.makedirs(MAPS, exist_ok=True)

    # ---- navigation map ----
    nav = new_grid()
    draw_walls(nav, OCC)
    for gid, (gx, gy, _k) in GOALS.items():
        fill_octagon(nav, gx, gy, GOAL_INRADIUS, OCC)
    draw_toggles(nav, OCC)
    draw_loaders(nav, OCC)
    write_pgm(os.path.join(MAPS, "override_field_map.pgm"), nav)
    write_yaml(os.path.join(MAPS, "override_field_map.yaml"),
               "override_field_map.pgm", "trinary")

    # ---- keepout mask (0 = keepout, 254 = free) ----
    keep_free = [[FREE_KEEP] * SIZE for _ in range(SIZE)]
    draw_walls(keep_free, KEEP)
    for gid, (gx, gy, _k) in GOALS.items():
        fill_octagon(keep_free, gx, gy, 0.30, KEEP)    # inflated goal zones
    draw_toggles(keep_free, KEEP)
    for lid, (lx, ly) in LOADERS.items():
        fill_rect(keep_free, lx - LOADER_HX - 0.05, ly - LOADER_HY - 0.05,
                  lx + LOADER_HX + 0.05, ly + LOADER_HY + 0.05, KEEP)
    write_pgm(os.path.join(MAPS, "override_keepout.pgm"), keep_free)
    write_yaml(os.path.join(MAPS, "override_keepout.yaml"),
               "override_keepout.pgm", "trinary")

    render_detail_png(os.path.join(MAPS, "override_field_detail.png"))
    print("maps generated: override_field_map, override_keepout, override_field_detail")


if __name__ == "__main__":
    main()
