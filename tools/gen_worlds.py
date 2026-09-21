#!/usr/bin/env python3
"""
gen_worlds.py - Generates the Override simulation worlds:

  worlds/override.sdf          full field (goals, toggles, loaders, elements)
  worlds/override_empty.sdf    field with no scoring elements

Element layout follows the official VEX V5RC Override field overview (Figure
FO-1) with GPS coordinates from the VEXcode VR V5RC Override playground
(millimeters, field center at origin). The 20 scoring objects are placed at
the official starting positions (12 pins + 8 cups).

Run:  python3 tools/gen_worlds.py
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(ROOT, "worlds")

WORLD_HEAD = """<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="override">
    <scene>
      <ambient>0.45 0.45 0.5 1</ambient>
      <background>0.2 0.2 0.25 1</background>
      <shadows>true</shadows>
      <sky>
        <clouds>
          <speed>1</speed>
        </clouds>
      </sky>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -1</direction>
    </light>
"""

WORLD_FOOT = """  </world>
</sdf>
"""


def include(model, name, x, y, z=0.0, yaw=0.0, roll=0.0, pitch=0.0):
    return """    <include>
      <name>%s</name>
      <uri>model://%s</uri>
      <pose>%g %g %g %g %g %g</pose>
    </include>
""" % (name, model, x, y, z, roll, pitch, yaw)


# official goal positions (meters, from VEXcode VR GPS data)
GOALS = [
    ("goal_center",  "goal-center",        (0.0,   0.0)),
    ("goal_neutral_1", "goal-neutral",     (-0.6,  1.2)),
    ("goal_neutral_2", "goal-neutral",     (-1.2,  0.6)),
    ("goal_neutral_3", "goal-neutral",     (1.2,  -0.6)),
    ("goal_neutral_4", "goal-neutral",     (0.6,  -1.2)),
    ("goal_red_1",   "goal-alliance-red",  (-1.2, -0.6)),
    ("goal_red_2",   "goal-alliance-red",  (-0.6, -1.2)),
    ("goal_blue_1",  "goal-alliance-blue", (0.6,   1.2)),
    ("goal_blue_2",  "goal-alliance-blue", (1.2,   0.6)),
]

# toggles at the center of each field wall (triangular prism runs along X;
# rotate 90 deg for the left/right walls)
TOGGLES = [
    ("toggle_1", (0.0,  1.78), 0.0),
    ("toggle_2", (1.78, 0.0),  1.5708),
    ("toggle_3", (0.0, -1.78), 0.0),
    ("toggle_4", (-1.78, 0.0), 1.5708),
]

# loaders adjacent to the alliance stations (red: left, blue: right)
LOADERS = [
    ("loader_1", (-1.74,  1.49)),
    ("loader_2", (-1.74, -1.49)),
    ("loader_3", (1.74,   1.49)),
    ("loader_4", (1.74,  -1.49)),
]

# official 20 starting positions (12 pins + 8 cups), VEXcode VR layout
ELEMENTS = [
    ("pin_ry_1",  "pin-ry", (-0.6,   1.745)),
    ("pin_by_1",  "pin-by", (0.6,    1.745)),
    ("cup_1",     "cup",    (-1.2,   1.2)),
    ("cup_2",     "cup",    (1.2,    1.2)),
    ("pin_ry_2",  "pin-ry", (-1.745, 0.6)),
    ("cup_3",     "cup",    (-0.6,   0.6)),
    ("pin_yy_1",  "pin-yy", (0.0,    0.6)),
    ("cup_4",     "cup",    (0.6,    0.6)),
    ("pin_by_2",  "pin-by", (1.745,  0.6)),
    ("pin_rb_1",  "pin-rb", (-0.6,   0.0)),
    ("pin_rb_2",  "pin-rb", (0.6,    0.0)),
    ("pin_by_3",  "pin-by", (-1.745, -0.6)),
    ("cup_5",     "cup",    (-0.6,   -0.6)),
    ("pin_yy_2",  "pin-yy", (0.0,   -0.6)),
    ("cup_6",     "cup",    (0.6,   -0.6)),
    ("pin_ry_3",  "pin-ry", (1.745, -0.6)),
    ("cup_7",     "cup",    (-1.2,  -1.2)),
    ("cup_8",     "cup",    (1.2,   -1.2)),
    ("pin_by_4",  "pin-by", (-0.6,  -1.745)),
    ("pin_ry_4",  "pin-ry", (0.6,   -1.745)),
]


def build_world(with_elements):
    parts = [WORLD_HEAD]
    parts.append(include("override-field", "override-field", 0, 0))
    for name, model, (x, y) in GOALS:
        parts.append(include(model, name, x, y))
    for name, (x, y), yaw in TOGGLES:
        parts.append(include("toggle", name, x, y, yaw=yaw))
    for name, (x, y) in LOADERS:
        parts.append(include("loader", name, x, y))
    if with_elements:
        for name, model, (x, y) in ELEMENTS:
            # model origin == element base (geometry starts at z=0); spawn a
            # hair above the floor so the element settles without penetrating
            parts.append(include(model, name, x, y, z=0.01))
    parts.append(WORLD_FOOT)
    return "".join(parts)


def main():
    os.makedirs(WORLDS, exist_ok=True)
    with open(os.path.join(WORLDS, "override.sdf"), "w") as f:
        f.write(build_world(True))
    with open(os.path.join(WORLDS, "override_empty.sdf"), "w") as f:
        f.write(build_world(False))
    print("worlds generated: override.sdf, override_empty.sdf")


if __name__ == "__main__":
    main()
