#!/usr/bin/env python3
"""
gen_models.py - Generates all Override game-element model files (model.sdf +
model.config) for the override_sim package.

Run:  python3 tools/gen_models.py
Idempotent: regenerates identical files every time.

Geometry follows the VEX V5RC Override Game Manual v1.1 Appendix A and the
team CAD placeholders (Spartan Design) -- see gen_meshes.py for the mesh
dimensions.
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS = os.path.join(ROOT, "models")

# ---------------------------------------------------------------------------
# materials
# ---------------------------------------------------------------------------
def mat(r, g, b, a=1.0, emissive=(0, 0, 0)):
    return """<material>
        <ambient>%g %g %g %g</ambient>
        <diffuse>%g %g %g %g</diffuse>
        <specular>0.1 0.1 0.1 1.0</specular>
        <emissive>%g %g %g 1</emissive>
      </material>""" % (r, g, b, a, r, g, b, a, *emissive)


RED = mat(0.75, 0.16, 0.12)
BLUE = mat(0.10, 0.22, 0.78)
YELLOW = mat(0.92, 0.74, 0.10)
NEUTRAL = mat(0.86, 0.86, 0.86)
GREY = mat(0.55, 0.55, 0.55)
DARK = mat(0.08, 0.08, 0.08)


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def model_config(name, description):
    return """<?xml version="1.0"?>
<model>
  <name>%s</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>override_sim</name>
    <email>override@example.com</email>
  </author>
  <description>%s</description>
  <ros>
    <version>jazzy</version>
  </ros>
</model>
""" % (name, description)


# ---------------------------------------------------------------------------
# pins
# ---------------------------------------------------------------------------
PIN_COLORS = {
    "pin-ry": (RED, YELLOW),   # bottom red, top yellow
    "pin-by": (BLUE, YELLOW),  # bottom blue, top yellow
    "pin-yy": (YELLOW, YELLOW),
    "pin-rb": (RED, BLUE),     # bottom red, top blue
}

PIN_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <velocity_decay>
        <linear>0.4</linear>
      </velocity_decay>
      <inertial>
        <pose>0 0 0.0825 0 0 0</pose>
        <mass>0.073</mass>
        <inertia>
          <ixx>0.000206</ixx>
          <iyy>0.000206</iyy>
          <izz>0.000081</izz>
        </inertia>
      </inertial>
      <visual name="bottom_half">
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/pin-bottom.stl</uri>
          </mesh>
        </geometry>
        {bottom_mat}
      </visual>
      <visual name="top_half">
        <pose>0 0 0.0907 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/pin-top.stl</uri>
          </mesh>
        </geometry>
        {top_mat}
      </visual>
      <collision name="collision">
        <pose>0 0 0.0825 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.047</radius>
            <length>0.165</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.6</mu>
              <mu2>0.6</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>20</update_frequency>
      <static_publisher>false</static_publisher>
      <static_update_frequency>1</static_update_frequency>
    </plugin>
    <plugin filename="rollingFriction" name="rolling_friction::RollingFrictionPlugin">
      <link_name>link</link_name>
      <coefficient>0.01</coefficient>
    </plugin>
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# cup
# ---------------------------------------------------------------------------
CUP_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="cup">
    <static>false</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <velocity_decay>
        <linear>0.4</linear>
      </velocity_decay>
      <inertial>
        <pose>0 0 0.08225 0 0 0</pose>
        <mass>0.078</mass>
        <inertia>
          <ixx>0.000235</ixx>
          <iyy>0.000235</iyy>
          <izz>0.000066</izz>
        </inertia>
      </inertial>
      <visual name="body">
        <geometry>
          <mesh>
            <uri>model://cup/meshes/cup.stl</uri>
          </mesh>
        </geometry>
        {body_mat}
      </visual>
      <visual name="clear_window">
        <geometry>
          <mesh>
            <uri>model://cup/meshes/cup-window.stl</uri>
          </mesh>
        </geometry>
        <material>
          <ambient>0.85 0.92 1.0 0.4</ambient>
          <diffuse>0.85 0.92 1.0 0.4</diffuse>
          <specular>0.8 0.8 0.8 1.0</specular>
          <emissive>0 0 0 1</emissive>
        </material>
        <transparency>0.65</transparency>
        <cast_shadows>false</cast_shadows>
      </visual>
      <collision name="collision">
        <pose>0 0 0.08225 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.041</radius>
            <length>0.1645</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.6</mu>
              <mu2>0.6</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_sensor_pose>false</publish_sensor_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>20</update_frequency>
      <static_publisher>false</static_publisher>
      <static_update_frequency>1</static_update_frequency>
    </plugin>
    <plugin filename="rollingFriction" name="rolling_friction::RollingFrictionPlugin">
      <link_name>link</link_name>
      <coefficient>0.01</coefficient>
    </plugin>
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# goals (octagonal posts, three heights)
# ---------------------------------------------------------------------------
def goal_sdf(name, height, color_mat, description):
    return """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <pose>0 0 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{name}/meshes/goal.stl</uri>
          </mesh>
        </geometry>
        {color_mat}
      </visual>
      <visual name="receptacle">
        <pose>0 0 {receptacle_z} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.030</radius>
            <length>0.008</length>
          </cylinder>
        </geometry>
        {dark_mat}
      </visual>
      <collision name="collision">
        <pose>0 0 {half_h} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.0715</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
""".format(name=name, height=height, half_h=height / 2,
           receptacle_z=height - 0.01, color_mat=color_mat, dark_mat=DARK)


# ---------------------------------------------------------------------------
# toggle (triangular prism, 3 faces: yellow / red / blue)
# ---------------------------------------------------------------------------
TOGGLE_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="toggle">
    <static>true</static>
    <link name="body">
      <pose>0 0 0 0 0 0</pose>
      <!-- 660 mm triangular prism; triangle points DOWN so the yellow (top)
           face is horizontal. Rotating the model 120 deg around its X axis
           cycles yellow -> red -> blue face up (state 0/1/2).
           Triangle cross-section lives in the YZ plane. -->
      <visual name="face_yellow">
        <pose>0 0 0.01507 0 0 0</pose>
        <geometry>
          <box><size>0.656 0.0562 0.004</size></box>
        </geometry>
        {yellow}
      </visual>
      <visual name="face_blue">
        <pose>0 -0.01305 -0.007535 0.5236 0 0</pose>
        <geometry>
          <box><size>0.656 0.0562 0.004</size></box>
        </geometry>
        {blue}
      </visual>
      <visual name="face_red">
        <pose>0 0.01305 -0.007535 2.618 0 0</pose>
        <geometry>
          <box><size>0.656 0.0562 0.004</size></box>
        </geometry>
        {red}
      </visual>
      <visual name="shaft_l">
        <pose>-0.328 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder><radius>0.015</radius><length>0.07</length></cylinder>
        </geometry>
        {dark}
      </visual>
      <visual name="shaft_r">
        <pose>0.328 0 0 0 1.5708 0</pose>
        <geometry>
          <cylinder><radius>0.015</radius><length>0.07</length></cylinder>
        </geometry>
        {dark}
      </visual>
      <collision name="collision">
        <pose>0 0 -0.0075 0 0 0</pose>
        <geometry>
          <box><size>0.656 0.06 0.05</size></box>
        </geometry>
        <surface>
          <friction>
            <ode><mu>0.5</mu><mu2>0.5</mu2></ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# loader (station shelf where match elements are loaded)
# ---------------------------------------------------------------------------
LOADER_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="loader">
    <static>true</static>
    <link name="body">
      <pose>0 0 0 0 0 0</pose>
      <visual name="platform">
        <pose>0 0 0.045 0 0 0</pose>
        <geometry>
          <box><size>0.34 0.24 0.09</size></box>
        </geometry>
        {grey}
      </visual>
      <visual name="opening">
        <pose>0 0 0.1 0 0 0</pose>
        <geometry>
          <cylinder><radius>0.10</radius><length>0.02</length></cylinder>
        </geometry>
        {dark}
      </visual>
      <visual name="rim">
        <pose>0 0 0.112 0 0 0</pose>
        <geometry>
          <cylinder><radius>0.115</radius><length>0.008</length></cylinder>
        </geometry>
        {neutral}
      </visual>
      <collision name="collision">
        <pose>0 0 0.045 0 0 0</pose>
        <geometry>
          <box><size>0.34 0.24 0.09</size></box>
        </geometry>
        <surface>
          <friction>
            <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""


# ---------------------------------------------------------------------------
# field (perimeter walls, floor, markings, alliance stations)
# ---------------------------------------------------------------------------
FIELD_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="override-field">
    <static>true</static>
    <link name="body">
      <pose>0 0 0 0 0 0</pose>
      {body}
    </link>
  </model>
</sdf>
"""


def box_visual(x, y, z, sx, sy, sz, mat_str, roll=0.0, pitch=0.0, yaw=0.0):
    return """      <visual name="v_{x}_{y}">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        {mat}
      </visual>
      <collision name="c_{x}_{y}">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
""".format(x=x, y=y, z=z, sx=sx, sy=sy, sz=sz,
           roll=roll, pitch=pitch, yaw=yaw, mat=mat_str)


def tape_visual(x, y, z, sx, sy, sz, yaw, mat_str=None):
    """marking tape: visual only, no collision"""
    if mat_str is None:
        mat_str = mat(0.96, 0.96, 0.96, 1.0, (0.05, 0.05, 0.05))
    return """      <visual name="tape_{x}_{y}">
        <pose>{x} {y} {z} 0 0 {yaw}</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        {mat}
      </visual>
""".format(x=x, y=y, z=z, sx=sx, sy=sy, sz=sz, yaw=yaw, mat=mat_str)


def build_field_sdf():
    half = 1.8288
    parts = []
    # --- floor -------------------------------------------------------------
    parts.append(box_visual(0, 0, -0.01, half * 2, half * 2, 0.02,
                            mat(0.42, 0.42, 0.44)))
    # --- perimeter walls ---------------------------------------------------
    wall_len = half * 2 + 0.1
    for s in (-1, 1):
        parts.append(box_visual(s * (half + 0.025), 0, 0.06, wall_len, 0.05, 0.12,
                                mat(0.15, 0.15, 0.17)))
        parts.append(box_visual(0, s * (half + 0.025), 0.06, 0.05, wall_len, 0.12,
                                mat(0.15, 0.15, 0.17)))
    # --- alliance stations (outside the walls, left = red, right = blue) ---
    parts.append(box_visual(-(half + 0.45), 0, 0.01, 0.7, 2.2, 0.02, RED))
    parts.append(box_visual(half + 0.45, 0, 0.01, 0.7, 2.2, 0.02, BLUE))
    # --- midfield diamond tape (|x|+|y| <= 0.6) ----------------------------
    edge = 0.6 * 2 ** 0.5
    for cx, cy, yaw in ((0.3, 0.3, 0.7854), (-0.3, 0.3, -0.7854),
                        (-0.3, -0.3, 0.7854), (0.3, -0.3, -0.7854)):
        parts.append(tape_visual(cx, cy, 0.005, edge, 0.03, 0.005, yaw))
    # --- quadrant divider / autonomous line segments (corners -> diamond) --
    seg = (half - 0.6) * 2 ** 0.5
    for cx, cy, yaw in ((-(half + 0.6) / 2, (half + 0.6) / 2, -0.7854),
                        ((half + 0.6) / 2, (half + 0.6) / 2, 2.3562),
                        (-(half + 0.6) / 2, -(half + 0.6) / 2, 0.7854),
                        ((half + 0.6) / 2, -(half + 0.6) / 2, 2.3562)):
        parts.append(tape_visual(cx, cy, 0.005, seg, 0.025, 0.005, yaw))
    # --- alliance corner markers (diagonal, standard VEX) ------------------
    for cx, cy, c in ((half, half, BLUE), (half, -half, RED),
                      (-half, -half, BLUE), (-half, half, RED)):
        parts.append(tape_visual(cx * 0.97, cy * 0.97, 0.005, 0.24, 0.24, 0.005, 0.0, c))
    return FIELD_SDF.format(body="\n".join(parts))


def main():
    # pins
    for name, (bottom, top) in PIN_COLORS.items():
        write(os.path.join(MODELS, name, "model.sdf"),
              PIN_SDF.format(name=name, bottom_mat=bottom, top_mat=top))
        write(os.path.join(MODELS, name, "model.config"),
              model_config(name, "Override bicolor pin (hex, 165 mm)"))

    # cup
    write(os.path.join(MODELS, "cup", "model.sdf"),
          CUP_SDF.format(body_mat=mat(0.72, 0.72, 0.74)))
    write(os.path.join(MODELS, "cup", "model.config"),
          model_config("cup", "Override hourglass cup (opaque + clear halves, 164.5 mm)"))

    # goals
    goals = (
        ("goal-alliance-red", 0.0825, RED, "Override red alliance goal (82.5 mm)"),
        ("goal-alliance-blue", 0.0825, BLUE, "Override blue alliance goal (82.5 mm)"),
        ("goal-neutral", 0.1465, NEUTRAL, "Override short neutral goal (146.5 mm)"),
        ("goal-center", 0.2227, NEUTRAL, "Override tall center goal (222.7 mm)"),
    )
    for name, h, c, desc in goals:
        write(os.path.join(MODELS, name, "model.sdf"), goal_sdf(name, h, c, desc))
        write(os.path.join(MODELS, name, "model.config"), model_config(name, desc))

    # toggle
    write(os.path.join(MODELS, "toggle", "model.sdf"),
          TOGGLE_SDF.format(yellow=YELLOW, red=RED, blue=BLUE, dark=DARK))
    write(os.path.join(MODELS, "toggle", "model.config"),
          model_config("toggle", "Override field toggle (660 mm triangular prism)"))

    # loader
    write(os.path.join(MODELS, "loader", "model.sdf"),
          LOADER_SDF.format(grey=GREY, dark=DARK, neutral=NEUTRAL))
    write(os.path.join(MODELS, "loader", "model.config"),
          model_config("loader", "Override match loader shelf"))

    # field
    write(os.path.join(MODELS, "override-field", "model.sdf"), build_field_sdf())
    write(os.path.join(MODELS, "override-field", "model.config"),
          model_config("override-field", "V5RC Override 12x12 field (walls, floor, markings)"))

    n = sum(len(files) for _, _, files in os.walk(MODELS))
    print("models generated. total files: %d" % n)


if __name__ == "__main__":
    main()
