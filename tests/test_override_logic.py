#!/usr/bin/env python3
"""Offline logic verification for override_sim nodes (no ROS required).

Stubs rclpy / ROS message packages / scipy, then imports the real
field_location.py and scoring.py from src/ and exercises their pure logic:
  * element -> goal / loader / field classification
  * goal stack building (pins + cups with yaw)
  * Override scoring (terminal points, yellow ownership, midfield, cups)
"""

import math
import os
import sys
import types

import numpy as np

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
sys.path.insert(0, SRC)

# ---------------------------------------------------------------------------
# minimal ROS stubs
# ---------------------------------------------------------------------------
class _Node:
    def __init__(self, name):
        self.name = name
        self._pub_history = {}
    def get_logger(self):
        class L:
            def info(self, *a): pass
            def warning(self, *a): pass
        return L()
    def create_subscription(self, *a, **k): pass
    def create_publisher(self, msg_type, topic, *a, **k):
        class P:
            def __init__(self, outer, topic):
                self._outer, self._topic = outer, topic
            def publish(self, msg):
                self._outer._pub_history[self._topic] = msg
        return P(self, topic)
    def create_service(self, *a, **k): pass
    def create_client(self, *a, **k): pass
    def create_timer(self, *a, **k): pass
    def declare_parameter(self, *a, **k): pass
    def get_parameter(self, n):
        class V:
            value = {"world_name": "override", "intake_capacity": 10,
                     "autonomous": False}.get(n, 0)
        return V()
    def get_clock(self):
        class C:
            def now(self):
                class N:
                    def to_msg(self): return None
                return N()
        return C()


rclpy_stub = types.ModuleType("rclpy")
rclpy_stub.__path__ = []
rclpy_stub.init = lambda *a, **k: None
rclpy_stub.ok = lambda: True
rclpy_stub.spin = lambda *a, **k: None
rclpy_stub.shutdown = lambda *a, **k: None
sys.modules["rclpy"] = rclpy_stub

_node_mod = types.ModuleType("rclpy.node")
_node_mod.Node = _Node
sys.modules["rclpy.node"] = _node_mod

_act = types.ModuleType("rclpy.action")
_act.ActionClient = object
sys.modules["rclpy.action"] = _act


def _msg_cls(**defaults):
    class M:
        def __init__(self, **kw):
            for k, v in defaults.items():
                setattr(self, k, _deepcopy(v))
            for k, v in kw.items():
                setattr(self, k, _deepcopy(v))
    return M


def _deepcopy(v):
    if isinstance(v, list):
        return [x for x in v]
    if isinstance(v, dict):
        return dict(v)
    return v


class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quat:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


_geom = types.ModuleType("geometry_msgs")
_geom.msg = types.ModuleType("geometry_msgs.msg")
_geom.msg.Point = _Point
_geom.msg.Quaternion = _Quat
_geom.msg.PoseArray = _msg_cls(poses=[])
_geom.msg.PoseStamped = _msg_cls(pose=_msg_cls(position=_Point(), orientation=_Quat())())
_geom.msg.Pose2D = _msg_cls(x=0.0, y=0.0, theta=0.0)
sys.modules["geometry_msgs"] = _geom
sys.modules["geometry_msgs.msg"] = _geom.msg

_std = types.ModuleType("std_msgs")
_std.msg = types.ModuleType("std_msgs.msg")
_std.msg.Int64MultiArray = _msg_cls(data=[])
_std.msg.Header = _msg_cls(stamp=None, frame_id="")
sys.modules["std_msgs"] = _std
sys.modules["std_msgs.msg"] = _std.msg

_sensor = types.ModuleType("sensor_msgs")
_sensor.msg = types.ModuleType("sensor_msgs.msg")
_sensor.msg.Joy = _msg_cls(buttons=[])
sys.modules["sensor_msgs"] = _sensor
sys.modules["sensor_msgs.msg"] = _sensor.msg

_ov = types.ModuleType("override_sim")
_ov.msg = types.ModuleType("override_sim.msg")
_ov.srv = types.ModuleType("override_sim.srv")
_ov.msg.PinElement = _msg_cls(
    object_name="", id=0, top_color=0, bottom_color=0,
    location=_Point(), orientation=_Quat())
_ov.msg.CupElement = _msg_cls(
    object_name="", id=0, location=_Point(), orientation=_Quat(), yaw=0.0)
_ov.msg.GoalState = _msg_cls(goal_id=0, goal_type=0, toggle_state=0,
                             quadrant="", pins=[], cups=[])
_ov.msg.GoalArray = _msg_cls(goals=[])
_ov.msg.ToggleState = _msg_cls(toggle_id=0, state=0, quadrant="")
_ov.msg.ToggleArray = _msg_cls(toggles=[])
_ov.msg.FieldElement = _msg_cls(
    object_name="", id=0, element_type=0, top_color=0, bottom_color=0,
    location=_Point(), orientation=_Quat())
_ov.msg.FieldElementArray = _msg_cls(elements=[])
_ov.msg.ActionState = _msg_cls()
_ov.msg.WorldState = _msg_cls()
for name in ("ScoreElement", "IntakeElement", "LoadElement", "FlipToggle"):
    setattr(_ov.srv, name, _msg_cls(Request=_msg_cls(), Response=_msg_cls()))
sys.modules["override_sim"] = _ov
sys.modules["override_sim.msg"] = _ov.msg
sys.modules["override_sim.srv"] = _ov.srv

# scipy stub (Rotation via numpy)
def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    ])


class _Rot:
    def __init__(self, quat):
        self.quat = np.asarray(quat, float)
    @classmethod
    def from_quat(cls, q):
        return cls(q)
    @classmethod
    def from_euler(cls, seq, angles):
        q = np.array([0.0, 0.0, 0.0, 1.0])
        for axis, ang in zip(seq, np.atleast_1d(angles)):
            h = ang / 2.0
            v = {"x": [np.sin(h), 0, 0, np.cos(h)],
                 "y": [0, np.sin(h), 0, np.cos(h)],
                 "z": [0, 0, np.sin(h), np.cos(h)]}[axis]
            q = _qmul(q, v)
        return cls(q)
    def as_euler(self, seq):
        x, y, z, w = self.quat
        return np.array([0.0, 0.0, np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))])
    def __mul__(self, other):
        return _Rot(_qmul(self.quat, other.quat))

_sp = types.ModuleType("scipy"); _sp.__path__ = []
sys.modules["scipy"] = _sp
_spat = types.ModuleType("scipy.spatial"); _spat.__path__ = []
sys.modules["scipy.spatial"] = _spat
_tr = types.ModuleType("scipy.spatial.transform"); _tr.__path__ = []
_tr.Rotation = _Rot
sys.modules["scipy.spatial.transform"] = _tr

# import the nodes
import field_location as fl_mod
import scoring as sc_mod

FAIL = []


def check(name, cond):
    if cond:
        print(f"  PASS  {name}")
    else:
        FAIL.append(name)
        print(f"  FAIL  {name}")


def mk_pin(name, gid=0, x=0.0, y=0.0, top=3, bottom=1, z=None, yaw=0.0):
    el = _ov.msg.FieldElement()
    el.object_name = name
    el.id = hash(name)
    el.element_type = 1
    el.top_color = top
    el.bottom_color = bottom
    gx, gy, _t, _q, h = fl_mod.GOALS[gid]
    el.location = _Point(gx + x, gy + y, h if z is None else z)
    el.orientation = _Quat(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    return el


def mk_cup(name, gid=0, x=0.0, y=0.0, z=None, yaw=0.0):
    el = _ov.msg.FieldElement()
    el.object_name = name
    el.id = hash(name)
    el.element_type = 2
    el.top_color = el.bottom_color = 0
    gx, gy, _t, _q, h = fl_mod.GOALS[gid]
    el.location = _Point(gx + x, gy + y, h if z is None else z)
    el.orientation = _Quat(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    return el


def classify(elements):
    node = fl_mod.FieldLocation()
    node.elements = elements
    node.robot_x = 0.0
    node.robot_y = 0.0
    node.robot_r = 0.0
    node.prev_buttons = [0, 0, 0, 0]
    node.toggle_states = {0: 0, 1: 0, 2: 0, 3: 0}
    msg = _ov.msg.FieldElementArray()
    msg.elements = elements
    node.object_location_callback(msg)
    node.last_goals = node._pub_history['/goals']
    node.field_elems = node._pub_history['/field_objects']
    node.loader_counts = node._pub_history['/loaders'].data
    return node


def goal_of(node, gid):
    for g in node.last_goals.goals:
        if g.goal_id == gid:
            return g
    return None


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
print("== field_location: classification ==")
node = classify([mk_pin("pin_ry_1", gid=3),                 # in red goal 3
                 mk_cup("cup_1", gid=3, z=0.0825 + 0.165),  # on top of pin
                 mk_pin("pin_by_1", gid=5),                 # in blue goal 5
                 mk_pin("pin_yy_1", gid=0),                 # on center goal
                 mk_pin("pin_rb_1", x=-1.74, y=1.49, z=0.08),  # on loader 0
                 mk_pin("pin_ry_5", x=-0.3, y=-0.3, z=0.0),     # on the field
                 ])
check("red goal 3 has 1 pin + 1 cup",
      len(goal_of(node, 3).pins) == 1 and len(goal_of(node, 3).cups) == 1)
check("blue goal 5 has 1 pin", len(goal_of(node, 5).pins) == 1)
check("center goal 0 has 1 pin", len(goal_of(node, 0).pins) == 1)
check("field keeps loose element", len(node.field_elems.elements) == 1)
check("loader 0 counts 1", node.loader_counts == [1, 0, 0, 0])

print("== scoring: terminal points ==")
sc = sc_mod.Scoring()
sc.robot_x, sc.robot_y = 0.0, 0.0
sc.opp_x, sc.opp_y = 0.0, 0.0
sc.auton_active = False
sc.auton_bonus = (0, 0)
_orig_calc = sc.score_calculator


def run_calc(ga):
    _orig_calc(ga)
    sc.last_score = sc._pub_history['/game_score'].data


sc.score_calculator = run_calc
# keep both robots outside the midfield for the pure terminal tests
sc.robot_x, sc.robot_y = 1.5, 1.5
sc.opp_x, sc.opp_y = 1.5, -1.5

ga = _ov.msg.GoalArray()
g = _ov.msg.GoalState()
g.goal_id, g.goal_type, g.quadrant, g.toggle_state = 3, 2, 'W', 1
p = _ov.msg.PinElement()
p.top_color, p.bottom_color = 3, 1
p.location = _Point(0, 0, 0.0825)
g.pins.append(p)
ga.goals.append(g)
sc.score_calculator(ga)
check("red goal pin-ry (toggle red): red=5, yellow=10 -> red 15",
      sc.last_score == [15, 0])

g = _ov.msg.GoalState()
g.goal_id, g.goal_type, g.quadrant, g.toggle_state = 6, 3, 'E', 0
p = _ov.msg.PinElement()
p.top_color, p.bottom_color = 3, 3
p.location = _Point(0, 0, 0.0825)
g.pins.append(p)
ga.goals.append(g)
sc.score_calculator(ga)
check("yellow pin on goal with neutral toggle scores 0",
      sc.last_score == [15, 0])

g = _ov.msg.GoalState()
g.goal_id, g.goal_type, g.quadrant, g.toggle_state = 6, 3, 'E', 2
p = _ov.msg.PinElement()
p.top_color, p.bottom_color = 3, 3
p.location = _Point(0, 0, 0.0825)
g.pins.append(p)
ga2 = _ov.msg.GoalArray()
ga2.goals.append(g)
sc.score_calculator(ga2)
check("yellow/yellow pin, blue toggle -> blue 20", sc.last_score == [0, 20])

print("== scoring: cup coverage ==")
g = _ov.msg.GoalState()
g.goal_id, g.goal_type, g.quadrant, g.toggle_state = 3, 2, 'W', 1
p = _ov.msg.PinElement()
p.top_color, p.bottom_color = 3, 1
p.location = _Point(0, 0, 0.0825)
g.pins.append(p)
c = _ov.msg.CupElement()
c.location = _Point(0, 0, 0.0825 + 0.165)
c.yaw = 0.0
g.cups.append(c)
ga = _ov.msg.GoalArray()
ga.goals.append(g)
sc.score_calculator(ga)
check("cup yaw 0 hides red bottom -> only yellow 10", sc.last_score == [10, 0])

g.cups[0].yaw = math.pi
ga = _ov.msg.GoalArray()
ga.goals.append(g)
sc.score_calculator(ga)
check("cup yaw pi hides yellow top -> only red 5", sc.last_score == [5, 0])

print("== scoring: midfield robots ==")
sc.robot_x, sc.robot_y = 0.1, 0.1
sc.opp_x, sc.opp_y = 1.5, 1.5
sc.score_calculator(_ov.msg.GoalArray())
check("red robot in midfield +8", sc.last_score == [8, 0])

sc.robot_x, sc.robot_y = 1.5, 0.0
sc.opp_x, sc.opp_y = 0.0, 0.0
sc.score_calculator(_ov.msg.GoalArray())
check("blue opponent in midfield +8", sc.last_score == [0, 8])

print("== scoring: center-goal yellow ownership by midfield ==")
g = _ov.msg.GoalState()
g.goal_id, g.goal_type, g.quadrant = 0, 0, 'C'
p = _ov.msg.PinElement()
p.top_color, p.bottom_color = 3, 3
p.location = _Point(0, 0, 0.2227)
g.pins.append(p)
ga = _ov.msg.GoalArray()
ga.goals.append(g)
sc.robot_x, sc.robot_y = 0.0, 0.0
sc.opp_x, sc.opp_y = 1.5, 1.5
sc.score_calculator(ga)
check("center yellow pin -> red (2 yellow terminals 20 + robot 8)",
      sc.last_score == [28, 0])
sc.opp_x, sc.opp_y = 0.0, 0.0
sc.score_calculator(ga)
check("center yellow pin tie -> no ownership, 8+8", sc.last_score == [8, 8])

print("== scoring: autonomous bonus ==")
sc.auton_started = False
sc.auton_active = False
sc.auton_bonus = (0, 0)
sc.phase_cb(_msg_cls(data=0)())
sc.robot_x, sc.robot_y = 1.5, 1.5
g = _ov.msg.GoalState()
g.goal_id, g.goal_type, g.quadrant, g.toggle_state = 3, 2, 'W', 1
p = _ov.msg.PinElement()
p.top_color, p.bottom_color = 3, 1
p.location = _Point(0, 0, 0.0825)
g.pins.append(p)
ga = _ov.msg.GoalArray()
ga.goals.append(g)
sc.score_calculator(ga)
check("auton snapshot red 15", sc.auton_red == 15 and sc.auton_blue == 0)
sc.phase_cb(_msg_cls(data=1)())
sc.score_calculator(ga)
check("auton bonus +12 red", sc.last_score[0] == 15 + 12)

print()
if FAIL:
    print("FAILED:", FAIL)
    sys.exit(1)
print("ALL LOGIC TESTS PASSED")
