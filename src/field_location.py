#!/usr/bin/env python3

import math

import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int64MultiArray

from override_sim.msg import (
    CupElement, FieldElement, FieldElementArray, GoalArray, GoalState,
    PinElement, ToggleArray,
)
from override_sim.srv import FlipToggle, IntakeElement, ScoreElement

# ---------------------------------------------------------------------------
# Override field reference data (meters, field center at origin)
# ---------------------------------------------------------------------------
# goal_id: (x, y, type, quadrant, height)
# type: 0 = tall neutral center, 1 = short neutral, 2 = red alliance, 3 = blue
GOALS = {
    0: (0.0, 0.0, 0, 'C', 0.2227),
    1: (-0.6, 1.2, 1, 'N', 0.1465),
    2: (-1.2, 0.6, 1, 'W', 0.1465),
    3: (-1.2, -0.6, 2, 'W', 0.0825),
    4: (-0.6, -1.2, 2, 'S', 0.0825),
    5: (0.6, 1.2, 3, 'N', 0.0825),
    6: (1.2, 0.6, 3, 'E', 0.0825),
    7: (1.2, -0.6, 1, 'E', 0.1465),
    8: (0.6, -1.2, 1, 'S', 0.1465),
}

TOGGLE_QUADRANTS = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}

LOADERS = {
    0: (-1.74, 1.49),   # red station, top-left
    1: (-1.74, -1.49),  # red station, bottom-left
    2: (1.74, 1.49),    # blue station, top-right
    3: (1.74, -1.49),   # blue station, bottom-right
}

GOAL_XY_TOL = 0.13       # horizontal distance from goal center that counts
LOADER_XY_TOL = 0.14
GOAL_ENGAGE_TOL = 0.02   # base may sink this much below the goal rim
GOAL_MAX_HEIGHT = 0.35   # element base may be this high above the rim


class FieldLocation(Node):
    """Classifies every scoring object into goals / loaders / field and
    publishes the resulting GoalArray. Also turns controller buttons into
    intake / placement / toggle-flip service calls."""

    def __init__(self):
        super().__init__('field_location')

        # teleop controller feedback
        self.create_subscription(Joy, '/joy', self.controller_callback, 10)
        # element poses from pose_bridge.py
        self.create_subscription(
            FieldElementArray, '/_object_locations', self.object_location_callback, 10)
        # robot pose
        self.create_subscription(PoseArray, '/otto_pose', self.robot_pose_callback, 10)
        # toggle states from world_services.py
        self.create_subscription(ToggleArray, '/toggles', self.toggle_callback, 10)

        # publishers
        self.goals = self.create_publisher(GoalArray, '/goals', 10)
        self.loaders = self.create_publisher(Int64MultiArray, '/loaders', 10)
        self.field_objects = self.create_publisher(
            FieldElementArray, '/field_objects', 10)

        # service clients
        self.intake_element = self.create_client(IntakeElement, '/robot_intake')
        self.score_element = self.create_client(ScoreElement, '/score_element')
        self.flip_toggle = self.create_client(FlipToggle, '/flip_toggle')

        # controller debounce
        self.prev_buttons = [0] * 4

        self.toggle_states = {0: 0, 1: 0, 2: 0, 3: 0}
        self.elements = FieldElementArray()

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def controller_callback(self, msg: Joy):
        buttons = list(msg.buttons[:4]) if len(msg.buttons) >= 4 else list(msg.buttons)
        if buttons[0] == 1 and self.prev_buttons[0] == 0:      # intake
            self.check_collision()
        elif buttons[1] == 1 and self.prev_buttons[1] == 0:    # place pin
            self.scoring_callback(1)
        elif buttons[2] == 1 and self.prev_buttons[2] == 0:    # place cup
            self.scoring_callback(2)
        elif buttons[3] == 1 and self.prev_buttons[3] == 0:    # flip toggle
            self.flip_nearest_toggle()
        self.prev_buttons = buttons

    def robot_pose_callback(self, msg: PoseArray):
        self.robot_x = msg.poses[-1].position.x
        self.robot_y = msg.poses[-1].position.y
        quat = msg.poses[-1].orientation
        q = np.array([quat.x, quat.y, quat.z, quat.w])
        q = q / (np.linalg.norm(q) or 1.0)
        self.robot_r = float(R.from_quat(q).as_euler('xyz')[2])

    def toggle_callback(self, msg: ToggleArray):
        for t in msg.toggles:
            self.toggle_states[t.toggle_id] = t.state

    def object_location_callback(self, msg: FieldElementArray):
        self.elements = msg

        goal_stacks = {gid: [] for gid in GOALS}
        loader_contents = {lid: [] for lid in LOADERS}
        field_elems = FieldElementArray()

        for el in msg.elements:
            base_z = el.location.z  # model origin == element base
            x, y = el.location.x, el.location.y
            placed = False

            # check goals (lowest id wins to keep assignment deterministic)
            for gid, (gx, gy, _gtype, _gquad, gheight) in GOALS.items():
                if math.hypot(x - gx, y - gy) < GOAL_XY_TOL:
                    if gheight - GOAL_ENGAGE_TOL <= base_z <= gheight + GOAL_MAX_HEIGHT:
                        goal_stacks[gid].append(el)
                        placed = True
                        break

            if placed:
                continue

            # check loaders
            for lid, (lx, ly) in LOADERS.items():
                if (abs(x - lx) < LOADER_XY_TOL and abs(y - ly) < LOADER_XY_TOL
                        and base_z < 0.4):
                    loader_contents[lid].append(el)
                    placed = True
                    break

            if not placed:
                field_elems.elements.append(el)

        # build the GoalArray
        goal_array = GoalArray()
        for gid in sorted(GOALS):
            gx, gy, gtype, gquad, gheight = GOALS[gid]
            stack = sorted(goal_stacks[gid], key=lambda e: e.location.z)
            gs = GoalState()
            gs.goal_id = gid
            gs.goal_type = gtype
            gs.quadrant = gquad
            gs.toggle_state = self.toggle_states.get(
                self.quadrant_to_toggle(gquad), 0)
            for el in stack:
                if el.element_type == 1:
                    pin = PinElement()
                    pin.object_name = el.object_name
                    pin.id = el.id
                    pin.top_color = el.top_color
                    pin.bottom_color = el.bottom_color
                    pin.location = el.location
                    pin.orientation = el.orientation
                    gs.pins.append(pin)
                else:
                    cup = CupElement()
                    cup.object_name = el.object_name
                    cup.id = el.id
                    cup.location = el.location
                    cup.orientation = el.orientation
                    cup.yaw = self.quat_to_yaw(el.orientation)
                    gs.cups.append(cup)
            goal_array.goals.append(gs)

        # publish
        self.goals.publish(goal_array)

        loader_msg = Int64MultiArray()
        loader_msg.data = [len(loader_contents[i]) for i in sorted(LOADERS)]
        self.loaders.publish(loader_msg)

        self.field_objects.publish(field_elems)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def quat_to_yaw(orientation):
        q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        q = q / (np.linalg.norm(q) or 1.0)
        return float(R.from_quat(q).as_euler('xyz')[2])

    @staticmethod
    def quadrant_to_toggle(quadrant):
        for tid, q in TOGGLE_QUADRANTS.items():
            if q == quadrant:
                return tid
        return -1

    def check_collision(self):
        """Is there a scoring element in the robot's intake zone?"""
        h, k, th = self.robot_x, self.robot_y, self.robot_r
        offset = 0.15
        ref_x = h + offset * math.cos(th)
        ref_y = k + offset * math.sin(th)

        for el in self.elements.elements:
            dx, dy = el.location.x - ref_x, el.location.y - ref_y
            dx_r = math.cos(th) * dx + math.sin(th) * dy
            dy_r = -math.sin(th) * dx + math.cos(th) * dy
            if abs(dx_r) < 0.10 and abs(dy_r) < 0.10 and el.location.z < 0.12:
                req = IntakeElement.Request()
                req.entity_id = el.id
                req.entity_type = el.element_type
                req.top_color = el.top_color
                req.bottom_color = el.bottom_color
                self.intake_element.call_async(req)
                self.get_logger().info(
                    f"intaking {el.object_name} (type {el.element_type})")
                return
        self.get_logger().info("no element in intake zone")

    def check_location(self):
        """Nearest goal id to the robot (0 if too far)."""
        best, best_d = 0, 0.7
        for gid, (gx, gy, *_rest) in GOALS.items():
            d = math.hypot(self.robot_x - gx, self.robot_y - gy)
            if d < best_d:
                best, best_d = gid, d
        return best

    def scoring_callback(self, element_type):
        goal_id = self.check_location()
        self.get_logger().info(f'placing element type {element_type} at goal {goal_id}')
        req = ScoreElement.Request()
        req.element_type = element_type
        req.top_color = 0
        req.bottom_color = 0
        req.goal_id = goal_id
        self.score_element.call_async(req)

    def flip_nearest_toggle(self):
        best, best_d = -1, 1.0
        for tid, quadrant in TOGGLE_QUADRANTS.items():
            tx, ty = self.toggle_position(tid)
            d = math.hypot(self.robot_x - tx, self.robot_y - ty)
            if d < best_d:
                best, best_d = tid, d
        if best == -1:
            self.get_logger().info("no toggle within range")
            return
        req = FlipToggle.Request()
        req.toggle_id = best
        req.state = -1
        self.flip_toggle.call_async(req)
        self.get_logger().info(f"flipping toggle {best}")

    @staticmethod
    def toggle_position(toggle_id):
        quadrant = TOGGLE_QUADRANTS[toggle_id]
        if quadrant == 'N':
            return (0.0, 1.78)
        if quadrant == 'E':
            return (1.78, 0.0)
        if quadrant == 'S':
            return (0.0, -1.78)
        return (-1.78, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = FieldLocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
