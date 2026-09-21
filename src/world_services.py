#!/usr/bin/env python3

import math
import os
import random
from collections import deque

import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int64MultiArray

from override_sim.msg import ToggleArray, ToggleState
from override_sim.srv import FlipToggle, IntakeElement, LoadElement, ScoreElement

from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from ros_gz_interfaces.msg import Entity, EntityFactory

from ament_index_python.packages import get_package_share_directory

# color codes: 1 = red, 2 = blue, 3 = yellow
PIN_MODELS = {
    (1, 3): 'pin-ry',
    (2, 3): 'pin-by',
    (3, 3): 'pin-yy',
    (1, 2): 'pin-rb',
}

# official element counts
ELEMENTS_LEFT_INIT = [20, 20, 19, 4, 56]   # ry, by, yy, rb, cups

GOAL_HEIGHTS = {
    0: 0.2227, 1: 0.1465, 2: 0.1465, 3: 0.0825, 4: 0.0825,
    5: 0.0825, 6: 0.0825, 7: 0.1465, 8: 0.1465,
}
GOAL_POSITIONS = {
    0: (0.0, 0.0), 1: (-0.6, 1.2), 2: (-1.2, 0.6), 3: (-1.2, -0.6),
    4: (-0.6, -1.2), 5: (0.6, 1.2), 6: (1.2, 0.6), 7: (1.2, -0.6),
    8: (0.6, -1.2),
}
LOADER_POSITIONS = {
    0: (-1.74, 1.49), 1: (-1.74, -1.49), 2: (1.74, 1.49), 3: (1.74, -1.49),
}
# toggle world poses: position + wall yaw; state roll = state * 120 deg
TOGGLE_POSES = {
    0: (0.0, 1.78, 0.0),
    1: (1.78, 0.0, math.pi / 2),
    2: (0.0, -1.78, 0.0),
    3: (-1.78, 0.0, math.pi / 2),
}


class WorldServices(Node):
    """Owns world-side game actions: robot intake, element placement on goals,
    match loading onto loaders, and Toggle flipping. Also publishes toggle
    state and the match phase."""

    def __init__(self):
        super().__init__('world_services')

        self.declare_parameter('world_name', 'override')
        self.declare_parameter('intake_capacity', 10)
        self.declare_parameter('autonomous', False)
        self.world_name = self.get_parameter('world_name').value
        self.intake_capacity = self.get_parameter('intake_capacity').value
        self.autonomous = self.get_parameter('autonomous').value

        self.pkg_path = get_package_share_directory('override_sim')

        # robot pose (for dropping misplaced elements)
        self.create_subscription(PoseArray, '/otto_pose', self.robot_pose_callback, 10)

        # publishers
        self.robot_elements = self.create_publisher(
            Int64MultiArray, '/robot_elements', 10)
        self.elements_remaining = self.create_publisher(
            Int64MultiArray, '/elements_remaining', 10)
        self.toggles_pub = self.create_publisher(ToggleArray, '/toggles', 10)
        self.phase_pub = self.create_publisher(Int64MultiArray, '/game_phase', 10)

        # services
        self.srv_intake = self.create_service(
            IntakeElement, '/robot_intake', self.intake_element)
        self.srv_score = self.create_service(
            ScoreElement, '/score_element', self.score_element)
        self.srv_loader = self.create_service(
            LoadElement, '/loader', self.load_element)
        self.srv_toggle = self.create_service(
            FlipToggle, '/flip_toggle', self.flip_toggle)

        # gazebo service clients
        self.remove_entity = self.create_client(
            DeleteEntity, f'/world/{self.world_name}/remove')
        self.spawn_entity = self.create_client(
            SpawnEntity, f'/world/{self.world_name}/create')
        self.set_pose = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')

        # state
        self.robot_intake = deque(maxlen=self.intake_capacity)
        self.goal_stack_count = {gid: 0 for gid in GOAL_HEIGHTS}
        self.toggle_states = {tid: 0 for tid in TOGGLE_POSES}
        self.elements_left = list(ELEMENTS_LEFT_INIT)
        self.name_counters = {}

        self.publish_elements_remaining()
        self.publish_toggles()

        # match phase management
        self.phase = 1
        if self.autonomous:
            self.phase = 0
            self.create_timer(15.0, self.end_autonomous)
        self.create_timer(1.0, self.publish_phase)

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def robot_pose_callback(self, msg: PoseArray):
        self.robot_x = msg.poses[-1].position.x
        self.robot_y = msg.poses[-1].position.y

    def end_autonomous(self):
        if self.phase == 0:
            self.get_logger().info("autonomous period ended")
            self.phase = 1

    def publish_phase(self):
        msg = Int64MultiArray()
        msg.data = [self.phase]
        self.phase_pub.publish(msg)

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def intake_element(self, request: IntakeElement.Request, response):
        if len(self.robot_intake) >= self.intake_capacity:
            self.get_logger().info("robot intake full")
            response.success = False
            return response

        self.delete_entity(request.entity_id)
        self.robot_intake.append(
            (request.entity_type, request.top_color, request.bottom_color))
        self.get_logger().info(
            f"robot intake: {list(self.robot_intake)}")
        self.publish_robot_elements()
        response.success = True
        return response

    def score_element(self, request: ScoreElement.Request, response):
        # pick the element to place: explicit colors win, else first in intake
        if request.top_color != 0 or request.bottom_color != 0:
            el_type = request.element_type
            top, bottom = request.top_color, request.bottom_color
        else:
            if not self.robot_intake:
                self.get_logger().info("nothing to place")
                response.success = False
                return response
            el_type, top, bottom = self.robot_intake.popleft()

        goal_id = request.goal_id
        if goal_id not in GOAL_POSITIONS:
            # not at a goal: drop near the robot
            x = self.robot_x + random.uniform(-0.3, 0.3)
            y = self.robot_y + random.uniform(-0.3, 0.3)
            self.spawn_element(el_type, top, bottom, x, y, 0.3)
            self.get_logger().info("dropped element near robot")
        else:
            gx, gy = GOAL_POSITIONS[goal_id]
            z = GOAL_HEIGHTS[goal_id] + self.goal_stack_count[goal_id] * 0.17 + 0.02
            self.spawn_element(el_type, top, bottom, gx, gy, z)
            self.goal_stack_count[goal_id] += 1
            self.get_logger().info(
                f"placed element type {el_type} on goal {goal_id}")

        self.publish_robot_elements()
        response.success = True
        return response

    def load_element(self, request: LoadElement.Request, response):
        loader_id = request.loader_id
        if loader_id not in LOADER_POSITIONS:
            response.success = False
            return response

        model_name = self.element_model(
            request.element_type, request.top_color, request.bottom_color)
        if model_name is None or not self.take_one_element(
                request.element_type, request.top_color, request.bottom_color):
            self.get_logger().info("no matching elements left")
            response.success = False
            return response

        lx, ly = LOADER_POSITIONS[loader_id]
        self.spawn_element(request.element_type, request.top_color,
                           request.bottom_color, lx, ly, 0.45)
        self.get_logger().info(f"loaded element onto loader {loader_id}")
        self.publish_elements_remaining()
        response.success = True
        return response

    def flip_toggle(self, request: FlipToggle.Request, response):
        toggle_id = request.toggle_id
        if toggle_id not in TOGGLE_POSES:
            response.success = False
            return response

        if request.state < 0:
            new_state = (self.toggle_states[toggle_id] + 1) % 3
        else:
            new_state = request.state % 3

        self.toggle_states[toggle_id] = new_state
        self.rotate_toggle(toggle_id, new_state)
        self.publish_toggles()
        self.get_logger().info(
            f"toggle {toggle_id} -> state {new_state}")
        response.success = True
        response.new_state = new_state
        return response

    # ------------------------------------------------------------------
    # gazebo helpers
    # ------------------------------------------------------------------
    def element_model(self, el_type, top, bottom):
        if el_type == 1:                      # pin
            return PIN_MODELS.get((bottom, top))
        if el_type == 2:                      # cup
            return 'cup'
        return None

    def spawn_element(self, el_type, top, bottom, x, y, z, yaw=0.0):
        model = self.element_model(el_type, top, bottom)
        if model is None:
            self.get_logger().warning("unknown element, not spawning")
            return

        counter = self.name_counters.setdefault(model, 100)
        self.name_counters[model] = counter + 1
        name = f"{model.replace('-', '_')}_{counter}"

        factory = EntityFactory()
        factory.name = name
        factory.allow_renaming = True
        factory.sdf_filename = os.path.join(
            self.pkg_path, 'models', model, 'model.sdf')
        factory.pose.position.x = x
        factory.pose.position.y = y
        factory.pose.position.z = z
        q = R.from_euler('z', yaw).as_quat()
        factory.pose.orientation.x = q[0]
        factory.pose.orientation.y = q[1]
        factory.pose.orientation.z = q[2]
        factory.pose.orientation.w = q[3]

        req = SpawnEntity.Request()
        req.entity_factory = factory
        self.spawn_entity.call_async(req)

    def delete_entity(self, entity_id):
        entity = Entity()
        entity.id = int(entity_id)
        req = DeleteEntity.Request()
        req.entity = entity
        self.remove_entity.call_async(req)

    def rotate_toggle(self, toggle_id, state):
        x, y, base_yaw = TOGGLE_POSES[toggle_id]
        roll = state * (2 * math.pi / 3)
        q = (R.from_euler('z', base_yaw) * R.from_euler('x', roll)).as_quat()
        req = SetEntityPose.Request()
        req.entity.name = f'toggle_{toggle_id + 1}'
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = 0.0
        req.pose.orientation.x = q[0]
        req.pose.orientation.y = q[1]
        req.pose.orientation.z = q[2]
        req.pose.orientation.w = q[3]
        self.set_pose.call_async(req)

    # ------------------------------------------------------------------
    # bookkeeping publishers
    # ------------------------------------------------------------------
    def take_one_element(self, el_type, top, bottom):
        idx = 4 if el_type == 2 else list(PIN_MODELS).index((bottom, top))
        if self.elements_left[idx] <= 0:
            return False
        self.elements_left[idx] -= 1
        return True

    def publish_robot_elements(self):
        msg = Int64MultiArray()
        msg.data = []
        for el_type, top, bottom in self.robot_intake:
            msg.data.extend([el_type, top, bottom])
        self.robot_elements.publish(msg)

    def publish_elements_remaining(self):
        msg = Int64MultiArray()
        msg.data = self.elements_left
        self.elements_remaining.publish(msg)

    def publish_toggles(self):
        msg = ToggleArray()
        quadrant_map = {0: 'N', 1: 'E', 2: 'S', 3: 'W'}
        for tid in sorted(self.toggle_states):
            t = ToggleState()
            t.toggle_id = tid
            t.state = self.toggle_states[tid]
            t.quadrant = quadrant_map[tid]
            msg.toggles.append(t)
        self.toggles_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WorldServices()
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
