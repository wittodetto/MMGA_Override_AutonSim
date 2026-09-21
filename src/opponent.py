#!/usr/bin/env python3

import random

import rclpy
import numpy as np
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.srv import SetEntityPose

# blue-side waypoints (meters); the opponent is the blue alliance robot
MOVESET = [
    (0.50, 1.50), (1.00, 1.10), (1.55, 0.50), (1.55, -0.50),
    (1.00, -1.10), (0.50, -1.50), (1.30, 0.00), (0.90, 1.55),
    (0.90, -1.55),
]

MOVE_GRAPH = {
    0: [1, 7],
    1: [0, 2, 7],
    2: [1, 3, 6],
    3: [2, 4],
    4: [3, 5, 8],
    5: [4, 8],
    6: [2],
    7: [0, 1],
    8: [4, 5],
}


class Opponent(Node):
    """Simple teleporting opponent robot that wanders the blue side."""

    def __init__(self):
        super().__init__('opponent')

        self.declare_parameter('world_name', 'override')
        self.world_name = self.get_parameter('world_name').value

        self.create_subscription(
            PoseStamped, '/opponent/pose', self.get_opponent_pose, 10)
        self.create_timer(2.0, self.dumb_behavior)

        self.move_pose = self.create_client(
            SetEntityPose, f'/world/{self.world_name}/set_pose')

        self.opp_x = None
        self.opp_y = None
        self.current_pose_key = 0
        self.height = 0.2

    def get_opponent_pose(self, msg: PoseStamped):
        self.opp_x = msg.pose.position.x
        self.opp_y = msg.pose.position.y

    def dumb_behavior(self):
        """wander the blue side pose graph"""
        if np.random.binomial(n=1, p=0.7) == 0:
            return
        potential = MOVE_GRAPH[self.current_pose_key]
        new_index = random.choice(potential)
        self.teleport_to_pose(MOVESET[new_index])
        self.current_pose_key = new_index

    def teleport_to_pose(self, pose):
        req = SetEntityPose.Request()
        req.entity.name = 'opponent'
        req.pose.position.x = pose[0]
        req.pose.position.y = pose[1]
        req.pose.position.z = self.height
        req.pose.orientation.w = 1.0
        self.move_pose.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = Opponent()
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
