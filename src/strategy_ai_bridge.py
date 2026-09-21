#!/usr/bin/env python3

import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node

from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Pose2D, PoseArray, PoseStamped

from override_sim.msg import (
    FieldElementArray, GoalArray, ToggleArray, WorldState,
)


class StrategyAIBridge(Node):
    """Subscribes to every simulation topic and packs one override_sim/WorldState
    message for the Strategy AI node on /sai_input."""

    def __init__(self):
        super().__init__('sai_bridge')

        self.create_subscription(
            Int64MultiArray, '/game_score', self.score_cb, 10)
        self.create_subscription(
            GoalArray, '/goals', self.goals_cb, 10)
        self.create_subscription(
            ToggleArray, '/toggles', self.toggles_cb, 10)
        self.create_subscription(
            PoseArray, '/otto_pose', self.robot_pose_cb, 10)
        self.create_subscription(
            Int64MultiArray, '/robot_elements', self.intake_cb, 10)
        self.create_subscription(
            FieldElementArray, '/field_objects', self.field_elements_cb, 10)
        self.create_subscription(
            Int64MultiArray, '/loaders', self.loader_cb, 10)
        self.create_subscription(
            Int64MultiArray, '/elements_remaining', self.elements_left_cb, 10)
        self.create_subscription(
            PoseStamped, '/opponent/pose', self.opponent_pose_cb, 10)

        self.create_timer(1.0, self.update_sai_world_state)
        self.to_sai = self.create_publisher(WorldState, '/sai_input', 10)

        self.world_state = WorldState()

    @staticmethod
    def pose2d_from_pose_array(msg: PoseArray):
        pose = Pose2D()
        pose.x = float(msg.poses[-1].position.x)
        pose.y = float(msg.poses[-1].position.y)
        q = msg.poses[-1].orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        quat = quat / (np.linalg.norm(quat) or 1.0)
        pose.theta = float(R.from_quat(quat).as_euler('xyz')[2])
        return pose

    def score_cb(self, msg):
        self.world_state.score = msg

    def goals_cb(self, msg):
        self.world_state.goals = msg

    def toggles_cb(self, msg):
        self.world_state.toggles = msg

    def robot_pose_cb(self, msg):
        self.world_state.robot_pose = self.pose2d_from_pose_array(msg)

    def intake_cb(self, msg):
        self.world_state.robot_intake = msg

    def field_elements_cb(self, msg):
        self.world_state.field_elements = msg

    def loader_cb(self, msg):
        self.world_state.loaders = msg

    def elements_left_cb(self, msg):
        self.world_state.elements_left = msg

    def opponent_pose_cb(self, msg: PoseStamped):
        pose = Pose2D()
        pose.x = msg.pose.position.x
        pose.y = msg.pose.position.y
        q = msg.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        quat = quat / (np.linalg.norm(quat) or 1.0)
        pose.theta = float(R.from_quat(quat).as_euler('xyz')[2])
        self.world_state.opponent_pose = pose

    def update_sai_world_state(self):
        self.world_state.header.stamp = self.get_clock().now().to_msg()
        self.world_state.header.frame_id = 'map'
        self.to_sai.publish(self.world_state)


def main(args=None):
    rclpy.init(args=args)
    node = StrategyAIBridge()
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
