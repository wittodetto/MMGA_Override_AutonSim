#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker

from override_sim.msg import ActionState
from override_sim.srv import IntakeElement, ScoreElement


class AIDriver(Node):
    """Executes the Strategy AI output: drives to the requested pose via Nav2
    and forwards intake / placement / toggle actions to the world services."""

    def __init__(self):
        super().__init__('ai_driver')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(Marker, '/ai_goal_marker', 10)

        self.create_subscription(
            ActionState, '/sai_output', self.interpret_action, 10)
        self.create_subscription(
            TwistStamped, '/cmd_vel', self.store_current_velocity, 10)

        self.intake_element = self.create_client(
            IntakeElement, '/robot_intake')
        self.score_element = self.create_client(
            ScoreElement, '/score_element')

        self._marker_id_counter = 0

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return [
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            sy * cp * sr + cy * sp * cr,
            sy * cp * cr - cy * sp * sr,
        ]

    def store_current_velocity(self, msg: TwistStamped):
        self.avg_vel = (msg.twist.linear.x + msg.twist.linear.y) / 2

    def publish_goal_marker(self, x, y, robot_color):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self._marker_id_counter
        self._marker_id_counter += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.lifetime.sec = 99999
        marker.lifetime.nanosec = 0
        if robot_color == 'red':
            marker.color.r = 1.0
        else:
            marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

    def drive_to(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        q = self.quaternion_from_euler(0, 0, theta)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.nav_client.send_goal_async(goal_msg)

    def interpret_action(self, msg: ActionState):
        self.publish_goal_marker(msg.red_robot.x, msg.red_robot.y, 'red')
        self.publish_goal_marker(msg.blue_robot.x, msg.blue_robot.y, 'blue')

        if msg.red_robot_action == 0:
            self.drive_to(msg.red_robot.x, msg.red_robot.y, msg.red_robot.theta)
        # action codes 1..4 (intake / place pin / place cup / flip toggle) are
        # handled by the robot's own controller / future manipulator stack.


def main(args=None):
    rclpy.init(args=args)
    node = AIDriver()
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
