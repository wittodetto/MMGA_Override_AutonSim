#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

from geometry_msgs.msg import PoseArray, PoseStamped

from override_sim.msg import GoalArray

# color codes: 1 = red, 2 = blue, 3 = yellow

PIN_TERMINAL_POINTS = 5
YELLOW_PIN_POINTS = 10
ROBOT_IN_MIDFIELD_POINTS = 8
AUTON_BONUS = 12
AUTON_BONUS_TIE = 6

MIDFIELD_HALF = 0.6       # midfield diamond |x|+|y| <= 0.6 m
ROBOT_RADIUS = 0.15       # any part of the robot counts -> inflate by this


class Scoring(Node):
    """VEX V5RC Override scoring.

    - Each placed pin has two terminals (top / bottom color halves).
    - A terminal is worth 5 pts if red/blue, 10 pts if yellow.
    - A terminal is hidden if the cup stacked on that pin has its opaque half
      facing it (simplified: cup yaw in [0, pi) hides the bottom terminal,
      [pi, 2pi) hides the top terminal).
    - Yellow pins in a quadrant are owned by the alliance whose color matches
      the quadrant Toggle; yellow pins on the center (midfield) goal are owned
      by the alliance with more robots in the Midfield (tie = no one).
    - A robot with any part inside the Midfield scores 8 pts.
    - Autonomous bonus: +12 to the alliance with more autonomous points
      (pins only), +6 each on a tie.
    """

    def __init__(self):
        super().__init__('scoring')

        self.create_subscription(GoalArray, '/goals', self.score_calculator, 10)
        self.create_subscription(PoseArray, '/otto_pose', self.robot_pose_cb, 10)
        self.create_subscription(
            PoseStamped, '/opponent/pose', self.opponent_pose_cb, 10)
        self.create_subscription(
            Int64MultiArray, '/game_phase', self.phase_cb, 10)

        self.score = self.create_publisher(Int64MultiArray, '/game_score', 10)
        self.score_detail = self.create_publisher(
            Int64MultiArray, '/score_detail', 10)

        # match phase state
        self.auton_active = False
        self.auton_started = False
        self.auton_bonus = (0, 0)
        self.auton_red = 0
        self.auton_blue = 0

        self.robot_x, self.robot_y = 0.0, 0.0
        self.opp_x, self.opp_y = 0.0, 0.0

    # ------------------------------------------------------------------
    def phase_cb(self, msg):
        phase = int(msg.data)
        if phase == 0:
            if not self.auton_started:
                self.auton_started = True
                self.auton_active = True
                self.auton_red = 0
                self.auton_blue = 0
                self.auton_bonus = (0, 0)
                self.get_logger().info("autonomous period started")
        elif phase == 1 and self.auton_active:
            self.auton_active = False
            if self.auton_red > self.auton_blue:
                self.auton_bonus = (AUTON_BONUS, 0)
            elif self.auton_blue > self.auton_red:
                self.auton_bonus = (0, AUTON_BONUS)
            else:
                self.auton_bonus = (AUTON_BONUS_TIE, AUTON_BONUS_TIE)
            self.get_logger().info(
                f"autonomous bonus: red {self.auton_bonus[0]}, "
                f"blue {self.auton_bonus[1]}")

    def robot_pose_cb(self, msg: PoseArray):
        self.robot_x = msg.poses[-1].position.x
        self.robot_y = msg.poses[-1].position.y

    def opponent_pose_cb(self, msg: PoseStamped):
        self.opp_x = msg.pose.position.x
        self.opp_y = msg.pose.position.y

    # ------------------------------------------------------------------
    @staticmethod
    def in_midfield(x, y):
        return abs(x) + abs(y) <= MIDFIELD_HALF + ROBOT_RADIUS

    def midfield_counts(self):
        red = 1 if self.in_midfield(self.robot_x, self.robot_y) else 0
        blue = 1 if self.in_midfield(self.opp_x, self.opp_y) else 0
        return red, blue

    def yellow_owner(self, gs, red_mid, blue_mid):
        """Return 1 (red), 2 (blue) or 0 (no one) for a yellow terminal."""
        if gs.quadrant == 'C':        # midfield goal
            if red_mid > blue_mid:
                return 1
            if blue_mid > red_mid:
                return 2
            return 0
        return gs.toggle_state

    def score_calculator(self, goals: GoalArray):
        red = 0
        blue = 0
        red_pins = 0
        blue_pins = 0
        red_yellow = 0
        blue_yellow = 0

        red_mid, blue_mid = self.midfield_counts()

        for gs in goals.goals:
            # combine pins and cups into one stack, sorted bottom-up
            stack = []
            for pin in gs.pins:
                stack.append((pin.location.z, 'pin', pin))
            for cup in gs.cups:
                stack.append((cup.location.z, 'cup', cup))
            stack.sort(key=lambda item: item[0])

            for base_z, kind, obj in stack:
                if kind != 'pin':
                    continue
                covered_half = self.cup_covering_half(obj, stack)

                terminals = []
                if covered_half != 'top':
                    terminals.append(obj.top_color)
                if covered_half != 'bottom':
                    terminals.append(obj.bottom_color)

                for color in terminals:
                    if color == 1:
                        red += PIN_TERMINAL_POINTS
                        red_pins += PIN_TERMINAL_POINTS
                    elif color == 2:
                        blue += PIN_TERMINAL_POINTS
                        blue_pins += PIN_TERMINAL_POINTS
                    elif color == 3:
                        owner = self.yellow_owner(gs, red_mid, blue_mid)
                        if owner == 1:
                            red += YELLOW_PIN_POINTS
                            red_yellow += YELLOW_PIN_POINTS
                        elif owner == 2:
                            blue += YELLOW_PIN_POINTS
                            blue_yellow += YELLOW_PIN_POINTS

        # robots in the midfield
        red += red_mid * ROBOT_IN_MIDFIELD_POINTS
        blue += blue_mid * ROBOT_IN_MIDFIELD_POINTS

        # autonomous pin-score snapshot (midfield points excluded per SC7)
        if self.auton_active:
            self.auton_red = red_pins + red_yellow
            self.auton_blue = blue_pins + blue_yellow

        red += self.auton_bonus[0]
        blue += self.auton_bonus[1]

        score_array = Int64MultiArray()
        score_array.data = [red, blue]
        self.score.publish(score_array)

        detail = Int64MultiArray()
        detail.data = [red_pins, blue_pins, red_yellow, blue_yellow,
                       red_mid * ROBOT_IN_MIDFIELD_POINTS,
                       blue_mid * ROBOT_IN_MIDFIELD_POINTS,
                       self.auton_bonus[0], self.auton_bonus[1]]
        self.score_detail.publish(detail)

    def cup_covering_half(self, pin, stack):
        """Return 'top'/'bottom'/'none': which pin terminal the nearest cup
        above this pin hides. Cup yaw in [0, pi) hides the bottom terminal;
        [pi, 2pi) hides the top terminal (simplified SC3 model)."""
        best_d = None
        covering_cup = None
        for cup_z, kind, obj in stack:
            if kind != 'cup':
                continue
            dz = cup_z - pin.location.z
            if 0.10 <= dz <= 0.28:
                if best_d is None or dz < best_d:
                    best_d = dz
                    covering_cup = obj
        if covering_cup is None:
            return None
        return 'bottom' if covering_cup.yaw < math.pi else 'top'


def main(args=None):
    rclpy.init(args=args)
    node = Scoring()
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
