#!/usr/bin/env python3

import json
import subprocess

import rclpy
from rclpy.node import Node

from override_sim.msg import FieldElement, FieldElementArray


class PoseBridge(Node):
    """Polls Gazebo's dynamic pose topic and republishes the scoring objects
    (pins and cups) as a FieldElementArray on /_object_locations."""

    # color codes: 1 = red, 2 = blue, 3 = yellow
    PIN_COMBOS = {
        'ry': (1, 3),   # bottom red, top yellow
        'by': (2, 3),   # bottom blue, top yellow
        'yy': (3, 3),
        'rb': (1, 2),   # bottom red, top blue
    }

    NON_ELEMENT_NAMES = [
        'link', 'Otto', 'opponent', 'wheel', 'field', 'goal', 'toggle',
        'loader', 'ground',
    ]

    def __init__(self):
        super().__init__('pose_bridge')

        self.declare_parameter('world_name', 'override')
        self.world_name = self.get_parameter('world_name').value

        self.auto_update = self.create_timer(0.5, self.update_locations)
        self.element_locations = self.create_publisher(
            FieldElementArray, '/_object_locations', 10)

    def echo_gz_topic(self):
        """Fetch one JSON sample from the gz dynamic pose topic."""
        try:
            result = subprocess.run(
                ["gz", "topic", "-e", "-t",
                 f"world/{self.world_name}/dynamic_pose/info",
                 "-n", "1", "--json-output"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warning("gz topic command timed out after 5 seconds")
            return {"pose": []}
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Unexpected error running gz topic: {e}")
            return {"pose": []}

        if result.returncode != 0:
            self.get_logger().warning(
                f"gz topic returned code {result.returncode}: {result.stderr.strip()}")
            return {"pose": []}

        try:
            for line in result.stdout.strip().split('\n'):
                if line.strip():
                    try:
                        return json.loads(line)
                    except json.JSONDecodeError:
                        continue
            self.get_logger().warning("No valid JSON found in output")
            return {"pose": []}
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"Unexpected error: {e}, stdout: '{result.stdout}', "
                f"stderr: '{result.stderr}'")
            return {"pose": []}

    @staticmethod
    def parse_pin_colors(name):
        """pin_ry_3 -> (bottom=1, top=3)"""
        for combo, (bottom, top) in PoseBridge.PIN_COMBOS.items():
            if f'pin_{combo}_' in name or name.startswith(f'pin_{combo}'):
                return bottom, top
        return 0, 0

    def update_locations(self):
        objects = FieldElementArray()
        data = self.echo_gz_topic()

        for pose in data.get("pose", []):
            name = pose.get("name", "")
            if any(tok in name for tok in self.NON_ELEMENT_NAMES):
                continue

            position = pose.get("position", {})
            if "x" not in position or "y" not in position or "z" not in position:
                continue

            el = FieldElement()
            el.object_name = name
            el.id = pose.get("id", 0)
            el.location.x = float(position["x"])
            el.location.y = float(position["y"])
            el.location.z = float(position["z"])
            o = pose.get("orientation", {})
            el.orientation.x = float(o.get("x", 0.0))
            el.orientation.y = float(o.get("y", 0.0))
            el.orientation.z = float(o.get("z", 0.0))
            el.orientation.w = float(o.get("w", 1.0))

            if 'cup' in name:
                el.element_type = 2
            elif 'pin' in name:
                el.element_type = 1
                el.bottom_color, el.top_color = self.parse_pin_colors(name)
            else:
                continue

            objects.elements.append(el)

        self.element_locations.publish(objects)


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridge()
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
