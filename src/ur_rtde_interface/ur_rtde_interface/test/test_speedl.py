#!/usr/bin/env python3

import argparse
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import itertools


class SpeedPublisher(Node):
    def __init__(self, namespace: str = "/robot"):
        super().__init__("speed_publisher")
        base = namespace.rstrip("/")
        topic = f"{base}/cmd_speed_l"
        self.pub = self.create_publisher(Float32MultiArray, topic, 10)

        # Define the sequence of speeds to alternate
        self.speeds = itertools.cycle([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # stop
            [0.05, 0.0, 0.0, 0.0, 0.0, 0.0],  # small velocity
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # stop
            [-0.05, 0.0, 0.0, 0.0, 0.0, 0.0], # small velocity opposite
        ])

        self.timer = self.create_timer(2.0, self.publish_speed)  # 1 Hz

    def publish_speed(self):
        msg = Float32MultiArray()
        msg.data = next(self.speeds)
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--ns', '--namespace', dest='ns', default='/robot', help='Base namespace or topic prefix (e.g. /robot or robot)')
    parsed, remaining = parser.parse_known_args(argv)

    namespace = parsed.ns
    rclpy.init(args=remaining)
    node = SpeedPublisher(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
