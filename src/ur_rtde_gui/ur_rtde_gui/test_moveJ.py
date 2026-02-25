#!/usr/bin/env python3

import argparse
import sys
import rclpy
from rclpy.node import Node
from ur_rtde_srv.srv import MoveToJoint  # Make sure this package is correctly built


class Tester(Node):
    def __init__(self, namespace: str = "/robot"):
        super().__init__("tester")
        base = namespace.rstrip("/")
        srv_name = f"{base}/move_to_joint"
        self.get_logger().info(f"Creating client for service: {srv_name}")
        self.cli = self.create_client(MoveToJoint, srv_name)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {srv_name} not available, waiting...")

    def call_service(self):
        req = MoveToJoint.Request()
        req.joint_angles = [1.8, -1.42, -1.9, -1.35, 1.55, 0.0]
        self.get_logger().info(f"Requesting MoveToJoint with joint angles: {req.joint_angles}")

        self.get_logger().info("Calling MoveToJoint service...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f"Service response: {res.success}")


def main(argv=None):
    # parse namespace argument while leaving ROS / rclpy args untouched
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--ns', '--namespace', dest='ns', default='/robot', help='Base namespace or topic prefix (e.g. /robot or robot)')
    parsed, remaining = parser.parse_known_args(argv)

    namespace = parsed.ns

    # init rclpy with remaining args so ROS-specific CLI options still work
    rclpy.init(args=remaining)
    tester = Tester(namespace=namespace)
    tester.call_service()
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
