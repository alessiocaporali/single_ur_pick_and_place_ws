#!/usr/bin/env python3

import argparse
import sys
import rclpy
from rclpy.node import Node
from ur_rtde_srv.srv import MoveToTarget
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import copy


class Tester(Node):
    def __init__(self, namespace: str = "/robot"):
        super().__init__("tester")
        base = namespace.rstrip("/")

        srv_name = f"{base}/move_to_target"
        srv_async_name = f"{base}/move_to_target_async"
        is_robot_moving_topic = f"{base}/is_robot_async_moving"

        self.cli = self.create_client(MoveToTarget, srv_name)
        self.cli_async = self.create_client(MoveToTarget, srv_async_name)
        self.is_moving_sub = self.create_subscription(Bool, is_robot_moving_topic, self.is_moving_callback, 10)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {srv_name}...")
        while not self.cli_async.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {srv_async_name}...")

        self.is_moving = False

    def is_moving_callback(self, msg: Bool):
        self.is_moving = msg.data
        self.get_logger().info(f"Is robot moving: {self.is_moving}")

    def call(self, target_msg: Pose):
        # Synchronous call
        req = MoveToTarget.Request()
        req.target_pose = target_msg
        req.frame_name = "tcp"
        req.speed = 0.01
        req.acceleration = 0.01
        self.get_logger().info("Calling MoveToTarget service...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Service response: {future.result()}")

        # Asynchronous move
        target_msg2 = copy.deepcopy(target_msg)
        target_msg2.position.z += 0.05

        req_async = MoveToTarget.Request()
        req_async.target_pose = target_msg2
        req_async.frame_name = "tcp"
        req_async.speed = 0.01
        req_async.acceleration = 0.01
        self.get_logger().info("Calling MoveToTargetAsync service...")
        future_async = self.cli_async.call_async(req_async)
        rclpy.spin_until_future_complete(self, future_async)
        self.get_logger().info(f"Service response async: {future_async.result()}")

        # Wait until robot finishes moving
        while self.is_moving:
            self.get_logger().info("Waiting for robot to finish moving...")
            rclpy.spin_once(self, timeout_sec=0.1)


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--ns', '--namespace', dest='ns', default='/robot', help='Base namespace or topic prefix (e.g. /robot or robot)')
    parsed, remaining = parser.parse_known_args(argv)

    namespace = parsed.ns
    rclpy.init(args=remaining)
    tester = Tester(namespace=namespace)

    target_msg = Pose()
    target_msg.position.x = -0.2
    target_msg.position.y = -0.4
    target_msg.position.z = 0.4
    target_msg.orientation.x = 0.0
    target_msg.orientation.y = 1.0
    target_msg.orientation.z = 0.0
    target_msg.orientation.w = 0.0

    tester.call(target_msg)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
