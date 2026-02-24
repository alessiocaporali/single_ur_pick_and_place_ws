#!/usr/bin/env python3

import argparse
import sys
import rclpy
from rclpy.node import Node
from ur_rtde_srv.srv import MoveToTrajectory
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class Tester(Node):
    def __init__(self, namespace: str = "/robot"):
        super().__init__("tester")
        base = namespace.rstrip("/")

        srv_name = f"{base}/move_to_trajectory"
        self.cli = self.create_client(MoveToTrajectory, srv_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {srv_name}...")

        is_moving_topic = f"{base}/is_robot_traj_moving"
        self.is_moving_sub = self.create_subscription(Bool, is_moving_topic, self.is_moving_callback, 10)

        self.is_robot_moving = False

    def is_moving_callback(self, msg: Bool):
        self.is_robot_moving = msg.data

    def call(self, target_msgs):
        req = MoveToTrajectory.Request()
        req.list_poses.extend(target_msgs)
        req.frame_name = "tcp"

        self.get_logger().info("Calling MoveToTrajectory service...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Service response: {future.result()}")

        # Wait until robot finishes moving
        while self.is_robot_moving:
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

    target_msg2 = Pose()
    target_msg2.position.x = -0.2
    target_msg2.position.y = -0.3
    target_msg2.position.z = 0.5
    target_msg2.orientation.x = 0.0
    target_msg2.orientation.y = 1.0
    target_msg2.orientation.z = 0.0
    target_msg2.orientation.w = 0.0

    tester.call([target_msg, target_msg2])

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
