#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ur_rtde_srv.srv import MoveGripper


class Tester(Node):
    def __init__(self):
        super().__init__("tester")

        srv_name = "/robot/move_gripper"
        self.cli = self.create_client(MoveGripper, srv_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {srv_name}...")

    def call(self, position: float = 0.03):
        req = MoveGripper.Request()
        req.position = position

        self.get_logger().info(f"Calling MoveGripper service with position {position}...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Service response: {future.result()}")


def main(args=None):
    rclpy.init(args=args)
    tester = Tester()
    tester.call(0.03)  # Open gripper
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
