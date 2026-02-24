#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import ParallelGripperCommand


class Tester(Node):
    def __init__(self):
        super().__init__("tester")

        action_name = "/gripper_action_controller/gripper_cmd"
        self._action_client = ActionClient(self, ParallelGripperCommand, action_name)

        self.get_logger().info(f"Waiting for action server {action_name}...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"Action server {action_name} not available")

        self.get_logger().info("Action server is available.")

    def send_goal(self, position: float = 0.03):
        """
        position: target position value (same value replicated across n_joints)
        """
        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command.position = [float(position)] 

        self.get_logger().info(f"Sending ParallelGripperCommand goal: position={goal_msg.command.position}")

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error("No goal handle returned (send_goal_async failed).")
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error("No result returned.")
            return

        self.get_logger().info(f"Result status: {result.status}")
        self.get_logger().info(f"Result: {result.result}")

    def _feedback_cb(self, feedback_msg):
        # feedback_msg.feedback is a ParallelGripperCommand.Feedback
        fb = feedback_msg.feedback
        # Depending on implementation, fields may include position/effort/stalled/reached_goal, etc.
        self.get_logger().debug(f"Feedback: {fb}")


def main(args=None):
    rclpy.init(args=args)
    tester = Tester()
    input("Press Enter to send goal...")
    tester.send_goal(position=0.025)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()