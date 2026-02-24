#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class UR5JointPublisher(Node):
    def __init__(self):
        super().__init__('ur5_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.start_time = time.time()

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        t = time.time() - self.start_time

        # Example: simple sinusoidal motion
        msg.position = [
            math.sin(t),
            math.sin(t + 1),
            math.sin(t + 2),
            math.sin(t + 3),
            math.sin(t + 4),
            math.sin(t + 5)
        ]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing JointState: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = UR5JointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
