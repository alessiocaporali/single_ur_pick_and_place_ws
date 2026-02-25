#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ur_rtde_interface.ur import UR
from ur_rtde_srv.srv import (
    MoveToTarget,
    MoveToJoint,
    MoveToTrajectory,
    SetPinFrame,
    MoveGripper,
)
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray, Bool, Int32

from pipy.tf import Frame, Vector, Rotation
import threading
import numpy as np
from scipy.spatial import transform


# -----------------------------
# Helper Functions
# -----------------------------
def compute_trajectory_slerp(self, list_frames, speed_scale=0.5, convert_to_tcp=True):
    speed_scale = np.clip(speed_scale, 0.0, 1.0)
    range_scale = [0.00002, 0.0002]
    scale = range_scale[0] + (range_scale[1] - range_scale[0]) * speed_scale

    positions = [frame.p.to_numpy() for frame in list_frames]
    rotations = [frame.M.get_quaternion() for frame in list_frames]
    trajectory = []

    for i in range(len(list_frames) - 1):
        start_pos = positions[i]
        end_pos = positions[i + 1]
        distance = np.linalg.norm(end_pos - start_pos)
        num_pos_points = max(int(distance / scale), 2)

        R = (list_frames[i].M * list_frames[i + 1].M.transpose()).to_numpy()
        theta = np.arccos((np.trace(R) - 1) / 2)
        num_rot_points = max(int(theta / (scale * 10)), 2) if theta > 1e-6 else 2

        num_points = max(num_pos_points, num_rot_points)
        interp_positions = np.linspace(start_pos, end_pos, num=num_points)

        key_times = [0, 1]
        key_rots = transform.Rotation.from_quat(np.array([rotations[i], rotations[i + 1]]))
        slerp = transform.Slerp(key_times, key_rots)
        interp_times = np.linspace(0, 1, num=num_points)
        interp_rots = slerp(interp_times)

        for pos, rot in zip(interp_positions, interp_rots):
            quat = rot.as_quat()
            frame = Frame(Rotation.quaternion(*quat), Vector(*pos))

            frame_tcp = (
                self.robot.convert_pin_frame_to_tcp_frame(frame)
                if convert_to_tcp else frame
            )

            tcp_pos = frame_tcp.p.to_numpy()
            tcp_rot = frame_tcp.M.get_rot()
            trajectory.append([*tcp_pos, *tcp_rot])

    return trajectory


def frame_from_pose(pose):
    pos = Vector(pose.position.x, pose.position.y, pose.position.z)
    quat = Rotation.quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    return Frame(quat, pos)


# -----------------------------
# ROS2 Node Class
# -----------------------------
class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        # Parameters
        self.declare_parameter("robot_ip", "172.17.0.2")
        self.declare_parameter("default_speed", 0.1)
        self.declare_parameter("default_acceleration", 0.1)
        self.declare_parameter("gripper_active", False)
        self.declare_parameter("namespace", "robot")

        robot_ip = self.get_parameter("robot_ip").value
        self.default_speed = self.get_parameter("default_speed").value
        self.default_acceleration = self.get_parameter("default_acceleration").value
        self.gripper_active = self.get_parameter("gripper_active").value
        self.namespace = self.get_parameter("namespace").value

        self.stop_flag = False

        # Robot driver
        self.robot = UR(robot_ip, gripper=self.gripper_active)
        self.gripper_range = [0.0, 0.055]
        if self.gripper_active and self.robot.gripper is not None:
            self.gripper_range = [
                self.robot.gripper.get_min_stroke(),
                self.robot.gripper.get_max_stroke(),
            ]
            self.get_logger().info(
                f"RTDE gripper stroke range: [{self.gripper_range[0]:.4f}, {self.gripper_range[1]:.4f}] m"
            )

        ns = self.namespace

        # -----------------------------
        # Services
        # -----------------------------
        self.create_service(MoveGripper, f"/{ns}/move_gripper", self.move_gripper_callback)
        self.create_service(SetPinFrame, f"/{ns}/set_pin_frame", self.set_pin_frame_callback)
        self.create_service(MoveToJoint, f"/{ns}/move_to_joint", self.movetojoint_callback)
        self.create_service(MoveToTarget, f"/{ns}/move_to_target", self.movetotarget_callback)
        self.create_service(MoveToTarget, f"/{ns}/move_to_target_async", self.movetotarget_async_callback)
        self.create_service(MoveToTrajectory, f"/{ns}/move_to_trajectory", self.movetotrajectory_callback)
        self.create_service(Trigger, f"/{ns}/stop_speed_l", self.stop_speedl_callback)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.moving_traj_pub = self.create_publisher(Bool, f"/{ns}/is_robot_traj_moving", 1)
        self.moving_async_pub = self.create_publisher(Bool, f"/{ns}/is_robot_async_moving", 1)

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Int32, f"/{ns}/traj_control_value", self.trajectory_control_value_callback, 10)
        self.create_subscription(Float32MultiArray, f"/{ns}/cmd_speed_l", self.speed_l_callback, 10)
        self.create_subscription(Bool, f"/{ns}/stop_signal", self.stop_signal_callback, 10)

        self.traj_control_value = 0
        self.max_traj_control_value = 5
        self.trajectory = []
        self.traj_idx = 0
        self.execute_traj = False

        # Threads
        threading.Thread(target=self.robot_state_loop, daemon=True).start()
        threading.Thread(target=self.main_loop, daemon=True).start()

        self.get_logger().info("Robot Controller Node Initialized")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def trajectory_control_value_callback(self, msg):
        self.traj_control_value = int(np.clip(msg.data, -self.max_traj_control_value, self.max_traj_control_value))

    def stop_signal_callback(self, msg):
        self.stop_flag = bool(msg.data)
        if self.stop_flag and (self.execute_traj or self.robot.is_robot_moving()):
            self.robot.stop()
            self.execute_traj = False

    def set_pin_frame_callback(self, req, resp=None):
        self.robot.set_pin_frame(frame_from_pose(req.pin_pose))
        return SetPinFrame.Response(success=True)

    def speed_l_callback(self, msg):
        self.robot.set_speedl(list(msg.data))

    def stop_speedl_callback(self, req, resp=None):
        self.robot.stop_speedl()
        return Trigger.Response(success=True, message="Speedl stopped")

    def move_gripper_callback(self, req, resp=None):
        if not self.gripper_active:
            return MoveGripper.Response(success=False)
        if self.robot.gripper is None:
            self.get_logger().error("gripper_active is true but UR gripper backend is unavailable")
            return MoveGripper.Response(success=False)
        pos = np.clip(req.position, self.gripper_range[0], self.gripper_range[1])
        self.robot.move_gripper(pos)
        return MoveGripper.Response(success=True)

    def movetojoint_callback(self, req, resp=None):
        speed = req.speed or self.default_speed
        acc = req.acceleration or self.default_acceleration
        self.robot.move_to_joint(req.joint_angles, speed=speed, acceleration=acc)
        return MoveToJoint.Response(success=True)

    def movetotarget_callback(self, req, resp=None):
        speed = req.speed or self.default_speed
        acc = req.acceleration or self.default_acceleration
        target = frame_from_pose(req.target_pose)
        if req.frame_name == "tcp":
            self.robot.move_to_tcp_frame(target, asynchronous=False, speed=speed, acceleration=acc)
        else:
            self.robot.move_to_pin_frame(target, asynchronous=False, speed=speed, acceleration=acc)
        return MoveToTarget.Response(success=True)

    def movetotarget_async_callback(self, req, resp=None):
        speed = req.speed or self.default_speed
        acc = req.acceleration or self.default_acceleration
        target = frame_from_pose(req.target_pose)
        if req.frame_name == "tcp":
            self.robot.move_to_tcp_frame(target, asynchronous=True, speed=speed, acceleration=acc)
        else:
            self.robot.move_to_pin_frame(target, asynchronous=True, speed=speed, acceleration=acc)
        return MoveToTarget.Response(success=True)

    def movetotrajectory_callback(self, req, resp=None):
        if len(req.list_poses) == 0:
            return MoveToTrajectory.Response(success=False)
        convert_to_tcp = req.frame_name == "pin"
        curr_frame = self.robot.get_pin_frame() if convert_to_tcp else self.robot.get_tcp_frame()
        list_frames = [frame_from_pose(p) for p in req.list_poses]
        list_frames.insert(0, curr_frame)
        self.trajectory = compute_trajectory_slerp(self, list_frames, req.speed_scale, convert_to_tcp)
        self.traj_idx = 0
        self.execute_traj = True
        self.traj_target_pose = self.trajectory[-1]
        return MoveToTrajectory.Response(success=True)

    # -----------------------------
    # Threads
    # -----------------------------
    def robot_state_loop(self):
        rate = self.create_rate(50)
        while rclpy.ok():
            self.moving_traj_pub.publish(Bool(data=self.execute_traj))
            self.moving_async_pub.publish(Bool(data=self.robot.is_robot_moving()))
            rate.sleep()

    def main_loop(self):
        rate = self.create_rate(500)
        while rclpy.ok():
            if self.stop_flag and not self.execute_traj:
                self.robot.stop()
                self.stop_flag = False
            elif self.stop_flag and self.execute_traj:
                self.execute_traj = False
                self.traj_idx = 0
                self.trajectory = []
                self.robot.servo_stop()
                self.stop_flag = False
            elif self.execute_traj:
                if self.traj_control_value != 0:
                    self.traj_idx += self.traj_control_value
                self.traj_idx = max(0, self.traj_idx)
                if self.traj_idx >= len(self.trajectory):
                    for _ in range(1000):
                        error = np.linalg.norm(np.array(self.traj_target_pose) - np.array(self.robot.get_tcp_pose()))
                        if error < 1e-4:
                            break
                        self.robot.servo_step(self.trajectory[-1])
                        rate.sleep()
                    self.trajectory = []
                    self.robot.servo_stop()
                    self.execute_traj = False
                    continue
                self.robot.servo_step(self.trajectory[self.traj_idx])
            rate.sleep()


# -----------------------------
# Main
# -----------------------------
def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
