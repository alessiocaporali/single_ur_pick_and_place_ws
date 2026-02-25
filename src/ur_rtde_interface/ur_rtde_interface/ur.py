import numpy as np
import sys
import time
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from pipy.tf import Frame, Vector, Rotation

from ur_rtde_interface.robotiq import GripperControl

class UR:

    def __init__(self, robot_ip, home_joints=None, only_receiver=False, gripper=True):
        """
        robot_ip: str - IP address of the robot
        camera_tool0_pose: list - [x, y, z, qx, qy, qz, qw] pose of the camera in the tool0 frame
        home_joints: list - [j1, j2, j3, j4, j5, j6] joint values for the home position
        """

        self.home_joints = home_joints

        self.speed = 0.1
        self.acceleration = 0.5

        self.servo_dt = 1.0 / 500  # 2ms
        self.servo_lookahead_time = 0.1
        self.servo_gain = 300

        self.rtde_r = RTDEReceiveInterface(robot_ip)
        if not only_receiver:
            self.rtde_c = RTDEControlInterface(robot_ip)

        self.pin_tool0_frame = Frame(Rotation(), Vector(0, 0, 0.2278))
        self.pin_tool0_frame_original = Frame(Rotation(), Vector(0, 0, 0.2278))

        
        if gripper:
            self.gripper = GripperControl("Hand_E", robot_ip=robot_ip, port=63352)
            if not self.gripper.initialize():
                print("Failed to initialize gripper")
                sys.exit(1)

            self.move_gripper(position=0.02, speed=100, force=100)
            print("Gripper initialized successfully")
        else:
            self.gripper = None
        
    def homing(self):
        self.rtde_c.moveJ(self.home_joints, 0.1, 0.1, False)

    def get_joint_positions(self):
        return self.rtde_r.getActualQ()

    def get_tcp_pose(self):
        """
        xyz and rotvec of the tcp
        """
        return self.rtde_r.getActualTCPPose()

    def get_tcp_frame(self):
        """
        Frame() object of the tcp
        """
        tcp = self.get_tcp_pose()
        orientation = Rotation.rot(rotvec=tcp[3:], angle=np.linalg.norm(tcp[3:]))
        pos = Vector(tcp[0], tcp[1], tcp[2])
        return Frame(orientation, pos)

    def reset_pin_frame(self):
        """
        Reset the pin frame to the original position.
        This is useful if the pin frame has been modified and needs to be reset.
        """
        self.pin_tool0_frame = self.pin_tool0_frame_original.copy()

    def set_pin_frame(self, pin_frame):
        self.pin_tool0_frame = pin_frame.copy()

    def get_pin_frame(self):
        return self.get_tcp_frame() * self.pin_tool0_frame

    def convert_pin_frame_to_tcp_frame(self, pin_frame):
        if self.pin_tool0_frame is None:
            raise ValueError("Pin frame not set")
        return pin_frame * self.pin_tool0_frame.inverse()

    def move_to_pin_frame(self, pin_frame, asynchronous=False, speed=None, acceleration=None):

        if speed is None:
            speed = self.speed
        if acceleration is None:
            acceleration = self.acceleration
        if self.pin_tool0_frame is None:
            raise ValueError("Pin frame not set")

        tcp_frame = self.convert_pin_frame_to_tcp_frame(pin_frame)
        self.move_to_tcp_frame(tcp_frame, asynchronous=asynchronous, speed=speed, acceleration=acceleration)

    def move_to_tcp_frame(self, frame, asynchronous=False, speed=None, acceleration=None):

        if speed is None:
            speed = self.speed
        if acceleration is None:
            acceleration = self.acceleration

        tcp_position = frame.p.to_numpy()
        tcp_orientation = frame.M.get_rot()
        # print("Moving to: ", tcp_position, tcp_orientation)
        pose = [*tcp_position, *tcp_orientation]
        self.rtde_c.moveL(pose, speed, acceleration, asynchronous=asynchronous)

    def move_to_joint(self, joint_values, speed=None, acceleration=None):

        if speed is None:
            speed = self.speed
        if acceleration is None:
            acceleration = self.acceleration

        self.rtde_c.moveJ(joint_values, speed=speed, acceleration=acceleration)

    def is_robot_moving(self):
        """
        Check if the robot is moving by an asynchronous operation.
        Returns:
            bool: True if the robot is moving, False otherwise.
        """
        x = self.rtde_c.getAsyncOperationProgress()
        if x == 0:
            return True
        else:
            return False

    def move_gripper(self, position, speed=None, force=None):
        if self.gripper is None:
            raise RuntimeError("Gripper backend is not enabled")

        position = float(position)
        if speed is None:
            speed = 50
        if force is None:
            force = 100

        self.gripper.go_to(position, speed, force)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.05)

            act_pos = self.gripper.gripper.getActualPos()
            if act_pos < 0:
                continue
            diff = abs(act_pos - position)
            if diff < 0.001:
                break

    def move_to_tcp_frame_servo(self, tcp_poses):
        for i in range(len(tcp_poses)):
            self.servo_step(tcp_poses[i])
        self.servo_stop()

    def stop(self):
        self.rtde_c.stopL()

    def servo_stop(self):
        self.rtde_c.servoStop(a=0.1)

    def servo_step(self, tcp_pose):
        t_start = self.rtde_c.initPeriod()
        self.rtde_c.servoL(
            tcp_pose,
            0.0,
            0.0,
            self.servo_dt,
            self.servo_lookahead_time,
            self.servo_gain,
        )
        self.rtde_c.waitPeriod(t_start)

    def get_robot_status(self):
        """
        1 if everything is ok
        """
        return self.rtde_r.getSafetyStatusBits()

    def set_speedl(self, values):
        if len(values) != 6:
            raise ValueError("Speedl values must be a list of 6 elements")
        self.rtde_c.speedL(values, 0.5, 0.02)

    def stop_speedl(self):
        self.rtde_c.speedStop(a=1.0)
