import os
import numpy as np
from typing import Iterable, Dict

import agx
import agxSDK
import agxOSG
import agxPython
import agxModel
import osg

import agxROS2

from pipy.tf import Rotation, Vector, Frame
from termcolor import cprint

class Joint():
    def __init__(self,
                 name: str,
                 hinge: agx.Hinge):
        self.name = name
        self.hinge = hinge
        self.parent = hinge.getBodyAt(1)
        self.child = hinge.getBodyAt(0)
        self.motor = hinge.getMotor1D()
        self.motor.setForceRange(agx.RangeReal(-np.inf, np.inf))
        self.lock = hinge.getLock1D()

    def get_angle(self) -> float:
        return self.hinge.getAngle()

    def get_speed(self) -> float:
        return self.hinge.getCurrentSpeed()


class PrismaticJoint:
    def __init__(self, name: str, prismatic: agx.Prismatic):
        self.name = name
        self.prismatic = prismatic
        self.parent = prismatic.getBodyAt(1)
        self.child  = prismatic.getBodyAt(0)

        self.motor = prismatic.getMotor1D()
        self.motor.setForceRange(agx.RangeReal(-np.inf, np.inf))

        self.lock = prismatic.getLock1D()

    def get_pos(self) -> float:
        # displacement along axis (meters)
        return self.prismatic.getAngle()

    def get_speed(self) -> float:
        return self.prismatic.getCurrentSpeed()


class UrRobot():
    def __init__(self,
                 sim: agxSDK.Simulation,
                 initial_position: Iterable[float] = None,
                 initial_orientation: Iterable[float] = None,
                 disable_self_collision: bool = True,
                 create_visual=True,
                 prefix: str = "ur",
                 package_path: str = "ur_description",
                 urdf_file: str = "ur_no_gripper.urdf"):
        super().__init__()

        # URDF 
        assembly_ref = self.load_from_urdf(package_path, urdf_file)
        assembly = assembly_ref.get()
        self.m_visual_node = None

        #################

        self.link_names = [
            f"{prefix}_base_link_inertia", 
            f"{prefix}_shoulder_link",
            f"{prefix}_upper_arm_link",
            f"{prefix}_forearm_link",
            f"{prefix}_wrist_1_link",
            f"{prefix}_wrist_2_link",
            f"{prefix}_wrist_3_link"]

        self.base = assembly.getRigidBody(self.link_names[0])
        self.base.setMotionControl(agx.RigidBody.STATIC)    
        self.ee = assembly.getRigidBody(self.link_names[-1])

        #################

        if disable_self_collision:
            sim.getSpace().setEnablePair(assembly.getName(), assembly.getName(), False)

        if create_visual:
            self.m_visual_node = agxOSG.createVisual(assembly, agxPython.getContext().environment.getSceneRoot())

        if initial_position is not None:
            assembly.setPosition(*initial_position)
        if initial_orientation is not None:
            assembly.setRotation(agx.EulerAngles(*initial_orientation))

        sim.add(assembly)
        self.assembly = assembly

        # setup constraints
        all_c = list(assembly.getConstraints())

        # set the damping time just in case the time step has changed
        [c.setDamping(2 * sim.getTimeStep()) for c in all_c]

        self.joints = [Joint(c.getName(), c.asHinge()) for c in all_c if c.asHinge()]
        self.prismatics = [PrismaticJoint(c.getName(), c.asPrismatic()) for c in all_c if c.asPrismatic()]
        self.locks = [c.asLockJoint() for c in all_c if c.asLockJoint()]

        ##########################
        self.links = []
        for name in self.link_names:
            link = assembly.getRigidBody(name)
            if link is not None:
                self.links.append(link)
            else:
                raise ValueError(f"Link '{name}' not found in the UR robot assembly.")

        cprint(f"Links found: {len(self.links)}", "yellow")
        for l in self.link_names:
            print(l)

        cprint(f"Joints found: {len(self.joints)}", "yellow")
        for j in self.joints:
            print(j.name)

        cprint(f"Prismatics found: {len(self.prismatics)}", "yellow")
        for j in self.prismatics:
            print(j.name)

        print("\n")
        #############################

        self.chain = agxModel.SerialKinematicChain(sim, self.base, self.ee)

        #############################


    def set_initial_pose(self, q: Iterable[float]) -> None:
        _, transforms = self.chain.computeForwardKinematicsAll(q)
        print(len(transforms), len(self.links))
        for transform, link in zip(transforms, self.links):
            link.getFrame().setLocalMatrix(transform)


    @property
    def node(self):
        return self.m_visual_node
    

    def set_prismatic_positions(self, x: Iterable[float]) -> None:
        for pj, xi in zip(self.prismatics, x):
            pj.lock.setEnable(True)      # ensure lock active
            pj.lock.setPosition(xi)      # meters

    def get_prismatic_positions(self) -> np.ndarray:
        return np.array([j.get_pos() for j in self.prismatics])

    def get_prismatic_velocities(self) -> np.ndarray:
        return np.array([j.get_speed() for j in self.prismatics])

    def enable_motors(self, enable: bool) -> None:
        [j.motor.setEnable(enable) for j in self.joints]

    def enable_locks(self, enable: bool) -> None:
        [j.lock.setEnable(enable) for j in self.joints]

    def get_joint_names(self) -> list:
        return [j.name for j in self.joints]

    def get_joint_positions(self) -> np.ndarray:
        return np.array([j.get_angle() for j in self.joints])

    def set_joint_positions(self, q: Iterable[float]) -> None:
        for j, qi in zip(self.joints, q):
            j.lock.setPosition(qi)

    def get_joint_velocities(self) -> np.ndarray:
        return np.array([j.get_speed() for j in self.joints])

    def set_joint_velocities(self, qd: Iterable[float]) -> None:
        for i, qdi in enumerate(qd):
            self.joints[i].motor.setSpeed(qdi)

    def set_joint_torques(self, tau: Iterable[float]) -> None:
        for i, t in enumerate(tau):
            self.joints[i].motor.setForceRange(t, t)

    def get_tt_pose(self) -> agx.AffineMatrix4x4:
        if self.tooltip is None:
            raise ValueError("Cannot call 'get_tt_pose' without a tooltip.")
        return self.tooltip.getFrame().getMatrix()

    @staticmethod
    def load_from_urdf(package_path: str = "ur_description", urdf_file: str = "ur_no_gripper.urdf") -> agxSDK.AssemblyRef:

        urdf_file = os.path.join(package_path, urdf_file)
        urdf_settings = agxModel.UrdfReaderSettings(fixToWorld_=True,
                                                    disableLinkedBodies_=False,
                                                    mergeKinematicLinks_=False)
        robot_ref = agxModel.UrdfReader.read(urdf_file, package_path, None, urdf_settings)

        return robot_ref

class UrRobotRos2Mapping(agxSDK.StepEventListener):
    """
    ROS2 joint_states → AGX robot driver using PD POSITION CONTROL.
    """

    ROS_TOPIC = "joint_states"
    NUM_JOINTS = 6

    def __init__(self, robot, kp=6.0, kd=0.1, prefix=None):
        super().__init__()
        
        # attention: topic name should not start with '/' or it won't work
        if "left" in prefix:
            topic = "sim_left/joint_states"
        elif "right" in prefix:
            topic = "sim_right/joint_states"
        else:
            topic = "joint_states"
        print(f"[Ros2] Subscribing to topic: {topic}")

        # ROS subscriber
        self.sub_joint_states = agxROS2.SubscriberSensorMsgsJointState(topic)
        self.msg_joint_state = agxROS2.SensorMsgsJointState()

        self.initialized = False  
        self.debug = True     # enable debug prints
        self.robot = robot
        self.kp = kp          # proportional gain
        self.kd = kd          # damping gain

        if prefix is None:
            prefix = "ur"
        else:
            prefix = prefix.rstrip("_")

        # Order coming from ROS
        self.ros_joint_order = [
            "gripper_robotiq_hande_left_finger_joint",
            f"{prefix}_elbow_joint",
            f"{prefix}_shoulder_lift_joint",
            f"{prefix}_shoulder_pan_joint",
            f"{prefix}_wrist_1_joint",
            f"{prefix}_wrist_2_joint",
            f"{prefix}_wrist_3_joint"
        ]

        # Correct AGX joint order
        self.agx_joint_order = [
            f"{prefix}_shoulder_pan_joint",
            f"{prefix}_shoulder_lift_joint",
            f"{prefix}_elbow_joint",
            f"{prefix}_wrist_1_joint",
            f"{prefix}_wrist_2_joint",
            f"{prefix}_wrist_3_joint",
        ]

        self.agx_gripper_joint = "gripper_robotiq_hande_left_finger_joint"
        self.agx_gripper_joint_idx = self.ros_joint_order.index(self.agx_gripper_joint)

        # Build index map AGX index → ROS index
        self.joint_index_map = {
            i: self.ros_joint_order.index(name)
            for i, name in enumerate(self.agx_joint_order)
        }

        # Cache AGX motors (fast)
        self.agx_motors = []
        for joint_name in self.agx_joint_order:
            joint = self.robot.assembly.getConstraint1DOF(joint_name)
            if not joint:
                raise RuntimeError(f"[Ros2] AGX joint not found: {joint_name}")
            motor = joint.getMotor1D()
            self.agx_motors.append(motor)

        self.agx_gripper_motors = []
        gripper_joint = self.robot.assembly.getConstraint1DOF(self.agx_gripper_joint)
        if gripper_joint:
            m = gripper_joint.getMotor1D()
            m.setEnable(True)
            m.setForceRange(agx.RangeReal(-1e6, 1e6))  # or bigger if needed
            self.agx_gripper_motors.append(m)

            
        # Initialize desired positions (avoid None checks later)
        self.q_desired = [0.0] * self.NUM_JOINTS

        print("[Ros2] Joint mapping:", self.joint_index_map)
        print("[Ros2] Using PD position control (Kp=%.2f, Kd=%.2f)" % (kp, kd))
    
        pj = self.robot.prismatics[0]

        pj.lock.setEnable(False)                    # don’t fight the motor
        pj.motor.setEnable(True)
        pj.motor.setForceRange(agx.RangeReal(-1e6, 1e6))  # strong enough
        pj.motor.setSpeed(0.0)    

        print("pos:", pj.get_pos(), "vel:", pj.get_speed(),
            "motor_en:", pj.motor.getEnable(),
            "lock_en:", pj.lock.getEnable())

    # ----------------------------------------------------------------------

    def pre(self, time):

        # Try receiving updated msg (non-blocking)
        new_msg = self.sub_joint_states.receiveMessage(self.msg_joint_state)
    
        # If no new msg, keep using previous desired positions
        if new_msg:
            # Map ROS → AGX order
            self.q_desired = [
                self.msg_joint_state.position[self.joint_index_map[i]]
                for i in range(self.NUM_JOINTS)
            ]
        else:
            print("[Ros2] No new joint_states message received; using previous desired positions.")
        # -------------------------------------------------------
        # INITIALIZATION — RUN ONLY ONE TIME
        # -------------------------------------------------------
        if not self.initialized:
            # we need at least one valid message before initializing
            if new_msg:
                self.robot.set_initial_pose(self.q_desired)
                self.initialized = True
                print("[Ros2] Initial pose set:", self.q_desired)
            return     # skip control until initialized

        # -------------------------------------------------------
        # NORMAL CONTROL STARTS HERE
        # -------------------------------------------------------
        q_current = self.robot.get_joint_positions()
        qdot_current = self.robot.get_joint_velocities()

        # Compute commanded velocities
        for i in range(self.NUM_JOINTS):

            # PD controller:
            # vel_cmd = kp*(q_desired - q_current) - kd*qdot_current
            error = self.q_desired[i] - q_current[i]
            derror = -qdot_current[i]

            vel_cmd = self.kp * error + self.kd * derror

            # Apply to AGX motor
            self.agx_motors[i].setSpeed(vel_cmd)

        #############################
        # GRIPPER
        q_gripper_current = self.robot.get_prismatic_positions()
        qdot_gripper_current = self.robot.get_prismatic_velocities()
        q_gripper_desired = self.msg_joint_state.position[self.agx_gripper_joint_idx]
        qdot_gripper_desired = self.msg_joint_state.velocity[self.agx_gripper_joint_idx]

        error = q_gripper_desired - q_gripper_current[0]
        derror = qdot_gripper_desired - qdot_gripper_current[0]

        gripper_vel_cmd = self.kp * error + self.kd * derror
        #print("[Ros2] gripper:", q_gripper_desired, q_gripper_current[0], error, gripper_vel_cmd)
        self.robot.prismatics[0].motor.setSpeed(gripper_vel_cmd)

        # Debug print once per second
        if time % 1.0 < 0.01 and self.debug:
            print("[Ros2] q_error:", [(float(round(self.q_desired[i] - q_current[i], 4))) for i in range(self.NUM_JOINTS)])

            pos = self.robot.ee.getFrame().getTranslate()
            print("[Ros2] robot EE position:", pos)

            rot = Rotation.quaternion(*self.robot.ee.getFrame().getRotate())
            print("[Ros2] robot EE rotation:", [float(x) for x in rot.get_quaternion()])
