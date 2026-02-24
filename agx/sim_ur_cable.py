import numpy as np

import agx
import agxOSG
import agxRender
from agxPythonModules.utils.environment import simulation, init_app, application, root

from cable_interface import Cable, CableRos2Publisher
from robot_interface import UrRobot, UrRobotRos2Mapping
from utils import set_default_camera, pipy_frame_from_agx
from pipy.tf import Frame, Rotation, Vector




# Build robot scene
def buildScene():
    app = application()
    sim = simulation()
    sim.setTimeStep(0.01)

    # camera
    set_default_camera(app)

    ################################################
    # Create the robot, using the Universal Robots UR loaded from urdf.

    package_path = "/home/alessio/dev25/prova_agx/ur/ur_description"
    urdf_file = "ur_no_gripper.urdf"
    prefix = "ur"
    robot = UrRobot(simulation(), initial_position=[0.0, 0.0, 0.0], package_path=package_path, urdf_file=urdf_file, prefix=prefix)
    robot.enable_motors(True)
    agxOSG.createAxes(robot.base, agx.AffineMatrix4x4(), root(), 0.1)
    agxOSG.createAxes(robot.ee, agx.AffineMatrix4x4(), root(), 0.1)

    robot_ros2 = UrRobotRos2Mapping(robot, prefix=prefix)
    sim.add(robot_ros2)

    ####################################################
    # cable
    cable_generator = Cable(radius=0.005, length=1.0, density=0.01)
    cable = cable_generator.initialize_straight_cable(direction='y')
    sim.add(cable)
    cable_generator.set_cable_properties(cable, bend_youngs_modulus=1.0E7, twist_youngs_modulus=1.0E7)
    node = agxOSG.createVisual(cable, root())
    agxOSG.setDiffuseColor(node, agxRender.Color.Blue())

    cable_ros2 = CableRos2Publisher(cable, cable_generator)
    sim.add(cable_ros2)

    frame_ee = pipy_frame_from_agx(robot.ee.getFrame())

    begin_frame = frame_ee * Frame(Rotation(), Vector(0, 0, 0.05))

    end_frame = frame_ee * Frame(Rotation(), Vector(-0.5, 0.0, 0.05))
    end_frame.M.do_rotX(np.pi)
    #end_frame.M.do_rotZ(np.pi)

    cable.begin().getRigidBody().setPosition(begin_frame.p.x(), begin_frame.p.y(), begin_frame.p.z())
    cable.begin().getRigidBody().setRotation(agx.Quat(*begin_frame.M.get_quaternion()))

    cable.findLast().getRigidBody().setPosition(end_frame.p.x(), end_frame.p.y(), end_frame.p.z())
    cable.findLast().getRigidBody().setRotation(agx.Quat(*end_frame.M.get_quaternion()))

    att_begin = agx.LockJoint(robot.ee, cable.begin().getRigidBody(), cable.begin().getRigidBody().getPosition())
    sim.add(att_begin)

    att_end = agx.LockJoint(robot.ee, cable.findLast().getRigidBody(), cable.findLast().getRigidBody().getPosition())
    sim.add(att_end)

    agxOSG.createAxes(cable.begin().getRigidBody(), agx.AffineMatrix4x4(), root(), 0.1)
    agxOSG.createAxes(cable.findLast().getRigidBody(), agx.AffineMatrix4x4(), root(), 0.1)

# Entry point when this script is started with python executable
init = init_app(
    name=__name__,
    scenes=[(buildScene, "1")],
    autoStepping=True,  # Default: False
    onInitialized=lambda app: print("App successfully initialized."),
    onShutdown=lambda app: print("App successfully shut down."),
)
