import os
os.environ["ROS_DOMAIN_ID"] = "0"

import agx
import agxOSG
from agxPythonModules.utils.environment import simulation, init_app, application, root

from utils import set_default_camera
from robot_interface import UrRobot, UrRobotRos2Mapping


# Build robot scene
def buildScene():
    app = application()
    sim = simulation()
    sim.setTimeStep(0.01)

    # camera
    set_default_camera(app)

    ################################################
    # Create the robot, using the Universal Robots UR loaded from urdf.

    package_path = "robot_description"
    urdf_file = f"ur_robotiq_hande.urdf"
    robot = UrRobot(simulation(), initial_position=[0.0, 0.0, 0.0], package_path=package_path, urdf_file=urdf_file)
    robot.enable_motors(True)
    agxOSG.createAxes(robot.base, agx.AffineMatrix4x4(), root(), 0.1)
    agxOSG.createAxes(robot.ee, agx.AffineMatrix4x4(), root(), 0.1)


    robot_ros2 = UrRobotRos2Mapping(robot, prefix="ur")
    sim.add(robot_ros2)


# Entry point when this script is started with python executable
init = init_app(
    name=__name__,
    scenes=[(buildScene, "1")],
    autoStepping=True,  # Default: False
    onInitialized=lambda app: print("App successfully initialized."),
    onShutdown=lambda app: print("App successfully shut down."),
)
