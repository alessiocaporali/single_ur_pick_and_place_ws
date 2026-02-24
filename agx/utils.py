import numpy as np
from pipy.tf import Frame, Rotation, Vector

import agx


def set_default_camera(app):
    cam_rot = np.array([[-0.85316517, -0.4995813,  -0.15009234],
        [ 0.49907197, -0.86545313,  0.04379551],
        [-0.15177731, -0.03754207,  0.98770149]])

    cam_pos = np.array([2.16002666, -0.94980901,  0.8256767])
    cam_frame = Frame(Rotation(*cam_rot.flatten()), Vector(*cam_pos))
    set_camera_from_frame(app, cam_frame)

def set_camera_from_frame(app, frame):
    """
    Set the AGX camera based on a PiPy Frame.

    The camera position is taken from the frame origin.
    The camera looks in the direction of the frame's X axis.
    The camera up direction is the frame's Z axis.

    Parameters
    ----------
    app : agxOSG.ExampleApplication
        AGX application handling the camera.
    frame : Frame
        PiPy Frame object defining camera pose.
    """
    # Camera position
    eye_np = frame.p.to_numpy()
    eye = agx.Vec3(*eye_np)

    # Camera forward (look direction)
    forward = frame.M.unitX()
    center_np = eye_np + forward  # distance = 1 automatically
    center = agx.Vec3(*center_np)

    # Camera up axis
    up = agx.Vec3(*frame.M.unitZ())

    # Apply camera
    app.setCameraHome(eye, center, up)



def pipy_frame_from_agx(agx_frame: agx.Frame) -> Frame:
    pos = agx_frame.getTranslate()
    rot = agx_frame.getRotate()
    rotation = Rotation.quaternion(rot.w(), rot.x(), rot.y(), rot.z())
    position = Vector(pos.x(), pos.y(), pos.z())
    return Frame(rotation, position)
