
import agx
import agxSDK

from agxPythonModules.sensors.camera_sensors import (
    VirtualCameraSensor,
    VirtualCameraSensorRBFollower,
)

import cv2
import numpy as np
from pipy.tf import Frame, Rotation, Vector

def set_default_camera(app):
    cam_rot = np.array([[-0.85316517, -0.4995813,  -0.15009234],
        [ 0.49907197, -0.86545313,  0.04379551],
        [-0.15177731, -0.03754207,  0.98770149]])

    cam_pos = np.array([2.16002666, -0.94980901,  0.8256767])
    cam_frame = Frame(Rotation(*cam_rot.flatten()), Vector(*cam_pos))
    set_camera_from_frame(app, cam_frame)


def compute_camera_frame(eye, target):
    # X axis of camera frame = forward direction (eye -> target)
    x_axis = target - eye
    x_axis = x_axis / np.linalg.norm(x_axis)

    # Use world Z as reference up, then orthogonalize
    world_up = np.array([0.0, 0.0, 1.0])

    # Z axis: "up" for camera, orthogonal to X
    z_axis = world_up - np.dot(world_up, x_axis) * x_axis
    z_axis = z_axis / np.linalg.norm(z_axis)

    # Y axis completes right-handed frame: z × x
    y_axis = np.cross(z_axis, x_axis)

    # Build rotation matrix with columns [x y z]
    R = np.column_stack((x_axis, y_axis, z_axis))

    # PiPy Rotation expects 9 numbers row-major, same as cam_rot.flatten()
    cam_rot = Rotation(*R.flatten())
    cam_pos = Vector(*eye)

    return Frame(cam_rot, cam_pos)



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

def add_virtual_rgb_camera(eye, target):
    """
    eye, target: numpy arrays of shape (3,)
    """
    # Convert numpy -> agx Vec3
    eye_vec    = agx.Vec3(float(eye[0]), float(eye[1]), float(eye[2]))
    center_vec = agx.Vec3(float(target[0]), float(target[1]), float(target[2]))
    up_vec     = agx.Vec3(0.0, 0.0, 1.0)

    width, height = 640, 480
    fovy  = 60.0
    near_ = 0.1
    far_  = 10.0   # a bit larger so it sees the whole scene

    rgb_cam = VirtualCameraSensor(
        width,
        height,
        eye_vec,
        center_vec,
        up_vec,
        fovy=fovy,
        near=near_,
        far=far_,
        depth_camera=False,  # RGB
    )

    return rgb_cam



class OpenCVCameraViewer(agxSDK.StepEventListener):
    """
    Live viewer for one RGB camera and optional depth camera using OpenCV.
    """

    def __init__(self, rgb_cam, depth_cam=None, window_prefix="Camera"):
        super().__init__(agxSDK.StepEventListener.LAST_STEP)
        self.rgb_cam = rgb_cam
        self.depth_cam = depth_cam
        self.window_prefix = window_prefix

        self.rgb_window_name = f"{window_prefix} - RGB"
        self.depth_window_name = f"{window_prefix} - Depth"

        cv2.namedWindow(self.rgb_window_name, cv2.WINDOW_NORMAL)
        if self.depth_cam is not None:
            cv2.namedWindow(self.depth_window_name, cv2.WINDOW_NORMAL)

    def last(self, t):
        self._show_rgb()
        self._show_depth()
        cv2.waitKey(1)

    def _show_rgb(self):
        img = self.rgb_cam.image
        if img is None:
            return

        img = np.asarray(img)

        # If float image, assume [0,1] and convert to uint8
        if img.dtype != np.uint8:
            img = np.clip(img, 0.0, 1.0)
            img = (img * 255.0).astype(np.uint8)

        # If single-channel, convert to 3-channel RGB
        if img.ndim == 2:
            img = img[..., np.newaxis]
        if img.shape[2] == 1:
            img = np.repeat(img, 3, axis=2)

        # VirtualCameraSensor gives RGB, OpenCV expects BGR
        img_bgr = img[..., ::-1]

        cv2.imshow(self.rgb_window_name, img_bgr)

    def _show_depth(self):
        if self.depth_cam is None:
            return

        depth = self.depth_cam.image
        if depth is None:
            return

        depth = np.asarray(depth).squeeze()  # (H,W,1) -> (H,W)

        # Depth is usually in meters (float). Normalize for visualization.
        if not np.any(np.isfinite(depth)):
            return

        # Clamp extreme values, then normalize to [0,255]
        d = np.copy(depth)
        d[~np.isfinite(d)] = 0.0

        d_min = np.percentile(d, 5)
        d_max = np.percentile(d, 95)
        if d_max <= d_min:
            d_min, d_max = d.min(), d.max()
        if d_max <= d_min:
            return  # all same value, nothing useful to show

        d = np.clip(d, d_min, d_max)
        d_norm = (d - d_min) / (d_max - d_min)
        d_u8 = (d_norm * 255.0).astype(np.uint8)

        # Optional: apply colormap for nicer depth visualization
        d_color = cv2.applyColorMap(d_u8, cv2.COLORMAP_JET)

        cv2.imshow(self.depth_window_name, d_color)

