import socket
import time
from enum import Enum

import numpy as np


class RobotiqSocketCmds:
    cmd_activate = b"SET ACT 1\n"
    cmd_deactivate = b"SET ACT 0\n"

    cmd_enable_move = b"SET GTO 1\n"
    cmd_disable_move = b"SET GTO 0\n"

    cmd_full_close = b"SET POS 255\n"
    cmd_full_open = b"SET POS 0\n"

    cmd_set_pos = b"SET POS "
    cmd_set_speed = b"SET SPE "
    cmd_set_force = b"SET FOR "

    cmd_get_pos = b"GET POS\n"
    cmd_get_speed = b"GET SPE\n"
    cmd_get_force = b"GET FOR\n"
    cmd_get_pos_request = b"GET PRE\n"
    cmd_object_detected = b"GET OBJ\n"
    cmd_get_activation_status = b"GET STA\n"
    cmd_get_fault = b"GET FLT\n"
    cmd_get_current = b"GET COU\n"


class ActivationStatus(Enum):
    NOT_CONNECTED = -1
    RESET = 0
    ACTIVATION_BUSY = 1
    NOT_USED = 2
    ACTIVATION_DONE = 3


class GripperSocket:
    """Robotiq socket client compatible with the existing GripperControl API."""

    def __init__(self, gripper_type: str, robot_ip="192.168.0.102", port=63352, timeout=0.5):
        if gripper_type not in ["2F_85", "Hand_E"]:
            raise ValueError("gripper_type must be '2F_85' or 'Hand_E'")

        self.gripper_type = gripper_type
        if gripper_type == "2F_85":
            self.min_stroke = 0.0
            self.max_stroke = 0.085
        else:
            self.min_stroke = 0.0
            self.max_stroke = 0.055
        self.stroke = self.max_stroke

        self.skt_ip = robot_ip
        self.skt_port = int(port)
        self.timeout = float(timeout)
        self.is_connected = False

        self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.skt.settimeout(self.timeout)

    def connect(self):
        try:
            self.skt.connect((self.skt_ip, self.skt_port))
            self.is_connected = True
            time.sleep(0.1)
            return True
        except Exception:
            self.is_connected = False
            return False

    def connectionClose(self):
        try:
            self.skt.close()
        finally:
            self.is_connected = False
        return True

    def _send_command(self, cmd):
        if not self.is_connected and not self.connect():
            return False, -1

        try:
            self.skt.sendall(cmd)
            data = self.skt.recv(1024)
            text = data.decode("utf-8", errors="ignore").strip().lower()
            if not text or "ack" in text or text == "ok":
                return True, -1

            parts = data.split()
            if len(parts) >= 2:
                try:
                    return True, int(parts[1])
                except ValueError:
                    return True, -1
            return True, -1
        except (socket.timeout, OSError):
            self.connectionClose()
            return False, -1

    def _position_to_byte(self, pos):
        pos = float(np.clip(pos, self.min_stroke, self.max_stroke))
        if self.gripper_type == "2F_85":
            return int(np.clip((3.0 - 230.0) / self.stroke * pos + 230.0, 0, 255))
        return int(np.clip((255.0 - (250.0 * pos / self.stroke)), 0, 255))

    def _byte_to_position(self, pos):
        pos = float(pos)
        if self.gripper_type == "2F_85":
            return float(np.clip(self.stroke / (3.0 - 230.0) * (pos - 230.0), 0.0, self.stroke))
        return float(np.clip(self.stroke / 255.0 * (255.0 - pos), 0.0, self.stroke))

    def _check_gripper_status(self):
        ok, value = self._send_command(RobotiqSocketCmds.cmd_get_activation_status)
        if not ok:
            return False, ActivationStatus.NOT_CONNECTED
        try:
            return True, ActivationStatus(int(value))
        except ValueError:
            return True, ActivationStatus.NOT_CONNECTED

    def initialize(self):
        if not self.is_connected and not self.connect():
            return False

        _, status = self._check_gripper_status()
        if status == ActivationStatus.RESET:
            self._send_command(RobotiqSocketCmds.cmd_activate)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            _, status = self._check_gripper_status()
            if status == ActivationStatus.ACTIVATION_DONE:
                return True
            time.sleep(0.1)
        return False

    def deactivate(self):
        return self._send_command(RobotiqSocketCmds.cmd_deactivate)

    def isReady(self):
        _, status = self._check_gripper_status()
        return status == ActivationStatus.ACTIVATION_DONE

    def isSleeping(self):
        _, status = self._check_gripper_status()
        return status == ActivationStatus.RESET

    def open_(self):
        return self._send_command(RobotiqSocketCmds.cmd_full_open)

    def close_(self):
        return self._send_command(RobotiqSocketCmds.cmd_full_close)

    def _send_move_routine(self, pos_cmd, speed, force):
        ok_pos, _ = self._send_command(pos_cmd)
        ok_spe, _ = self._send_command(RobotiqSocketCmds.cmd_set_speed + str(int(speed)).encode() + b"\n")
        ok_for, _ = self._send_command(RobotiqSocketCmds.cmd_set_force + str(int(force)).encode() + b"\n")

        if ok_pos and ok_spe and ok_for:
            return self._send_command(RobotiqSocketCmds.cmd_enable_move)[0]
        return False

    def goTo(self, pos, speed, force):
        pos_byte = self._position_to_byte(pos)
        speed = int(np.clip(speed, 1, 100))
        force = int(np.clip(force, 1, 100))
        pos_cmd = RobotiqSocketCmds.cmd_set_pos + str(pos_byte).encode() + b"\n"
        return self._send_move_routine(pos_cmd, speed, force)

    def move_perc_pos(self, pos_perc, speed, force):
        pos_perc = float(np.clip(pos_perc, 0.0, 100.0))
        pos_byte = int(255 - int(255 * pos_perc / 100.0))
        pos_cmd = RobotiqSocketCmds.cmd_set_pos + str(pos_byte).encode() + b"\n"
        return self._send_move_routine(pos_cmd, speed, force)

    def graspDetected(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_object_detected)
        return fdbk in (1, 2)

    def isMoving(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_activation_status)
        return fdbk == 0

    def getFaultId(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_fault)
        return fdbk

    def getVelocityEcho(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_speed)
        return fdbk

    def getForceEcho(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_force)
        return fdbk

    def getRequestedPosition(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_pos_request)
        if fdbk < 0:
            return self.getActualPos()
        return self._byte_to_position(fdbk)

    def getActualPos(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_pos)
        if fdbk < 0:
            return -1.0
        return self._byte_to_position(fdbk)

    def getCurrent(self):
        _, fdbk = self._send_command(RobotiqSocketCmds.cmd_get_current)
        return fdbk

    def getGripperStatus(self):
        return [
            self.isReady(),
            self.isSleeping(),
            self.isMoving(),
            self.graspDetected(),
            self.getFaultId(),
            self.getActualPos(),
            self.getRequestedPosition(),
            self.getCurrent(),
        ]
