import sys
import time
from ur_rtde_interface.robotiq_socket import GripperSocket


class GripperControl:
    def __init__(self, gripper_type: str, **kwargs) -> None:

        self.gripper = GripperSocket(
            robot_ip=kwargs.get("robot_ip", "192.168.0.102"),
            port=kwargs.get("port", 63352),
            gripper_type=gripper_type,
        )

    def set_min_stroke(self, min_stroke: float) -> None:
        self.gripper.min_stroke = float(min_stroke)

    def set_max_stroke(self, max_stroke: float) -> None:
        self.gripper.max_stroke = float(max_stroke)

    def set_stroke(self, stroke: float) -> None:
        self.gripper.stroke = float(stroke)

    def get_stroke(self) -> float:
        return float(self.gripper.stroke)

    def get_max_stroke(self) -> float:
        return self.gripper.max_stroke

    def get_min_stroke(self) -> float:
        return self.gripper.min_stroke

    def set_max_force(self, max_force: float) -> None:
        self.max_force = float(max_force)

    def get_max_force(self) -> float:
        return float(getattr(self, "max_force", 100.0))

    def get_status(self, as_dict=False) -> dict:
        """
        Return the status of the gripper if as_dict is True, otherwise return the status as a list
        """
        if not as_dict:
            return self.gripper.getGripperStatus()
        status = self.gripper.getGripperStatus()
        return {
            "Ready": status[0],
            "Reset": status[1],
            "Is Moving": status[2],
            "Object Detected": status[3],
            "Fault Code": status[4],
            "Pos": status[5],
            "Requested Pos": status[6],
            "Current": status[7],
        }

    def initialize(self) -> bool:
        self.init = False
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if self.gripper.initialize():
                self.init = True
                return self.init
            time.sleep(1)
        return self.init

    def open(self) -> None:
        self.gripper.open_()

    def close(self) -> None:
        self.gripper.close_()

    def go_to(self, position, speed, force) -> None:
        self.gripper.goTo(position, speed, force)


if __name__ == "__main__":
    import time

    gripper_control_socket = GripperControl("Hand_E", robot_ip="192.168.0.103", port=63352)

    if not gripper_control_socket.initialize():
        print("Failed to initialize gripper")
        sys.exit(1)
    print("Gripper initialized successfully")

    gripper_control_socket.go_to(0.04, 50, 100)

    time.sleep(2)

    gripper_control_socket.go_to(0.00, 50, 100)

    for i in range(100):
        time.sleep(0.1)
        print(f"Gripper position: {gripper_control_socket.gripper.getActualPos()}")
