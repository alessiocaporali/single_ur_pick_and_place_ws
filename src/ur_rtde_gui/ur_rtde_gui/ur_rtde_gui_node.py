#!/usr/bin/env python3
import sys
import threading
from dataclasses import dataclass
import signal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand

from PyQt5 import QtCore, QtWidgets


@dataclass
class GripperConfig:
    action_name: str = "/gripper_action_controller/gripper_cmd"
    n_joints: int = 1
    pos_min: float = 0.0
    pos_max: float = 0.025
    open_pos: float = 0.025
    close_pos: float = 0.0


class GripperActionNode(Node):
    def __init__(self, cfg: GripperConfig):
        super().__init__("gripper_gui")
        self.cfg = cfg
        self._client = ActionClient(self, GripperCommand, cfg.action_name)
        self._lock = threading.Lock()
        self._busy = False

    def wait_server(self, timeout_sec: float = 2.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def is_busy(self) -> bool:
        with self._lock:
            return self._busy

    def _set_busy(self, val: bool):
        with self._lock:
            self._busy = val

    def send_position(self, position: float, feedback_cb=None, done_cb=None):
        if self.is_busy():
            self.get_logger().warn("Action busy; ignoring new goal.")
            return

        pos = float(position)
        pos = max(self.cfg.pos_min, min(self.cfg.pos_max, pos))

        goal = GripperCommand.Goal()
        goal.command.position = pos

        self._set_busy(True)

        send_future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)

        def _on_goal_response(fut):
            goal_handle = fut.result()
            if goal_handle is None or not goal_handle.accepted:
                self._set_busy(False)
                if done_cb:
                    done_cb(False, "rejected")
                return

            result_future = goal_handle.get_result_async()

            def _on_result(res_fut):
                res = res_fut.result()
                self._set_busy(False)
                if res is None:
                    if done_cb:
                        done_cb(False, "no_result")
                    return
                if done_cb:
                    done_cb(True, f"status={res.status}")

            result_future.add_done_callback(_on_result)

        send_future.add_done_callback(_on_goal_response)


class GripperWindow(QtWidgets.QWidget):
    # Signal to safely append text from ANY thread -> GUI thread
    log_signal = QtCore.pyqtSignal(str)

    def __init__(self, ros_node: GripperActionNode, cfg: GripperConfig):
        super().__init__()
        self.node = ros_node
        self.cfg = cfg

        self.setWindowTitle("Gripper Control")
        self.setMinimumWidth(420)

        # Widgets
        self.status_lbl = QtWidgets.QLabel("Status: disconnected")
        self.status_lbl.setStyleSheet("font-weight: bold;")

        self.pos_lbl = QtWidgets.QLabel("Position: --")

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)
        self.slider.setValue(int(1000 * (cfg.open_pos - cfg.pos_min) / (cfg.pos_max - cfg.pos_min)))
        self.slider.valueChanged.connect(self._on_slider)

        self.pos_spin = QtWidgets.QDoubleSpinBox()
        self.pos_spin.setDecimals(4)
        self.pos_spin.setSingleStep(0.001)
        self.pos_spin.setRange(cfg.pos_min, cfg.pos_max)
        self.pos_spin.setValue(cfg.open_pos)
        self.pos_spin.valueChanged.connect(self._on_spin)

        self.send_btn = QtWidgets.QPushButton("Send")
        self.send_btn.clicked.connect(self._send_current)

        self.open_btn = QtWidgets.QPushButton("Open")
        self.open_btn.clicked.connect(lambda: self._send_pos(cfg.open_pos))

        self.close_btn = QtWidgets.QPushButton("Close")
        self.close_btn.clicked.connect(lambda: self._send_pos(cfg.close_pos))

        self.fb_box = QtWidgets.QPlainTextEdit()
        self.fb_box.setReadOnly(True)
        self.fb_box.setMaximumBlockCount(300)

        # Connect signal -> GUI update
        self.log_signal.connect(self._append_gui)

        # Layout
        grid = QtWidgets.QGridLayout(self)
        grid.addWidget(self.status_lbl, 0, 0, 1, 4)

        grid.addWidget(QtWidgets.QLabel("Position setpoint"), 1, 0, 1, 2)
        grid.addWidget(self.pos_spin, 1, 2, 1, 2)
        grid.addWidget(self.slider, 2, 0, 1, 4)
        grid.addWidget(self.pos_lbl, 3, 0, 1, 2)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addWidget(self.open_btn)
        btn_row.addWidget(self.close_btn)
        btn_row.addStretch(1)
        btn_row.addWidget(self.send_btn)
        grid.addLayout(btn_row, 6, 0, 1, 4)

        grid.addWidget(QtWidgets.QLabel("Feedback / logs"), 7, 0, 1, 4)
        grid.addWidget(self.fb_box, 8, 0, 1, 4)

        # Timer to update connection/busy state
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(250)

        self._sync_from_value(cfg.open_pos)

    def _append_gui(self, text: str):
        """Runs on GUI thread."""
        self.fb_box.appendPlainText(text)

    def log(self, text: str):
        """Safe to call from any thread."""
        self.log_signal.emit(text)

    def _value_to_slider(self, value: float) -> int:
        span = self.cfg.pos_max - self.cfg.pos_min
        if span <= 0:
            return 0
        return int(round(1000.0 * (value - self.cfg.pos_min) / span))

    def _slider_to_value(self, s: int) -> float:
        span = self.cfg.pos_max - self.cfg.pos_min
        return self.cfg.pos_min + (float(s) / 1000.0) * span

    def _sync_from_value(self, value: float):
        self.pos_spin.blockSignals(True)
        self.slider.blockSignals(True)
        self.pos_spin.setValue(value)
        self.slider.setValue(self._value_to_slider(value))
        self.slider.blockSignals(False)
        self.pos_spin.blockSignals(False)
        self.pos_lbl.setText(f"Position: {value:.4f}")

    def _on_slider(self, s: int):
        v = self._slider_to_value(s)
        self._sync_from_value(v)

    def _on_spin(self, v: float):
        self._sync_from_value(float(v))

    def _tick(self):
        ok = self.node.wait_server(timeout_sec=0.0)
        busy = self.node.is_busy()

        if ok:
            self.status_lbl.setText(f"Status: connected  |  busy: {busy}")
        else:
            self.status_lbl.setText("Status: disconnected (action server not found)")

        enabled = ok and (not busy)
        self.send_btn.setEnabled(enabled)
        self.open_btn.setEnabled(enabled)
        self.close_btn.setEnabled(enabled)

    def _send_current(self):
        self._send_pos(self.pos_spin.value())

    def _send_pos(self, pos: float):
        self.log(f"> sending pos={pos:.4f}")

        def feedback_cb(msg):
            # ROS thread -> emit string to GUI thread
            self.log(f"feedback: {msg.feedback}")

        def done_cb(ok: bool, info: str):
            self.log(f"done: ok={ok} {info}")

        self.node.send_position(pos, feedback_cb=feedback_cb, done_cb=done_cb)


def main():
    cfg = GripperConfig()

    rclpy.init(args=None)
    node = GripperActionNode(cfg)

    app = QtWidgets.QApplication(sys.argv)
    win = GripperWindow(node, cfg)
    win.show()

    # Start rclpy spinning in a joinable thread (not daemon)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=False)
    spin_thread.start()

    # When the Qt app is quitting, cleanly shutdown rclpy and wait for the spin thread
    def _cleanup():
        # First request rclpy to shutdown so rclpy.spin returns
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # wait briefly for spin thread to exit
        if spin_thread.is_alive():
            spin_thread.join(timeout=2.0)
        # finally destroy the node (only once spin has stopped)
        try:
            node.destroy_node()
        except Exception:
            pass

    # Connect Qt's aboutToQuit signal to cleanup
    app.aboutToQuit.connect(_cleanup)

    # Make Ctrl-C (SIGINT) trigger app.quit() which will emit aboutToQuit -> cleanup
    signal.signal(signal.SIGINT, lambda sig, frame: app.quit())

    try:
        rc = app.exec_()
    except KeyboardInterrupt:
        # fallback: if something else catches the SIGINT, force cleanup
        _cleanup()
        rc = 0

    # Ensure cleanup ran (in case exec_ returned without aboutToQuit firing)
    _cleanup()

    sys.exit(rc)

if __name__ == "__main__":
    main()
