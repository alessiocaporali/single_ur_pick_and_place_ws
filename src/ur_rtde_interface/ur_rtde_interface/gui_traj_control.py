#!/usr/bin/env python3
"""
ros_button_slider_gui.py
ROS2 GUI with +1 / -1 momentary buttons and a slider (1–5).
Publishes Int32 messages to /robot/traj_control_value:
 +N when +1 button pressed,
 -N when -1 button pressed,
 0 when neither pressed.
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider
)
from PyQt5.QtCore import Qt
from rclpy.qos import QoSProfile


class ButtonPublisherGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "ros_button_slider_gui")
        QWidget.__init__(self)
        self.setWindowTitle("ROS2 Button + Slider Publisher")
        self.setMinimumSize(320, 200)

        # --- ROS2 publisher ---
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Int32, "robot/traj_control_value", qos)

        # --- Internal state ---
        self.value = 0
        self.slider_value = 1
        self.running = True

        # --- Layout ---
        layout = QVBoxLayout()

        # --- Title ---
        title = QLabel("Press buttons to publish ±(slider value)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-weight: bold; font-size: 14pt;")
        layout.addWidget(title)

        # --- Slider ---
        slider_layout = QVBoxLayout()
        self.slider_label = QLabel(f"Slider value: {self.slider_value}")
        self.slider_label.setAlignment(Qt.AlignCenter)
        self.slider_label.setStyleSheet("font-weight: bold; font-size: 12pt;")

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(1)
        self.slider.setMaximum(5)
        self.slider.setValue(1)
        self.slider.setTickInterval(1)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.update_slider_value)

        slider_layout.addWidget(self.slider_label)
        slider_layout.addWidget(self.slider)
        layout.addLayout(slider_layout)

        # --- Buttons ---
        btn_row = QHBoxLayout()
        self.minus_btn = QPushButton("− Button")
        self.plus_btn = QPushButton("+ Button")

        self.minus_btn.setStyleSheet("font-size: 12pt; font-weight: bold; padding: 8px;")
        self.plus_btn.setStyleSheet("font-size: 12pt; font-weight: bold; padding: 8px;")

        self.minus_btn.pressed.connect(lambda: self.set_value(-self.slider_value))
        self.minus_btn.released.connect(lambda: self.set_value(0))
        self.plus_btn.pressed.connect(lambda: self.set_value(self.slider_value))
        self.plus_btn.released.connect(lambda: self.set_value(0))

        btn_row.addWidget(self.minus_btn)
        btn_row.addWidget(self.plus_btn)
        layout.addLayout(btn_row)

        self.setLayout(layout)

        # --- Start background publishing thread ---
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()

    def update_slider_value(self, val):
        self.slider_value = val
        self.slider_label.setText(f"Slider value: {val}")

    def set_value(self, val):
        self.value = val

    def publish_loop(self):
        """Continuous publisher loop."""
        while rclpy.ok() and self.running:
            msg = Int32()
            msg.data = self.value
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def closeEvent(self, event):
        self.running = False
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = ButtonPublisherGUI()
    gui.show()
    ret = app.exec_()
    gui.running = False
    gui.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
