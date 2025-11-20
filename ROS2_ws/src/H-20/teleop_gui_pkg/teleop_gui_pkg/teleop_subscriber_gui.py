#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt


class TeleopGUI(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "teleop_gui")
        QWidget.__init__(self)

        self.setWindowTitle("Teleop Key GUI")
        self.resize(300, 200)

        self.label = QLabel("Waiting for teleop keys...", self)
        self.label.setAlignment(Qt.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.subscription = self.create_subscription(
            String,
            'teleop/key',
            self.key_callback,
            10
        )

    def key_callback(self, msg):
        self.label.setText(f"Pressed key: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    gui = TeleopGUI()
    gui.show()

    # ROS2 + Qt event loop 통합
    timer = gui.create_timer(0.01, lambda: None)

    try:
        app.exec_()
    finally:
        gui.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()