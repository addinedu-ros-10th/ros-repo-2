from PyQt6.QtCore import QObject, pyqtSignal

class BridgeSignaller(QObject):
    robot_signal = pyqtSignal(int, float, float)
