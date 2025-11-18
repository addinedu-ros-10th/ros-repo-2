from PyQt6.QtCore import QObject, pyqtSignal

class BridgeSignaller(QObject):
    robot_signal = pyqtSignal(int, float, float)
    staff_rfid_signal_1 = pyqtSignal(str)   # RFID_1 UID 전달
    staff_rfid_signal_2 = pyqtSignal(str)   # RFID_2 UID 전달
    staff_rfid_signal_3 = pyqtSignal(str)   # RFID_3 UID 전달
    
    staff_list_add = pyqtSignal(dict)           # GUI → 서버 dict 전달
    staff_list_signal = pyqtSignal(object)      # 서버 → GUI DataFrame 전달
