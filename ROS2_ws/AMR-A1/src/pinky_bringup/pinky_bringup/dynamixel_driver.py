import time
from dynamixel_sdk import *

class DynamixelDriver:
    def __init__(self, port, baudrate, dxl_ids):
        self.ADDR_OPERATING_MODE    = 11
        self.ADDR_TORQUE_ENABLE     = 64
        self.ADDR_LED_RED           = 65
        self.ADDR_GOAL_VELOCITY     = 104
        self.ADDR_PROFILE_ACCEL     = 108
        self.ADDR_PRESENT_VELOCITY  = 128
        self.ADDR_PRESENT_POSITION  = 132

        self.LEN_GOAL_VELOCITY      = 4
        self.LEN_PRESENT_VELOCITY   = 4
        self.LEN_PRESENT_POSITION   = 4

        self.PROTOCOL_VERSION       = 2.0
        self.DXL_IDS                = dxl_ids
        self.BAUDRATE               = baudrate
        self.DEVICENAME             = port

        self.RPM_TO_VALUE_SCALE = 1 / 0.229

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_VELOCITY, self.LEN_GOAL_VELOCITY)
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

    def begin(self):
        if not self.portHandler.openPort(): return False
        if not self.portHandler.setBaudRate(self.BAUDRATE): return False
        return True

    def terminate(self):
        self.set_double_rpm(0, 0)
        time.sleep(0.1)
        for dxl_id in self.DXL_IDS:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_LED_RED, 0) # LED OFF
        time.sleep(0.1)
        self.portHandler.closePort()

    def initialize_motors(self, profile_accel=200):
        for dxl_id in self.DXL_IDS:
            try: 
                self.packetHandler.reboot(self.portHandler, dxl_id)
                time.sleep(0.5)
            except Exception as e:
                print(f"Warning: Could not reboot motor {dxl_id}. Error: {e}")

            if self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_OPERATING_MODE, 1)[0] != COMM_SUCCESS: return False
            if self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PROFILE_ACCEL, profile_accel)[0] != COMM_SUCCESS: return False
            if self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)[0] != COMM_SUCCESS: return False
            if self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_LED_RED, 1)[0] != COMM_SUCCESS: return False
        return True

    def set_double_rpm(self, rpm_l, rpm_r):
        velocities = [rpm_l, rpm_r]
        self.groupSyncWrite.clearParam()

        for i, dxl_id in enumerate(self.DXL_IDS):
            dxl_vel = int(velocities[i] * self.RPM_TO_VALUE_SCALE)
            param = [DXL_LOBYTE(DXL_LOWORD(dxl_vel)), DXL_HIBYTE(DXL_LOWORD(dxl_vel)),
                     DXL_LOBYTE(DXL_HIWORD(dxl_vel)), DXL_HIBYTE(DXL_HIWORD(dxl_vel))]
            if not self.groupSyncWrite.addParam(dxl_id, param): return False

        return self.groupSyncWrite.txPacket() == COMM_SUCCESS

    def get_feedback(self):
        self.groupBulkRead.clearParam()
        read_len = self.LEN_PRESENT_VELOCITY + self.LEN_PRESENT_POSITION
        
        for dxl_id in self.DXL_IDS:
            self.groupBulkRead.addParam(dxl_id, self.ADDR_PRESENT_VELOCITY, read_len)

        if self.groupBulkRead.txRxPacket() != COMM_SUCCESS:
            return None, None, None, None
        
        id_l, id_r = self.DXL_IDS[0], self.DXL_IDS[1]
        if not self.groupBulkRead.isAvailable(id_l, self.ADDR_PRESENT_VELOCITY, read_len) or \
           not self.groupBulkRead.isAvailable(id_r, self.ADDR_PRESENT_VELOCITY, read_len):
            return None, None, None, None

        vel_raw_l = self.groupBulkRead.getData(id_l, self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY)
        pos_raw_l = self.groupBulkRead.getData(id_l, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        
        vel_raw_r = self.groupBulkRead.getData(id_r, self.ADDR_PRESENT_VELOCITY, self.LEN_PRESENT_VELOCITY)
        pos_raw_r = self.groupBulkRead.getData(id_r, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
        
        if vel_raw_l > 2**31: vel_raw_l -= 2**32
        if pos_raw_l > 2**31: pos_raw_l -= 2**32
        rpm_l = vel_raw_l / self.RPM_TO_VALUE_SCALE

        if vel_raw_r > 2**31: vel_raw_r -= 2**32
        if pos_raw_r > 2**31: pos_raw_r -= 2**32
        rpm_r = vel_raw_r / self.RPM_TO_VALUE_SCALE

        return rpm_l, rpm_r, pos_raw_l, pos_raw_r