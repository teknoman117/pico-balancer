import ctypes

PACKET_TYPE_UNKNOWN = 0

PACKET_TYPE_ORIENTATION = 1
class OrientationPacket(ctypes.Structure):
    _fields_ = [("yaw", ctypes.c_float),
                ("pitch", ctypes.c_float),
                ("roll", ctypes.c_float)]

PACKET_TYPE_PID_RESPONSE = 2
class PIDResponsePacket(ctypes.Structure):
    _fields_ = [("P", ctypes.c_float),
                ("I", ctypes.c_float),
                ("D", ctypes.c_float)]

PACKET_TYPE_PID_CONFIGURATION = 3
class PIDConfigurationPacket(ctypes.Structure):
    _fields_ = [("Kp", ctypes.c_float),
                ("Ki", ctypes.c_float),
                ("Kd", ctypes.c_float)]

class MotionPacketData(ctypes.Union):
    _fields_ = [("orientation", OrientationPacket),
                ("pid_response", PIDResponsePacket),
                ("pid_configuration", PIDConfigurationPacket)]

class MotionPacket(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32),
                ("data", MotionPacketData)]