import ctypes

PACKET_TYPE_UNKNOWN = 0

PACKET_TYPE_STATE = 1
class StatePacket(ctypes.Structure):
    _fields_ = [("timestamp", ctypes.c_uint32),
                ("yaw", ctypes.c_float),
                ("pitch", ctypes.c_float),
                ("roll", ctypes.c_float),
                ("pitch_target", ctypes.c_float),
                ("P", ctypes.c_float),
                ("I", ctypes.c_float),
                ("D", ctypes.c_float),
                ("velocity", ctypes.c_float),
                ("velocity_target", ctypes.c_float),
                ("vP", ctypes.c_float),
                ("vI", ctypes.c_float),
                ("vD", ctypes.c_float),
                ("yaw_target", ctypes.c_float),
                ("yP", ctypes.c_float),
                ("yI", ctypes.c_float),
                ("yD", ctypes.c_float)]

PACKET_TYPE_CONTROL = 2
class ControlPacket(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float),
                ("y", ctypes.c_float)]

PACKET_TYPE_PID_CONFIGURATION_TILT = 3
PACKET_TYPE_PID_CONFIGURATION_VELOCITY = 4
PACKET_TYPE_PID_CONFIGURATION_YAW = 5
class PIDConfigurationPacket(ctypes.Structure):
    _fields_ = [("Kp", ctypes.c_float),
                ("Ki", ctypes.c_float),
                ("Kd", ctypes.c_float)]

class MotionPacketData(ctypes.Union):
    _fields_ = [("state", StatePacket),
                ("pid_configuration", PIDConfigurationPacket),
                ("control", ControlPacket)]

class MotionPacket(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32),
                ("data", MotionPacketData)]