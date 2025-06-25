#!/usr/bin/env python3

import asyncio
import asyncudp
import argparse
import evdev

from balancer.types import MotionPacket, PACKET_TYPE_CONTROL, PACKET_TYPE_PID_CONFIGURATION_TILT, PACKET_TYPE_PID_CONFIGURATION_VELOCITY, PACKET_TYPE_PID_CONFIGURATION_YAW

def clamp(n, lower=-1.0, upper=1.0):
    return max(min(upper, n), lower)

def find_gamepad():
    support_devices = [
        # DualShock 4 - CUH-ZCT2U
        ("Wireless Controller", 1356, 2508)
    ]

    devices = evdev.list_devices()
    for path in devices:
        device = evdev.InputDevice(path)
        if (device.name, device.info.vendor, device.info.product) in support_devices:
            return device
    return None

async def main(control = False, Kp = 0.1, Ki = 1.0, Kd = 0.010, vKp = 0.0013, vKi = 0.0, vKd = 0.00003, yKp = 0.02, yKi = 0.001, yKd = 0.001):
    sock = await asyncudp.create_socket(remote_addr=('pico-balancer.local', 3000))
    print('Connected to Balancer')

    # send tilt PID loop constants
    packet = MotionPacket()
    packet.type = PACKET_TYPE_PID_CONFIGURATION_TILT
    packet.data.pid_configuration.Kp = Kp
    packet.data.pid_configuration.Ki = Ki
    packet.data.pid_configuration.Kd = Kd
    print(f'Setting Tilt Configuration - Kp:{Kp}, Ki:{Ki}, Kd:{Kd}')
    sock.sendto(bytes(packet))

    # send velocity PID loop constants
    packet = MotionPacket()
    packet.type = PACKET_TYPE_PID_CONFIGURATION_VELOCITY
    packet.data.pid_configuration.Kp = vKp
    packet.data.pid_configuration.Ki = vKi
    packet.data.pid_configuration.Kd = vKd
    print(f'Setting Velocity Configuration - Kp:{vKp}, Ki:{vKi}, Kd:{vKd}')
    sock.sendto(bytes(packet))

    # send velocity PID loop constants
    packet = MotionPacket()
    packet.type = PACKET_TYPE_PID_CONFIGURATION_YAW
    packet.data.pid_configuration.Kp = yKp
    packet.data.pid_configuration.Ki = yKi
    packet.data.pid_configuration.Kd = yKd
    print(f'Setting Yaw Configuration - Kp:{yKp}, Ki:{yKi}, Kd:{yKd}')
    sock.sendto(bytes(packet))

    # quit if we're not in RC mode
    if not control:
        return

    # send all gamepad events to balancer
    stick_x = None
    stick_y = None
    gamepad = find_gamepad()
    print('Entering Remote Control Mode')
    async for event in gamepad.async_read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_RX:
                stick_x = clamp((event.value - 128) / 127)
                if abs(stick_x) < 0.05:
                    stick_x = 0

            elif event.code == evdev.ecodes.ABS_Y:
                stick_y = clamp((event.value - 128) / 127)
                if abs(stick_y) < 0.05:
                    stick_y = 0

        # send control event to the robot
        if stick_x is not None and stick_y is not None:
            packet = MotionPacket()
            packet.type = PACKET_TYPE_CONTROL
            packet.data.control.x = stick_x
            packet.data.control.y = stick_y
            sock.sendto(bytes(packet))

# Command Line Arguments
parser = argparse.ArgumentParser(
                    prog='PID Configuration Setter',
                    description='Updates PicoW Balancer PID Configuration')
parser.add_argument('-c', '--control', action='store_true')
parser.add_argument('-p', '--Kp', default=0.1)
parser.add_argument('-i', '--Ki', default=1.0)
parser.add_argument('-d', '--Kd', default=0.010)
parser.add_argument('--vKp', default=0.0013)
parser.add_argument('--vKi', default=0.0)
parser.add_argument('--vKd', default=0.00003)
parser.add_argument('--yKp', default=0.02)
parser.add_argument('--yKi', default=0.001)
parser.add_argument('--yKd', default=0.001)

args = parser.parse_args()
asyncio.run(main(control=bool(args.control), Kp=float(args.Kp), Ki=float(args.Ki), Kd=float(args.Kd),
                 vKp=float(args.vKp), vKi=float(args.vKi), vKd=float(args.vKd),
                 yKp=float(args.yKp), yKi=float(args.yKi), yKd=float(args.yKd)))