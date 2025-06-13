#!/usr/bin/env python3

import asyncio
import asyncudp
import evdev

from balancer.types import MotionPacket, ControlPacket, PACKET_TYPE_CONTROL

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

async def main():
    # connect to the balancer
    sock = await asyncudp.create_socket(remote_addr=('pico-balancer.local', 3000))
    print('connected to balancer')

    # known stick positions
    stick_x = None
    stick_y = None

    gamepad = find_gamepad()
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

asyncio.run(main())