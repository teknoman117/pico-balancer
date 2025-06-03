#!/usr/bin/env python3

import asyncio
import asyncudp
from balancer.types import MotionPacket

async def main():
    sock = await asyncudp.create_socket(local_addr=('0.0.0.0', 3000))

    print("socket created, entering loop")
    while True:
        data, addr = await sock.recvfrom()

        packet = MotionPacket.from_buffer_copy(data)
        print(f'orientation: yaw: {packet.data.orientation.yaw}, pitch: {packet.data.orientation.pitch}, roll: {packet.data.orientation.roll}, from = {addr}')

asyncio.run(main())