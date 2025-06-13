#!/usr/bin/env python3

import asyncio
import asyncudp
import argparse

from balancer.types import MotionPacket, PIDConfigurationPacket, PACKET_TYPE_PID_CONFIGURATION

async def main(Kp = 0.1, Ki = 1.0, Kd = 0.01):
    sock = await asyncudp.create_socket(remote_addr=('pico-balancer.local', 3000))

    # decode command line parameters
    configuration = PIDConfigurationPacket()
    configuration.Kp = Kp
    configuration.Ki = Ki
    configuration.Kd = Kd

    packet = MotionPacket()
    packet.type = PACKET_TYPE_PID_CONFIGURATION
    packet.data.pid_configuration = configuration

    print(f'Setting Configuration - Kp:{Kp}, Ki:{Ki}, Kd:{Kd}')
    sock.sendto(bytes(packet))

parser = argparse.ArgumentParser(
                    prog='PID Configuration Setter',
                    description='Updates PicoW Balancer PID Configuration')
parser.add_argument('-p', '--Kp', default=0.1)
parser.add_argument('-i', '--Ki', default=1.0)
parser.add_argument('-d', '--Kd', default=0.01)

args = parser.parse_args()
asyncio.run(main(Kp=float(args.Kp), Ki=float(args.Ki), Kd=float(args.Kd)))