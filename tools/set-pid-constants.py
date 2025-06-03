#!/usr/bin/env python3

import asyncio
import asyncudp
import argparse

from balancer.types import MotionPacket, PIDConfigurationPacket, PACKET_TYPE_PID_CONFIGURATION

# volatile float Kp = 1.f / 5.f;
# volatile float Ki = 0.025f;
# volatile float Kd = 1.1f;

async def main(Kp = 0.2, Ki = 0.025, Kd = 1.1):
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
parser.add_argument('-p', '--Kp', default=0.2)
parser.add_argument('-i', '--Ki', default=0.025)
parser.add_argument('-d', '--Kd', default=1.1)

args = parser.parse_args()
asyncio.run(main(Kp=float(args.Kp), Ki=float(args.Ki), Kd=float(args.Kd)))