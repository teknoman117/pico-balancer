#!/usr/bin/env python3

import asyncio
import asyncudp

async def main():
    sock = await asyncudp.create_socket(local_addr=('0.0.0.0', 3000))

    print("socket created, entering loop")
    while True:
        data, addr = await sock.recvfrom()
        print(data, addr)

asyncio.run(main())