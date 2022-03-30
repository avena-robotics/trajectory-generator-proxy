#!/usr/bin/env python
from serial import Serial, SerialTimeoutException
from collections import defaultdict
import os
import time
import asyncio
import struct

from umodbus.server.serial import get_server
from umodbus.server.serial.rtu import RTUServer
from umodbus.client.serial.redundancy_check import CRCError

class ModbusServer:

    def __init__(self):
        self.s = Serial(os.path.expanduser('~')+'/dev/vcomslave')
        self.s.timeout = 0.1
        self.data_store = defaultdict(int)
        self.jtc_status=[0]
        self.app = get_server(RTUServer, self.s)


        @self.app.route(slave_ids=[1], function_codes=[1, 2, 3], addresses=list(range(0, 1000)))
        def read_data_store(slave_id, function_code, address):
            print(address, self.jtc_status[0][address])
            # """" Return value of address. """
            return self.jtc_status[0][address]

        @self.app.route(slave_ids=[1], function_codes=[5, 15, 16], addresses=list(range(0, 1000)))
        def write_data_store(slave_id, function_code, address, value):
            # print(self.jtc_status[0])
            """" Set value for address. """
            print(f'Address: {address}, value: {value}')
            self.data_store[address] = value


    async def handle_request(self):
        while 1:
            await asyncio.sleep(0.05)
            try:
                self.app.serve_once()
            except (CRCError, struct.error) as e:
                print('Can\'t handle request: {0}'.format(e))
            except (SerialTimeoutException, ValueError):
                pass

if __name__ == '__main__':
    ms=ModbusServer()
    asyncio.run(ms.handle_request())