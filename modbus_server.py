#!/usr/bin/env python
from serial import Serial, SerialTimeoutException, serialutil
from collections import defaultdict
import os
import time
import asyncio
import struct

from umodbus.server.serial import get_server
from umodbus.server.serial.rtu import RTUServer
from umodbus.client.serial.redundancy_check import CRCError

MB_START_PATH_REG = 751

class ModbusServer:

    def __init__(self):
        self.s = Serial(
            port=os.path.expanduser('~')+'/dev/vcomslave',
            baudrate=115200,
            bytesize=serialutil.EIGHTBITS,
            parity=serialutil.PARITY_NONE,
            stopbits=serialutil.STOPBITS_ONE,
        )
        self.s.timeout = 0.1
        self.data_store = defaultdict(int)
        self.jtc_status=[0]
        self.app = get_server(RTUServer, self.s)
        self.flags=[False]


        @self.app.route(slave_ids=[1], function_codes=[1, 2, 3], addresses=list(range(0, 1000)))
        def read_data_store(slave_id, function_code, address):
            # """" Return value of address. """
            return self.jtc_status[0][address]

        @self.app.route(slave_ids=[1], function_codes=[5, 15, 16], addresses=list(range(751, 774)))
        def write_data_store(slave_id, function_code, address, value):
            """" Set value for address. """
            # print(f'Address: {address}, value: {value}')
            self.data_store[address] = value
            if address == MB_START_PATH_REG + 22: # Point 0 received
                self.flags[0] = True

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