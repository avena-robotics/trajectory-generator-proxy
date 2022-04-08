import os
import asyncio
import struct
import time
from collections import defaultdict

from serial import Serial, SerialTimeoutException, serialutil
from umodbus.server.serial import get_server
from umodbus.server.serial.rtu import RTUServer
from umodbus.client.serial.redundancy_check import CRCError

MB_START_TELEMETRY_REG = 1
MB_END_TELEMETRY_REG = 122

MB_START_PATH_REG = 751
MB_END_PATH_REG = MB_START_PATH_REG + 22

MB_START_CONTROL_REG = 201
MB_END_CONTROL_REG = MB_START_CONTROL_REG + 9


class Flags:
    send_waypoints = False
    send_control_word = False


class ModbusServer:

    def __init__(self):
        self.s = Serial(
            port=os.path.expanduser('~')+'/dev/vcomslave',
            baudrate=115200,
            bytesize=serialutil.EIGHTBITS,
            parity=serialutil.PARITY_NONE,
            stopbits=serialutil.STOPBITS_ONE,
            timeout = 0.1
        )
        self.data_store = defaultdict(int)
        self.control_words = defaultdict(int)
        # self.control_words = {
        #     'clear_current_errors': False,
        #     'clear_occured_errors': False,
        #     'friction_table_use_default': False,
        #     'pid_param_use_default': False,
        #     'arm_model_use_default': False,
        #     'teaching_mode_enable': False,
        #     'teaching_mode_disable': False,
        #     'friction_polynomial_use_default': False,
        # }
        self.jtc_status=[0]
        self.app = get_server(RTUServer, self.s)
        self.flags = Flags()
        
        @self.app.route(slave_ids=[1], function_codes=[3], addresses=list(range(MB_START_TELEMETRY_REG, MB_END_TELEMETRY_REG + 1)))
        def read_data_store(slave_id, function_code, address):
            # """" Return value of address. """
            # print(f'Telemetry. Address: {address}')
            return self.jtc_status[0][address]

        @self.app.route(slave_ids=[1], function_codes=[16], addresses=list(range(MB_START_PATH_REG, MB_END_PATH_REG + 1)))
        def write_waypoints(slave_id, function_code, address, value):
            """" Set value for address. """
            # print(f'{__name__}. Address: {address}, value: {value}')
            self.data_store[address] = value
            if address == MB_END_PATH_REG: # Point 0 received
                self.flags.send_waypoints = True

        @self.app.route(slave_ids=[1], function_codes=[16], addresses=list(range(MB_START_CONTROL_REG, MB_END_CONTROL_REG + 1)))
        def write_control_words(slave_id, function_code, address, value):
            """" Set control words. """
            # print(f'{__name__}. Address: {address}, value: {value}')
            self.control_words[address] = value
            if address == MB_END_CONTROL_REG:
                self.flags.send_control_word = True

    async def handle_request(self):
        while True:
            await asyncio.sleep(0.05)
            try:
                self.app.serve_once()
            except (CRCError, struct.error) as e:
                print('Can\'t handle request: {0}'.format(e))
            except SerialTimeoutException as e:
                print('[ERROR]: SerialTimeoutException', e)
            except ValueError as e:
                # print('[ERROR]: ValueError', e)
                pass

if __name__ == '__main__':
    ms=ModbusServer()
    asyncio.run(ms.handle_request())