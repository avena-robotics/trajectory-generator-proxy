import os
import asyncio, threading
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
MB_END_PATH_REG = MB_START_PATH_REG + 316

MB_START_CONTROL_REG = 201
MB_END_CONTROL_REG = MB_START_CONTROL_REG + 99


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
        self.s.flush()
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

        self.path_mtx = threading.Lock()
        self.ctrl_mtx = threading.Lock()
        self.tele_mtx = threading.Lock()

        self.handle_modbus_thread = threading.Thread(target=self.handle_request_threaded, daemon=True).start()
        
        @self.app.route(slave_ids=[1], function_codes=[3], addresses=list(range(MB_START_TELEMETRY_REG, MB_END_TELEMETRY_REG + 1)))
        def read_data_store(slave_id, function_code, address):
            if address == MB_START_TELEMETRY_REG:
                self.tele_mtx.acquire()
            elif address == MB_END_TELEMETRY_REG:
                self.tele_mtx.release()
            # """" Return value of address. """
            # print(f'[read_data_store]: Telemetry. Address: {address}')
            # if address == MB_END_TELEMETRY_REG:
            #     print(time.time_ns())
            return self.jtc_status[0][address]

        @self.app.route(slave_ids=[1], function_codes=[16], addresses=list(range(MB_START_PATH_REG, MB_END_PATH_REG + 1)))
        def write_waypoints(slave_id, function_code, address, value):
            """" Set value for address. """
            # print(f'[write_waypoints]: Address: {address}, value: {value}')
            if address == MB_START_PATH_REG:
                self.path_mtx.acquire()
                self.data_store[address] = value
            elif address == MB_END_PATH_REG: # All points received
                self.path_mtx.release()
                self.flags.send_waypoints = True


        @self.app.route(slave_ids=[1], function_codes=[16], addresses=list(range(MB_START_CONTROL_REG, MB_END_CONTROL_REG + 1)))
        def write_control_words(slave_id, function_code, address, value):
            """" Set control words. """
            # print(f'[write_control_words]: Address: {address}, value: {value}')
            if address == MB_START_CONTROL_REG:
                self.ctrl_mtx.acquire()
                self.control_words[address] = value
            if address == MB_END_CONTROL_REG:
                self.ctrl_mtx.release()
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

    def handle_request_threaded(self):
        while True:
            time.sleep(0.05)
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