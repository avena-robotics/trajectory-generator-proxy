import enum
import struct
import serial
import asyncio
import time
import threading

from commons import get_crc
import params, jtc_vars, trajectory


class RSResponseInfo:
    frame_type = params.Host_FT(0)
    frame_status = params.Host_RxFS(0)
    data_status = params.Host_RxDS(0)
    tes = params.TES(0)

    def __repr__(self):
        return f'RSResponseInfo: {self.frame_type}, {self.frame_status}, {self.data_status}, {self.tes}'


class RSComm:
    def __init__(self):
        self.port = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, parity=serial.PARITY_NONE,
                                  stopbits=1, bytesize=8, timeout=1)
        self.port.flush()
        self.jtc_status = []
        self.response_info = RSResponseInfo()
        self.current_config = [0] * 6  # current robotic arm values in radians
        self.read_bytes_buffer = b''
        self.read_bytes_buffer_mtx = threading.Lock()
        self.read_serial_thread = threading.Thread(target=self.read_serial_buffer, daemon=True).start()

    def read_serial_buffer(self):
        while True:
            new_buffer = self.port.read_all()
            # if len(new_buffer) > 500:
            #     print(f'\nBIG: {len(new_buffer)}\n')
            with self.read_bytes_buffer_mtx:
                self.read_bytes_buffer += new_buffer
            time.sleep(0.01)

    def read_st_frame(self):
        # Validate accumulated byte stream
        if len(self.read_bytes_buffer) < 4:
            return
        if len(self.read_bytes_buffer) > params.COMBUFREADMAX:
            self.read_bytes_buffer = b''
            return
        if (header_idx := self.read_bytes_buffer.find(params.Host_FT.Header.value)) > params.COMBUFREADMAX - 4:
            self.read_bytes_buffer = b''
            return
        if self.read_bytes_buffer[header_idx+1] != params.Host_FT.JtcStatus.value:
            self.read_bytes_buffer = b''
            return
        nd = int.from_bytes(self.read_bytes_buffer[header_idx + 2:header_idx + 4], 'big', signed=False)
        if len(self.read_bytes_buffer) < (header_idx + nd):
            return
        if nd < 4:
            return
        if nd > params.COMBUFREADMAX:
            self.read_bytes_buffer = b''
            # print('nd > params.COMBUFREADMAX')
            return

        # Save valid buffer
        read_buffer = self.read_bytes_buffer[header_idx:header_idx + nd]

        ##################################################
        # DEBUG
        # print(f'len(self.read_bytes): {len(self.read_bytes_buffer)}')
        # print(f'header_idx:           {header_idx}')
        # print(f'nd:                   {nd}')
        # print(f'header_idx + nd:      {header_idx + nd}')
        # print([int(read_buffer[i]) for i in range(10)])
        # print(f'new buffer: {len(new_buffer)}')
        ##################################################

        # self.jtc_status = self.read_jtc_status(read_buffer)
        ret = self.translate_to_modbus(read_buffer)
        self.response_info = self.translate_frame(read_buffer)
        # print('received info:', self.response_info)

        # Store remaining buffer for further processing
        self.read_bytes_buffer = self.read_bytes_buffer[header_idx + nd:]
        return ret

    def translate_frame(self, frame) -> RSResponseInfo:
        response = RSResponseInfo()
        response.frame_type = params.Host_FT(frame[4])
        response.frame_status = params.Host_RxFS(frame[5])
        response.data_status = params.Host_RxDS(frame[6])
        response.tes = params.TES(frame[16])
        ################
        # DEBUG
        if frame[4]:
            print('response:', response)
        ################
        return response

    def translate_to_modbus(self, frame):
        mb_frame = [-1]  # There is nothing at index 0 (according to Excel documentation describing registers)

        # JTC current FSM
        mb_frame += list(struct.unpack('>H', b'\x00' + frame[9:10]))

        # JTC current errors
        mb_frame += list(struct.unpack('>H', frame[10:12]))

        # JTC occured errors
        mb_frame += list(struct.unpack('>H', frame[12:14]))

        # JTC init status
        mb_frame += list(struct.unpack('>H', b'\x00' + frame[14:15]))

        # JTC joint init status
        mb_frame += list(struct.unpack('>H', b'\x00' + frame[15:16]))

        # Traj execution status
        mb_frame += list(struct.unpack('>H', b'\x00' + frame[16:17]))

        # Traj num current point
        mb_frame += list(struct.unpack('>HH', frame[17:21]))

        # Friction type
        mb_frame += list(struct.unpack('>H', b'\x00' + frame[21:22]))

        # CAN current status
        mb_frame += list(struct.unpack('>H', b'\x00' + frame[22:23]))

        # CAN current errors
        mb_frame += list(struct.unpack('>HH', frame[23:27]))

        # CAN occured errors
        mb_frame += list(struct.unpack('>HH', frame[27:31]))

        # Joints 0 - 5
        for i in range(params.JOINTS_MAX):
            offset = 31 + i * 22
            # Joint current FSM
            mb_frame += list(struct.unpack('>H', b'\x00' + frame[offset:offset + 1]))

            # Joint MC current errors
            mb_frame += list(struct.unpack('>H', b'\x00' + frame[offset + 1:offset + 2]))

            # Joint MC occured errors
            mb_frame += list(struct.unpack('>H', b'\x00' + frame[offset + 2:offset + 3]))

            # Joint current errors
            mb_frame += list(struct.unpack('>H', b'\x00' + frame[offset + 3:offset + 4]))

            # Joint current warnings
            mb_frame += list(struct.unpack('>H', b'\x00' + frame[offset + 4:offset + 5]))

            # Joint internal errors
            mb_frame += list(struct.unpack('>H', frame[offset + 5:offset + 7]))

            # Joint internal occured errors
            mb_frame += list(struct.unpack('>H', frame[offset + 7:offset + 9]))

            # Joint current position
            mb_frame += list(struct.unpack('>HH', frame[offset + 9:offset + 13]))

            #####################################################
            # Save current position for i-th joint
            self.current_config[i] = struct.unpack('>f', frame[offset + 9:offset + 13])[0]
            #####################################################

            # Joint current velocity
            mb_frame += list(struct.unpack('>HH', frame[offset + 13:offset + 17]))

            # Joint current torque
            mb_frame += list(struct.unpack('>HH', frame[offset + 17:offset + 21]))

            # Joint temperature
            mb_frame += list(struct.unpack('>H', b'\x00' + frame[offset + 21:offset + 22]))

        # Joint 6 (reserved for future use)
        mb_frame += [0] * 14

        # Digital input
        mb_frame += [0]

        # Digital output
        mb_frame += [0]

        # Analog inputs 0 - 3
        mb_frame += [0] * 4

        # Analog outputs 0 - 3
        mb_frame += [0] * 4

        return mb_frame

    # def read_jtc_status(self, frame):
    #     output = [jtc_vars.JTCStatus(), jtc_vars.CANStatus(), trajectory.Trajectory(), [jtc_vars.JointStatus()] * 6]
    #     output[0].jtcFsm = frame[9]
    #     output[0].jtcErrors = int.from_bytes(frame[10:12], 'big', signed=False)
    #     output[0].jtcOccurredErrors = int.from_bytes(frame[12:14], 'big', signed=False)
    #     output[0].jtcInitStatus = frame[14]
    #     output[0].jointsInitStatus = frame[15]
    #     # TRAJ
    #     output[2].currentStatus = frame[16]
    #     output[2].numCurrentPoint = int.from_bytes(frame[17:21], 'big', signed=False)
    #     output[0].jtcFricType = frame[21]

    #     output[1].canStatus = frame[22]
    #     output[1].canStatusFlags = int.from_bytes(frame[23:27], 'big', signed=False)
    #     output[1].canOccurredFlags = int.from_bytes(frame[27:31], 'big', signed=False)

    #     for i in range(6):
    #         offset = 31 + i * 22
    #         output[3][i].currentFsm = frame[offset]
    #         output[3][i].mcCurrentError = frame[offset + 1]
    #         output[3][i].mcOccurredError = frame[offset + 2]
    #         output[3][i].currentError = frame[offset + 3]
    #         output[3][i].currentWarning = frame[offset + 4]
    #         output[3][i].internalErrors = int.from_bytes(frame[offset + 5:offset + 7], 'big', signed=False)
    #         output[3][i].internalOccurredErrors = int.from_bytes(frame[offset + 7:offset + 9], 'big', signed=False)

    #         [output[3][i].pos] = struct.unpack('>f', frame[offset + 9:offset + 13])
    #         [output[3][i].vel] = struct.unpack('>f', frame[offset + 13:offset + 17])
    #         [output[3][i].torque] = struct.unpack('>f', frame[offset + 17:offset + 21])

    #         output[3][i].temperature = frame[offset + 22]

    #     return output

    # killme

    def send_command(self, command: enum.Enum, param_list: list = []):
        msg = struct.pack('>BB', params.Host_FT.Header.value, command.value)
        msg += struct.pack('>H', 0) # placeholder for nd
        nd = len(msg) + 2
        for param in param_list:
            if type(param) == float:
                msg += struct.pack('>f', param)
                nd += 4
            else:
                # Currently there are only float and uint8 values as parameter possible
                msg += struct.pack('>B', param)
                nd += 1
        msg = msg.replace(struct.pack('>H', 0), struct.pack('>H', nd), 1)
        crc = get_crc(msg)
        msg += struct.pack('>H', crc)
        self.port.write(msg)

    async def send_trajectory(self, traj):
        seg_num = 0
        # print(f'len(traj.seg): {len(traj.seg)}')
        while seg_num < len(traj.seg):
            # print(f'seg: {[traj.seg[seg_num].strToSend[i] for i in range(10)]}')
            self.port.write(traj.seg[seg_num].strToSend)
            # self.port.flush()
            while True:
                # print('Waiting to get traj status')
                if self.response_info.frame_type == params.Host_FT.Trajectory:
                    if self.response_info.frame_status == params.Host_RxFS.NoError and \
                            self.response_info.data_status == params.Host_RxDS.NoError:
                        # Trajectory successfully sent, send next segment
                        seg_num += 1
                    else:

                        print('[ERROR]: JTC returned error while receiving trajectory. Response info:',
                              self.response_info)
                        seg_num = 0
                    break
                await asyncio.sleep(0.005)
            await asyncio.sleep(0.01)

        # Wait for TES_Stop status meaning that trajectory was successfully received by JTC
        print('Waiting for TES to change to Stop...')
        while True:
            # print(self.response_info)
            if self.response_info.frame_type == params.Host_FT.Null and \
                    self.response_info.frame_status == params.Host_RxFS.Idle and \
                    self.response_info.data_status == params.Host_RxDS.Idle and \
                    self.response_info.tes == params.TES.Stop:
                break
            await asyncio.sleep(0.015)

    async def update_stm_status(self, mb_server):
        while True:
            with self.read_bytes_buffer_mtx:
                ret = self.read_st_frame()
            if ret is not None:
                with mb_server.tele_mtx:
                    mb_server.jtc_status[0] = ret
            await asyncio.sleep(0.005)

    async def msg_dispatch(self, q):
        pass


if __name__ == '__main__':
    rs_com = RSComm()
    serial_data_container = [0]
    asyncio.run(rs_com.update_stm_status(serial_data_container))
