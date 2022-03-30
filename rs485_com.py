import ctypes
import enum
import math
import struct

import serial
import os
import time
import asyncio
import params, jtc_vars, trajectory


def get_crc(frame, nbytes):
    crc = 0
    for byte in range(nbytes):
        crc = ctypes.c_uint16(ctypes.c_uint16(crc).value ^ (frame[byte] << 8)).value
        for bit in range(8):
            if crc & 0x8000:
                crc = ctypes.c_uint16(ctypes.c_uint16(crc << 1).value ^ 0x1021).value
            else:
                crc = ctypes.c_uint16(crc << 1).value
    return ctypes.c_uint16(crc).value


class RSComm:

    def __init__(self):
        self.port = serial.Serial(port='/dev/ttyACM0', baudrate=115200, parity=serial.PARITY_NONE,
                                  stopbits=1, bytesize=8, timeout=1, write_timeout=1, rtscts=1)
        self.read_bytes_buffer = b''
        self.jtc_status = []

    def read_st_frame(self):

        # while True:
        self.read_bytes_buffer += self.port.read_all()

        if len(self.read_bytes_buffer) < 4:
            return
        if len(self.read_bytes_buffer) > params.COMBUFREADMAX:
            self.read_bytes_buffer = b''
            return
        if (header_idx := self.read_bytes_buffer.find(params.Host_FT['Header'].value)) > params.COMBUFREADMAX - 4:
            self.read_bytes_buffer = b''
            return
        nd = int.from_bytes(self.read_bytes_buffer[header_idx + 2:header_idx + 4], 'big', signed=False)
        if len(self.read_bytes_buffer) < (header_idx+nd):
            return
        if nd < 4:
            return
        if nd > params.COMBUFREADMAX:
            self.read_bytes_buffer = b''
            return
        read_buffer = []
        for i in range(nd):
            read_buffer.append(self.read_bytes_buffer[header_idx + i])

        self.jtc_status = self.read_jtc_status(self.read_bytes_buffer[header_idx:header_idx + nd])
        ret=self.translate_to_modbus(self.read_bytes_buffer[header_idx:header_idx + nd])
        self.translate_frame(read_buffer[header_idx:header_idx + nd])
        self.read_bytes_buffer = self.read_bytes_buffer[header_idx + nd:]

        return ret
        # if len(read_bytes_buffer) <= 4:
        #     return

    def translate_frame(self, frame):
        response = []
        response.append(params.Host_FT(frame[4]).name)
        response.append(params.Host_RxFS(frame[5]).name)
        response.append(params.Host_RxDS(frame[6]).name)
        if frame[4]:
            print(response)

    def translate_to_modbus(self,frame):
        mb_frame=[]
        mb_frame+=list(struct.unpack('>H',b'\x00'+frame[9:10]))
        mb_frame+=list(struct.unpack('>HH',frame[10:14]))
        mb_frame+=list(struct.unpack('>H',b'\x00'+frame[14:15]))
        mb_frame+=list(struct.unpack('>H',b'\x00'+frame[15:16]))
        mb_frame+=list(struct.unpack('>H',b'\x00'+frame[16:17]))
        mb_frame+=list(struct.unpack('>HH',frame[17:21]))
        mb_frame+=list(struct.unpack('>H',b'\x00'+frame[21:22]))
        mb_frame+=list(struct.unpack('>H',b'\x00'+frame[22:23]))
        mb_frame+=list(struct.unpack('>HH',frame[23:27]))
        mb_frame+=list(struct.unpack('>HH',frame[27:31]))

        for i in range(6):
            offset = 31 + i * 22
            mb_frame+=list(struct.unpack('>H',b'\x00'+frame[offset:offset+1]))
            mb_frame+=list(struct.unpack('>H',b'\x00'+frame[offset+1:offset+2]))
            mb_frame+=list(struct.unpack('>H',b'\x00'+frame[offset+2:offset+3]))
            mb_frame+=list(struct.unpack('>H',b'\x00'+frame[offset+3:offset+4]))
            mb_frame+=list(struct.unpack('>H',b'\x00'+frame[offset+4:offset+5]))
            mb_frame+=list(struct.unpack('>HH',frame[offset+5:offset+9]))
            mb_frame+=list(struct.unpack('>ff',frame[offset+9:offset+17]))
            mb_frame+=list(struct.unpack('>f',frame[offset+17:offset+21]))
            mb_frame+=list(struct.unpack('>H',b'\x00'+frame[offset+21:offset+22]))

        mb_frame+=list(struct.unpack('>H',frame[163:165]))

        return mb_frame


    def read_jtc_status(self, frame):
        output = [jtc_vars.JTCStatus(), jtc_vars.CANStatus(), trajectory.Trajectory(), [jtc_vars.JointStatus()] * 6]
        output[0].jtcFsm = frame[9]
        output[0].jtcErrors = int.from_bytes(frame[10:12], 'big', signed=False)
        output[0].jtcOccurredErrors = int.from_bytes(frame[12:14], 'big', signed=False)
        output[0].jtcInitStatus = frame[14]
        output[0].jointsInitStatus = frame[15]
        # TRAJ
        output[2].currentStatus = frame[16]
        output[2].numCurrentPoint = int.from_bytes(frame[17:21], 'big', signed=False)
        output[0].jtcFricType = frame[21]

        output[1].canStatus = frame[22]
        output[1].canStatusFlags = int.from_bytes(frame[23:27], 'big', signed=False)
        output[1].canOccurredFlags = int.from_bytes(frame[27:31], 'big', signed=False)

        for i in range(6):
            offset = 31 + i * 22
            output[3][i].currentFsm = frame[offset]
            output[3][i].mcCurrentError = frame[offset + 1]
            output[3][i].mcOccurredError = frame[offset + 2]
            output[3][i].currentError =frame[offset +3]
            output[3][i].currentWarning =frame[offset +4]
            output[3][i].internalErrors = int.from_bytes(frame[offset + 5:offset + 7], 'big', signed=False)
            output[3][i].internalOccurredErrors = int.from_bytes(frame[offset + 7:offset + 9], 'big', signed=False)


            [output[3][i].pos] = struct.unpack('>f', frame[offset + 9:offset + 13])
            [output[3][i].vel] = struct.unpack('>f', frame[offset + 13:offset + 17])
            [output[3][i].torque] = struct.unpack('>f', frame[offset + 17:offset + 21])

            output[3][i].temperature = frame[offset + 22]

        return output

    # killme

    def send_command(self, command):
        msg = struct.pack('>BB', params.Host_FT.Header.value, command.value)
        nd = len(msg) + 4
        msg += struct.pack('>H', nd)
        crc = get_crc(msg, len(msg))
        msg += struct.pack('>H', crc)
        self.port.write(msg)

    def send_trajectory(self, traj):
        self.port.write(traj.numOfSegToSend.strToSend)

    async def update_stm_status(self,status_container):
        while (True):
            ret = self.read_st_frame()
            if ret is not None:
                status_container[0]=ret
            await asyncio.sleep(0.015)

    async def msg_dispatch(self, q):
        pass

    async def main(self):
        pass


if __name__ == '__main__':
    pass