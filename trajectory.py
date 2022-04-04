import struct
import time
from fastcrc import crc16

import params, rs485_com


class TrajectorySegment:
    numOfTraj = 0
    lenPointsInTraj = 0
    lenSegsInTraj = 0
    stepTime = 0
    lenPointsInSeg = 0
    lenBytesInSeg = 0
    numOfSeg = 0
    crc = 0
    strToSend = b''
    value = []


class Trajectory:
    targetStatus = 0
    currentStatus = 0
    numCurrentPoint = 0
    isSend = 0
    numOfTraj = 0
    lenPointsInTraj = 0
    lenSegsInTraj = 0
    stepTime = 10
    seg = []
    value = []
    numOfSegToSend = 0
    trajString = ''
    wasRead = 0

    def prepare_trajectory_to_send(self):
        self.lenPointsInTraj = self.value[0, :, 0].size
        if self.lenPointsInTraj % params.TRAJ_MAXPOINTINSEG == 0:
            self.lenSegsInTraj = int(self.lenPointsInTraj / params.TRAJ_MAXPOINTINSEG)
        else:
            self.lenSegsInTraj = int(self.lenPointsInTraj / params.TRAJ_MAXPOINTINSEG + 1)
        self.seg = []
        for i in range(self.lenSegsInTraj):
            self.seg.append(TrajectorySegment())
            self.prepare_segments(self.seg[i], i)
            self.prepare_segments_to_send(self.seg[i])
        self.numOfSegToSend = 0
        self.isSend = True
        print(f'Number of points in trajectory: {self.lenPointsInTraj}')

    def prepare_segments(self, seg, num):
        seg.numOfTraj = self.numOfTraj
        seg.stepTime = self.stepTime
        seg.lenPointsInTraj = self.lenPointsInTraj
        seg.lenSegsInTraj = self.lenSegsInTraj
        seg.numOfSeg = num

        if seg.lenPointsInTraj % params.TRAJ_MAXPOINTINSEG == 0:
            seg.lenPointsInSeg = params.TRAJ_MAXPOINTINSEG
        else:
            if seg.numOfSeg < (seg.lenSegsInTraj - 1):
                seg.lenPointsInSeg = params.TRAJ_MAXPOINTINSEG
            else:
                seg.lenPointsInSeg = seg.lenPointsInTraj % params.TRAJ_MAXPOINTINSEG

        seg.lenBytesInSeg = 36 * seg.lenPointsInSeg + 14

        start_idx = seg.numOfSeg * params.TRAJ_MAXPOINTINSEG
        finish_idx = start_idx + seg.lenPointsInSeg
        seg.value = self.value[:, start_idx:finish_idx, :]

    def prepare_segments_to_send(self, seg: TrajectorySegment):
        # start = time.time()
        seg.strToSend = bytearray(seg.lenBytesInSeg)
        seg.strToSend[0:1] = int.to_bytes(params.Host_FT.Header.value, 1, 'big', signed=False)
        seg.strToSend[1:2] = int.to_bytes(params.Host_FT.Trajectory.value, 1, 'big', signed=False)
        seg.strToSend[2:4] = int.to_bytes(0, 2, 'big', signed=False)  # placeholder for nd

        seg.strToSend[4:6] = int.to_bytes(seg.numOfTraj, 2, 'big', signed=False)
        seg.strToSend[6:8] = int.to_bytes(seg.numOfSeg, 2, 'big', signed=False)
        seg.strToSend[8:10] = int.to_bytes(seg.lenSegsInTraj, 2, 'big', signed=False)
        seg.strToSend[10:12] = int.to_bytes(seg.stepTime, 2, 'big', signed=False)

        s0 = time.time()
        for i in range(seg.lenPointsInSeg):
            offset = 12 + i * 36
            val = (seg.value[0][i] / params.JOINT_POSMAX * params.MAX_INT16).astype(int)
            seg.strToSend[offset:offset+12] = struct.pack('>hhhhhh', *val)
            val = (seg.value[1][i] / params.JOINT_VELMAX * params.MAX_INT16).astype(int)
            seg.strToSend[offset+12:offset+24] = struct.pack('>hhhhhh', *val)
            val = (seg.value[2][i] / params.JOINT_ACCMAX * params.MAX_INT16).astype(int)
            seg.strToSend[offset+24:offset+36] = struct.pack('>hhhhhh', *val)
        s1 = time.time()
        print(f'append traj: {s1 - s0}')

        nd = len(seg.strToSend) + 2
        seg.strToSend[2:4] = int.to_bytes(nd, 2, 'big', signed=False)
        # crc = rs485_com.get_crc(seg.strToSend, len(seg.strToSend) - 2)
        # crc = self.crc_calc.calculate_checksum(seg.strToSend[:-2])
        s2 = time.time()
        crc = crc16.xmodem(bytes(seg.strToSend[:-2]))
        s3 = time.time()
        print(f'new CRC: {s3 - s2}')
        # print(f'our crc: {crc}; new CRC: {new_crc}')
        seg.strToSend[-2:] = int.to_bytes(crc, 2, 'big', signed=False)

        # print('prepare single seg:', time.time() - start)
