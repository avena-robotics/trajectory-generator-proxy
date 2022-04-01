import struct
import numpy as np
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
        self.seg = [TrajectorySegment()] * self.lenSegsInTraj
        for i in range(self.lenSegsInTraj):
            self.prepare_segments(self.seg[i], i)
            self.prepare_segments_to_send(self.seg[i])
        self.numOfSegToSend = 0
        self.isSend = True

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
        seg.strToSend = b''
        seg.strToSend += int.to_bytes(params.Host_FT.Header.value, 1, 'big', signed=False)
        seg.strToSend += int.to_bytes(params.Host_FT.Trajectory.value, 1, 'big', signed=False)
        seg.strToSend += int.to_bytes(0, 2, 'big', signed=False)  # nd

        seg.strToSend += int.to_bytes(seg.numOfTraj, 2, 'big', signed=False)
        seg.strToSend += int.to_bytes(seg.numOfSeg, 2, 'big', signed=False)
        seg.strToSend += int.to_bytes(seg.lenSegsInTraj, 2, 'big', signed=False)
        seg.strToSend += int.to_bytes(seg.stepTime, 2, 'big', signed=False)

        for i in range(seg.lenPointsInSeg):
            # val = (seg.value[0][i] / params.JOINT_POSMAX * params.MAX_INT16).astype(int)
            # seg.strToSend += struct.pack('>hhhhhh', *val)
            # val = (seg.value[1][i] / params.JOINT_VELMAX * params.MAX_INT16).astype(int)
            # seg.strToSend += struct.pack('>hhhhhh', *val)
            # val = (seg.value[2][i] / params.JOINT_ACCMAX * params.MAX_INT16).astype(int)
            # seg.strToSend += struct.pack('>hhhhhh', *val)
            seg.strToSend += struct.pack('>ffffff', seg.value[0][i])
            seg.strToSend += struct.pack('>ffffff', seg.value[1][i])
            seg.strToSend += struct.pack('>ffffff', seg.value[2][i])


        nd = len(seg.strToSend) + 2
        seg.strToSend = seg.strToSend.replace(seg.strToSend[2:4], int.to_bytes(nd, 2, 'big', signed=False), 1)
        crc = rs485_com.get_crc(seg.strToSend, len(seg.strToSend))
        seg.strToSend += int.to_bytes(crc, 2, 'big', signed=False)
