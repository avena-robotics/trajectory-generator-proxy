from concurrent.futures import ThreadPoolExecutor
import modbus_server
import rs485_com
import trajectory
import asyncio
import numpy as np
import struct
import params
import time
from roboticstoolbox.tools import jtraj

np.set_printoptions(suppress=True)


def convert_to_float(high_16bit: int, low_16bit: int) -> float:
    val = struct.pack('>H', high_16bit) + struct.pack('>H', low_16bit)
    res = struct.unpack('>f', val)
    return res[0]


def calculate_trajectory(start_config: np.array, goal_config: np.array, max_speed: float) -> np.ndarray:
    '''
    Joint which has the longest angular distance from start to goal value,
    moves the fastest and it determines full time of 
    '''
    max_diff = np.max(np.abs(goal_config - start_config))
    max_speed = min(max_speed, params.MAX_VEL)
    time = max_diff / max_speed
    time_vec = np.linspace(0, time, int(time / params.TIME_STEP))
    rtb_traj = jtraj(start_config, goal_config, time_vec)
    print(f'Trajectory max speed: {max(rtb_traj.qd.max(), abs(rtb_traj.qd.min()))} rad/s')
    print(f'Trajectory max acc: {max(rtb_traj.qdd.max(), abs(rtb_traj.qdd.min()))} rad/s^2')
    traj = np.array([rtb_traj.q, rtb_traj.qd, rtb_traj.qdd])
    return traj


async def handle_flags(mb_server: modbus_server.ModbusServer, rs_com: rs485_com.RSComm, executor: ThreadPoolExecutor):
    while True:
        if mb_server.flags.send_waypoints:
            mb_server.flags.send_waypoints = False
            await calculate_and_send_traj(mb_server, rs_com, executor)
        elif mb_server.flags.send_control_word:
            mb_server.flags.send_control_word = False
            await send_control_word(mb_server, rs_com)
        await asyncio.sleep(0.1)


async def calculate_and_send_traj(mb_server: modbus_server.ModbusServer, rs_com: rs485_com.RSComm,
                                  executor: ThreadPoolExecutor):
    print('Calculate and send trajectory...')
    # Deserialize data receive from Modbus to get goal config
    mb_server.path_mtx.acquire()
    wp_num = mb_server.data_store[modbus_server.MB_START_PATH_REG + 1]
    print('Received', wp_num, 'waypoints')
    traj = trajectory.Trajectory()
    start_config = np.array(rs_com.current_config)
    for wp in range(wp_num):
        wp_offset = 2 + wp * 21
        goal_config = []
        for i in range(params.JOINTS_MAX):
            offset = modbus_server.MB_START_PATH_REG + 1 + i * 2 + wp_offset
            goal_config.append(convert_to_float(mb_server.data_store[offset], mb_server.data_store[offset + 1]))
        print(f'Goal config {wp + 1} / {wp_num}: {goal_config}')
        max_speed = convert_to_float(mb_server.data_store[modbus_server.MB_START_PATH_REG + 13 + wp_offset],
                                     mb_server.data_store[modbus_server.MB_START_PATH_REG + 13 + wp_offset + 1])
        print('Max speed:', max_speed, 'rad/s')

        s0 = time.time()
        fut = executor.submit(calculate_trajectory, start_config, np.array(goal_config), max_speed)
        start_config = np.array(goal_config)
        while fut.running():
            await asyncio.sleep(0.05)
        if type(traj.value) is np.ndarray:
            traj.value = np.append(traj.value, fut.result(),1)
        else:
            traj.value = fut.result()
        print('calculate_trajectory:', time.time() - s0, 's')

        await asyncio.sleep(0.02)

    mb_server.path_mtx.release()

    if np.size(traj.value,1) > params.TRAJ_MAX_POINTS:
        print('Trajectory is too long.')
        return 'dupa'

    s0 = time.time()
    fut2 = executor.submit(traj.prepare_trajectory_to_send)
    while fut2.running():
        await asyncio.sleep(0.05)
    fut2.result()
    print('prepare_trajectory_to_send:', time.time() - s0, 's')
    await asyncio.sleep(0.02)

    print('Sending trajectory to JTC')
    await rs_com.send_trajectory(traj)
    print('Trajectory successfully send')


async def send_control_word(mb_server: modbus_server.ModbusServer, rs_com: rs485_com.RSComm):
    command = params.Host_FT(0)
    param_list = []
    mb_server.ctrl_mtx.acquire()
    for i in range(modbus_server.MB_START_CONTROL_REG, modbus_server.MB_END_CONTROL_REG + 1):
        if mb_server.control_words[i]:
            if i == modbus_server.MB_START_CONTROL_REG:
                command = params.Host_FT.ClearCurrentErrors
            elif i == modbus_server.MB_START_CONTROL_REG + 1:
                command = params.Host_FT.ClearOccuredErrors
            elif i == modbus_server.MB_START_CONTROL_REG + 2:
                command = params.Host_FT.FrictionTableUseDefault
            elif i == modbus_server.MB_START_CONTROL_REG + 3:
                command = params.Host_FT.PidParamUseDefault
            elif i == modbus_server.MB_START_CONTROL_REG + 4:
                command = params.Host_FT.ArmModelUseDefault
            elif i == modbus_server.MB_START_CONTROL_REG + 5:
                command = params.Host_FT.TeachingModeEnable
            elif i == modbus_server.MB_START_CONTROL_REG + 6:
                command = params.Host_FT.TeachingModeDisable
            elif i == modbus_server.MB_START_CONTROL_REG + 7:
                command = params.Host_FT.FrictionPolynomialUseDefault
            elif i == modbus_server.MB_START_CONTROL_REG + 9:
                print('Sending TES with value:', params.TES(mb_server.control_words[i]))
                command = params.Host_FT.TrajSetExecStatus
                param_list.append(params.TES(mb_server.control_words[i]).value)
    mb_server.ctrl_mtx.release()
    rs_com.send_command(command, param_list)


async def main():
    print('Starting proxy program between Codesys and STM JTC')
    rs_com = rs485_com.RSComm()
    mb_serv = modbus_server.ModbusServer()
    traj_calc_exec = ThreadPoolExecutor(max_workers=1)

    print('Adding tasks')
    tasks = []
    tasks.append(asyncio.create_task(rs_com.update_stm_status(mb_serv)))
    tasks.append(asyncio.create_task(handle_flags(mb_serv, rs_com, traj_calc_exec)))

    print('Running...')
    g = await asyncio.gather(*tasks)


a = asyncio.run(main())
