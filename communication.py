from concurrent.futures import ThreadPoolExecutor
import modbus_server
import rs485_com
import trajectory
import asyncio
import numpy as np
import struct
import params
import time
from typing import DefaultDict, List
from roboticstoolbox.tools import jtraj

np.set_printoptions(suppress=True)   

def convert_to_float(high_16bit: int, low_16bit: int) -> float:
    val = struct.pack('>H', high_16bit) + struct.pack('>H', low_16bit)
    res = struct.unpack('>f', val)
    return res[0]

def calculate_trajectory(start_config: np.array, goal_config: np.array):
    max_diff = np.max(np.abs(goal_config - start_config))
    time = max_diff / params.MAX_VEL
    time_vec = np.linspace(0, time, int(time / params.TIME_STEP))
    rtb_traj = jtraj(start_config, goal_config, time_vec)
    traj = np.array([rtb_traj.q, rtb_traj.qd, rtb_traj.qdd])
    # print(traj)
    return traj

async def calculate_and_send_traj(flag: List, traj_container: DefaultDict, rs_com: rs485_com.RSComm, executor: ThreadPoolExecutor):
    while True:
        if flag[0]:
            print('Calculate and send trajectory...')
            flag[0] = False
            # Deserialize data receive from Modbus to get goal config
            goal_config = [] 
            for i in range(params.JOINTS_MAX):
                offset = modbus_server.MB_START_PATH_REG + 3 + i * 2
                goal_config.append(convert_to_float(traj_container[offset], traj_container[offset+1]))
            print('goal_config:', goal_config)
            traj = trajectory.Trajectory()
            
            ###
            # traj.value = calculate_trajectory(np.array(rs_com.current_config), np.array(goal_config))
            ###
            start = time.time()
            fut = executor.submit(calculate_trajectory,np.array(rs_com.current_config), np.array(goal_config))
            while fut.running():
                await asyncio.sleep(0.05)
            traj.value=fut.result()
            print('calculate_trajectory:', time.time() - start)

            start = time.time()
            fut = executor.submit(traj.prepare_trajectory_to_send)
            while fut.running():
                await asyncio.sleep(0.05)
            # traj.prepare_trajectory_to_send()
            print('prepare_trajectory_to_send:', time.time() - start)

            print('Sending trajectory to JTC')
            await rs_com.send_trajectory(traj)
            print('Trigger trajectory execution')
            rs_com.execute_trajectory()
            print('Trajectory successfully send')
        await asyncio.sleep(0.1)
        

async def main():
    print('Starting proxy program between Codesys and STM JTC')
    rs_com = rs485_com.RSComm()
    mb_serv = modbus_server.ModbusServer()
    traj_calc_exec = ThreadPoolExecutor(max_workers=1)

    print('Adding tasks')
    tasks = []
    tasks.append(asyncio.create_task(rs_com.update_stm_status(mb_serv)))
    tasks.append(asyncio.create_task(mb_serv.handle_request()))
    tasks.append(asyncio.create_task(calculate_and_send_traj(mb_serv.flags, mb_serv.data_store, rs_com, traj_calc_exec)))
    
    # mb_thread = threading.Thread(target=mb_serv.handle_request)
    # mb_thread.start()

    print('Running...')
    g = await asyncio.gather(*tasks)


a = asyncio.run(main())