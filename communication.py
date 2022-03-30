import modbus_server
import rs485_com
import trajectory
import asyncio
import numpy as np
import struct
from typing import DefaultDict, List
from roboticstoolbox.tools import jtraj
    
MAX_VEL = 0.5
TIME_STEP = 0.01 # sec

def convert_to_float(high_16bit: int, low_16bit: int) -> float:
    val = struct.pack('>H', high_16bit) + struct.pack('>H', low_16bit)
    res = struct.unpack('>f', val)
    return res[0]

def calculate_trajectory(start_config: np.array, goal_config: np.array):
    max_diff = np.max(np.abs(goal_config - start_config))
    time = max_diff / MAX_VEL
    time_vec = np.linspace(0, time, int(time / TIME_STEP))
    return jtraj(start_config, goal_config, time_vec)

async def calculate_and_send_traj(flag: List, traj_container: DefaultDict, rs_com: rs485_com.RSComm):
    while True:
        if flag[0]:
            flag[0] = False
            # Deserialize data receive from Modbus to get goal config
            goal_config = [] 
            for i in range(6):
                offset = modbus_server.MB_START_PATH_REG + 3 + i * 2
                goal_config.append(convert_to_float(traj_container[offset], traj_container[offset+1]))
            print('goal_config:', goal_config)
            traj = trajectory.Trajectory()
            traj.value = calculate_trajectory(np.array(rs_com.current_config), np.array(goal_config))
            traj.prepare_trajectory_to_send()
        await asyncio.sleep(0.1)
        

async def main():
    print('Starting proxy program between Codesys and STM JTC')
    rs_com = rs485_com.RSComm()
    mb_serv = modbus_server.ModbusServer()

    print('Adding tasks')
    t1=asyncio.create_task(rs_com.update_stm_status(mb_serv.jtc_status))
    t2=asyncio.create_task(mb_serv.handle_request())
    t3=asyncio.create_task(calculate_and_send_traj(mb_serv.flags, mb_serv.data_store, rs_com))
    
    print('Running...')
    g = await asyncio.gather(t1, t2, t3)


a = asyncio.run(main())