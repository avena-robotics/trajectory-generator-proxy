import numpy as np
import roboticstoolbox as rtb
# import trajectory

MAX_VEL = 0.5
TIME_STEP = 0.01 # sec

if __name__ == '__main__':
    start_config=np.array([0,0,0,0,0,0])
    goal_config=np.array([1,2.137,0.3,-0.69,-1,0.420])

    max_diff = np.max(np.abs(goal_config - start_config))
    time = max_diff / MAX_VEL
    time_vec = np.linspace(0, time, int(time / TIME_STEP))

    print(max_diff, time, time_vec)

    traj = rtb.jtraj(start_config, goal_config, time_vec)

    # print(traj)
    # t = trajectory.Trajectory()
    # t.value=np.array([traj.q,traj.qd,traj.qdd])
    # t.prepare_trajectory_to_send()
