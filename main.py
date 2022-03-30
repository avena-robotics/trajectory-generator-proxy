import numpy as np
import roboticstoolbox as rtb
import trajectory



if __name__ == '__main__':
    start_config=np.array([0,0,0,0,0,0])
    goal_config=np.array([1,2.137,0.3,-0.69,-1,0.420])
    traj = rtb.jtraj(start_config,goal_config,100)
    t = trajectory.Trajectory()
    t.value=np.array([traj.q,traj.qd,traj.qdd])
    t.prepare_trajectory_to_send()