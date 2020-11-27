import modern_robotics as mr
import numpy as np

'''
This file inlcudes code for milestone2 which computes the trajectory as a list of N SE(3) matrices
and save it as a .csv file so it can be simulated in coppeliaSim scene.

Instruction for usage:
User can simply run the code and the program will generate a .csv file called 'End_eff_sim' which can be used for simulation 
'''  

def matrix2list(Traj,traj,N,gripper_state):
    '''This function converts the transformation matrix into N x 13 matrices list '''
    list = np.zeros((N,13), dtype=float)
    for i in range(N):
        list[i][0] = traj[i][0][0]
        list[i][1] = traj[i][0][1]
        list[i][2] = traj[i][0][2]
        list[i][3] = traj[i][1][0]
        list[i][4] = traj[i][1][1]
        list[i][5] = traj[i][1][2]
        list[i][6] = traj[i][2][0]
        list[i][7] = traj[i][2][1]
        list[i][8] = traj[i][2][2]
        list[i][9] = traj[i][0][3]
        list[i][10] = traj[i][1][3]
        list[i][11] = traj[i][2][3]
        list[i][12] = gripper_state
        Traj.append(list[i].tolist())
    return Traj




def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff,k):
    '''
    Computes the trajectory as a list of N SE(3) matrices and generates a .csv file for simulation

    Input:
    :parameter Tse_initial: The initial configuration of the end-effector relative to ground
    :parameter Tsc_initial: THe initial cofiguration of the cube relative to ground
    :parameter Tsc_final: The final configuration of the cube relative to ground 
    :parameter Tce_grasp: The configuration of the end-effector relative to the cube when grasping
    :parameter Tce_standoff: The configuration of the end_effector's standoff relative to the cube before and after grasping the cube 
    
    
    
    Output: 
    A discretized trajctory lists as N x 13 matrices. The first 12 entries are [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz] based on
    the original transformation matrix:
    T = np.array([[r11, r12, r13, px],
                  [r21, r22, r23, py],
                  [r31, r32, r33, pz],
                  [0, 0, 0, 1]])
    The last entry is the gripper state which is eithr 0(open) or 1(closed)
  
    '''


    ''' Create an empty list for Trajectory'''
    Traj = []
    
    '''
    :parameter Tse_init_standoff: The initial configuration of the end_effector's standoff relative to the ground before and after grasping the cube 
    :parameter Tse_init_grasp: The initial configuration of the end-effector relative to the ground when grasping
    :parameter Tse_fin_standoff: The final configuration of the end_effector's standoff relative to the ground before and after grasping the cube 
    :parameter Tse_fin_grasp: The final configuration of the end-effector relative to the ground when grasping
    :parameter Tf: Total time of the motion in seconds from rest to rest
    :parameter N: The number of points N > 1 (Start and stop) in the discrete representation of the trajectory
    :parameter method: The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling and 5 indicates quintic
                   (fifth-order polynomial) time scaling
    :parameter gripper_state: 0 = open , 1 = close 
    '''
    
    Tse_init_standoff = np.dot(Tsc_initial,Tce_standoff)
    Tse_init_grasp = np.dot(Tsc_initial,Tce_grasp)
    Tse_fin_standoff = np.dot(Tsc_final,Tce_standoff)
    Tse_fin_grasp = np.dot(Tsc_final,Tce_grasp)

    Tf = 3
    N = 200
    method = 5
    gripper_state = 0
    
    '''1st step: Move to initial standoff position'''    
    traj1 = np.asarray(mr.CartesianTrajectory(Tse_initial,Tse_init_standoff,Tf,N,method))
    Traj = matrix2list(Traj,traj1,N,gripper_state)

    '''2nd step: Move to initial grasp position '''
    N = 100
    traj2 = np.asarray(mr.CartesianTrajectory(Tse_init_standoff,Tse_init_grasp,Tf,N,method))
    Traj = matrix2list(Traj,traj2,N,gripper_state)

    '''3rd step: Close the gripper'''
    N = 100
    gripper_state = 1
    traj3 = np.asarray(mr.CartesianTrajectory(Tse_init_grasp,Tse_init_grasp,Tf,N,method))
    Traj = matrix2list(Traj,traj3,N,gripper_state)

    '''4th step: Back to initial standoff position'''
    N = 100
    traj4 = np.asarray(mr.CartesianTrajectory(Tse_init_grasp,Tse_init_standoff,Tf,N,method))
    Traj = matrix2list(Traj,traj4,N,gripper_state)

    '''5th step: Move to the final standoff position'''
    N = 400
    traj5 = np.asarray(mr.CartesianTrajectory(Tse_init_standoff,Tse_fin_standoff,Tf,N,method))
    Traj = matrix2list(Traj,traj5,N,gripper_state)


    '''6th step: Move to the final grasp position'''
    N = 100
    traj6 = np.asarray(mr.CartesianTrajectory(Tse_fin_standoff,Tse_fin_grasp,Tf,N,method))
    Traj = matrix2list(Traj,traj6,N,gripper_state)


    '''7th step: Open the gripper'''
    N = 100
    gripper_state = 0
    traj7 = np.asarray(mr.CartesianTrajectory(Tse_fin_grasp,Tse_fin_grasp,Tf,N,method))
    Traj = matrix2list(Traj,traj7,N,gripper_state)



    '''8th step: Back to the final standoff position'''
    N = 100
    traj8 = np.asarray(mr.CartesianTrajectory(Tse_fin_grasp,Tse_fin_standoff,Tf,N,method))
    Traj = matrix2list(Traj,traj8,N,gripper_state)


    return Traj


'''Example input from website'''

Tsb = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]])

Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])
M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])

Tse_ini = Tsb.dot(Tb0).dot(M0e)


Tsc_ini = np.array([[1, 0, 0, 1],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
Tsc_fin = np.array([[0, 1, 0, 0],
                    [-1, 0, 0, -1],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]]) 



Tce_grp = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2, 0],
					[ 0, 1, 0, 0],
					[-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
					[ 0, 0, 0, 1]])

Tce_sta = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2,   0],
					[ 0, 1, 0,   0],
					[-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0.15],  
					[ 0, 0, 0,   1]])
k = 1


Trajectory = TrajectoryGenerator(Tse_ini, Tsc_ini, Tsc_fin, Tce_grp, Tce_sta, k)
np.savetxt('End_effector_sim.csv', Trajectory, delimiter=',')
#print(Trajectory)