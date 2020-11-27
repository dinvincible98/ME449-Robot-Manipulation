import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
import logging
from numpy import sin, cos

''' This code file inlcudes and integrates milestone 1-3 to accomplish youBot simulatiom task, user can simply run
this file and get the .csv file for Coppeliasim scene6. 
'''
''' Initializing log file '''
LOG_FILENAME = 'newTask.log'
logging.basicConfig(filename=LOG_FILENAME, level=logging.DEBUG)


def NextState(current_config, speed, timestep, max_omg):
    '''
    This function compute the robot configuration in next timestep
    Input:
    :param current_config:A 12-vector representing the current configuration of the robot 
    (3 variables for the chassis configuration, 5 variables for the arm configuration, 
    and 4 variables for the wheel angles).

    :param speed:A 9-vector of controls indicating the arm joint speeds theta_dot (5 variables) 
    and the wheel speeds u (4 variables).

    :param timestep:A timestep

    :param max_omg:A positive real value indicating the maximum angular speed of the arm joints and the wheels. 

    Output:
    :param next_config: A 12-vector representing the configuration of the robot after delta t(timestep) 


    Note: The function NextState is based on a simple first-order Euler step, i.e.,
    
    new arm joint angles = (old arm joint angles) + (joint speeds) * Δt
    new wheel angles = (old wheel angles) + (wheel speeds) * Δt
    new chassis configuration is obtained from odometry, as described in Chapter 13.4
    '''
    

    # Define variables
    r = 0.0475                     # Radius of wheel
    l = 0.235                     
    w = 0.15

    # Get configuration for chasis, arm and wheel from input current_config
    chasis_config = np.array([[current_config[0]],
                             [current_config[1]],
                             [current_config[2]]])
    
    arm_config = np.array([[current_config[3]],
                           [current_config[4]],
                           [current_config[5]],
                           [current_config[6]],
                           [current_config[7]]])
    
    
    wheel_config = np.array([[current_config[8]],
                             [current_config[9]],
                             [current_config[10]],
                             [current_config[11]]])            

    #Check if the arm and wheel speed is under limit
    speed_update = np.zeros(9)
    for i in range(len(speed)):
        if speed[i] >= 0:
            if speed[i] > max_omg:
                speed[i] = max_omg
                speed_update[i] = speed[i]
            else:
                speed_update[i] = speed[i]
        else:
            if abs(speed[i]) > max_omg:
                speed[i] = -max_omg
                speed_update[i] = speed[i]
            else:
                speed_update[i] = speed[i]

    



    # Compute next joint configuration
    #Joint speed
    theta_dot = np.array([[speed_update[0]],
                          [speed_update[1]],
                          [speed_update[2]],
                          [speed_update[3]],
                          [speed_update[4]]])    
    det_theta = theta_dot * timestep

    next_joint_angles = arm_config + det_theta

    # Compute next wheel configuration
    # Wheel speed
    u = np.array([[speed_update[5]],
                  [speed_update[6]],
                  [speed_update[7]],
                  [speed_update[8]]])
    det_u = u * timestep

    next_wheel_angles = wheel_config + det_u


    # Compute next chasis configuration
    # From Chapter 13.4, the chasis configuration is obtained from odometry
    # V_b = F * det_u
    
    F = r / 4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                          [1, 1, 1, 1],
                          [-1, 1, -1, 1]])
    V_b = np.dot(F,det_u).reshape(3,)

    
    w_bz = V_b[0]
    v_bx = V_b[1]
    v_by = V_b[2]

    det_qb = np.zeros(3,)
    
    if w_bz < 1e-3:
        det_qb = np.array([[0],
                           [v_bx],
                           [v_by]])
    else:
        det_qb = np.array([[w_bz],
                           [v_bx * sin(w_bz) + v_by * (cos(w_bz) - 1)/w_bz],
                           [v_by * sin(w_bz) + v_bx * (1 - cos(w_bz))/w_bz]])

    # transform matrix for det_qb
    
    trans = np.array([[1, 0, 0],
                      [0, cos(current_config[0]), -sin(current_config[0])],
                      [0, sin(current_config[0]), cos(current_config[0])]])
    det_q = np.dot(trans, det_qb)
 
    
    next_chasis_config = chasis_config + det_q
    # print(next_chasis_config)

    next_config = np.concatenate((next_chasis_config,next_joint_angles,next_wheel_angles), axis=None)
    
    return next_config

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
    :param k: The number of trajectory reference configurations per 0.01 seconds(1)
    
    
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
    N = 500
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
    N = 600
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

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, timestep,integral,robot_config):
    '''This function calculate the kinematic task-space feedforward plus feedback control law
    Input:
    param: X: The current actual end-effector configuration. 
    param: Xd: The current end-effector reference configuration. 
    param: Xd_next: The end-effector reference configuration at the next timestep in the reference 
                    trajectory delta t later.
    param: Kp: The P gain matrix
    param: Ki: The I gain matrix
    param: timestep: A timestep delta t between reference trajectory configurations

    Output:
    param: commanded_V: The commanded end-effector twist expressed in the end-effector frame 
    '''

    # Define variables
    r = 0.0475                     # Radius of wheel
    l = 0.235                     
    w = 0.15

    T_b0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])
    M_Oe = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]])
    B_list = np.array([[0, 0, 1, 0, 0.033, 0],
                       [0, -1, 0, -0.5076, 0, 0],
                       [0, -1, 0, -0.3526, 0, 0],
                       [0, -1, 0, -0.2176, 0, 0],
                       [0, 0, 1, 0, 0, 0]]).T
    
    theta_list = robot_config[3:8]
    F_6 = r / 4 * np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1],
                            [0, 0, 0, 0]])
    
    
    T_0e = mr.FKinBody(M_Oe,B_list,theta_list)
    Vd = mr.se3ToVec((1/timestep) * mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next)))
    #print(Vd)

    Ad_xxd = mr.Adjoint(np.dot(mr.TransInv(X),Xd))
    #print(Ad_xxd)
    Ad_xxd_Vd = np.dot(Ad_xxd,Vd)
    #print(Ad_xxd_Vd)
    X_err = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
    #print(X_err)
    
    #Define global variable integral

    V = Ad_xxd_Vd + np.dot(Kp,X_err) + np.dot(Ki, integral+ X_err*timestep)
    integral += X_err * timestep
    #print(V)

    #From Chapter 13.5
    J_arm = mr.JacobianBody(B_list,theta_list)
    #print(J_arm) 
    a = np.dot(mr.TransInv(T_0e),mr.TransInv(T_b0))
    J_base = np.dot(mr.Adjoint(a),F_6)
    #print(J_base)
    Je = np.concatenate((J_base,J_arm),axis=1) 
    #print(Je)
    Je_inv = np.linalg.pinv(Je)
    #print(Je_inv.shape)
    
    command_V = Je_inv.dot(V)                                     #Commanded Twist
    #print(command_V)
    return command_V,X_err



'''Main function'''

def main(Tsc_ini,Tsc_fin,Kp,Ki,robot_config):
    '''The main fucntion includes milestone1 - 3. It plans a trajectory for the end-effector of the 
        youBot, uses the feedback control to drive the youBot and let it pick up the cube at a specific location 
        and then carry to the desired location and put it down.
    Input:  
        :param  Tsc_ini: The cube's initial configruation relative to the ground
        :param  Tsc_fin: The cube's final configruation relative to the ground
        :param  Kp: P controller gain
        :param  Ki: I controller gain
        :param  robot_config: The initial configuration of the youBot

    Other parameters:
        :param Tse_ini: The initial configuration of the end-effector relative to ground
        :param Tce_grp: The configuration of the end-effector relative to the cube when grasping
        :param Tce_sta: The configuration of the end_effector's standoff relative to the cube before and after grasping the cube 
    Output:
        : A list of 1700 x 13 matrices saved as a .csv file. The first 12 entries of each row represents
        the transformation matrix Tse at that time instant(dt = 0.01) in SE(3). The last entry represents 
        the gripper state (0 = open, 1 = close) 
        : A list of 1699 X 6 matrices saved as .csv file(X_err)
        : A log file (Based on mode) 
    '''

    #Define initial variables

    Tse_ini = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])

    Tce_grp = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2, 0],
                        [ 0, 1, 0, 0],
                        [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
                        [ 0, 0, 0, 1]])

    Tce_sta = np.array([[ -np.sqrt(2)/2, 0, np.sqrt(2)/2,   0],
                        [ 0, 1, 0,   0],
                        [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0.15],  
                        [ 0, 0, 0,   1]])
	
    Tb0 = np.array([[ 1, 0, 0, 0.1662],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.0026],
				    [ 0, 0, 0,   1]])
    
    M0e = np.array([[ 1, 0, 0, 0.033],
				    [ 0, 1, 0,   0],
				    [ 0, 0, 1, 0.6546],
				    [ 0, 0, 0,   1]])
    
    B_list = np.array([[0, 0, 1, 0, 0.033, 0],
                       [0, -1, 0, -0.5076, 0, 0],
                       [0, -1, 0, -0.3526, 0, 0],
                       [0, -1, 0, -0.2176, 0, 0],
                       [0, 0, 1, 0, 0, 0]]).T
    k = 1
    timestep = 0.01
    max_omg = 15 

    #Create a list of trajectory for robot and an array for X_err
    robot_traj = []
    X_err_arr = []

    #Define global variable integral
    global integral
    integral =  np.zeros((6,),dtype=float)


    '''1st step: Call the TrajectoryGenereator function to get desired configuration of 
       the end effector of the robot'''
    # Trajectory = TrajectoryGenerator(Tse_ini, Tsc_ini, Tsc_fin, Tce_grp, Tce_sta, k)
    # np.savetxt('End_effector_sim2.csv', Trajectory, delimiter=',')
    traj = np.asarray(TrajectoryGenerator(Tse_ini,Tsc_ini,Tsc_fin,Tce_grp,Tce_sta,k))
    # print(traj)
    '''2nd step: Append the initial configuration of robot to the robot_traj'''
    robot_traj.append(robot_config.tolist())

    '''3rd step: Looping '''
    for i in range(1699):                          # 14s 
        theta_list = robot_config[3:8]
        Xd = np.array([[traj[i][0],traj[i][1],traj[i][2],traj[i][9]],
                       [traj[i][3],traj[i][4],traj[i][5],traj[i][10]],
                       [traj[i][6],traj[i][7],traj[i][8],traj[i][11]],
                       [0, 0, 0, 1]])
        Xd_next = np.array([[traj[i+1][0],traj[i+1][1],traj[i+1][2],traj[i+1][9]],
                            [traj[i+1][3],traj[i+1][4],traj[i+1][5],traj[i+1][10]],
                            [traj[i+1][6],traj[i+1][7],traj[i+1][8],traj[i+1][11]],
                            [0, 0, 0, 1]])
		
        Tsb = np.array([[cos(robot_config[0]), -sin(robot_config[0]), 0, robot_config[1]],
				   	    [sin(robot_config[0]), cos(robot_config[0]), 0, robot_config[2]], 
				        [              0       ,          0            , 1,        0.0963 ],
				        [              0       ,          0            , 0,             1]])
        T0e = mr.FKinBody(M0e,B_list,theta_list)

        Tbe = np.dot(Tb0,T0e)
        X = np.dot(Tsb,Tbe)

        
        '''4th step: Get the commanded Twist and error vector from FeedbackControl function'''
        
        command_V, X_err =  FeedbackControl(X,Xd,Xd_next,Kp,Ki,timestep,integral,robot_config)

        X_err_arr.append(X_err.tolist())
        #print(X_err_arr)
        #print("@",command_V)
        wheel_speed = command_V[:4]
        #print("###",wheel_speed)
        arm_speed = command_V[4:9]
        #print("---",arm_speed)
    
        '''5th step: Update the robot_config'''    
        # Update robot_config based on feedback control
        # The input command of Nextstate and the returned command of feedback control is fliped
        # Speed = [Arm_speed+wheel_speed] 
        control = np.concatenate((arm_speed,wheel_speed),axis=None)
        # print("^^^",control) 
        robot_config = NextState(robot_config[:12],control,timestep,max_omg)
        robot_current_traj = np.concatenate((robot_config,traj[i][12]),axis=None)
        robot_traj.append(robot_current_traj.tolist())

    '''6th step: Save the X_err vector'''
    logging.debug("Generating X_err file")
    np.savetxt("X_err_newTask.csv", X_err_arr,delimiter=',')

    '''7th step: Generating .csv file for robot'''
    logging.debug("Generating .csv file ")
    np.savetxt("newTask.csv",robot_traj,delimiter=',')
    
    '''8th step: Plot X_err'''
    logging.debug('Plotting the X_err')
    
    t = np.linspace(0,16.99,1699)
    x = np.asarray(X_err_arr)
    plt.plot(t,x[:,0])
    plt.plot(t,x[:,1])    
    plt.plot(t,x[:,2])
    plt.plot(t,x[:,3])  
    plt.plot(t,x[:,4])
    plt.plot(t,x[:,5])
    plt.title("X_err plot")
    plt.xlim([0,17])
    plt.xlabel("Time(s)")
    plt.ylabel("Error")
    plt.legend([r'$Xerr[1]$',r'$Xerr[2]$',r'$Xerr[3]$',r'$Xerr[4]$',r'$Xerr[5]$',r'$Xerr[6]$'])
    plt.grid(True)
    plt.show()
    

    logging.debug("Done")




'''Example input'''
'''There are three mode, user needs uncommented the mode for usage. The default is newTask which is the latest mode.'''
'''Best'''
# kp = 20
# ki = 0.5
# Tsc_ini = np.array([[1, 0, 0, 1],
#                     [0, 1, 0, 0],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])
# Tsc_fin = np.array([[0, 1, 0, 0],
#                     [-1, 0, 0, -1],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])

'''Overshoot'''
# kp = 2
# ki = 0.001
# Tsc_ini = np.array([[1, 0, 0, 1],
#                     [0, 1, 0, 0],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])
# Tsc_fin = np.array([[0, 1, 0, 0],
#                     [-1, 0, 0, -1],
#                     [0, 0, 1, 0.025],
#                     [0, 0, 0, 1]])
'''newTask'''
kp = 20
ki = 0.1
Tsc_ini = np.array([[0, -1, 0, 1],
                    [1, 0, 0, 1.0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
Tsc_fin = np.array([[1, 0, 0, 2],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])

 
Kp = np.diag([kp,kp,kp,kp,kp,kp])
Ki = np.diag([ki,ki,ki,ki,ki,ki])

#Initial configuration of robot
robot_config = np.array([0.1, -0.2, 0.2, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

#Call main function
main(Tsc_ini,Tsc_fin,Kp,Ki,robot_config)





        


