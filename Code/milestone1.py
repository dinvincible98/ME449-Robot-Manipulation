import modern_robotics as mr
import numpy as np
from numpy import sin,cos

'''Milestone2'''

'''
This milestone includes a customized function NextState which calculate the kinematics of the youbot and generates
a Next_config.csv file that can be used for simulation in CoppeliaSim scene 6.
'''

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
    
    if w_bz == 0:
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






# Test input
current_config = np.zeros(12)
speed = np.array([0, 0, 0, 0, 0, 10, 10, 10, 10])       # Should spin counterclockwise 
timestep = 0.01
max_omg = 15                                     # Maximun speed for joints and wheels 
next_config = NextState(current_config, speed, timestep, max_omg)
#print(next_config)

# Test for 100 timesteps
Traj = []                                                 #A N x 13 matrices
# Initial states
current_traj = np.concatenate((current_config,0), axis=None)         
Traj.append(current_config.tolist())


# Iterations
for i in range(100):
    next_config = NextState(current_config,speed,timestep,max_omg)
    current_config = next_config
    current_traj = np.concatenate((current_config,0), axis=None)
    Traj.append(current_config.tolist())

# Generate the .csv file
np.savetxt('Next_config.csv', Traj, delimiter = ',')
# print(Traj)











