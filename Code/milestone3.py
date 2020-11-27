import modern_robotics as mr
import numpy as np

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, timestep):
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

    robot_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
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
    theta_list = robot_config[3:]
    F_6 = r / 4 * np.array([[0, 0, 0, 0],
                            [0, 0, 0, 0],
                            [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1],
                            [0, 0, 0, 0]])
    
    integral = np.zeros((6,),dtype=float)
    
    T_0e = mr.FKinBody(M_Oe,B_list,theta_list)
    Vd = mr.se3ToVec((1/timestep) * mr.MatrixLog6(np.dot(mr.TransInv(Xd),Xd_next)))
    #print(Vd)

    Ad_xxd = mr.Adjoint(np.dot(mr.TransInv(X),Xd))
    #print(Ad_xxd)
    Ad_xxd_Vd = np.dot(Ad_xxd,Vd)
    #print(Ad_xxd_Vd)
    X_err = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
    print(X_err)

    V = Ad_xxd_Vd + np.dot(Kp,X_err) + np.dot(Ki, integral + X_err * timestep)
    integral += X_err * timestep
    # print(V)

    #From Chapter 13.5
    J_arm = mr.JacobianBody(B_list,theta_list)
    #print(J_arm) 
    a = np.dot(mr.TransInv(T_0e),mr.TransInv(T_b0))
    J_base = np.dot(mr.Adjoint(a),F_6)                        
    #print(J_base)
    Je = np.concatenate((J_base,J_arm),axis=1) 
    print(Je)
    Je_inv = np.linalg.pinv(Je,1e-4)
    # print(Je_inv)
    
    command_V = Je_inv.dot(V)                                     #Commanded Twist
    print(command_V)
    # print("---",command_V[:4])
    return X_err,command_V



#Test input

Xd = np.array([[0, 0, 1, 0.5],
               [0, 1, 0, 0],
               [-1, 0, 0, 0.5],
               [0, 0, 0, 1]])
Xd_next = np.array([[0, 0, 1, 0.6],
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0.3],
                    [0, 0, 0, 1]])
                 
X = np.array([[0.17, 0, 0.985, 0.387],
              [0, 1, 0, 0],
              [-0.985, 0, 0.17, 0.57],
              [0, 0, 0, 1]])

Kp = 0
Ki = 0
timestep = 0.01

X_err, V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,timestep)
# print(V)

