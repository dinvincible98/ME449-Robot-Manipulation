The result of this package is derived from feedforward + PI controller

The feedback gains are:
kp = 20
ki = 0.1

The initial configuration of the cube is:
Tsc_ini = np.array([[0, -1, 0, 1],
                    [1, 0, 0, 1.0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])
                    
The final configuration of the cube is:                 
Tsc_fin = np.array([[1, 0, 0, 2],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.025],
                    [0, 0, 0, 1]])

