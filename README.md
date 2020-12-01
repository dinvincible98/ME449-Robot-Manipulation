# ME449-Robot-Manipulation

# Overview

* This is the final project of the cource ME 449 Robotic Manipulation at Northwestern University. It is also the capstone project for the course "Modern Robotics" on Cousera. Here is a wiki page includes the detailed information.

* The goal of the project is to wirte a program that enables KUKA youBot to finish a pick and place task in V-Rep simulation scene.

# Package description

* Code

1.milestone1.py: This file includes a customized function NextState which calculate the kinematics of the youbot and generates a Next_config.csv file that can be used for simulation in CoppeliaSim scene 6.

2.milestone2.py: This file inlcudes code for milestone2 which computes the trajectory as a list of N SE(3) matrices and save it as a .csv file so it can be simulated in coppeliaSim scene.

3.milestone3.py: This file includes a function to calculate the kinematic task-space feedforward plus feedback control law.

4.The_full_program.py: This file inlcudes and integrates milestone 1-3 then generates a .csv file that can accomplish youBot simulatiom task.

* Results

There are three subdirectories and each contain two .csv file(one for simulation and one for plot of end-effector error), one log file, one plot picture, one simulation video and a README that states the controller and gain values(Kp and Ki)
 
1.Best: It solves a pick-and-place task with a mini Xerr value where the initial and final configurations of the cube are at the default locations in the capstone CoppeliaSim scene.
	
	The initial configuration of the cube:
	
	Tsc_ini = np.array([[1, 0, 0, 1],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0.025],
                       [0, 0, 0, 1]])
                       
	The final(goal) configuration of the cube:
	
	Tsc_fin = np.array([[0, 1, 0, 0],
		               [-1, 0, 0, -1],
		               [0, 0, 1, 0.025],
		               [0, 0, 0, 1]])
	Gains:
	
	Kp = 20
	Ki = 0.5
	
* Demo
![best](https://user-images.githubusercontent.com/70287453/100411600-dcc27c80-3037-11eb-9b2b-3de78e7b406f.gif)
	
* Xerr plot:
![Best](https://user-images.githubusercontent.com/70287453/100800894-d6901f80-33ec-11eb-985d-63261fdd943c.png)
  
		               
	
	2. Overshoot: It solves the same task as "Best" but the has larger Xerr value.

	3. Newtask: It solves the pick-and-place task where the initial and final configuration of the cube is self-customized.




* Overshoot

![overshoot](https://user-images.githubusercontent.com/70287453/100411731-35921500-3038-11eb-8d46-5e6ad207eea3.gif)

* NewTask

![newTask](https://user-images.githubusercontent.com/70287453/100411792-58242e00-3038-11eb-8eaa-48703d2cd7ae.gif)


![Best](https://user-images.githubusercontent.com/70287453/100800894-d6901f80-33ec-11eb-985d-63261fdd943c.png)
![newTask](https://user-images.githubusercontent.com/70287453/100800916-dc860080-33ec-11eb-879c-ea6c93688a7f.png)
![overshoot](https://user-images.githubusercontent.com/70287453/100800924-e0198780-33ec-11eb-9c03-43a4eb9e2a90.png)
