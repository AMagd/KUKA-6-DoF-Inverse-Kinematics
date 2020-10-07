# KUKA-6-DoF-Inverse-Kinematics

Author ~ Ahmed Magd Aly
Innopolis University

YouTube: https://www.youtube.com/playlist?list=PL9nxOfGsvMusy76Hr64nVc0glB2h7BjB4

## This project consists of 11 MATLAB files:

- 6 of them "namely (Tx, Ty, Tz, Rx, Ry and Rz)" are functions for the homogeneous tranformation matrices

- 2 files "FK and IK" are functions for solving forward kinematics (FK) and inverse kinematics (IK)
	- FK function has 7 inputs, which are joint varibales q1 -> q6 and the last input is a logical input that if true allows the function to plot the robot and if false does not provide any plots
	- IK takes 6 inputs which are (x,y,z) and (angles z,y,x) for which represents the orientation of the target point in euler's angles

- "FK_test" file provides a test for the forward kinematics function

- "IK_test" provides a test for the inverse kinematics function.
	- line 10 "ShapeToDraw = 1;" of this function controls the shape that will be drawn, where if ShapeToDraw was:
		- 1 -> it will plot a circle that is made of a sine wave
		- 2 -> will plot a 3D spiral shape
		- 3 -> will plot an array of circles
		- 4 -> will plot a sine wave with all possible configurations of the robot

- "IK_live_script" is a MATLAB live script file explaining the inverse kinematics of the robot
