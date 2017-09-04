## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc/kuka_arm_schematic.png
[image2]: ./misc/kuka_arm_geometric_analysis.png
[image3]: ./misc/kuka_arm_testing.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The Modified DH paramters:

   | theta_i | alpha_i | a_i-1 | d_i
-- | ------- | ------- | ----- | ------
1  | theta_1 | 0       | 0     | 0.75
2  | theta_2 | -pi/2   | 0.35  | 0 
3  | theta_3 | 0       | 1.25  | 0
4  | theta_4 | -pi/2   |-0.054 | 1.50
5  | theta_5 | pi/2    | 0     | 0
6  | theta_6 | -pi/2   | 0     | 0.303

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
The modified transformation matrix and individual transformation matrices are found on lines 39~98.  The generalized homogeneous transform between the base link and gripper link is shown on lines 100~106. 


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Theta1 was found by deriving the wrist center and projecting onto the x0 and y0 plane.  Theta2 and 3 were found projecting onto the x1, y1 plane.  The following image shows these derivations.

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The IKServer.py code was implemented as follows:
    - DH were parameters derived from .xacro file and schematic diagram
    - The modified transformation matrix was defined from the DH paramters
    - Individual transformation matrices defined
    - The wrist center was found using rotational part of total transformation matrix and rpy values
    - Using the geometric IK method, theta1 was found using the wrist center. Theta2, theta3 were found using the derivations in (image 2)
    - Once the the first three joint angles were found the Euler angles were derived for a ZYZ Euler angle transformation using R3_6 = inv(R0_3) * R_rpy


And just for fun, another example image:
![alt text][image3]


