#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    i = 0
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        # Define DH param symbols
        rospy.loginfo("Defining DH param symbols...")
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
                    
        # Define Modified DH Transformation matrix
        rospy.loginfo("Modified DH transformatin matrix...")
        s = {alpha0:    0, a0:      0, d1:  0.75, q1:      q1,
            alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
            alpha2:     0, a2:   1.25, d3:     0, q3:      q3,
            alpha3: -pi/2, a3: -0.054, d4:  1.501, q4: 	   q4,
            alpha4:  pi/2, a4:      0, d5:     0, q5:      q5,
            alpha5: -pi/2, a5:      0, d6:     0, q6:      q6,
            alpha6:     0, a6:      0, d7: 0.303, q7:       0}

        # Create individual transformation matrices
        rospy.loginfo("Create individual transformation matrices...")            
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                       [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                   0,                   0,            0,               1]])

        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                       [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                   0,                   0,            0,               1]])

        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                       [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                   0,                   0,            0,               1]])

        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
                       [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                   0,                   0,            0,               1]])

        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
                       [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                   0,                   0,            0,               1]])

        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
                       [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                   0,                   0,            0,               1]])

        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
                       [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                   0,                   0,            0,               1]])
 
        T6_G = T6_G.subs(s)

        # Composition of Homogeneous Transforms
        T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
        T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
        T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
        T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
        T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
        T0_G = simplify(T0_6 * T6_G) # base_link to gripper_link

        R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]  #base_link to link_3
         
        r, p, y = symbols('r p y')

        # Rotation Matrix about the x-axis
        R_x = Matrix([[1,      0,       0],
                      [0, cos(r), -sin(r)],
                      [0, sin(r),  cos(r)]]) # roll

        # Rotation Matrix about the y-axis
        R_y = Matrix([[ cos(p), 0, sin(p)],
                      [      0, 1,      0],
                      [-sin(p), 0, cos(p)]]) # pitch

        # Rotation Matrix about the z-axis
        R_z = Matrix([[cos(y), -sin(y), 0],
                      [sin(y),  cos(y), 0],
                      [     0,       0, 1]]) # yaw

        # The generalized Rotation Matrix
        R_rpy = simplify(R_z * R_y * R_x)

        # Correction Needed to Account for Orientation Difference bet.
        # definition of Gripper Link in URDF vs. DH Convetion
        R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
        R_rpy = R_rpy * R_corr

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            i = i + 1
            rospy.loginfo("End effector pose i = %d (px,py,pz): %.2f, %.2f, %.2f" % (i,px,py,pz))
	    
            rospy.loginfo("Roll: %.2f" % roll)
            rospy.loginfo("Pitch: %.2f" % pitch)
            rospy.loginfo("Yaw: %.2f" % yaw)

            # find the WC relative to the base frame
            # wx = px - (d6 + l) * nx; where px is end effector position, d6 from DH parameters, 
            # wy = py - (d6 + l) * ny; l is end effector length,
            # wz = pz - (d6 + l) * nz; nx is the vector along gripper link z-axis
            # d6 + l = 0.193 + 0.11 = 0.303

            R_rpy = R_rpy.subs({'r':roll, 'p':pitch, 'y':yaw})
           
            r13 = R_rpy[0,2]
            r23 = R_rpy[1,2]
            r33 = R_rpy[2,2]

            wx = px - (0.303) * r13 # roll
            wy = py - (0.303) * r23 # pitch
            wz = pz - (0.303) * r33 # yaw   
            rospy.loginfo("Wrist center i = %d (wx,wy,wz): %.2f, %.2f, %.2f" % (i,wx,wy,wz))
         
            # Calculate joint angles using Geometric IK method
            # using trigonometry and projections
            # 1: project onto x0,y0
            theta1 = atan2(wy, wx)
 
            # 2: project onto x1,y1 to find theta2, theta3
            # use law of cosines to find theta2 and theta3
            a1 = 0.35
            a2 = 1.25
            d1 = 0.75
            d4 = 1.501
            j = sqrt(wx**2 + wy**2) - a1
            k = wz - d1
            l = sqrt( j**2 + k**2)
            theta_0 = atan2( k, j)
            theta_00 = acos((a2**2 + l**2 - d4**2)/(2*a2*l))
            theta2 = (pi/2 - theta_0 - theta_00)

            theta_000 = acos((a2**2 + d4**2 - l**2)/(2*a2*d4))
            theta3 = pi/2 - (theta_000 - 0.036) # O.036: small error for joint 3 to joint 4 dip

            
            # Find the set of angles corresponding to a ZYZ Euler Angle Transformation
            R3_6 = (R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3}).T * R_rpy)
            rospy.loginfo(R3_6)

            r13 = R3_6[0,2] #row 0, column 2
            r23 = R3_6[1,2] #row 1, column 2
            r33 = R3_6[2,2] #row 2, column 2
            r21 = R3_6[1,0] #row 1, column 0
            r22 = R3_6[1,1] #row 1, column 1

            theta4 = atan2(r33, -r13)
            theta5 = atan2(sqrt(r13**2 + r33**2), r23)
            theta6 = atan2(-r22, r21)

            # Clamp angles to uppper/lower limits
            theta1 = max(min(theta1, 3.22), -3.22)
            theta2 = max(min(theta2, 1.48), -0.78)
            theta3 = max(min(theta3, 1.13), -3.66)
            theta4 = max(min(theta4, 6.10), -6.10)
            theta5 = max(min(theta5, 2.18), -2.18)
            theta6 = max(min(theta6, 6.10), -6.10)

            rospy.loginfo("Theta1: %.2f" % theta1)
            rospy.loginfo("Theta2: %.2f" % theta2)
            rospy.loginfo("Theta3: %.2f" % theta3)
            rospy.loginfo("Theta4: %.2f" % theta4)
            rospy.loginfo("Theta5: %.2f" % theta5)
            rospy.loginfo("Theta6: %.2f" % theta6)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service

    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
