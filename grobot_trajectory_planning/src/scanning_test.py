#!/usr/bin/env python

import sys
import string
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import *

def inverse_kinematics_Arm1(px,py,pz,phi):
   theta_1= atan2(py,px)
   q3y=((px*cos(theta_1)+py*sin(theta_1)-0.33*cos(phi))**2+(pz-0.3-0.33*sin(phi))**2-2.88)
   q3x=2.88
   theta_3= -acos(q3y/q3x)
   q2y=(1.2*cos(theta_3)+1.2)*(pz-0.3-0.33*sin(phi))-1.2*sin(theta_3)*(px*cos(theta_1)+py*sin(theta_1)-0.33*cos(phi))
   q2x=(1.2*cos(theta_3)+1.2)*(px*cos(theta_1)+py*sin(theta_1)-0.33*cos(phi))+1.2*sin(theta_3)*(pz-0.3-0.33*sin(phi))
   theta_2= atan2(q2y,q2x)
   theta_4= phi-theta_2-theta_3
   return [theta_1,-theta_2,-theta_3,-theta_4]

def inverse_kinematics_Arm2(px,py,pz,phi_2,az,nz):
    Q5= atan(-az/nz)
    c= 0.0875*sin(Q5)-0.1274
    r=((2*c*py)**2)-4*((py)**2+(px)**2)*(-(c)**2-(px)**2)
    q11= (-2*c*py+sqrt(r))/(2*(py**2+px**2))
    #q12= (-2*c*py-sqrt(r))/(2*(py**2+px**2))
    Q1= acos(q11)
    Q3y=(px*cos(Q1)+py*sin(Q1)-0.0875*cos(phi_2)*cos(Q5)+0.1705*sin(phi_2))**2+(pz-0.3-0.0875*sin(phi_2)*cos(Q5)-0.1705*cos(phi_2))**2-2.25
    Q3x=2.16
    Q3= -acos(Q3y/Q3x)
    Q2y=(0.9*cos(Q3)+1.2)*(pz-0.3-0.0875*sin(phi_2)*cos(Q5)-0.1705*cos(phi_2))-0.9*sin(Q3)*(px*cos(Q1)+py*sin(Q1)-0.0875*cos(phi_2)*cos(Q5)+0.1705*sin(phi_2))
    Q2x=(0.9*cos(Q3)+1.2)*(px*cos(Q1)+py*sin(Q1)-0.0875*cos(phi_2)*cos(Q5)+0.1705*sin(phi_2))+0.9*sin(Q3)*(pz-0.3-0.0875*sin(phi_2)*cos(Q5)-0.1705*cos(phi_2))
    Q2= atan2(Q2y,Q2x)
    Q4= phi_2-Q2-Q3
    return [Q1,-Q2,-Q3,-Q4, Q5]

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

arm1_group = moveit_commander.MoveGroupCommander("arm1_grasping")
arm2_group = moveit_commander.MoveGroupCommander("arm2_inspection")

#Moving Arm 1 back to initial position
print("Move Arm 1 to initial position...")
joint_goal = arm1_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -2.2256
joint_goal[2] = 2.40
joint_goal[3] = 0
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

#Scanning point
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.20, 0.8, 1.5, 0, 0, 1)
print("Move to scanning pose...")
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

print("Start scanning...")
joint_goal = arm2_group.get_current_joint_values()
joint_goal[4] = 0.2
plan2 = arm2_group.plan()
for x in range(1, 5, 1):
   joint_goal = arm2_group.get_current_joint_values()
   joint_goal[4] -= 0.262
   plan2 = arm2_group.plan()
   arm2_group.go(joint_goal, wait=True)
   if joint_goal[4]== -1.510:
       break

print("Return.")
for x in range(1, 5, 1):
   joint_goal = arm2_group.get_current_joint_values()
   joint_goal[4] += 0.262
   plan2 = arm2_group.plan()
   arm2_group.go(joint_goal, wait=True)
   if joint_goal[4]== 0.2:
       break

print("Stop scanning.")

print("Focusing on Grape 1.")

#Grape 1 focusing point
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.1, 0.4, 1.5, 0, 0, 1)

joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

joint_goal = arm2_group.get_current_joint_values()
joint_goal[4] = -0.2
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

command=raw_input("Continue? Y/N   ")
if command == "Y" or "y":
    print("Focusing on Grape 2.")
else:
    print("Closing program...")
    print("5")
    rospy.sleep(1)
    print("4")
    rospy.sleep(1)
    print("3")
    rospy.sleep(1)
    print("2")
    rospy.sleep(1)
    print("1")
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    sys.exit()

#Drop grape
print("Drop Grape 1 to basket...")
joint_goal = arm1_group.get_current_joint_values()
joint_goal[0] = 1.5880
joint_goal[1] = -2.1221
joint_goal[2] = 2.4000
joint_goal[3] = 0.5003
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

#Set Arm 1 Link 1 to initial pose
print("Grasping Grape 2...")
joint_goal = arm1_group.get_current_joint_values()
joint_goal[2] = 1.8
joint_goal[3] = 0
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

joint_goal = arm1_group.get_current_joint_values()
joint_goal[0] = 0
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

#Grape 2 focusing point
#Viapoint 1(to avoid obstacle)
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.5, 0.9, 1.3, 0, 0, 1)
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

#Viapoint 2(to avoid obstacle)
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.6, 1.2, 1.1, 0, 0, 1)
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

#Final point
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.8, 1.3, 1.0, 0, 0, 1)
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

joint_goal = arm2_group.get_current_joint_values()
joint_goal[4] = -0.25
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

command=raw_input("Continue? Y/N   ")
if command == "Y" or "y":
    print("Focusing on Grape 2.")
else:
    print("Closing program...")
    print("5")
    rospy.sleep(1)
    print("4")
    rospy.sleep(1)
    print("3")
    rospy.sleep(1)
    print("2")
    rospy.sleep(1)
    print("1")
    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()
    sys.exit()

#Reset all robot arms back to initial positions
print("Move Arm 2 to initial position...")

#Viapoint 1(to avoid obstacle)
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.6, 1.2, 1.1, 0, 0, 1)
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

#Viapoint 2(to avoid obstacle)
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.5, 0.9, 1.3, 0, 0, 1)
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

#Viapoint 3
Q_1,Q_2,Q_3,Q_4,Q_5= inverse_kinematics_Arm2(0.1, 0.4, 1.5, 0, 0, 1)
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = Q_1
joint_goal[1] = Q_2
joint_goal[2] = Q_3
joint_goal[3] = Q_4
joint_goal[4] = Q_5
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

#Final pose for Arm 2
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -1.9668
joint_goal[2] = 1.9967
joint_goal[3] = 0
joint_goal[4] = 0
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

#Moving Arm 1 back to initial position
print("Move Arm 1 to initial position...")
joint_goal = arm1_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -2.2256
joint_goal[2] = 2.40
joint_goal[3] = 0
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

rospy.sleep(3)
moveit_commander.roscpp_shutdown()
