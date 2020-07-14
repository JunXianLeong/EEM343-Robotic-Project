#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

#start at initial position
arm2_group = moveit_commander.MoveGroupCommander("arm2_inspection") #'start' pose
print("Move Arm 2 to starting position...")
joint_goal = arm2_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -1.9668
joint_goal[2] = 1.9967
joint_goal[3] = 0
joint_goal[4] = 0
plan2 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)

arm1_group = moveit_commander.MoveGroupCommander("arm1_grasping") #'start' pose
print("Move Arm 1 to starting position...")
joint_goal = arm1_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -2.2256
joint_goal[2] = 2.40
joint_goal[3] = 0
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

gripper_group = moveit_commander.MoveGroupCommander("gripper") #'open' pose
print("Reset gripper to its initial pose...")
joint_goal = gripper_group.get_current_joint_values()
joint_goal[0] = 0.9144
joint_goal[1] = -0.9144
plan3 = gripper_group.plan()
gripper_group.go(joint_goal, wait=True)

rospy.sleep(3)
moveit_commander.roscpp_shutdown()