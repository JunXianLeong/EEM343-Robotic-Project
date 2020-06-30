#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

#start at initial position
arm1_group = moveit_commander.MoveGroupCommander("arm1_grasping")
arm1_group.set_named_target("start")
plan1 = arm1_group.go()

gripper_group = moveit_commander.MoveGroupCommander("gripper")
gripper_group.set_named_target("open")
plan2 = gripper_group.go()

arm2_group = moveit_commander.MoveGroupCommander("arm2_inspection")
arm2_group.set_named_target("rest")
plan3 = arm2_group.go()
rospy.sleep(5)

#move to scanning point to start scanning
arm2_group.set_named_target("scanning_center")
plan3 = arm2_group.go()

#rotate joint 5 of arm 2 to scan the environments
joint_goal = arm2_group.get_current_joint_values()
joint_goal[4] = -2.8563
plan3 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)
rospy.sleep(2)

joint_goal = arm2_group.get_current_joint_values()
joint_goal[4] = -1.2863
plan3 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)
rospy.sleep(2)

joint_goal = arm2_group.get_current_joint_values()
joint_goal[4] = -2.0713
plan3 = arm2_group.plan()
arm2_group.go(joint_goal, wait=True)
rospy.sleep(2)

#focus on the grapes
arm2_group.set_named_target("focusing")
plan3 = arm2_group.go()

#start grasping grapes
arm1_group.set_named_target("grasping grapes")
plan1 = arm1_group.go()

#close gripper
gripper_group.set_named_target("close")
plan2 = gripper_group.go()

#arm 1 is moving upward and backward to make sure that the grapes is grasped properly
joint_goal = arm1_group.get_current_joint_values()
joint_goal[1] = -0.2
plan1 = arm1_group.plan()
arm1_group.go(joint_goal, wait=True)

#camera move back to the center view
arm2_group.set_named_target("scanning_center")
plan3 = arm2_group.go()

#arm1 moves to the basket
arm1_group.set_named_target("drop_grapes")
plan1 = arm1_group.go()

#gripper open to make the grape falls
gripper_group.set_named_target("open")
plan2 = gripper_group.go()

#back to start position
arm1_group.set_named_target("start")
plan1 = arm1_group.go()

arm2_group.set_named_target("rest")
plan3 = arm2_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()