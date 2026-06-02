#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("print_tip_pose")

group = moveit_commander.MoveGroupCommander("right_arm")

print("Planning frame:", group.get_planning_frame())
print("Default end effector:", group.get_end_effector_link())

try:
    group.set_end_effector_link("right_gripper_tip")
    print("New end effector:", group.get_end_effector_link())
    print("Current gripper tip pose:")
    print(group.get_current_pose("right_gripper_tip"))
except Exception as e:
    print("Could not set right_gripper_tip as end effector:")
    print(e)
