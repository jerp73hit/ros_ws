#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("print_moveit_info")

group = moveit_commander.MoveGroupCommander("right_arm")

print("Planning frame:", group.get_planning_frame())
print("End effector link:", group.get_end_effector_link())
print("Current pose:")
print(group.get_current_pose())
