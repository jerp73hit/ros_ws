#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize([])
rospy.init_node("moveit_test")

group = moveit_commander.MoveGroupCommander("right_arm")

group.set_planning_time(5)

pose = Pose()
pose.position.x = 0.5
pose.position.y = 0.0
pose.position.z = 0.3
pose.orientation.w = 1.0

group.set_pose_target(pose)

plan = group.go(wait=True)

group.stop()
group.clear_pose_targets()

print("Movimiento completado")