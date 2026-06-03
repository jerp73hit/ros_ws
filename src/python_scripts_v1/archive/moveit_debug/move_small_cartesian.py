#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_small_cartesian")

group = moveit_commander.MoveGroupCommander("right_arm")

group.set_start_state_to_current_state()
group.set_planning_time(10.0)
group.set_num_planning_attempts(20)
group.set_max_velocity_scaling_factor(0.10)
group.set_max_acceleration_scaling_factor(0.10)

current = group.get_current_pose().pose

pose = Pose()
pose.position.x = current.position.x + 0.05
pose.position.y = current.position.y
pose.position.z = current.position.z + 0.05

pose.orientation = current.orientation

print("Current pose:")
print(current)

print("Target pose:")
print(pose)

group.set_pose_target(pose)

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Movimiento cartesiano pequeño completado")
else:
    print("Movimiento cartesiano pequeño falló")
