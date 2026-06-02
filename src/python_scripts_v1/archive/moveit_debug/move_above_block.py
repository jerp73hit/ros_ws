#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_above_block")

group = moveit_commander.MoveGroupCommander("right_arm")

group.set_start_state_to_current_state()
group.set_planning_time(10.0)
group.set_num_planning_attempts(20)
group.set_max_velocity_scaling_factor(0.15)
group.set_max_acceleration_scaling_factor(0.15)

pose = Pose()

# Pose encima del bloque
pose.position.x = 0.55
pose.position.y = 0.10
pose.position.z = 1.00

# Orientación tipo "gripper mirando hacia abajo"
pose.orientation.x = 0.0
pose.orientation.y = 1.0
pose.orientation.z = 0.0
pose.orientation.w = 0.0

group.set_pose_target(pose)

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Movimiento sobre el bloque completado")
else:
    print("No se pudo mover sobre el bloque")
