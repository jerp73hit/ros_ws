#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_tip_above_block_position_only")

group = moveit_commander.MoveGroupCommander("right_arm")

group.set_end_effector_link("right_gripper_tip")
group.set_pose_reference_frame("base")
group.set_start_state_to_current_state()

group.set_planning_time(20.0)
group.set_num_planning_attempts(50)
group.set_max_velocity_scaling_factor(0.08)
group.set_max_acceleration_scaling_factor(0.08)

group.set_goal_position_tolerance(0.03)

# Objetivo en frame base, encima del bloque
target_position = [0.55, 0.10, 0.05]

print("Planning frame:", group.get_planning_frame())
print("Pose reference frame:", group.get_pose_reference_frame())
print("End effector:", group.get_end_effector_link())
print("Target position:", target_position)

group.set_position_target(target_position, "right_gripper_tip")

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Gripper tip movido encima del bloque usando solo posición.")
else:
    print("No se pudo mover usando solo posición.")
