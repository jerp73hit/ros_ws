#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_tip_above_block_oriented")

group = moveit_commander.MoveGroupCommander("right_arm")

group.set_end_effector_link("right_gripper_tip")
group.set_pose_reference_frame("base")
group.set_start_state_to_current_state()

group.set_planning_time(25.0)
group.set_num_planning_attempts(80)
group.set_max_velocity_scaling_factor(0.06)
group.set_max_acceleration_scaling_factor(0.06)

group.set_goal_position_tolerance(0.03)
group.set_goal_orientation_tolerance(0.35)

pose = Pose()

# Coordenadas calibradas en frame base.
# Bloque en Planning Scene: x=0.55, y=0.10, z=-0.115
# Vamos encima del bloque.
pose.position.x = 0.55
pose.position.y = 0.10
pose.position.z = 0.10

# Orientación overhead del demo original.
# Busca que el gripper apunte hacia abajo.
pose.orientation.x = -0.00142460053167
pose.orientation.y = 0.999994209902
pose.orientation.z = -0.00177030764765
pose.orientation.w = 0.00253311793936

print("Planning frame:", group.get_planning_frame())
print("Pose reference frame:", group.get_pose_reference_frame())
print("End effector:", group.get_end_effector_link())
print("Target pose:")
print(pose)

group.set_pose_target(pose, "right_gripper_tip")

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Gripper orientado encima del bloque.")
else:
    print("No se pudo mover con orientación overhead.")
