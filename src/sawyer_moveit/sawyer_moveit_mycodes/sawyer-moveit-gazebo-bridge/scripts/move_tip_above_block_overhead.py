#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_tip_above_block_overhead")

group = moveit_commander.MoveGroupCommander("right_arm")

# Usar la punta real del gripper como efector final
group.set_end_effector_link("right_gripper_tip")
group.set_start_state_to_current_state()

group.set_planning_time(20.0)
group.set_num_planning_attempts(50)
group.set_max_velocity_scaling_factor(0.08)
group.set_max_acceleration_scaling_factor(0.08)

group.set_goal_position_tolerance(0.02)
group.set_goal_orientation_tolerance(0.20)

pose = Pose()

# Coordenadas compatibles con el frame base/demo original.
# Es una pose de aproximación, todavía NO es de agarre.
pose.position.x = 0.55
pose.position.y = 0.10
pose.position.z = 0.05

# Orientación overhead del demo original
pose.orientation.x = -0.00142460053167
pose.orientation.y = 0.999994209902
pose.orientation.z = -0.00177030764765
pose.orientation.w = 0.00253311793936

print("Planning frame:", group.get_planning_frame())
print("End effector:", group.get_end_effector_link())
print("Target pose:")
print(pose)

group.set_pose_target(pose, "right_gripper_tip")

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Gripper tip ubicado encima del bloque con orientación overhead.")
else:
    print("No se pudo ubicar el gripper encima del bloque.")
