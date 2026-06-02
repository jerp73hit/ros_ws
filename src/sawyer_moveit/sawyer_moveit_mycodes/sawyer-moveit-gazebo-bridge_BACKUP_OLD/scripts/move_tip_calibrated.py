#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_tip_calibrated")

group = moveit_commander.MoveGroupCommander("right_arm")

group.set_end_effector_link("right_gripper_tip")
group.set_pose_reference_frame("base")
group.set_start_state_to_current_state()

group.set_planning_time(25.0)
group.set_num_planning_attempts(80)
group.set_max_velocity_scaling_factor(0.06)
group.set_max_acceleration_scaling_factor(0.06)

group.set_goal_position_tolerance(0.025)
group.set_goal_orientation_tolerance(0.45)

pose = Pose()

# Punto base del cubo en nuestra escena calibrada
cube_x = 0.55
cube_y = 0.10

# Offsets de calibración visual
# Cambia estos dos valores hasta que la garra quede centrada sobre el cubo.
offset_x = 0.00
offset_y = 0.00

pose.position.x = cube_x + offset_x
pose.position.y = cube_y + offset_y
pose.position.z = 0.10

# Orientación overhead
pose.orientation.x = -0.00142460053167
pose.orientation.y = 0.999994209902
pose.orientation.z = -0.00177030764765
pose.orientation.w = 0.00253311793936

print("Target pose:")
print(pose)

group.set_pose_target(pose, "right_gripper_tip")

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

if success:
    print("Movimiento calibrado ejecutado.")
else:
    print("Movimiento calibrado falló.")
