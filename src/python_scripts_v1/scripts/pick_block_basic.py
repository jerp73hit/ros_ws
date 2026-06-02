#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import intera_interface

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("pick_block_basic")

# Enable robot
print("Enabling robot...")
rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
rs.enable()
rospy.sleep(1.0)

# Gripper
gripper = intera_interface.Gripper()

# MoveIt group
group = moveit_commander.MoveGroupCommander("right_arm")
group.set_end_effector_link("right_gripper_tip")
group.set_pose_reference_frame("base")
group.set_start_state_to_current_state()

group.set_planning_time(20.0)
group.set_num_planning_attempts(50)
group.set_max_velocity_scaling_factor(0.06)
group.set_max_acceleration_scaling_factor(0.06)
group.set_goal_position_tolerance(0.025)

def move_tip_to(position):
    group.set_start_state_to_current_state()
    group.set_position_target(position, "right_gripper_tip")

    print("Moving gripper tip to:", position)
    success = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()

    if not success:
        print("Move failed:", position)
        return False

    rospy.sleep(0.5)
    return True

# Coordenadas en frame base.
# Ya confirmaste que esta zona llega encima del bloque.
above = [0.55, 0.10, 0.10]

# Aproximación más baja.
# Si choca o falla, subimos este valor.
pre_grasp = [0.55, 0.10, 0.02]

# Pick básico
print("Opening gripper...")
gripper.open()
rospy.sleep(1.0)

if not move_tip_to(above):
    sys.exit(1)

if not move_tip_to(pre_grasp):
    sys.exit(1)

print("Closing gripper...")
gripper.close()
rospy.sleep(1.5)

print("Lifting...")
if not move_tip_to(above):
    sys.exit(1)

print("Basic pick sequence finished.")
