#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import intera_interface

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("pick_block_position_only")

# Habilitar robot
print("Enabling robot...")
rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
rs.enable()
rospy.sleep(1.0)

# Gripper
gripper = intera_interface.Gripper()

# MoveIt
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("right_arm")

group.set_end_effector_link("right_gripper_tip")
group.set_pose_reference_frame("base")
group.set_start_state_to_current_state()

group.set_planning_time(20.0)
group.set_num_planning_attempts(50)
group.set_max_velocity_scaling_factor(0.05)
group.set_max_acceleration_scaling_factor(0.05)
group.set_goal_position_tolerance(0.025)

def move_tip_to(position, label=""):
    group.set_start_state_to_current_state()
    group.clear_pose_targets()

    print("\nMoving to", label, position)
    group.set_position_target(position, "right_gripper_tip")

    success = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    if success:
        print("OK:", label)
    else:
        print("FAILED:", label)

    return success

# Coordenadas calibradas
# El gripper ya llegó cerca de:
# x=0.557, y=0.122, z=0.092
# Entonces usamos un pequeño ajuste en y.
above = [0.55, 0.12, 0.10]

# Bajada progresiva. No bajes de una sola vez.
mid = [0.55, 0.12, 0.03]

# Pre-grasp cerca del bloque.
# Si choca o falla, sube este valor a -0.02, 0.00, 0.02.
pre_grasp = [0.55, 0.12, -0.04]

print("Opening gripper...")
gripper.open()
rospy.sleep(1.0)

# Importante:
# Removemos el bloque del Planning Scene para que MoveIt permita acercarse.
# La mesa queda como obstáculo.
print("Removing block collision object from MoveIt scene...")
scene.remove_world_object("block")
rospy.sleep(1.0)

if not move_tip_to(above, "above block"):
    sys.exit(1)

if not move_tip_to(mid, "mid approach"):
    sys.exit(1)

if not move_tip_to(pre_grasp, "pre grasp"):
    sys.exit(1)

print("Closing gripper...")
gripper.close()
rospy.sleep(2.0)

print("Lifting...")
if not move_tip_to(above, "lift"):
    sys.exit(1)

print("\nPick básico terminado.")
