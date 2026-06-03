#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import intera_interface


# ──────────────────────────────────────────────
# CONSTANTS — edit these to tune behaviour
# ──────────────────────────────────────────────
HOVER_Z       =  0.500   # Safe hover height above the block
GRASP_Z       = -0.130   # Z where the gripper meets the block
LIFT_Z        =  0.200   # Z to lift to after grasping

HOVER_X       =  0.58
HOVER_Y       =  0.125

TABLE_NAME    = "cafe_table"   # Must match the name in your scene script
BLOCK_NAME    = "block"

GRIPPER_TOUCH_LINKS = [
    "right_gripper_l_finger",
    "right_gripper_r_finger",
    "right_gripper_l_finger_tip",
    "right_gripper_r_finger_tip",
    "right_electric_gripper_base",
]


# ──────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────
def build_pose(arm_group, x, y, z):
    """Return a copy of the current EEF pose with x/y/z overridden."""
    pose = arm_group.get_current_pose("right_gripper_tip").pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    return pose


def move_to(arm_group, pose, label):
    """Plan and execute to a pose target; exit on failure."""
    rospy.loginfo(f"Moving to: {label}  →  ({pose.position.x:.3f}, "
                  f"{pose.position.y:.3f}, {pose.position.z:.3f})")
    arm_group.set_pose_target(pose, "right_gripper_tip")
    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    if not success:
        rospy.logerr(f"FAILED to reach: {label}")
        sys.exit(1)

    rospy.loginfo(f"Reached: {label}")
    rospy.sleep(0.5)


# ──────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────
def main():
    # --- ROS / MoveIt init ---
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sawyer_pick", anonymous=True)

    rospy.loginfo("=== Initialising gripper ===")
    gripper = intera_interface.Gripper("right_gripper")
    gripper.open()
    rospy.sleep(1.0)

    arm_group = moveit_commander.MoveGroupCommander("right_arm")
    arm_group.set_end_effector_link("right_gripper_tip")

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)

    # --- Planner settings ---
    arm_group.set_planning_time(20.0)
    arm_group.set_num_planning_attempts(50)
    arm_group.set_max_velocity_scaling_factor(0.08)
    arm_group.set_max_acceleration_scaling_factor(0.08)

    # ── STEP 1: Collision setup ──────────────────
    rospy.loginfo("=== Setting up collision rules ===")

    # Remove block so fingers can reach it
    scene.remove_world_object(BLOCK_NAME)

    # Give fingers immunity against the table surface
    scene.attach_box(
        link="right_gripper_tip",
        name=TABLE_NAME,
        touch_links=GRIPPER_TOUCH_LINKS,
    )
    rospy.sleep(1.0)
    arm_group.set_start_state_to_current_state()

    # ── STEP 2: Hover above block ────────────────
    rospy.loginfo("=== PHASE 2: Moving to hover position ===")
    hover_pose = build_pose(arm_group, HOVER_X, HOVER_Y, HOVER_Z)
    move_to(arm_group, hover_pose, "Hover (0.58, 0.125, 0.50)")

    # ── STEP 3: Descend to block ─────────────────
    rospy.loginfo("=== PHASE 3: Descending to grasp position ===")
    grasp_pose = build_pose(arm_group, HOVER_X, HOVER_Y, GRASP_Z)
    move_to(arm_group, grasp_pose, "Grasp (0.58, 0.125, -0.13)")

    # ── STEP 4: Close gripper ────────────────────
    rospy.loginfo("=== PHASE 4: Closing gripper ===")
    gripper.close()
    rospy.sleep(1.5)

    # ── STEP 5: Lift ─────────────────────────────
    rospy.loginfo("=== PHASE 5: Lifting block ===")
    lift_pose = build_pose(arm_group, HOVER_X, HOVER_Y, LIFT_Z)
    move_to(arm_group, lift_pose, f"Lift Z={LIFT_Z}")

    # ── STEP 6: Cleanup collision rules ──────────
    rospy.loginfo("=== Cleaning up collision attachments ===")
    scene.remove_attached_object("right_gripper_tip", name=TABLE_NAME)
    rospy.sleep(0.5)

    rospy.loginfo("=== Pick sequence complete! ===")


if __name__ == "_main_":
    try:
        main()
    except rospy.ROSInterruptException:
        pas
