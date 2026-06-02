#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import intera_interface
from math import sqrt

# ──────────────────────────────────────────────────────────────
# SPEED SETTINGS — increase these to make the robot go faster
#   Range: 0.0 (stopped) → 1.0 (full speed)
#   ⚠️  In a real lab, never exceed 0.3 until you are confident
#       the path is safe. In Gazebo simulation you can go up to 1.0
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING     = 0.08   # ← CHANGE THIS to go faster (e.g. 0.3, 0.5, 1.0)
ACCELERATION_SCALING = 0.08   # ← CHANGE THIS to accelerate faster (e.g. 0.3, 0.5, 1.0)

# ──────────────────────────────────────────────────────────────
# WORKSPACE LIMITS — defines the reachability box for validation
#   Adjust these to match your real/simulated table setup
# ──────────────────────────────────────────────────────────────
X_MIN, X_MAX =  0.30,  0.90   # metres forward from robot base
Y_MIN, Y_MAX = -0.60,  0.60   # metres left/right
Z_MIN, Z_MAX = -0.20,  0.80   # metres up/down from base frame

# Minimum distance from the robot base (avoids self-collision zone)
MIN_RADIUS = 0.25   # metres — points closer than this are rejected

TABLE_NAME   = "cafe_table"
BLOCK_NAME   = "block"
EEF_LINK     = "right_gripper_tip"
MOVE_GROUP   = "right_arm"

GRIPPER_TOUCH_LINKS = [
    "right_gripper_l_finger",
    "right_gripper_r_finger",
    "right_gripper_l_finger_tip",
    "right_gripper_r_finger_tip",
    "right_electric_gripper_base",
]


# ──────────────────────────────────────────────────────────────
# VALIDATION
# ──────────────────────────────────────────────────────────────
def validate_coordinates(x, y, z):
    """
    Returns (ok: bool, reason: str).
    Checks bounding box + minimum radius from base.
    """
    # 1. Bounding box check
    if not (X_MIN <= x <= X_MAX):
        return False, f"X={x:.3f} is out of range [{X_MIN}, {X_MAX}]"
    if not (Y_MIN <= y <= Y_MAX):
        return False, f"Y={y:.3f} is out of range [{Y_MIN}, {Y_MAX}]"
    if not (Z_MIN <= z <= Z_MAX):
        return False, f"Z={z:.3f} is out of range [{Z_MIN}, {Z_MAX}]"

    # 2. Minimum radius check (too close to the base)
    radius = sqrt(x**2 + y**2 + z**2)
    if radius < MIN_RADIUS:
        return False, (f"Point is too close to the robot base "
                       f"(radius={radius:.3f} m, min={MIN_RADIUS} m)")

    return True, "OK"


# ──────────────────────────────────────────────────────────────
# INPUT
# ──────────────────────────────────────────────────────────────
def ask_coordinates():
    """
    Prompts the user for x, y, z until valid values are entered.
    Type 'q' at any prompt to quit the loop.
    Returns (x, y, z) or raises SystemExit on quit.
    """
    while True:
        print("\n" + "─" * 50)
        print("Enter target coordinates (or 'q' to quit):")

        coords = {}
        quit_requested = False

        for axis in ("x", "y", "z"):
            while True:
                raw = input(f"  {axis.upper()} (metres): ").strip().lower()
                if raw == "q":
                    quit_requested = True
                    break
                try:
                    coords[axis] = float(raw)
                    break
                except ValueError:
                    print(f"  ⚠  Please enter a number (e.g. 0.58)")

            if quit_requested:
                break

        if quit_requested:
            print("Quit requested — shutting down.")
            sys.exit(0)

        x, y, z = coords["x"], coords["y"], coords["z"]
        ok, reason = validate_coordinates(x, y, z)

        if ok:
            print(f"  ✓  Target accepted: ({x:.3f}, {y:.3f}, {z:.3f})")
            return x, y, z
        else:
            print(f"  ✗  Invalid target: {reason}")
            print("     Please try again.\n")
            print(f"     Allowed ranges:  X[{X_MIN},{X_MAX}]  "
                  f"Y[{Y_MIN},{Y_MAX}]  Z[{Z_MIN},{Z_MAX}]")


# ──────────────────────────────────────────────────────────────
# MOTION
# ──────────────────────────────────────────────────────────────
def build_pose(arm_group, x, y, z):
    """Return current EEF pose with x/y/z overridden."""
    pose = arm_group.get_current_pose(EEF_LINK).pose
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    return pose


def move_to(arm_group, x, y, z):
    """Plan and execute to (x, y, z). Returns True on success."""
    pose = build_pose(arm_group, x, y, z)

    rospy.loginfo(f"Planning to ({x:.3f}, {y:.3f}, {z:.3f}) ...")
    arm_group.set_pose_target(pose, EEF_LINK)
    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    if success:
        rospy.loginfo("✓ Target reached.")
    else:
        rospy.logwarn("✗ Planner could not find a path to that target.")

    return success


# ──────────────────────────────────────────────────────────────
# SETUP
# ──────────────────────────────────────────────────────────────
def setup_moveit():
    """Initialise MoveIt, scene, and planner settings. Returns arm_group."""
    arm_group = moveit_commander.MoveGroupCommander(MOVE_GROUP)
    arm_group.set_end_effector_link(EEF_LINK)

    # ── Speed ──────────────────────────────────────────────────
    # To make the robot faster: increase VELOCITY_SCALING and
    # ACCELERATION_SCALING at the top of this file.
    arm_group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    arm_group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)

    # ── Planner robustness ─────────────────────────────────────
    arm_group.set_planning_time(20.0)
    arm_group.set_num_planning_attempts(50)

    return arm_group


def setup_scene(arm_group, scene):
    """Apply collision rules: ghost block, give fingers table immunity."""
    rospy.loginfo("Setting up collision rules ...")
    scene.remove_world_object(BLOCK_NAME)
    scene.attach_box(link=EEF_LINK, name=TABLE_NAME,
                     touch_links=GRIPPER_TOUCH_LINKS)
    rospy.sleep(1.0)
    arm_group.set_start_state_to_current_state()


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("sawyer_interactive_move", anonymous=True)

    # Gripper
    rospy.loginfo("Opening gripper ...")
    gripper = intera_interface.Gripper("right_gripper")
    gripper.open()
    rospy.sleep(1.0)

    # MoveIt
    arm_group = setup_moveit()
    scene     = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1.0)
    setup_scene(arm_group, scene)

    print("\n" + "=" * 50)
    print(" Sawyer interactive positioning mode")
    print(f" Velocity scaling     : {VELOCITY_SCALING}  ← edit to go faster")
    print(f" Acceleration scaling : {ACCELERATION_SCALING}  ← edit to go faster")
    print(f" Workspace X : [{X_MIN}, {X_MAX}]")
    print(f" Workspace Y : [{Y_MIN}, {Y_MAX}]")
    print(f" Workspace Z : [{Z_MIN}, {Z_MAX}]")
    print("=" * 50)

    # ── Interactive loop ───────────────────────────────────────
    while not rospy.is_shutdown():
        x, y, z = ask_coordinates()          # blocks until valid input or 'q'
        move_to(arm_group, x, y, z)          # attempt the move


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
