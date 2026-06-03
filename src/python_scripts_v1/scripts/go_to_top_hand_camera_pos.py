#!/usr/bin/env python3
"""
Move Sawyer through a predefined list of joint positions and exit.
Add or remove waypoints in WAYPOINTS list at the top of the file.
Joint order: right_j0, right_j1, right_j2, right_j3, right_j4, right_j5, right_j6
"""
import rospy
import intera_interface
from intera_interface import CHECK_VERSION

# ──────────────────────────────────────────────────────────────
# WAYPOINTS — edit these to define the sequence of positions
# Each entry is a dict {joint_name: angle_radians}
# Copy the values from:
#   rostopic echo -n 1 /joint_states
# and pick only right_j0 → right_j6
# ──────────────────────────────────────────────────────────────
WAYPOINTS = [
    # Current position from your rostopic echo — use as reference/home
    {
        "right_j0": -0.0334,
        "right_j1": -2.0752,
        "right_j2":  0.0657,
        "right_j3":  1.3148,
        "right_j4": -0.0649,
        "right_j5":  0.3539,
        "right_j6":  1.7660,
    },
    # ── Add more waypoints below ──────────────────────────────
    # Example: second position
    # {
    #     "right_j0":  0.00,
    #     "right_j1": -1.80,
    #     "right_j2":  0.00,
    #     "right_j3":  1.50,
    #     "right_j4":  0.00,
    #     "right_j5":  0.50,
    #     "right_j6":  0.00,
    # },
]

# ──────────────────────────────────────────────────────────────
# SETTINGS
# ──────────────────────────────────────────────────────────────
SPEED        = 0.15   # 0.0 → 1.0  (raise to go faster)
TIMEOUT      = 15.0   # seconds per waypoint before giving up
PAUSE_AFTER  = 0.5    # seconds to wait at each waypoint


def move_to_waypoints():
    """Move through WAYPOINTS in order and return when done."""

    rospy.init_node("move_to_waypoints", anonymous=True)

    rospy.loginfo("Getting robot state...")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    rospy.loginfo("Robot enabled.")

    limb = intera_interface.Limb("right")
    limb.set_joint_position_speed(SPEED)

    rospy.loginfo(f"Executing {len(WAYPOINTS)} waypoint(s) at speed {SPEED}...")

    for i, waypoint in enumerate(WAYPOINTS):
        if rospy.is_shutdown():
            break

        rospy.loginfo(f"Moving to waypoint {i + 1}/{len(WAYPOINTS)}...")
        limb.move_to_joint_positions(waypoint, timeout=TIMEOUT)
        rospy.sleep(PAUSE_AFTER)
        rospy.loginfo(f"Waypoint {i + 1} reached.")

    rospy.loginfo("All waypoints complete.")


if __name__ == "__main__":
    try:
        move_to_waypoints()
    except rospy.ROSInterruptException:
        pass