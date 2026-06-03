#!/usr/bin/env python3

import sys
import rospy
import intera_interface
from math import sqrt, radians, degrees, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from intera_core_msgs.msg import EndpointState
from trac_ik_python.trac_ik import IK


# ──────────────────────────────────────────────────────────────
# SPEED
#   Controls intera_interface.Limb joint motion speed.
#   Range: 0.0 → 1.0
#   Real robot: keep ≤ 0.3 until paths are verified safe
#   Gazebo:     up to 0.6 before PID overshoots
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING = 0.08     # ← raise to go faster
MOVE_TIMEOUT     = 20.0     # seconds — increase if arm times out mid-motion


# ──────────────────────────────────────────────────────────────
# SAWYER GEOMETRY
# ──────────────────────────────────────────────────────────────
SAWYER_BASE_Z = 0.93    # robot base height above Gazebo world origin (metres)

# trac_ik chain — verify these match your URDF with:
#   rosparam get /robot_description | grep -o 'name="[^"]*"' | head -30
# For Sawyer the base link is typically "base" or "right_arm_base_link"
IK_BASE_LINK = "base"               # ← change if IK fails to initialise
IK_TIP_LINK  = "right_gripper_tip"

# Sawyer right arm joint names in order (must match URDF chain above)
JOINT_NAMES = [
    "right_j0", "right_j1", "right_j2", "right_j3",
    "right_j4", "right_j5", "right_j6"
]


# ──────────────────────────────────────────────────────────────
# WORKSPACE  (world frame: Z=0 at ground level)
# ──────────────────────────────────────────────────────────────
X_MIN, X_MAX =  0.30,  0.90
Y_MIN, Y_MAX = -0.60,  0.60
Z_MIN, Z_MAX = 0.75,  2.0

MIN_MOVE_DIST = 0.05    # metres — below this the robot is already close enough
MAX_MOVE_DIST = 3.0    # metres — above this is likely a typo


# ──────────────────────────────────────────────────────────────
# ENDPOINT TOPIC
# Watch orientation live:
#   rostopic echo /robot/limb/right/endpoint_state | grep -A4 "orientation"
# ──────────────────────────────────────────────────────────────
ENDPOINT_TOPIC = "/robot/limb/right/endpoint_state"


# ──────────────────────────────────────────────────────────────
# IK SOLVER  (trac_ik — same role as sns_ik but with Python bindings)
# ──────────────────────────────────────────────────────────────
def make_ik_solver():
    """
    Creates a trac_ik solver for the Sawyer right arm.
    pos_tol  : 1 mm position tolerance
    ori_tol  : 0.1 rad (~5.7°) orientation tolerance — gives the solver
               freedom near kinematic boundaries (fixes the Z floor issue)
    """
    rospy.loginfo(f"Initialising trac_ik: {IK_BASE_LINK} → {IK_TIP_LINK}")
    solver = IK(
        IK_BASE_LINK,
        IK_TIP_LINK,
        timeout=0.05,       # seconds per IK attempt
        epsilon=0.001,      # position tolerance (metres)
        solve_type="Speed", # "Speed" | "Distance" | "Manip1" | "Manip2"
                            # Speed  → fastest solution
                            # Distance → solution closest to seed (smoother)
    )
    rospy.loginfo(f"trac_ik ready — {solver.number_of_joints} joints found")
    return solver


def solve_ik(solver, limb, x_world, y_world, z_world,
             roll_deg, pitch_deg, yaw_deg):
    """
    Solves IK for a world-frame target.
    Converts world → robot-base frame before calling trac_ik.
    Uses current joint positions as the seed (produces smoother motions).

    Returns list of joint angles (radians) or None on failure.
    """
    # World → robot base frame (only Z needs the offset)
    x_rb = x_world
    y_rb = y_world
    z_rb = z_world - SAWYER_BASE_Z

    rospy.loginfo(
        f"IK target — robot-base frame: "
        f"({x_rb:.3f}, {y_rb:.3f}, {z_rb:.3f})  "
        f"rpy=({roll_deg:.1f}°, {pitch_deg:.1f}°, {yaw_deg:.1f}°)"
    )

    # Target orientation as quaternion
    qx, qy, qz, qw = quaternion_from_euler(
        radians(roll_deg), radians(pitch_deg), radians(yaw_deg)
    )

    # Seed: current joint positions (dict → ordered list)
    current = limb.joint_angles()
    seed = [current.get(j, 0.0) for j in JOINT_NAMES]

    solution = solver.get_ik(
        seed,
        x_rb, y_rb, z_rb,
        qx, qy, qz, qw,
        # Per-axis tolerances (matching epsilon and ori_tol above)
        bx=0.001, by=0.001, bz=0.001,   # position (metres)
        brx=0.1,  bry=0.1,  brz=0.1     # orientation (radians)
    )

    if solution is None:
        rospy.logwarn(
            "trac_ik found no solution. "
            "Try a slightly different orientation or position."
        )
    return solution


# ──────────────────────────────────────────────────────────────
# CURRENT EEF STATE
# ──────────────────────────────────────────────────────────────
def get_current_eef_state():
    """
    Reads /robot/limb/right/endpoint_state.
    Returns:
        pos     : (x, y, z) in world frame
        rpy_deg : (roll, pitch, yaw) in degrees
        quat    : (x, y, z, w) quaternion
    or (None, None, None) on failure.

    Watch live:
        rostopic echo /robot/limb/right/endpoint_state | grep -A4 "orientation"
    """
    try:
        msg = rospy.wait_for_message(ENDPOINT_TOPIC, EndpointState, timeout=5.0)
        p = msg.pose.position
        o = msg.pose.orientation

        world_z = p.z + SAWYER_BASE_Z
        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])

        return (
            (p.x, p.y, world_z),
            (degrees(roll), degrees(pitch), degrees(yaw)),
            (o.x, o.y, o.z, o.w)
        )
    except rospy.ROSException as e:
        rospy.logwarn(f"Could not read {ENDPOINT_TOPIC}: {e}")
        return None, None, None


# ──────────────────────────────────────────────────────────────
# VALIDATION
# ──────────────────────────────────────────────────────────────
def validate_position(tx, ty, tz):
    """Bounding box + distance from current EEF."""
    if not (X_MIN <= tx <= X_MAX):
        return False, f"X={tx:.3f} out of range [{X_MIN}, {X_MAX}] m"
    if not (Y_MIN <= ty <= Y_MAX):
        return False, f"Y={ty:.3f} out of range [{Y_MIN}, {Y_MAX}] m"
    if not (Z_MIN <= tz <= Z_MAX):
        return False, (f"Z={tz:.3f} out of range [{Z_MIN}, {Z_MAX}] m "
                       f"(world frame, ground=0, robot base={SAWYER_BASE_Z} m)")

    pos, _, _ = get_current_eef_state()
    if pos is None:
        return True, "OK (distance check skipped — could not read EEF)"

    cx, cy, cz = pos
    dist = sqrt((tx-cx)**2 + (ty-cy)**2 + (tz-cz)**2)
    rospy.loginfo(
        f"EEF (world): ({cx:.3f}, {cy:.3f}, {cz:.3f})  "
        f"→ target: ({tx:.3f}, {ty:.3f}, {tz:.3f})  dist={dist:.4f} m"
    )

    if dist < MIN_MOVE_DIST:
        return False, f"Already at target (dist={dist:.4f} m < {MIN_MOVE_DIST} m)"
    if dist > MAX_MOVE_DIST:
        return False, f"Target too far (dist={dist:.3f} m > {MAX_MOVE_DIST} m)"

    return True, f"OK (dist={dist:.3f} m)"


# ──────────────────────────────────────────────────────────────
# INPUT HELPERS
# ──────────────────────────────────────────────────────────────
def ask_float(prompt):
    """Returns float, or exits on 'q'."""
    while True:
        raw = input(prompt).strip().lower()
        if raw == "q":
            print("Quit — shutting down.")
            sys.exit(0)
        try:
            return float(raw)
        except ValueError:
            print("  ⚠  Enter a number, e.g. 0.58")


def ask_angle(label, current_deg):
    """Returns float (degrees). Press Enter to keep current."""
    while True:
        raw = input(f"  {label} (deg) [{current_deg:.1f}]: ").strip().lower()
        if raw == "q":
            print("Quit — shutting down.")
            sys.exit(0)
        if raw == "":
            return current_deg
        try:
            return float(raw)
        except ValueError:
            print(f"  ⚠  Bad input — keeping {current_deg:.1f}°")


def ask_target():
    """
    Interactive prompt for position + orientation.
    Shows current EEF state as reference before every input.
    Returns (x, y, z, roll_deg, pitch_deg, yaw_deg).
    """
    while True:
        print("\n" + "─" * 58)
        print(" Enter target  (world frame, metres)  —  'q' to quit")
        print(f"  Workspace  X[{X_MIN},{X_MAX}]  "
              f"Y[{Y_MIN},{Y_MAX}]  Z[{Z_MIN},{Z_MAX}]")

        pos, rpy, quat = get_current_eef_state()
        if pos:
            print(f"\n  Current position    : "
                  f"x={pos[0]:.3f}  y={pos[1]:.3f}  z={pos[2]:.3f}")
            print(f"  Current orientation : "
                  f"roll={rpy[0]:.1f}°  pitch={rpy[1]:.1f}°  yaw={rpy[2]:.1f}°")
            print(f"  Current quaternion  : "
                  f"x={quat[0]:.4f}  y={quat[1]:.4f}  "
                  f"z={quat[2]:.4f}  w={quat[3]:.4f}")
            print(f"\n  (Live orientation: "
                  f"rostopic echo {ENDPOINT_TOPIC} | grep -A4 'orientation')")

        # Position
        print("\n  --- Position ---")
        x = ask_float("  X (m): ")
        y = ask_float("  Y (m): ")
        z = ask_float("  Z (m): ")

        ok, reason = validate_position(x, y, z)
        if not ok:
            print(f"  ✗  Rejected: {reason}\n")
            continue

        # Orientation  (Enter = keep current)
        print("\n  --- Orientation  (Enter keeps current value) ---")
        cur_r = rpy[0] if rpy else 0.0
        cur_p = rpy[1] if rpy else 0.0
        cur_y = rpy[2] if rpy else 0.0

        roll  = ask_angle("Roll ", cur_r)
        pitch = ask_angle("Pitch", cur_p)
        yaw   = ask_angle("Yaw  ", cur_y)

        print(f"\n  ✓  Position    : ({x:.3f}, {y:.3f}, {z:.3f})  {reason}")
        print(f"  ✓  Orientation : "
              f"roll={roll:.1f}°  pitch={pitch:.1f}°  yaw={yaw:.1f}°")

        return x, y, z, roll, pitch, yaw


# ──────────────────────────────────────────────────────────────
# MOTION  — trac_ik + intera direct joint control
# ──────────────────────────────────────────────────────────────
def move_to(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg):
    """
    1. Solves IK with trac_ik (no MoveIt, no OctoMap consulted).
    2. Executes via intera_interface.Limb.move_to_joint_positions()
       (direct joint control — completely bypasses MoveIt collision world).
    """
    solution = solve_ik(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg)

    if solution is None:
        rospy.logwarn("✗ No IK solution found — skipping move.")
        return False

    joint_goal = dict(zip(JOINT_NAMES, solution))
    rospy.loginfo(f"IK solution found: { {k: f'{v:.3f}' for k,v in joint_goal.items()} }")
    rospy.loginfo(f"Moving at speed {VELOCITY_SCALING} ...")

    # set_joint_position_speed controls intera's internal joint velocity limit
    limb.set_joint_position_speed(VELOCITY_SCALING)

    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)

    rospy.loginfo("✓ Motion complete.")
    return True


# ──────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────
def main():
    rospy.init_node("sawyer_tracik_move", anonymous=True)

    rospy.loginfo("Opening gripper ...")
    gripper = intera_interface.Gripper("right_gripper")
    gripper.open()
    rospy.sleep(1.0)

    rospy.loginfo("Initialising limb ...")
    limb = intera_interface.Limb("right")

    rospy.loginfo("Initialising IK solver ...")
    solver = make_ik_solver()

    print("\n" + "=" * 58)
    print(" Sawyer — trac_ik direct joint control")
    print(f"  Speed          : {VELOCITY_SCALING}   ← raise to go faster")
    print(f"  Z offset       : topic_z + {SAWYER_BASE_Z} = world_z")
    print(f"  IK chain       : {IK_BASE_LINK} → {IK_TIP_LINK}")
    print(f"  OctoMap        : bypassed (no MoveIt planner)")
    print(f"\n  Live orientation:")
    print(f"    rostopic echo {ENDPOINT_TOPIC} | grep -A4 'orientation'")
    print("=" * 58)

    while not rospy.is_shutdown():
        x, y, z, roll, pitch, yaw = ask_target()
        move_to(solver, limb, x, y, z, roll, pitch, yaw)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
