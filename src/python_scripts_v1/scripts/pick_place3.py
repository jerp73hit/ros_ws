#!/usr/bin/env python3

import sys
import math
import random
import rospy
import intera_interface
from math import radians, degrees
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from trac_ik_python.trac_ik import IK
from gazebo_msgs.srv import GetModelState

# ──────────────────────────────────────────────────────────────
# GLOBAL CONFIGURATION
# ──────────────────────────────────────────────────────────────
VELOCITY_SCALING = 0.1      # Default smooth speed
MOVE_TIMEOUT     = 28.0     # Max time for IK solver / execution
SAWYER_BASE_Z    = 0.93     
CUBE_NAME        = "block"  # Gazebo model name

# Vital: Positive to grab from above, avoiding table collisions
Z_OFFSET = 0.012

IK_BASE_LINK = "base"
IK_TIP_LINK  = "right_gripper_tip"

JOINT_NAMES = [
    "right_j0", "right_j1", "right_j2", "right_j3",
    "right_j4", "right_j5", "right_j6"
]

SAFE_POSE = [
    -0.041663, -1.025829, 0.029368, 2.175181,
    -0.067030, 0.396837, 1.765965
]

# Workingspace limits and default orientations
PLACE_X_MIN, PLACE_X_MAX =  0.45, 0.75
PLACE_Y_MIN, PLACE_Y_MAX = -0.30, 0.30
SAFE_Z = 0.93   

BASE_ROLL  = 180.0
BASE_PITCH = 0.0
DROP_YAW   = 180.0

# ──────────────────────────────────────────────────────────────
# IK & KINEMATICS
# ──────────────────────────────────────────────────────────────
def create_ik_solver() -> IK:
    return IK(
        IK_BASE_LINK, IK_TIP_LINK,
        timeout=0.05, epsilon=0.001, 
        solve_type="Speed"
    )

def compute_ik(solver, limb, x_world, y_world, z_world, roll_deg, pitch_deg, yaw_deg):
    z_rb = z_world - SAWYER_BASE_Z 
    qx, qy, qz, qw = quaternion_from_euler(
        radians(roll_deg), radians(pitch_deg), radians(yaw_deg)
    )

    current = limb.joint_angles()
    seed = [current.get(j, 0.0) for j in JOINT_NAMES]

    return solver.get_ik(
        seed, x_world, y_world, z_rb, qx, qy, qz, qw,
        bx=0.001, by=0.001, bz=0.001,
        brx=0.1,  bry=0.1,  brz=0.1
    )

def move_to_pose(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg, velocity=None) -> bool:
    if velocity is None:
        velocity = VELOCITY_SCALING

    solution = compute_ik(solver, limb, x, y, z, roll_deg, pitch_deg, yaw_deg)
    if not solution:
        return False
    
    joint_goal = dict(zip(JOINT_NAMES, solution))
    limb.set_joint_position_speed(velocity)
    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)
    return True

def move_to_safe_pose(limb):
    """Moves arm directly to predefined safe joint positions."""
    rospy.loginfo("Moving to safe joint pose...")
    joint_goal = dict(zip(JOINT_NAMES, SAFE_POSE))
    limb.set_joint_position_speed(VELOCITY_SCALING)
    limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)

def move_straight_z(solver, limb, x, y, z_start, z_end, roll, pitch, yaw, steps=10) -> bool:
    """Forces vertical movement via Cartesian waypoints to prevent IK contortions."""
    distance = z_end - z_start
    step_inc = distance / steps
    
    for i in range(1, steps + 1):
        z_current = z_start + (step_inc * i)
        success = move_to_pose(solver, limb, x, y, z_current, roll, pitch, yaw, velocity=0.05)
        if not success:
            return False
        rospy.sleep(0.05) 
    return True

# ──────────────────────────────────────────────────────────────
# SENSORY INTERFACE (GAZEBO 3D MATH)
# ──────────────────────────────────────────────────────────────
def get_cube_state() -> tuple:
    """Retrieves object state using rotation matrices to avoid Gimbal Lock."""
    rospy.loginfo(f"Locating '{CUBE_NAME}' in Gazebo...")
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(CUBE_NAME, 'world') 
        
        if not resp.success:
            rospy.logwarn(f"Object '{CUBE_NAME}' not found.")
            return None, None, None, None, None, None
            
        pos = resp.pose.position
        ori = resp.pose.orientation
        
        # Quaternion to rotation matrix for roll/pitch immunity
        q = [ori.x, ori.y, ori.z, ori.w]
        R = quaternion_matrix(q)
        
        # Directional vectors for local axes
        vec_x, vec_y, vec_z = R[:3, 0], R[:3, 1], R[:3, 2]
        
        # Find the axis most aligned with the horizontal plane (smallest Z)
        axes = [vec_x, vec_y, vec_z]
        flat_axis = min(axes, key=lambda v: abs(v[2]))
        
        true_yaw_rad = math.atan2(flat_axis[1], flat_axis[0])
        
        return pos.x, pos.y, pos.z, 0.0, 0.0, math.degrees(true_yaw_rad)
        
    except Exception as e:
        rospy.logerr(f"Gazebo service failed: {e}")
        return None, None, None, None, None, None

def generate_valid_destination(solver, limb, z_world) -> tuple:
    """Finds a reachable XY drop coordinate within limits."""
    for _ in range(10):
        rx = random.uniform(PLACE_X_MIN, PLACE_X_MAX)
        ry = random.uniform(PLACE_Y_MIN, PLACE_Y_MAX)
        if compute_ik(solver, limb, rx, ry, z_world, BASE_ROLL, BASE_PITCH, DROP_YAW):
            return rx, ry
    return None, None

# ──────────────────────────────────────────────────────────────
# MAIN SEQUENCE
# ──────────────────────────────────────────────────────────────
def main():
    rospy.init_node("sawyer_smart_pick_place", anonymous=True)

    gripper = intera_interface.Gripper("right_gripper")
    limb = intera_interface.Limb("right")
    solver = create_ik_solver()

    rospy.loginfo("Initializing hardware...")
    gripper.open()
    move_to_safe_pose(limb)

    while not rospy.is_shutdown():
        print("\n" + "=" * 60)
        user_input = input(" Press ENTER to scan and execute (or 'q' to quit): ")
        if user_input.strip().lower() == 'q':
            sys.exit(0)

        # 1. Perception
        pick_x, pick_y, pick_z, _, _, cube_yaw = get_cube_state()
        if pick_x is None: continue
            
        place_x, place_y = generate_valid_destination(solver, limb, pick_z)
        if place_x is None:
            rospy.logwarn("Destination unreachable. Retrying...")
            continue

        print(f" ✓ Origin: X={pick_x:.3f}, Y={pick_y:.3f} | Target Yaw: {cube_yaw:.1f}°")
        print("-" * 60)

        # ──────────────────────────────────────────────────
        # PHASE 1: APPROACH (XY)
        # ──────────────────────────────────────────────────
        gripper.open()
        rospy.sleep(0.5)

        rospy.loginfo("Phase 1: Hovering over target (XY)...")
        if not move_to_pose(solver, limb, pick_x, pick_y, SAFE_Z, BASE_ROLL, BASE_PITCH, 0.0):
            rospy.logwarn("Path blocked. Aborting.")
            move_to_safe_pose(limb)
            continue
        
        rospy.sleep(0.8) 

        # ──────────────────────────────────────────────────
        # PHASE 2: ALIGNMENT (Yaw Delta)
        # ──────────────────────────────────────────────────
        rospy.loginfo("Phase 2: Calculating yaw delta...")
        pose_actual = limb.endpoint_pose()
        q_actual = pose_actual['orientation']
        _, _, yaw_gripper_rad = euler_from_quaternion([q_actual.x, q_actual.y, q_actual.z, q_actual.w])
        yaw_gripper = degrees(yaw_gripper_rad)
        
        delta_yaw = cube_yaw - yaw_gripper

        # Geometry optimization: clamp to nearest 90-degree flat face
        while delta_yaw > 45.0:   delta_yaw -= 90.0
        while delta_yaw < -45.0:  delta_yaw += 90.0

        target_yaw_gripper = yaw_gripper + delta_yaw
        rospy.loginfo(f" -> Applying {delta_yaw:.1f}° wrist rotation...")
        
        move_to_pose(solver, limb, pick_x, pick_y, SAFE_Z, BASE_ROLL, BASE_PITCH, target_yaw_gripper)
        rospy.sleep(0.5)

        # ──────────────────────────────────────────────────
        # PHASE 3: EXTRACTION (Z-Linear)
        # ──────────────────────────────────────────────────
        rospy.loginfo("Phase 3: Linear Z descent...")
        move_straight_z(
            solver, limb, pick_x, pick_y, 
            SAFE_Z, pick_z + Z_OFFSET, 
            BASE_ROLL, BASE_PITCH, target_yaw_gripper, steps=10
        )
        
        rospy.loginfo(" -> Grasping...")
        gripper.close()
        rospy.sleep(1.0) 

        rospy.loginfo(" -> Linear Z ascent...")
        move_straight_z(
            solver, limb, pick_x, pick_y, 
            pick_z + Z_OFFSET, SAFE_Z, 
            BASE_ROLL, BASE_PITCH, target_yaw_gripper, steps=10
        )
        
        # ──────────────────────────────────────────────────
        # PHASE 4: DROP
        # ──────────────────────────────────────────────────
        rospy.loginfo("Phase 4: Moving to drop location...")
        move_to_pose(solver, limb, place_x, place_y, SAFE_Z, BASE_ROLL, BASE_PITCH, DROP_YAW)
        rospy.sleep(0.5)

        rospy.loginfo(" -> Linear Z descent...")
        move_straight_z(
            solver, limb, place_x, place_y, 
            SAFE_Z, pick_z + Z_OFFSET, 
            BASE_ROLL, BASE_PITCH, DROP_YAW, steps=10
        )
        
        rospy.loginfo(" -> Releasing...")
        gripper.open()
        rospy.sleep(1.0) 

        rospy.loginfo(" -> Linear Z ascent...")
        move_straight_z(
            solver, limb, place_x, place_y, 
            pick_z + Z_OFFSET, SAFE_Z, 
            BASE_ROLL, BASE_PITCH, DROP_YAW, steps=10
        )
        
        rospy.loginfo("Returning to standby...")
        move_to_safe_pose(limb) 

        rospy.loginfo("✓ Cycle completed successfully.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
