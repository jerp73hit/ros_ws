#!/usr/bin/env python3
import rospy
import json
import numpy as np
from gazebo_msgs.msg import ModelStates
from get_camera_pose import get_camera_pose

# Object half-extents from Bbox_to_world.py
OBJECT_CLASSES = {
    "mustard":            (0,  (0.038, 0.030, 0.075)),
    "coke_can":           (1,  (0.033, 0.033, 0.058)),
    "bowl":               (2,  (0.095, 0.095, 0.028)),
    "banana":             (3,  (0.110, 0.030, 0.022)),
    "strawberry":         (4,  (0.025, 0.025, 0.022)),
    "planta_maceta":      (5,  (0.065, 0.065, 0.090)),
    "esponja_lavaplatos": (6,  (0.050, 0.035, 0.022)),
    "papas_fritas":       (7,  (0.038, 0.038, 0.045)),
    "block":              (8,  (0.028, 0.028, 0.028)),
}

def get_gazebo_states():
    print("[INFO] Waiting for /gazebo/model_states topic...")
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
    gt_dict = {}
    for name, pose in zip(msg.name, msg.pose):
        # Extract ground truth XYZ
        gt_dict[name] = np.array([pose.position.x, pose.position.y, pose.position.z])
    return gt_dict

def main():
    rospy.init_node('depth_comparator', anonymous=True)
    
    # ── TRUE CAMERA POSITION ────────────────────────────────────────────────
    # This must match your physical world (TF Z + 0.93m Pedestal height)
    #cam_pos = get_camera_pose()#np.array([0.236, 0.141, 1.847]) 
    # ── TRUE CAMERA POSITION ────────────────────────────────────────────────
    # Unpack both the position and the quaternion from the function
    cam_pos, cam_quat = get_camera_pose()
    
    if cam_pos is None:
        print("[ERROR] Could not get camera pose. Exiting.")
        return
    # ────────────────────────────────────────────────────────────────────────

    try:
        gt_states = get_gazebo_states()
    except rospy.ROSException:
        print("[ERROR] Could not read /gazebo/model_states. Is Gazebo running?")
        return

    try:
        with open("detections_world.json", "r") as f:
            detections = json.load(f)
    except FileNotFoundError:
        print("[ERROR] detections_world.json not found. Run Bbox_to_world.py first.")
        return

    print("\n" + "=" * 105)
    print(f"{'Object':<20} | {'Cam Measured Z':<16} | {'GT Z-Depth (Top)':<16} | {'GT Z-Depth (Base)':<17} | {'Euclidean Dist'}")
    print("=" * 105)

    for det in detections:
        name = det["name"]
        if name not in gt_states:
            continue
            
        cam_depth = det["depth_m"]
        gt_base_xyz = gt_states[name]
        
        # Calculate where the physical top of the object is
        # Full height is 2 * half-extent (hz)
        hz = OBJECT_CLASSES[name][1][2]
        gt_top_xyz = gt_base_xyz.copy()
        gt_top_xyz[2] += (hz * 2)
        
        # True Z-Depth is the vertical drop from the lens to the target
        gt_depth_base = cam_pos[2] - gt_base_xyz[2]
        gt_depth_top = cam_pos[2] - gt_top_xyz[2]
        
        # Euclidean distance is the straight-line ray (tape measure distance)
        euclidean_dist = np.linalg.norm(cam_pos - gt_base_xyz)
        
        print(f"{name:<20} | {cam_depth:>14.3f} m | {gt_depth_top:>14.3f} m | {gt_depth_base:>15.3f} m | {euclidean_dist:>12.3f} m")
        
    print("-" * 105)
    print(" * 'Cam Measured Z' is what your depth.npy file recorded.")
    print(" * 'GT Z-Depth (Top)' is the physical Z distance from the lens to the upper surface of the object.")
    print(" * 'GT Z-Depth (Base)' is the physical Z distance to the table underneath the object.")
    print(" * 'Euclidean Dist' is the straight-line distance, which causes Parallax shift.")

if __name__ == "__main__":
    main()