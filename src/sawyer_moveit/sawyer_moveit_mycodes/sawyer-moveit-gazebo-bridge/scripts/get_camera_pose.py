#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np

def get_camera_pose(target_frame="right_hand_camera_optical", source_frame="world"):
    """
    Fetches the XYZ position and Quaternion orientation of a camera frame 
    relative to a source frame (e.g., 'world').
    """
    # Initialize the TF buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    print(f"[INFO] Waiting for transform from '{source_frame}' to '{target_frame}'...")
    
    try:
        # Wait up to 3 seconds for the transform to become available
        trans = tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(3.0))
        
        # Extract Translation (XYZ)
        pos = np.array([
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z+0.93
        ])
        
        # Extract Rotation (Quaternion)
        quat = np.array([
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ])
        
        return pos, quat

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(f"[ERROR] TF Lookup failed: {e}")
        return None, None

if __name__ == "__main__":
    rospy.init_node('camera_pose_listener', anonymous=True)
    
    # Change to "head_camera_optical" if you are using the head camera
    camera_frame = "right_hand_camera_optical" 
    
    pos, quat = get_camera_pose(target_frame=camera_frame, source_frame="world")
    
    if pos is not None:
        print("\n── Current Camera Pose ──────────────────────────────")
        print(f"Translation (XYZ) : {pos.tolist()}")
        print(f"Quaternion (XYZW) : {quat.tolist()}")
        print("─────────────────────────────────────────────────────\n")