#!/usr/bin/env python3
"""
bbox_to_3d.py
=============
ROS1 Noetic | Sawyer robot | right_hand_camera

Given a YOLO-format label file and an RGB image, this script estimates
the 3-D world-frame position (x, y, z) of every bounding box by
deprojecting the box centre through the depth image.

INPUTS
------
  Positional:
    image_path   Path to the RGB image (.png / .jpg)
    labels_path  Path to the YOLO .txt label file  (one line per object:
                 class_id cx cy w h  — all normalised 0..1)

  Optional:
    --camera-x, --camera-y, --camera-z
    --camera-qx, --camera-qy, --camera-qz, --camera-qw
        Camera pose in world frame.  If omitted the pose is looked up
        live from the TF tree  (right_hand_camera → world).

OUTPUTS
-------
  Prints a table of object positions to stdout.
  Optionally saves a visualisation image with projected boxes overlaid.

TOPICS USED
-----------
  /io/internal_camera/right_hand_camera/depth/image_raw   (32FC1, metres)
  /right_hand_camera/depth/camera_info
  /tf  /tf_static                                          (for auto pose)

REQUIREMENTS
------------
  pip install opencv-python numpy
  ROS: rospy, sensor_msgs, geometry_msgs, cv_bridge, tf, tf2_ros
"""

import argparse
import math
import sys
import os

import numpy as np
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as RosImage, CameraInfo

import tf2_ros
import tf2_geometry_msgs          # noqa – registers transforms
from geometry_msgs.msg import PointStamped, TransformStamped

# ═══════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════

DEPTH_TOPIC    = "/io/internal_camera/right_hand_camera/depth/image_raw"
CAMINFO_TOPIC  = "/right_hand_camera/depth/camera_info"

# TF frames
CAMERA_FRAME = "right_hand_camera"
WORLD_FRAME  = "world"

# How many seconds to wait for depth / camera_info at startup
TOPIC_TIMEOUT = 10.0

# Class names — must match the order used during training
CLASS_NAMES = [
    "mustard",            # 0
    "coke_can",           # 1
    "bowl",               # 2
    "banana",             # 3
    "strawberry",         # 4
    "planta_maceta",      # 5
    "esponja_lavaplatos",  # 6
    "papas_fritas",       # 7
    "block",              # 8
]

# Colours for visualisation (BGR)
COLOURS = [
    (0,   255, 0),    (0,   0,   255),  (255, 0,   0),
    (0,   255, 255),  (255, 0,   255),  (255, 255, 0),
    (128, 0,   255),  (0,   128, 255),  (255, 128, 0),
]

# ═══════════════════════════════════════════════════════════════════════════
#  ARGUMENT PARSING
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description="Estimate 3-D world positions from YOLO bboxes + depth.")

    p.add_argument("image_path",  help="Path to the RGB image")
    p.add_argument("labels_path", help="Path to the YOLO .txt label file")

    p.add_argument("--camera-x",  type=float, default=None)
    p.add_argument("--camera-y",  type=float, default=None)
    p.add_argument("--camera-z",  type=float, default=None)
    p.add_argument("--camera-qx", type=float, default=None)
    p.add_argument("--camera-qy", type=float, default=None)
    p.add_argument("--camera-qz", type=float, default=None)
    p.add_argument("--camera-qw", type=float, default=None)

    p.add_argument("--vis-out", default=None,
                   help="If given, save a visualisation PNG to this path.")
    p.add_argument("--depth-radius", type=int, default=5,
                   help="Pixel radius for median depth sampling (default 5).")

    # rospy injects its own args; strip them
    import sys
    args = p.parse_args([a for a in sys.argv[1:]
                         if not a.startswith("__")])
    return args


# ═══════════════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════════════

def load_yolo_labels(path, img_w, img_h):
    """
    Parse a YOLO label file and return a list of dicts:
      { class_id, class_name, cx_px, cy_px, w_px, h_px,
        x1, y1, x2, y2 }   (pixel coordinates)
    """
    detections = []
    if not os.path.isfile(path):
        rospy.logerr(f"[bbox3d] Label file not found: {path}")
        return detections

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 5:
                rospy.logwarn(f"[bbox3d] Skipping malformed line: {line}")
                continue
            cid, cx, cy, bw, bh = int(parts[0]), float(parts[1]), \
                                   float(parts[2]), float(parts[3]), float(parts[4])

            cx_px = cx * img_w
            cy_px = cy * img_h
            bw_px = bw * img_w
            bh_px = bh * img_h

            detections.append({
                "class_id":   cid,
                "class_name": CLASS_NAMES[cid] if cid < len(CLASS_NAMES) else str(cid),
                "cx_px": cx_px,
                "cy_px": cy_px,
                "w_px":  bw_px,
                "h_px":  bh_px,
                "x1": int(cx_px - bw_px / 2),
                "y1": int(cy_px - bh_px / 2),
                "x2": int(cx_px + bw_px / 2),
                "y2": int(cy_px + bh_px / 2),
            })
    return detections


def sample_depth(depth_img, cx_px, cy_px, radius):
    """
    Return the median of valid (non-NaN, non-zero, finite) depth values
    inside a square patch of side 2*radius+1 centred at (cx_px, cy_px).
    Returns NaN if no valid samples found.
    """
    h, w = depth_img.shape[:2]
    x0 = max(0, int(cx_px) - radius)
    x1 = min(w, int(cx_px) + radius + 1)
    y0 = max(0, int(cy_px) - radius)
    y1 = min(h, int(cy_px) + radius + 1)

    patch = depth_img[y0:y1, x0:x1].astype(np.float32)
    valid = patch[np.isfinite(patch) & (patch > 0.01)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


def deproject_pixel(u, v, depth_m, K):
    """
    Back-project pixel (u, v) at depth depth_m using camera matrix K
    into camera-frame 3-D point (x_cam, y_cam, z_cam).
    Standard optical convention: z forward, x right, y down.
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return np.array([x, y, z], dtype=float)


def camera_to_world(pt_cam, cam_pos, cam_quat_xyzw):
    """
    Transform a camera-frame point to world frame.
    cam_pos:        np.array [x, y, z]
    cam_quat_xyzw:  np.array [qx, qy, qz, qw]
    """
    from tf.transformations import quaternion_matrix
    q = cam_quat_xyzw
    R = quaternion_matrix(q)[:3, :3]   # camera_frame→world rotation

    # The ROS optical frame convention is:
    #   x right, y down, z forward  (camera frame)
    # world frame: x forward, y left, z up  (Gazebo/ROS convention)
    # We need to rotate from optical to Gazebo camera frame first:
    #   Gazebo cam: x forward, y left, z up
    #   Optical:    z forward, x right, y down
    # optical→gazebo_cam:
    R_opt2gz = np.array([
        [ 0, -1,  0],
        [ 0,  0, -1],
        [ 1,  0,  0],
    ], dtype=float)

    pt_gz  = R_opt2gz @ pt_cam        # optical → gazebo camera frame
    pt_world = R @ pt_gz + cam_pos    # gazebo camera frame → world
    return pt_world


# ═══════════════════════════════════════════════════════════════════════════
#  ROS NODE
# ═══════════════════════════════════════════════════════════════════════════

class BBoxTo3D:

    def __init__(self, args):
        self.args   = args
        self.bridge = CvBridge()

        self._depth_img  = None
        self._K          = None
        self._depth_lock = __import__("threading").Lock()

        # Subscribe to depth + camera info
        rospy.Subscriber(DEPTH_TOPIC,   RosImage,    self._depth_cb,   queue_size=1)
        rospy.Subscriber(CAMINFO_TOPIC, CameraInfo,  self._caminfo_cb, queue_size=1)

        # TF buffer (used only when camera pose not given on CLI)
        self._tf_buffer   = tf2_ros.Buffer(rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    # ── Callbacks ────────────────────────────────────────────────────────

    def _depth_cb(self, msg):
        try:
            # 32FC1 = float32 metres; 16UC1 = uint16 millimetres
            if msg.encoding == "32FC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            elif msg.encoding in ("16UC1", "mono16"):
                depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding="16UC1").astype(np.float32) / 1000.0
            else:
                rospy.logwarn_once(
                    f"[bbox3d] Unexpected depth encoding: {msg.encoding}")
                depth = self.bridge.imgmsg_to_cv2(msg).astype(np.float32)
            with self._depth_lock:
                self._depth_img = depth
        except CvBridgeError as e:
            rospy.logwarn(f"[bbox3d] Depth CvBridge error: {e}")

    def _caminfo_cb(self, msg):
        if self._K is None:
            self._K = np.array(msg.K, dtype=float).reshape(3, 3)
            rospy.loginfo("[bbox3d] Camera intrinsics received.")

    # ── Camera pose ───────────────────────────────────────────────────────

    def _get_camera_pose(self):
        """
        Returns (position np.array[3], quaternion np.array[4] xyzw).
        Uses CLI args if provided, otherwise looks up TF.
        """
        args = self.args
        cli_pos  = [args.camera_x, args.camera_y, args.camera_z]
        cli_quat = [args.camera_qx, args.camera_qy,
                    args.camera_qz, args.camera_qw]

        if all(v is not None for v in cli_pos + cli_quat):
            rospy.loginfo("[bbox3d] Using camera pose from command-line arguments.")
            return np.array(cli_pos, dtype=float), np.array(cli_quat, dtype=float)

        # Auto-detect from TF tree
        rospy.loginfo(f"[bbox3d] Looking up TF: {CAMERA_FRAME} → {WORLD_FRAME} …")
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                WORLD_FRAME,
                CAMERA_FRAME,
                rospy.Time(0),              # latest available
                rospy.Duration(5.0)
            )
            t = tf_stamped.transform.translation
            r = tf_stamped.transform.rotation
            pos  = np.array([t.x, t.y, t.z], dtype=float)
            quat = np.array([r.x, r.y, r.z, r.w], dtype=float)
            rospy.loginfo(
                f"[bbox3d] TF pose: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})"
                f"  quat=({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})"
            )
            return pos, quat
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(
                f"[bbox3d] TF lookup failed: {e}\n"
                "  → Provide camera pose via --camera-x/y/z and --camera-qx/y/z/w"
            )
            sys.exit(1)

    # ── Wait for data ─────────────────────────────────────────────────────

    def _wait_for_data(self):
        rospy.loginfo(f"[bbox3d] Waiting for depth image on {DEPTH_TOPIC} …")
        deadline = rospy.Time.now() + rospy.Duration(TOPIC_TIMEOUT)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self._depth_lock:
                has_depth = self._depth_img is not None
            if has_depth and self._K is not None:
                rospy.loginfo("[bbox3d] Depth image and camera info ready.")
                return
            if rospy.Time.now() > deadline:
                rospy.logerr(
                    "[bbox3d] Timed out waiting for depth data.\n"
                    f"  depth:  {'OK' if has_depth else 'MISSING'}\n"
                    f"  K matrix: {'OK' if self._K is not None else 'MISSING'}\n"
                    f"  Check topics:\n"
                    f"    rostopic hz {DEPTH_TOPIC}\n"
                    f"    rostopic hz {CAMINFO_TOPIC}"
                )
                sys.exit(1)
            rate.sleep()

    # ── Main ──────────────────────────────────────────────────────────────

    def run(self):
        self._wait_for_data()

        # Load RGB image
        rgb = cv2.imread(self.args.image_path)
        if rgb is None:
            rospy.logerr(f"[bbox3d] Cannot read image: {self.args.image_path}")
            sys.exit(1)
        img_h, img_w = rgb.shape[:2]
        rospy.loginfo(f"[bbox3d] Image: {img_w}×{img_h}  ({self.args.image_path})")

        # Load YOLO labels
        detections = load_yolo_labels(self.args.labels_path, img_w, img_h)
        if not detections:
            rospy.logwarn("[bbox3d] No detections found in label file.")
            sys.exit(0)
        rospy.loginfo(f"[bbox3d] {len(detections)} detection(s) loaded.")

        # Camera intrinsics + pose
        K = self._K
        with self._depth_lock:
            depth_img = self._depth_img.copy()

        cam_pos, cam_quat = self._get_camera_pose()

        # ── Process each detection ────────────────────────────────────────
        results = []
        for det in detections:
            cx_px = det["cx_px"]
            cy_px = det["cy_px"]

            depth_m = sample_depth(depth_img, cx_px, cy_px,
                                   self.args.depth_radius)

            if math.isnan(depth_m):
                world_pos = None
                cam_pt    = None
                rospy.logwarn(
                    f"[bbox3d]  {det['class_name']:22s} — no valid depth at "
                    f"pixel ({cx_px:.0f}, {cy_px:.0f})"
                )
            else:
                cam_pt    = deproject_pixel(cx_px, cy_px, depth_m, K)
                world_pos = camera_to_world(cam_pt, cam_pos, cam_quat)

            results.append({**det,
                            "depth_m":   depth_m,
                            "cam_pt":    cam_pt,
                            "world_pos": world_pos})

        # ── Print results table ───────────────────────────────────────────
        sep = "─" * 80
        print(f"\n{sep}")
        print(f"  {'CLASS':<22} {'ID':>3}  {'DEPTH':>7}  "
              f"{'X_world':>9}  {'Y_world':>9}  {'Z_world':>9}")
        print(sep)
        for r in results:
            if r["world_pos"] is not None:
                wx, wy, wz = r["world_pos"]
                print(f"  {r['class_name']:<22} {r['class_id']:>3}  "
                      f"{r['depth_m']:>6.3f}m  "
                      f"{wx:>+9.4f}  {wy:>+9.4f}  {wz:>+9.4f}")
            else:
                print(f"  {r['class_name']:<22} {r['class_id']:>3}  "
                      f"   N/A    {'':>9}  {'':>9}  {'':>9}  (no depth)")
        print(f"{sep}\n")

        # ── Visualisation ─────────────────────────────────────────────────
        if self.args.vis_out:
            vis = rgb.copy()
            for r in results:
                colour = COLOURS[r["class_id"] % len(COLOURS)]
                cv2.rectangle(vis,
                              (r["x1"], r["y1"]),
                              (r["x2"], r["y2"]),
                              colour, 2)
                if r["world_pos"] is not None:
                    wx, wy, wz = r["world_pos"]
                    label_str = (f"{r['class_name']}  "
                                 f"({wx:+.2f}, {wy:+.2f}, {wz:+.2f})")
                else:
                    label_str = f"{r['class_name']}  [no depth]"

                # Background for text legibility
                (tw, th), bl = cv2.getTextSize(
                    label_str, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                tx = r["x1"]
                ty = max(r["y1"] - 5, th + 4)
                cv2.rectangle(vis,
                              (tx, ty - th - bl - 2),
                              (tx + tw + 2, ty + 2),
                              colour, -1)
                cv2.putText(vis, label_str,
                            (tx + 1, ty - bl),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (0, 0, 0), 1, cv2.LINE_AA)

            cv2.imwrite(self.args.vis_out, vis)
            rospy.loginfo(f"[bbox3d] Visualisation saved → {self.args.vis_out}")


# ═══════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()
    rospy.init_node("bbox_to_3d", anonymous=True)
    node = BBoxTo3D(args)
    node.run()
