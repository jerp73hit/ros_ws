#!/usr/bin/env python3
"""
Object 3D Position Estimator — Right Hand Camera (top-down)
─────────────────────────────────────────────────────────────
Topics used:
  RGB   : /io/internal_camera/right_hand_camera/image_raw
  Depth : /io/internal_camera/right_hand_camera/depth/image_raw
  Info  : /io/internal_camera/right_hand_camera/camera_info

With a top-down camera:
  - depth directly = Z height of the object above the table
  - X/Y from deprojection = horizontal position on the table
  This gives the cleanest possible 3D estimate.
"""

import rospy
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from threading import Lock


# ──────────────────────────────────────────────────────────────────────────────
# TOPIC NAMES  (from your rostopic list output)
# ──────────────────────────────────────────────────────────────────────────────
RGB_TOPIC         = "/io/internal_camera/right_hand_camera/image_raw"
DEPTH_TOPIC       = "/io/internal_camera/right_hand_camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/io/internal_camera/right_hand_camera/camera_info"

# TF frames — verify with: rosrun tf tf_echo world right_hand_camera
WORLD_FRAME  = "world"
CAMERA_FRAME = "right_hand_camera"   # ← change if TF lookup fails

# ──────────────────────────────────────────────────────────────────────────────
# DEFAULT BOUNDING BOXES
# Measured from your new top-down images (1220×868 px approx).
# Replace with your AI detector output at runtime via get_object_positions().
#
# Object layout visible in your RGB image:
#   bowl         → large circle, left side
#   coke_can     → dark cylinder below bowl
#   planta_maceta→ green plant, top center
#   strawberry   → small red ball, center
#   mustard      → yellow bottle, bottom left
#   banana       → curved shape, bottom center
#   block_red    → red cube, right center
#   block_green  → green rectangle, bottom right
#   chips_bag    → small yellow pack, right of center
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_BOUNDING_BOXES = [
    {"label": "bowl",           "x1": 370, "y1": 200, "x2": 564, "y2": 354},
    {"label": "coke_can",       "x1": 333, "y1": 368, "x2": 420, "y2": 464},
    {"label": "planta_maceta",  "x1": 586, "y1": 164, "x2": 707, "y2": 315},
    {"label": "strawberry",     "x1": 622, "y1": 408, "x2": 669, "y2": 452},
    {"label": "mustard",        "x1": 491, "y1": 500, "x2": 565, "y2": 622},
    {"label": "banana",         "x1": 610, "y1": 511, "x2": 702, "y2": 697},
    {"label": "block_red",      "x1": 833, "y1": 482, "x2": 893, "y2": 545},
    {"label": "block_green",    "x1": 872, "y1": 549, "x2": 969, "y2": 658},
    {"label": "chips_bag",      "x1": 873, "y1": 389, "x2": 923, "y2": 465},
]


class ObjectPositionEstimator:
    """
    Converts 2D bounding boxes + depth map → 3D world positions.

    Top-down camera pipeline:
      1. Bounding box center pixel (u, v)
      2. Robust depth = median of valid pixels in bounding box interior
         (more reliable than single-pixel for top-down view)
      3. Deproject (u, v, depth) → camera frame using intrinsics
      4. TF transform → world frame
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.lock   = Lock()

        self.rgb_image   = None
        self.depth_image = None
        self.depth_encoding = "32FC1"

        # Camera intrinsics (filled from /camera_info)
        self.fx = self.fy = self.cx = self.cy = None

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(RGB_TOPIC,         Image,      self._rgb_cb,   queue_size=1)
        rospy.Subscriber(DEPTH_TOPIC,       Image,      self._depth_cb, queue_size=1)
        rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, self._info_cb,  queue_size=1)

        rospy.loginfo("Waiting for first RGB + depth frames...")
        self._wait_for_data()
        rospy.loginfo("Ready.")

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _rgb_cb(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        with self.lock:
            enc = msg.encoding
            self.depth_encoding = enc
            if enc == "32FC1":
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            elif enc in ("16UC1", "mono16"):
                # Gazebo often publishes in mm as 16-bit
                raw = self.bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32)
                self.depth_image = raw * 0.001   # → metres
            else:
                rospy.logwarn_once(f"Unrecognised depth encoding '{enc}', "
                                   "treating as 32FC1")
                self.depth_image = self.bridge.imgmsg_to_cv2(
                    msg, "passthrough").astype(np.float32)

    def _info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            rospy.loginfo(f"Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
                          f"cx={self.cx:.1f} cy={self.cy:.1f}")

    def _wait_for_data(self, timeout=10.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.rgb_image is not None and self.depth_image is not None:
                    return
            rate.sleep()
        rospy.logwarn("Timed out waiting for images. Check topic names.")

    # ── Intrinsics ────────────────────────────────────────────────────────────

    def _intrinsics(self):
        """
        Returns (fx, fy, cx, cy).
        Default estimated from your top-down image (1220×868, ~60° HFOV):
          fx = fy = 1220 / (2 * tan(30°)) ≈ 1056
        Adjust HFOV to match your Gazebo camera plugin <horizontal_fov> value.
        """
        if self.fx is not None:
            return self.fx, self.fy, self.cx, self.cy
        rospy.logwarn_once(
            f"No camera_info received from {CAMERA_INFO_TOPIC}. "
            "Using defaults — subscribe to camera_info for accuracy."
        )
        return 1056.0, 1056.0, 610.0, 434.0

    # ── Depth sampling ────────────────────────────────────────────────────────

    def _sample_depth_bbox(self, depth_img, x1, y1, x2, y2):
        """
        Samples depth inside the bounding box.
        Uses the inner 50% of the box to avoid sampling edges/background.
        Returns depth in metres, or None if no valid pixels.

        WHY: In a top-down view the bounding box is tightly aligned with
        the object top surface, so the median inside the box gives the
        true object height rather than the table surface.
        """
        h, w = depth_img.shape

        # Shrink to inner 50% to avoid object edges
        pad_x = max(1, int((x2 - x1) * 0.25))
        pad_y = max(1, int((y2 - y1) * 0.25))
        ix1 = max(0, x1 + pad_x)
        ix2 = min(w, x2 - pad_x)
        iy1 = max(0, y1 + pad_y)
        iy2 = min(h, y2 - pad_y)

        if ix2 <= ix1 or iy2 <= iy1:
            # Box too small to shrink — use full box
            ix1, ix2, iy1, iy2 = x1, x2, y1, y2

        patch = depth_img[iy1:iy2, ix1:ix2].flatten()
        valid = patch[np.isfinite(patch) & (patch > 0.01)]   # ignore zeros/nan

        if len(valid) < 3:
            return None

        return float(np.median(valid))

    # ── Deprojection ──────────────────────────────────────────────────────────

    def _deproject(self, u, v, depth_m):
        """Pixel (u,v) + depth → 3D point in camera frame (metres)."""
        fx, fy, cx, cy = self._intrinsics()
        X = (u - cx) * depth_m / fx
        Y = (v - cy) * depth_m / fy
        Z = depth_m
        return X, Y, Z

    # ── TF transform ──────────────────────────────────────────────────────────

    def _to_world(self, x_c, y_c, z_c):
        """
        Camera frame → world frame via TF.
        Falls back to camera-frame coords if TF is unavailable.
        """
        pt = PointStamped()
        pt.header.frame_id = CAMERA_FRAME
        pt.header.stamp    = rospy.Time(0)
        pt.point.x = x_c
        pt.point.y = y_c
        pt.point.z = z_c
        try:
            world = self.tf_buffer.transform(
                pt, WORLD_FRAME, timeout=rospy.Duration(1.0))
            return (world.point.x, world.point.y, world.point.z)
        except Exception as e:
            rospy.logwarn_once(
                f"TF {CAMERA_FRAME}→{WORLD_FRAME} failed: {e}. "
                "Returning camera-frame coordinates. "
                f"Verify CAMERA_FRAME='{CAMERA_FRAME}' with: "
                f"rosrun tf tf_echo {WORLD_FRAME} {CAMERA_FRAME}"
            )
            return (x_c, y_c, z_c)

    # ── Public API ────────────────────────────────────────────────────────────

    def get_object_positions(self, bounding_boxes=None):
        """
        Main function — receives AI bounding boxes, returns 3D positions.

        Parameters
        ----------
        bounding_boxes : list[dict] | None
            Format: [{"label": str, "x1": int, "y1": int,
                                    "x2": int, "y2": int}, ...]
            Pixel coords in the RGB image (same resolution as depth).
            Pass None to use DEFAULT_BOUNDING_BOXES from the provided photos.

        Returns
        -------
        dict: {label: {"world":  (x, y, z) metres in world frame,
                        "camera": (x, y, z) metres in camera frame,
                        "depth_m": float,
                        "pixel_center": (u, v)}}
        """
        if bounding_boxes is None:
            rospy.loginfo("Using DEFAULT bounding boxes from provided photos.")
            bounding_boxes = DEFAULT_BOUNDING_BOXES

        with self.lock:
            depth_img = (self.depth_image.copy()
                         if self.depth_image is not None else None)

        if depth_img is None:
            rospy.logerr("No depth image available.")
            return {}

        results = {}

        for bbox in bounding_boxes:
            label = bbox["label"]
            x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2

            depth_m = self._sample_depth_bbox(depth_img, x1, y1, x2, y2)

            if depth_m is None:
                rospy.logwarn(f"[{label}] No valid depth in bbox — skipping.")
                continue

            x_c, y_c, z_c = self._deproject(u, v, depth_m)
            world = self._to_world(x_c, y_c, z_c)

            results[label] = {
                "world":        world,
                "camera":       (x_c, y_c, z_c),
                "depth_m":      depth_m,
                "pixel_center": (u, v),
            }

            rospy.loginfo(
                f"[{label:<18}]  px=({u:4d},{v:4d})  "
                f"depth={depth_m:.3f}m  "
                f"world=({world[0]:+.3f}, {world[1]:+.3f}, {world[2]:+.3f})"
            )

        return results

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def debug_depth_range(self):
        """
        Prints depth statistics — useful because the depth image looks
        nearly black in rqt_image_view at 10m scale, but values are valid.
        Call this to confirm depth data is being received correctly.
        """
        with self.lock:
            d = self.depth_image.copy() if self.depth_image is not None else None
        if d is None:
            rospy.logwarn("No depth image yet.")
            return
        valid = d[np.isfinite(d) & (d > 0)]
        if len(valid) == 0:
            rospy.logwarn("Depth image has no valid pixels.")
            return
        rospy.loginfo(
            f"Depth stats (metres): "
            f"min={valid.min():.3f}  max={valid.max():.3f}  "
            f"median={np.median(valid):.3f}  "
            f"encoding={self.depth_encoding}\n"
            f"  NOTE: image looks black in rqt at 10m scale — "
            f"this is normal for a ~1m overhead camera."
        )

    def visualize(self, bounding_boxes=None, results=None, save_path=None):
        """
        Draws bounding boxes + 3D positions on the RGB image.
        save_path: if given, saves to file instead of showing window.
        """
        if bounding_boxes is None:
            bounding_boxes = DEFAULT_BOUNDING_BOXES
        if results is None:
            results = self.get_object_positions(bounding_boxes)

        with self.lock:
            vis = self.rgb_image.copy() if self.rgb_image is not None else None
        if vis is None:
            rospy.logwarn("No RGB image to visualize.")
            return

        for bbox in bounding_boxes:
            label = bbox["label"]
            color = (0, 220, 0)

            cv2.rectangle(vis,
                          (bbox["x1"], bbox["y1"]),
                          (bbox["x2"], bbox["y2"]),
                          color, 2)

            if label in results:
                wx, wy, wz = results[label]["world"]
                u,  v      = results[label]["pixel_center"]
                d          = results[label]["depth_m"]
                line1 = f"{label}"
                line2 = f"({wx:.2f},{wy:.2f},{wz:.2f})m"
                line3 = f"depth={d:.3f}m"
                cv2.putText(vis, line1, (bbox["x1"], bbox["y1"] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                cv2.putText(vis, line2, (bbox["x1"], bbox["y1"] - 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)
                cv2.putText(vis, line3, (bbox["x1"], bbox["y1"] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 180, 0), 1)
                cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)
            else:
                cv2.putText(vis, f"{label} [no depth]",
                            (bbox["x1"], bbox["y1"] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 80, 255), 1)

        if save_path:
            cv2.imwrite(save_path, vis)
            rospy.loginfo(f"Visualization saved to {save_path}")
        else:
            cv2.imshow("Object Positions", vis)
            cv2.waitKey(1)


# ──────────────────────────────────────────────────────────────────────────────
# STANDALONE TEST NODE
# ──────────────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node("object_position_estimator", anonymous=True)
    est = ObjectPositionEstimator()

    # Confirm depth is working (image looks black in rqt but values are valid)
    est.debug_depth_range()

    # Run with default boxes from your provided photos
    results = est.get_object_positions()

    print("\n" + "=" * 65)
    print(f"{'OBJECT':<22} {'WORLD X':>8} {'WORLD Y':>8} {'WORLD Z':>8} {'DEPTH':>8}")
    print("-" * 65)
    for label, data in results.items():
        wx, wy, wz = data["world"]
        print(f"{label:<22} {wx:+8.3f} {wy:+8.3f} {wz:+8.3f} "
              f"{data['depth_m']:8.3f}m")
    print("=" * 65)

    est.visualize(save_path="/tmp/object_positions.png")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass