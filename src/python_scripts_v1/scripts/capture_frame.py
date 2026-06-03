#!/usr/bin/env python3
"""
capture_frame.py
================
Capture one synchronised snapshot from the Sawyer wrist camera and save
everything needed to run offline YOLO detection + 3-D back-projection later.

Saved files (all in --outdir, default current directory)
---------------------------------------------------------
  rgb.png              Raw colour image  (use THIS for YOLO inference)
  depth.npy            float32 (H, W) depth array, metres. NaN = no reading.
  depth_preview.png    False-colour depth image for quick visual check.
  camera_info.npz      Full camera intrinsics + distortion coefficients.
                         Keys: K (3x3), D (1x5), fx, fy, cx, cy,
                               width, height, distortion_model
  camera_pose.npz      Camera position + quaternion in the world frame
                       at capture time (read from /tf, world → camera link).
                         Keys: position (3,), quaternion (4,) [x,y,z,w]
  meta.json            Human-readable summary of everything above.

Nothing involving bounding boxes is written — the rgb.png is the clean
input image, exactly as received from the camera.

Usage
-----
  # Single shot, save to current directory
  python3 capture_frame.py

  # Save to a specific folder
  python3 capture_frame.py --outdir ~/captures/shot_01

  # Loop: keep saving a new snapshot every N seconds (Ctrl-C to stop)
  python3 capture_frame.py --loop --interval 3.0 --outdir ~/captures

  # Adjust sync tolerance (default 0.05 s)
  python3 capture_frame.py --slop 0.1 --outdir ~/captures/shot_01

Topics
------
  /io/internal_camera/right_hand_camera/image_raw
  /io/internal_camera/right_hand_camera/depth/image_raw
  /io/internal_camera/right_hand_camera/camera_info
  /tf  (optional — for camera pose; skipped gracefully if unavailable)
"""

import os
import sys
import json
import time
import argparse
import threading

import numpy as np
import cv2
import rospy
import message_filters

from sensor_msgs.msg  import Image, CameraInfo
from cv_bridge        import CvBridge, CvBridgeError

# tf is optional — the rest of the script works without it
try:
    import tf
    _TF_AVAILABLE = True
except ImportError:
    _TF_AVAILABLE = False

# ── Topic names ───────────────────────────────────────────────────────────────
RGB_TOPIC   = "/io/internal_camera/right_hand_camera/image_raw"
DEPTH_TOPIC = "/io/internal_camera/right_hand_camera/depth/image_raw"
INFO_TOPIC  = "/io/internal_camera/right_hand_camera/camera_info"

# ── TF frame names (adjust if your URDF differs) ──────────────────────────────
WORLD_FRAME  = "world"
CAMERA_FRAME = "right_hand_camera"   # or "right_hand_camera_optical_frame"


# ─────────────────────────────────────────────────────────────────────────────
#  Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _ensure_metres(depth: np.ndarray) -> np.ndarray:
    """
    Convert depth to metres if it looks like it was published in millimetres.
    Heuristic: if the median of valid pixels > 10, assume mm encoding.
    Safe for typical table-top distances (0.4 - 2.5 m).
    """
    valid = depth[np.isfinite(depth) & (depth > 0)]
    if len(valid) == 0:
        return depth
    if float(np.median(valid)) > 10.0:
        rospy.logwarn(
            "[capture] Depth median > 10 — assuming millimetres, converting ÷1000.")
        depth = depth / 1000.0
    return depth


def _depth_to_colour(depth: np.ndarray) -> np.ndarray:
    """
    Produce a BGR false-colour image from a float32 depth array for preview.
    Invalid pixels (NaN, 0, inf) are shown in black.
    """
    valid_mask = np.isfinite(depth) & (depth > 0)
    if not np.any(valid_mask):
        return np.zeros((*depth.shape, 3), dtype=np.uint8)

    d_min = float(np.min(depth[valid_mask]))
    d_max = float(np.max(depth[valid_mask]))
    span  = max(d_max - d_min, 1e-6)

    norm = np.zeros_like(depth, dtype=np.float32)
    norm[valid_mask] = (depth[valid_mask] - d_min) / span   # 0 (near) … 1 (far)

    grey = (norm * 255).astype(np.uint8)
    colour = cv2.applyColorMap(grey, cv2.COLORMAP_TURBO)
    colour[~valid_mask] = 0   # black out invalid pixels
    return colour


def _get_camera_pose(tf_listener, timeout: float = 1.0):
    """
    Look up the transform world → camera_frame via /tf.
    Returns (position np(3,), quaternion np(4,) [x,y,z,w]) or (None, None).
    """
    if not _TF_AVAILABLE or tf_listener is None:
        return None, None
    try:
        tf_listener.waitForTransform(
            WORLD_FRAME, CAMERA_FRAME,
            rospy.Time(0), rospy.Duration(timeout))
        (trans, rot) = tf_listener.lookupTransform(
            WORLD_FRAME, CAMERA_FRAME, rospy.Time(0))
        return np.array(trans, dtype=float), np.array(rot, dtype=float)
    except Exception as e:
        rospy.logwarn(f"[capture] TF lookup failed ({e}) — pose not saved.")
        return None, None


def _depth_stats(depth: np.ndarray) -> dict:
    valid = depth[np.isfinite(depth) & (depth > 0)]
    if len(valid) == 0:
        return {"valid_pixels": 0, "total_pixels": int(depth.size),
                "min_m": None, "max_m": None, "median_m": None}
    return {
        "valid_pixels": int(len(valid)),
        "total_pixels": int(depth.size),
        "min_m":    round(float(np.min(valid)),    4),
        "max_m":    round(float(np.max(valid)),    4),
        "median_m": round(float(np.median(valid)), 4),
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Core: save one snapshot
# ─────────────────────────────────────────────────────────────────────────────

def save_snapshot(
    rgb_msg:   Image,
    depth_msg: Image,
    camera_info: CameraInfo,
    bridge:    CvBridge,
    tf_listener,
    outdir:    str,
    index:     int = 0,
    loop_mode: bool = False,
) -> str:
    """
    Decode, convert, and write all files for one frame pair.
    Returns the output directory path used.
    """

    # ── Decode images ─────────────────────────────────────────────────────
    try:
        rgb_frame = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"[capture] RGB decode failed: {e}")
        return outdir

    try:
        depth_raw = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.logerr(f"[capture] Depth decode failed: {e}")
        return outdir

    depth = _ensure_metres(depth_raw.astype(np.float32))

    # ── Output path ───────────────────────────────────────────────────────
    if loop_mode:
        snap_dir = os.path.join(outdir, f"frame_{index:04d}")
    else:
        snap_dir = outdir
    os.makedirs(snap_dir, exist_ok=True)

    # ── Camera intrinsics ─────────────────────────────────────────────────
    K = np.array(camera_info.K, dtype=float).reshape(3, 3)
    D = np.array(camera_info.D, dtype=float)           # distortion coefficients
    img_w    = camera_info.width
    img_h    = camera_info.height
    dist_model = camera_info.distortion_model

    # ── Camera pose from TF ───────────────────────────────────────────────
    cam_pos, cam_quat = _get_camera_pose(tf_listener)

    # ── Timestamps ────────────────────────────────────────────────────────
    rgb_stamp   = rgb_msg.header.stamp
    depth_stamp = depth_msg.header.stamp
    wall_time   = time.time()

    # ── Write files ───────────────────────────────────────────────────────

    # 1. Clean RGB image — run YOLO on this
    rgb_path = os.path.join(snap_dir, "rgb.png")
    cv2.imwrite(rgb_path, rgb_frame)

    # 2. Depth array (float32, metres)
    depth_path = os.path.join(snap_dir, "depth.npy")
    np.save(depth_path, depth)

    # 3. False-colour depth preview (for visual sanity check — NOT for YOLO)
    preview_path = os.path.join(snap_dir, "depth_preview.png")
    cv2.imwrite(preview_path, _depth_to_colour(depth))

    # 4. Camera intrinsics
    info_path = os.path.join(snap_dir, "camera_info.npz")
    np.savez(
        info_path,
        K                = K,
        D                = D,
        fx               = K[0, 0],
        fy               = K[1, 1],
        cx               = K[0, 2],
        cy               = K[1, 2],
        width            = img_w,
        height           = img_h,
        distortion_model = np.bytes_(dist_model),
    )

    # 5. Camera pose (if TF was available)
    if cam_pos is not None:
        pose_path = os.path.join(snap_dir, "camera_pose.npz")
        np.savez(
            pose_path,
            position   = cam_pos,
            quaternion = cam_quat,   # [x, y, z, w]
            world_frame  = np.bytes_(WORLD_FRAME),
            camera_frame = np.bytes_(CAMERA_FRAME),
        )
    else:
        pose_path = None

    # 6. Human-readable metadata
    stats = _depth_stats(depth)
    meta  = {
        "capture_wall_time":    wall_time,
        "rgb_stamp":            {"secs": rgb_stamp.secs,
                                 "nsecs": rgb_stamp.nsecs},
        "depth_stamp":          {"secs": depth_stamp.secs,
                                 "nsecs": depth_stamp.nsecs},
        "rgb_topic":            RGB_TOPIC,
        "depth_topic":          DEPTH_TOPIC,
        "image_width":          int(rgb_frame.shape[1]),
        "image_height":         int(rgb_frame.shape[0]),
        "depth_width":          int(depth.shape[1]),
        "depth_height":         int(depth.shape[0]),
        "depth_encoding":       depth_msg.encoding,
        "depth_stats":          stats,
        "camera_info": {
            "fx": round(K[0, 0], 4), "fy": round(K[1, 1], 4),
            "cx": round(K[0, 2], 4), "cy": round(K[1, 2], 4),
            "distortion_model": dist_model,
            "D": D.tolist(),
        },
        "camera_pose": {
            "world_frame":  WORLD_FRAME,
            "camera_frame": CAMERA_FRAME,
            "position":    cam_pos.tolist()  if cam_pos  is not None else None,
            "quaternion":  cam_quat.tolist() if cam_quat is not None else None,
        },
        "files": {
            "rgb":           "rgb.png",
            "depth_npy":     "depth.npy",
            "depth_preview": "depth_preview.png",
            "camera_info":   "camera_info.npz",
            "camera_pose":   "camera_pose.npz" if pose_path else None,
            "meta":          "meta.json",
        },
    }
    meta_path = os.path.join(snap_dir, "meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)

    # ── Log summary ───────────────────────────────────────────────────────
    pose_str = (f"({cam_pos[0]:+.3f}, {cam_pos[1]:+.3f}, {cam_pos[2]:+.3f})"
                if cam_pos is not None else "not available")
    rospy.loginfo(
        f"\n[capture] ── Snapshot saved to {snap_dir} ──\n"
        f"  rgb.png          {rgb_frame.shape[1]}×{rgb_frame.shape[0]}  BGR\n"
        f"  depth.npy        {depth.shape[1]}×{depth.shape[0]}  float32 m  "
        f"valid={stats['valid_pixels']}/{stats['total_pixels']}  "
        f"range=[{stats['min_m']}, {stats['max_m']}] m\n"
        f"  depth_preview    false-colour PNG\n"
        f"  camera_info.npz  fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  "
        f"cx={K[0,2]:.1f}  cy={K[1,2]:.1f}\n"
        f"  camera_pose      {pose_str}\n"
        f"  meta.json        full metadata"
    )
    return snap_dir


# ─────────────────────────────────────────────────────────────────────────────
#  Capture node
# ─────────────────────────────────────────────────────────────────────────────

class CaptureNode:

    def __init__(self, outdir: str, loop: bool, interval: float, slop: float):
        self.outdir   = outdir
        self.loop     = loop
        self.interval = interval
        self.bridge   = CvBridge()
        self._lock    = threading.Lock()
        self._count   = 0
        self._done    = False
        self._last_t  = 0.0

        os.makedirs(outdir, exist_ok=True)

        # ── Camera info (blocking, one-shot) ──────────────────────────────
        rospy.loginfo("[capture] Waiting for camera_info …")
        self._camera_info = rospy.wait_for_message(
            INFO_TOPIC, CameraInfo, timeout=15.0)
        rospy.loginfo(
            f"[capture] Camera info received  "
            f"{self._camera_info.width}x{self._camera_info.height}")

        # ── TF listener (optional) ────────────────────────────────────────
        if _TF_AVAILABLE:
            self._tf = tf.TransformListener()
            rospy.sleep(0.5)   # give TF a moment to fill its buffer
        else:
            self._tf = None
            rospy.logwarn("[capture] tf module not found — pose will not be saved.")

        # ── Synchronised RGB + depth ──────────────────────────────────────
        rgb_sub   = message_filters.Subscriber(RGB_TOPIC,   Image)
        depth_sub = message_filters.Subscriber(DEPTH_TOPIC, Image)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=slop)
        self._sync.registerCallback(self._cb)
        rospy.loginfo(
            f"[capture] Subscribed (slop={slop}s).  "
            f"Waiting for first synchronised pair …")

    def _cb(self, rgb_msg: Image, depth_msg: Image):
        # One-shot mode: ignore callbacks after the first save
        if not self.loop and self._done:
            return

        # Loop mode: throttle to the requested interval
        now = time.time()
        if self.loop and (now - self._last_t) < self.interval:
            return
        self._last_t = now

        with self._lock:
            self._count += 1
            idx = self._count

        save_snapshot(
            rgb_msg      = rgb_msg,
            depth_msg    = depth_msg,
            camera_info  = self._camera_info,
            bridge       = self.bridge,
            tf_listener  = self._tf,
            outdir       = self.outdir,
            index        = idx,
            loop_mode    = self.loop,
        )

        if not self.loop:
            self._done = True

    def done(self) -> bool:
        return self._done


# ─────────────────────────────────────────────────────────────────────────────
#  Utility: load a saved snapshot back into Python
# ─────────────────────────────────────────────────────────────────────────────

def load_snapshot(snap_dir: str) -> dict:
    """
    Load all files from a directory saved by this script.

    Returns a dict with keys:
        rgb           np.ndarray (H, W, 3) uint8  BGR
        depth         np.ndarray (H, W)    float32 metres
        K             np.ndarray (3, 3)    camera matrix
        D             np.ndarray (5,)      distortion coefficients
        cam_pos       np.ndarray (3,)      or None
        cam_quat      np.ndarray (4,)      [x,y,z,w] or None
        meta          dict

    Example
    -------
        from capture_frame import load_snapshot
        snap = load_snapshot("~/captures/shot_01")
        detections = results_to_world(
            yolo_result,
            cam_pos  = snap["cam_pos"],
            cam_quat = snap["cam_quat"],
            K        = snap["K"],
            depth_img= snap["depth"],
        )
    """
    snap_dir = os.path.expanduser(snap_dir)

    rgb   = cv2.imread(os.path.join(snap_dir, "rgb.png"))
    depth = np.load(os.path.join(snap_dir, "depth.npy"))

    info  = np.load(os.path.join(snap_dir, "camera_info.npz"))
    K     = info["K"].reshape(3, 3)
    D     = info["D"]

    pose_path = os.path.join(snap_dir, "camera_pose.npz")
    if os.path.exists(pose_path):
        pose     = np.load(pose_path)
        cam_pos  = pose["position"]
        cam_quat = pose["quaternion"]
    else:
        cam_pos  = None
        cam_quat = None

    meta_path = os.path.join(snap_dir, "meta.json")
    with open(meta_path) as f:
        meta = json.load(f)

    return {
        "rgb":      rgb,
        "depth":    depth,
        "K":        K,
        "D":        D,
        "cam_pos":  cam_pos,
        "cam_quat": cam_quat,
        "meta":     meta,
    }


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Save a synchronised RGB + depth snapshot from the Sawyer camera.")
    parser.add_argument(
        "--outdir", default="~/ros_ws/proof_imgs",
        help="Directory to write files into (created if needed).")
    parser.add_argument(
        "--loop", action="store_true",
        help="Keep saving snapshots instead of a single shot.")
    parser.add_argument(
        "--interval", type=float, default=3.0,
        help="Seconds between snapshots in loop mode (default 3.0).")
    parser.add_argument(
        "--slop", type=float, default=0.05,
        help="Max time difference (s) between RGB and depth frames (default 0.05).")
    args = parser.parse_args()

    rospy.init_node("capture_frame", anonymous=True)

    node = CaptureNode(
        outdir   = os.path.expanduser(args.outdir),
        loop     = args.loop,
        interval = args.interval,
        slop     = args.slop,
    )

    if args.loop:
        rospy.loginfo("[capture] Loop mode — press Ctrl-C to stop.")
        rospy.spin()
    else:
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not node.done():
            rate.sleep()
        rospy.loginfo("[capture] Single snapshot complete.")


if __name__ == "__main__":
    main()