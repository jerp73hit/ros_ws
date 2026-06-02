#!/usr/bin/env python3
"""
dome_photos_capture.py
======================
ROS1 Noetic | Gazebo | Sawyer + cafe_table scene

FEATURES
--------
  • 500+ photos along a hemispherical dome trajectory
  • Images split randomly into  train / test / valid  folders (70/20/10)
  • Reads /gazebo/model_states to get live object positions
  • Reads /io/internal_camera/right_hand_camera/camera_info for camera intrinsics
  • Projects every object's 3-D bounding box into the image plane
  • Saves one YOLO-format .txt label file per image (same stem, same folder)
  • Also writes  classes.txt  and  dataset.yaml  at the root output folder

OBJECT CLASSES (indices match YOLO class id)
--------------------------------------------
  0  mustard
  1  coke_can
  2  bowl
  3  banana
  4  strawberry
  5  planta_maceta
  6  esponja_lavaplatos
  7  papas_fritas
  8  block

DOME TRAJECTORY
---------------
  N_RINGS × N_PER_RING + 1 zenith  →  9×56+1 = 505 poses (all > 500)

REQUIREMENTS
------------
  pip install opencv-python
  ROS packages: rospy, sensor_msgs, geometry_msgs, gazebo_msgs, cv_bridge,
                tf, tf.transformations
"""

import os
import math
import random
import threading

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import quaternion_from_matrix, quaternion_matrix

# ═══════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════

# ── Focus point: centre of cafe_table top surface ──────────────────────────
# cafe_table origin is at (0.75, 0.0, 0.0) in world frame.
# The table top surface sits at z ≈ 0.755 m (table height ~0.755 m).
FOCUS_X = 0.75      # table centre x  (from /gazebo/model_states)
FOCUS_Y = 0.0       # table centre y
FOCUS_Z = 0.755     # table top surface z

# ── Dome parameters  →  9×56 + 1 = 505 photos ─────────────────────────────
DOME_RADIUS   = 1.0     # metres from focus point to camera
N_RINGS       = 9       # elevation bands (excluding zenith)
N_PER_RING    = 56      # azimuth steps per ring
MIN_ELEVATION = 10      # degrees — low enough for side views
MAX_ELEVATION = 80      # degrees — nearly overhead

# ── Dataset split ratios ───────────────────────────────────────────────────
SPLIT_TRAIN = 0.70
SPLIT_TEST  = 0.20
SPLIT_VALID = 0.10      # remainder goes here

# ── ROS topics ────────────────────────────────────────────────────────────
CAMERA_IMAGE_TOPIC  = "/dome_camera/image_raw"
CAMERA_INFO_TOPIC   = "/dome_camera/camera_info"
MODEL_STATES_TOPIC  = "/gazebo/model_states"

# ── Image resolution (must match SDF below) ───────────────────────────────
IMG_W = 1280
IMG_H = 720

# ── Output root ───────────────────────────────────────────────────────────
OUTPUT_DIR = os.path.expanduser("~/dome_dataset")

# ── Timing ────────────────────────────────────────────────────────────────
SETTLE_TIME = 0.6       # seconds to wait after moving camera

# ── Gazebo model name ─────────────────────────────────────────────────────
CAMERA_MODEL_NAME = "dome_camera"

# ── Object classes and their approximate half-extents (metres) ────────────
# Used to project a 3-D bounding box into the image.
# Tweak per-object sizes to match your actual models.
OBJECT_CLASSES = {
    #  name               class_id  (half_x, half_y, half_z)
    # Sizes tuned from visual inspection of debug images.
    # pose.position is the BASE of each model in Gazebo, so half_z is added
    # as a centre offset in _generate_labels before projection.
    "mustard":           (0,  (0.038, 0.030, 0.075)),   # ~11 cm tall bottle
    "coke_can":          (1,  (0.033, 0.033, 0.058)),   # standard 330 ml can
    "bowl":              (2,  (0.095, 0.095, 0.028)),   # wide shallow bowl
    "banana":            (3,  (0.110, 0.030, 0.022)),   # long flat fruit
    "strawberry":        (4,  (0.025, 0.025, 0.022)),   # small berry
    "planta_maceta":     (5,  (0.065, 0.065, 0.090)),   # pot + plant
    "esponja_lavaplatos":(6,  (0.050, 0.035, 0.022)),   # flat sponge
    "papas_fritas":      (7,  (0.038, 0.038, 0.045)),   # fries box
    "block":             (8,  (0.028, 0.028, 0.028)),   # cube
}

# ── Per-class BGR colours for debug visualisation ─────────────────────────
# Order matches class_id 0-8
CLASS_COLOURS = [
    (  0, 255,   0),   # 0 mustard           — green
    (  0,   0, 255),   # 1 coke_can          — red
    (255,   0,   0),   # 2 bowl              — blue
    (  0, 255, 255),   # 3 banana            — yellow
    (255,   0, 255),   # 4 strawberry        — magenta
    (255, 255,   0),   # 5 planta_maceta     — cyan
    (128,   0, 255),   # 6 esponja_lavaplatos— purple
    (  0, 128, 255),   # 7 papas_fritas      — orange
    (255, 128,   0),   # 8 block             — light blue
]

# ═══════════════════════════════════════════════════════════════════════════
#  CAMERA SDF
# ═══════════════════════════════════════════════════════════════════════════

CAMERA_SDF = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{CAMERA_MODEL_NAME}">
    <static>true</static>
    <link name="camera_link">
      <visual name="visual">
        <geometry><sphere><radius>0.02</radius></sphere></geometry>
      </visual>
      <sensor name="dome_cam_sensor" type="camera">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>{IMG_W}</width>
            <height>{IMG_H}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.02</near><far>300</far></clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>30</updateRate>
          <cameraName>dome_camera</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>dome_camera_optical_frame</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>"""

# ═══════════════════════════════════════════════════════════════════════════
#  GEOMETRY HELPERS
# ═══════════════════════════════════════════════════════════════════════════

def look_at_quaternion(cam_xyz, target_xyz):
    """
    Build a quaternion so the Gazebo camera at cam_xyz looks at target_xyz.
    Gazebo cameras point along their +X axis.
    """
    forward = np.array(target_xyz, dtype=float) - np.array(cam_xyz, dtype=float)
    norm = np.linalg.norm(forward)
    if norm < 1e-6:
        return Quaternion(x=0, y=0, z=0, w=1)
    forward /= norm

    world_up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(forward, world_up)) > 0.99:
        world_up = np.array([0.0, 1.0, 0.0])

    right  = np.cross(forward, world_up);  right  /= np.linalg.norm(right)
    up     = np.cross(right, forward)

    rot = np.eye(4)
    rot[0:3, 0] =  forward
    rot[0:3, 1] = -right
    rot[0:3, 2] =  up

    q = quaternion_from_matrix(rot)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def dome_poses(focus, radius, n_rings, n_per_ring, min_el_deg, max_el_deg):
    """Return list of ((x,y,z), Quaternion) for all dome positions."""
    fx, fy, fz = focus
    poses = []
    for el_deg in np.linspace(min_el_deg, max_el_deg, n_rings):
        el = math.radians(el_deg)
        for i in range(n_per_ring):
            az = 2 * math.pi * i / n_per_ring
            x = fx + radius * math.cos(el) * math.cos(az)
            y = fy + radius * math.cos(el) * math.sin(az)
            z = fz + radius * math.sin(el)
            poses.append(((x, y, z), look_at_quaternion((x,y,z), (fx,fy,fz))))
    # zenith
    x, y, z = fx, fy, fz + radius
    poses.append(((x, y, z), look_at_quaternion((x,y,z), (fx,fy,fz))))
    return poses


def world_to_camera(world_pt, cam_pos, cam_quat_ros):
    """
    Transform a world-frame 3-D point into the camera optical frame
    (OpenCV convention: +X right, +Y down, +Z forward into scene).

    The dome camera SDF pose (set via /gazebo/set_model_state) uses the
    Gazebo body convention:
        +X  forward  (look-at direction, built by look_at_quaternion)
        +Y  left
        +Z  up

    libgazebo_ros_camera internally rotates to the ROS optical convention
    before publishing images, applying:
        optical_X =  gazebo_Y  (right  = left-axis negated... wait)

    Confirmed mapping (Gazebo body → ROS optical):
        optical +X (right)   =  -Gazebo +Y  (right = -left)
        optical +Y (down)    =  -Gazebo +Z  (down  = -up)
        optical +Z (forward) =  +Gazebo +X  (forward = forward)

    So:  R_fix = [[ 0, -1,  0],
                  [ 0,  0, -1],
                  [ 1,  0,  0]]
    """
    q = [cam_quat_ros.x, cam_quat_ros.y,
         cam_quat_ros.z, cam_quat_ros.w]
    R = quaternion_matrix(q)[:3, :3]   # rotation: world axes → Gazebo body axes

    # Gazebo body → ROS optical frame
    R_fix = np.array([[ 0, -1,  0],
                      [ 0,  0, -1],
                      [ 1,  0,  0]], dtype=float)

    p_cam_pos = np.array([cam_pos.x, cam_pos.y, cam_pos.z], dtype=float)
    p_world   = np.array(world_pt, dtype=float)

    p_rel = p_world - p_cam_pos   # vector in world frame
    p_gz  = R.T @ p_rel           # rotate into Gazebo body frame
    p_opt = R_fix @ p_gz          # rotate into optical frame

    return p_opt   # [x_right, y_down, z_forward]


def project_point(pt_optical, K):
    """
    Project an optical-frame 3-D point with camera matrix K.
    Returns (u, v) pixel coordinates, or None if behind camera.
    """
    x, y, z = pt_optical
    if z <= 0.01:
        return None
    u = K[0, 0] * x / z + K[0, 2]
    v = K[1, 1] * y / z + K[1, 2]
    return (u, v)


def object_bbox_yolo(obj_pos, half_extents, cam_pos, cam_quat, K, img_w, img_h):
    """
    Project the 8 corners of an axis-aligned bounding box into the image,
    compute the 2-D enclosing rectangle, and return YOLO format:
      (class_cx_norm, class_cy_norm, class_w_norm, class_h_norm)
    Also returns pixel-space rect (x1, y1, x2, y2) as second value.
    Returns (None, None) if the object is entirely behind the camera or off-screen.
    """
    ox, oy, oz = obj_pos
    hx, hy, hz = half_extents

    corners_world = [
        (ox + sx*hx, oy + sy*hy, oz + sz*hz)
        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
    ]

    us, vs = [], []
    for cw in corners_world:
        p_opt = world_to_camera(cw, cam_pos, cam_quat)
        uv = project_point(p_opt, K)
        if uv is not None:
            us.append(uv[0])
            vs.append(uv[1])

    if not us:
        return None, None

    u_min, u_max = min(us), max(us)
    v_min, v_max = min(vs), max(vs)

    # Clip to image
    u_min = max(0.0, u_min);  u_max = min(float(img_w), u_max)
    v_min = max(0.0, v_min);  v_max = min(float(img_h), v_max)

    if u_max <= u_min or v_max <= v_min:
        return None, None

    cx = (u_min + u_max) / 2.0 / img_w
    cy = (v_min + v_max) / 2.0 / img_h
    bw = (u_max - u_min) / img_w
    bh = (v_max - v_min) / img_h

    px_rect = (int(u_min), int(v_min), int(u_max), int(v_max))
    return (cx, cy, bw, bh), px_rect


def draw_debug_image(frame, label_lines, px_rects):
    """
    Draw bounding boxes with class labels onto a copy of *frame*.

    label_lines : list of YOLO strings  "class_id cx cy w h"
    px_rects    : list of (x1, y1, x2, y2) in pixel space,
                  same order and length as label_lines.
    Returns the annotated image (frame is not modified).
    """
    vis = frame.copy()
    for line, rect in zip(label_lines, px_rects):
        if rect is None:
            continue
        cid = int(line.split()[0])
        colour = CLASS_COLOURS[cid % len(CLASS_COLOURS)]
        x1, y1, x2, y2 = rect

        # Box
        cv2.rectangle(vis, (x1, y1), (x2, y2), colour, 2)

        # Label background + text
        class_name = next(
            (n for n, (i, _) in OBJECT_CLASSES.items() if i == cid), str(cid)
        )
        label_str = f"{cid}:{class_name}"
        font       = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        thickness  = 1
        (tw, th), baseline = cv2.getTextSize(
            label_str, font, font_scale, thickness)
        tx = x1
        ty = max(y1 - 4, th + baseline + 2)
        cv2.rectangle(vis,
                      (tx, ty - th - baseline - 2),
                      (tx + tw + 4, ty + 2),
                      colour, -1)
        cv2.putText(vis, label_str,
                    (tx + 2, ty - baseline),
                    font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
    return vis

def overlap_metric(rectA, rectB):
    """
    Returns the MAXIMUM of three overlap measures:
      1. IoU(A,B)              — standard symmetric overlap
      2. containment(A in B)   — fraction of A that is inside B
      3. containment(B in A)   — fraction of B that is inside A

    Why: pure IoU misses the case where a small object (strawberry) sits
    entirely inside the projected box of a larger object (planta_maceta).
    In that case IoU = small_area/large_area which can be 0.04 — far below
    any sensible threshold — yet the small object is completely occluded.
    Taking the max of IoU and both containment ratios catches all three
    geometric cases with a single threshold.
    """
    ax1, ay1, ax2, ay2 = rectA
    bx1, by1, bx2, by2 = rectB

    ix1 = max(ax1, bx1);  ix2 = min(ax2, bx2)
    iy1 = max(ay1, by1);  iy2 = min(ay2, by2)

    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0

    area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1, (bx2 - bx1) * (by2 - by1))
    union  = area_a + area_b - inter

    iou_val        = inter / union          # symmetric overlap
    contain_a_in_b = inter / area_a        # fraction of A that is inside B
    contain_b_in_a = inter / area_b        # fraction of B that is inside A

    return max(iou_val, contain_a_in_b, contain_b_in_a)


def filter_occluded(detections, overlap_threshold=0.30):
    """
    Remove occluded objects using overlap_metric() instead of plain IoU.

    For every pair where overlap_metric > overlap_threshold:
      → keep the CLOSER object (smaller depth value)
      → discard the FARTHER object

    Threshold 0.30 means:
      - Objects whose boxes overlap by >30% (IoU) are filtered, OR
      - Objects where >30% of one box is inside the other are filtered.
    This is robust to both side-view partial overlaps and the
    containment case (small object fully inside larger object's box).
    """
    n        = len(detections)
    occluded = [False] * n

    for i in range(n):
        if occluded[i]:
            continue
        for j in range(i + 1, n):
            if occluded[j]:
                continue

            metric = overlap_metric(detections[i]['rect'], detections[j]['rect'])
            if metric > overlap_threshold:
                if detections[i]['depth'] <= detections[j]['depth']:
                    occluded[j] = True
                    rospy.loginfo(
                        f"[DomeCam] Occluded: {detections[j]['name']} "
                        f"behind {detections[i]['name']} "
                        f"(metric={metric:.2f}, "
                        f"d={detections[i]['depth']:.3f} vs "
                        f"{detections[j]['depth']:.3f} m)"
                    )
                else:
                    occluded[i] = True
                    rospy.loginfo(
                        f"[DomeCam] Occluded: {detections[i]['name']} "
                        f"behind {detections[j]['name']} "
                        f"(metric={metric:.2f}, "
                        f"d={detections[j]['depth']:.3f} vs "
                        f"{detections[i]['depth']:.3f} m)"
                    )
                    break

    return [d for d, occ in zip(detections, occluded) if not occ]

# ═══════════════════════════════════════════════════════════════════════════
#  MAIN CLASS
# ═══════════════════════════════════════════════════════════════════════════

class DomeCameraCapture:

    def __init__(self):
        rospy.init_node("dome_camera_capture", anonymous=True)
        self.bridge = CvBridge()

        # ── State ──────────────────────────────────────────────────────────
        self._latest_frame  = None
        self._frame_lock    = threading.Lock()

        self._camera_info   = None          # sensor_msgs/CameraInfo
        self._info_lock     = threading.Lock()

        self._model_states  = None          # gazebo_msgs/ModelStates
        self._states_lock   = threading.Lock()

        # ── Output folders ─────────────────────────────────────────────────
        self._dirs = {}
        for split in ("train", "test", "valid"):
            for sub in ("images", "labels", "debug"):
                p = os.path.join(OUTPUT_DIR, split, sub)
                os.makedirs(p, exist_ok=True)
                self._dirs[f"{split}_{sub}"] = p

        # ── Camera matrix (fallback = pinhole from FOV if info not received)─
        fov = 1.3962634   # radians  (80°)
        fx  = IMG_W / (2 * math.tan(fov / 2))
        self._K_default = np.array([
            [fx,  0, IMG_W / 2],
            [ 0, fx, IMG_H / 2],
            [ 0,  0,         1]
        ], dtype=float)

        # ── Subscriptions ──────────────────────────────────────────────────
        rospy.Subscriber(CAMERA_IMAGE_TOPIC, Image,
                         self._image_cb, queue_size=1)
        rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo,
                         self._info_cb,  queue_size=1)
        rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates,
                         self._states_cb, queue_size=1)

        # ── Gazebo services ────────────────────────────────────────────────
        rospy.loginfo("[DomeCam] Waiting for Gazebo services …")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/set_model_state")

        from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
        self._spawn     = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._delete    = rospy.ServiceProxy("/gazebo/delete_model",    DeleteModel)
        self._set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self._write_meta()

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._latest_frame = frame.copy()
        except CvBridgeError as e:
            rospy.logwarn(f"[DomeCam] CvBridge: {e}")

    def _info_cb(self, msg):
        with self._info_lock:
            if self._camera_info is None:
                self._camera_info = msg
                rospy.loginfo("[DomeCam] Camera info received.")

    def _states_cb(self, msg):
        with self._states_lock:
            self._model_states = msg

    # ── Accessors ───────────────────────────────────────────────────────────

    def _get_frame(self):
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _get_K(self):
        with self._info_lock:
            if self._camera_info is not None:
                K = np.array(self._camera_info.K, dtype=float).reshape(3, 3)
                return K
        return self._K_default

    def _get_object_positions(self):
        """Return dict: name → geometry_msgs/Pose for tracked objects only."""
        with self._states_lock:
            if self._model_states is None:
                return {}
            result = {}
            for name, pose in zip(self._model_states.name,
                                  self._model_states.pose):
                if name in OBJECT_CLASSES:
                    result[name] = pose
        return result

    # ── Dataset metadata ────────────────────────────────────────────────────

    def _write_meta(self):
        # classes.txt
        classes = sorted(OBJECT_CLASSES.items(), key=lambda kv: kv[1][0])
        with open(os.path.join(OUTPUT_DIR, "classes.txt"), "w") as f:
            for name, (cid, _) in classes:
                f.write(f"{name}\n")

        # dataset.yaml  (YOLOv5/v8 compatible)
        yaml_path = os.path.join(OUTPUT_DIR, "dataset.yaml")
        with open(yaml_path, "w") as f:
            f.write(f"path: {OUTPUT_DIR}\n")
            f.write(f"train: train/images\n")
            f.write(f"val:   valid/images\n")
            f.write(f"test:  test/images\n\n")
            f.write(f"nc: {len(OBJECT_CLASSES)}\n")
            f.write(f"names: [{', '.join(n for n,_ in classes)}]\n")

        rospy.loginfo(f"[DomeCam] Metadata written to {OUTPUT_DIR}")

    # ── Gazebo helpers ──────────────────────────────────────────────────────

    def _spawn_camera(self):
        pose = Pose(
            position=Point(x=FOCUS_X, y=FOCUS_Y, z=FOCUS_Z + DOME_RADIUS),
            orientation=Quaternion(x=0, y=0, z=0, w=1)
        )
        self._spawn(model_name=CAMERA_MODEL_NAME,
                    model_xml=CAMERA_SDF,
                    robot_namespace="/",
                    initial_pose=pose,
                    reference_frame="world")
        rospy.loginfo("[DomeCam] Camera spawned.")

    def _delete_camera(self):
        try:
            self._delete(CAMERA_MODEL_NAME)
        except Exception:
            pass

    def _move_camera(self, xyz, quat):
        state = ModelState()
        state.model_name      = CAMERA_MODEL_NAME
        state.reference_frame = "world"
        state.pose = Pose(
            position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=quat
        )
        self._set_state(state)

    # ── Split assignment ────────────────────────────────────────────────────

    @staticmethod
    def _choose_split():
        r = random.random()
        if r < SPLIT_TRAIN:
            return "train"
        elif r < SPLIT_TRAIN + SPLIT_TEST:
            return "test"
        else:
            return "valid"

    # ── Label generation ────────────────────────────────────────────────────

    def _generate_labels(self, cam_pos_msg, cam_quat_msg, K):
        """
            Return (label_lines, px_rects) for all visible, non-occluded objects.

            Pipeline:
            1. Project every tracked object into the image.
            2. Compute depth (distance from camera to object centre).
            3. Run occlusion filter: for overlapping boxes, discard the farther one.
            4. Return surviving YOLO lines and pixel rects.
        """
        
        obj_poses   = self._get_object_positions()
        cam_pos_arr = np.array(
            [cam_pos_msg.x, cam_pos_msg.y, cam_pos_msg.z], dtype=float)

        # ── Step 1 & 2: project + compute depth ──────────────────────────────
        raw = []
        for name, pose in obj_poses.items():
            class_id, half_ext = OBJECT_CLASSES[name]

            obj_xyz = (pose.position.x,
                    pose.position.y,
                    pose.position.z + half_ext[2])

            result, rect = object_bbox_yolo(
                obj_pos      = obj_xyz,
                half_extents = half_ext,
                cam_pos      = cam_pos_msg,
                cam_quat     = cam_quat_msg,
                K            = K,
                img_w        = IMG_W,
                img_h        = IMG_H,
            )
            if result is None:
                continue

            cx, cy, bw, bh = result
            depth = float(np.linalg.norm(
                np.array(obj_xyz, dtype=float) - cam_pos_arr))

            raw.append({
                'name':     name,
                'class_id': class_id,
                'line':     f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}",
                'rect':     rect,
                'depth':    depth,
            })

        # ── Step 3: occlusion filter ──────────────────────────────────────────
        raw.sort(key=lambda d: d['depth'])
        survivors = filter_occluded(raw, overlap_threshold=0.30)  # ← was iou_threshold=0.15

        # ── Step 4: unpack ────────────────────────────────────────────────────
        lines    = [d['line'] for d in survivors]
        px_rects = [d['rect'] for d in survivors]
        return lines, px_rects

    # ── Main loop ───────────────────────────────────────────────────────────

    def run(self):
        focus = (FOCUS_X, FOCUS_Y, FOCUS_Z)
        poses = dome_poses(
            focus      = focus,
            radius     = DOME_RADIUS,
            n_rings    = N_RINGS,
            n_per_ring = N_PER_RING,
            min_el_deg = MIN_ELEVATION,
            max_el_deg = MAX_ELEVATION,
        )
        total = len(poses)
        rospy.loginfo(f"[DomeCam] {total} dome poses planned "
                      f"({N_RINGS}×{N_PER_RING}+1).")

        self._spawn_camera()
        rospy.sleep(2.5)    # let plugin initialise + first frame arrive

        split_counts = {"train": 0, "test": 0, "valid": 0}
        saved = 0

        for idx, (xyz, quat) in enumerate(poses):
            if rospy.is_shutdown():
                break

            self._move_camera(xyz, quat)
            rospy.sleep(SETTLE_TIME)

            frame = self._get_frame()
            if frame is None:
                rospy.logwarn(f"[DomeCam] No frame at pose {idx+1} — skipping.")
                continue

            K     = self._get_K()
            split = self._choose_split()

            # ── Filename stem ──────────────────────────────────────────────
            az_deg = math.degrees(
                math.atan2(xyz[1] - FOCUS_Y, xyz[0] - FOCUS_X))
            el_deg = math.degrees(
                math.asin(np.clip((xyz[2] - FOCUS_Z) / DOME_RADIUS, -1, 1)))
            stem = f"dome_{idx+1:04d}_az{az_deg:+07.1f}_el{el_deg:05.1f}"

            # ── Save image ─────────────────────────────────────────────────
            img_path = os.path.join(self._dirs[f"{split}_images"],
                                    stem + ".png")
            cv2.imwrite(img_path, frame)

            # ── Generate & save labels ─────────────────────────────────────
            cam_pos   = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            lbl_lines, px_rects = self._generate_labels(cam_pos, quat, K)

            lbl_path = os.path.join(self._dirs[f"{split}_labels"],
                                    stem + ".txt")
            with open(lbl_path, "w") as f:
                f.write("\n".join(lbl_lines))

            # ── Save debug image (boxes drawn, separate file) ──────────────
            debug_frame = draw_debug_image(frame, lbl_lines, px_rects)
            debug_path  = os.path.join(self._dirs[f"{split}_debug"],
                                       stem + "_debug.png")
            cv2.imwrite(debug_path, debug_frame)

            saved += 1
            split_counts[split] += 1
            n_objs = len(lbl_lines)
            rospy.loginfo(
                f"[DomeCam] [{saved}/{total}] {split:5s} | "
                f"{n_objs} obj(s) | {stem}.png  +debug"
            )

        self._delete_camera()
        rospy.loginfo(
            f"[DomeCam] DONE — {saved} images saved.\n"
            f"  train: {split_counts['train']}  "
            f"test: {split_counts['test']}  "
            f"valid: {split_counts['valid']}\n"
            f"  Output: {OUTPUT_DIR}"
        )


# ═══════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    try:
        node = DomeCameraCapture()
        node.run()
    except rospy.ROSInterruptException:
        pass