import os
import rospy
import message_filters
import intera_interface
import numpy as np
import math
import copy
import threading

from math import radians
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from trac_ik_python.trac_ik import IK
from ultralytics import YOLO
from frame2world import get_positions, OBJECT_CLASSES

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_MODEL_PATH = os.path.realpath(os.path.join(_SCRIPT_DIR, "..", "..", "..", "yolo_model", "weights", "best.pt"))

RGB_TOPIC = "/io/internal_camera/right_hand_camera/image_raw"
DEPTH_TOPIC = "/io/internal_camera/right_hand_camera/depth/image_raw"
INFO_TOPIC = "/io/internal_camera/right_hand_camera/camera_info"
WORLD_FRAME = "world"
CAMERA_FRAME = "right_hand_camera"

VELOCITY_SCALING = 0.1
MOVE_TIMEOUT = 8.0
SAWYER_BASE_Z = 0.93
Z_OFFSET = 0.012
IK_BASE_LINK = "base"
IK_TIP_LINK = "right_gripper_tip"
JOINT_NAMES = [
    "right_j0", "right_j1", "right_j2", "right_j3",
    "right_j4", "right_j5", "right_j6",
]
SAFE_POSE = [
    -0.041663, -1.025829, 0.029368, 2.175181,
    -0.067030, 0.396837, 1.765965,
]
SAFE_Z = 0.93
X_RANGE = [0.30, 0.90]
Y_RANGE = [-0.60, 0.60]
Z_RANGE = [-0.20, 0.80]
TABLE_Z = 0.755
BASE_ROLL = 180.0
BASE_PITCH = 0.0

_node_initialized = False
_rec = None
_limb = None
_gripper = None
_solver = None


class receiver:
    def __init__(self, camera_frame="right_hand_camera"):
        self._lock = threading.Lock()
        self.bridge = CvBridge()
        self._camera_frame = camera_frame
        self.frame = None
        self.depth = None
        self.K = None
        self.cam_pos = None
        self.cam_quat = None

        self.model = YOLO(_MODEL_PATH)

        rospy.loginfo("Waiting for camera_info...")
        camera_info = rospy.wait_for_message(INFO_TOPIC, CameraInfo, timeout=15.0)
        self.K = np.array(camera_info.K, dtype=float).reshape(3, 3)
        rospy.loginfo("Camera info received.")

        import tf2_ros
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.sleep(0.5)

        rgb_sub = message_filters.Subscriber(RGB_TOPIC, Image)
        depth_sub = message_filters.Subscriber(DEPTH_TOPIC, Image)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.05,
        )
        self._sync.registerCallback(self._cb)
        rospy.loginfo("Subscribed to RGB + depth.")

    def _cb(self, rgb_msg, depth_msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            depth = depth_raw.astype(np.float32)
            valid = depth[np.isfinite(depth) & (depth > 0)]
            if len(valid) > 0 and float(np.median(valid)) > 10.0:
                depth = depth / 1000.0
            self._update_pose()
            with self._lock:
                self.frame = frame
                self.depth = depth
        except Exception as e:
            rospy.logerr("receiver callback error: %s", str(e))

    def _update_pose(self):
        try:
            trans = self._tf_buffer.lookup_transform(
                WORLD_FRAME, self._camera_frame, rospy.Time(0), rospy.Duration(0.1),
            )
            pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ])
            quat = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ])
            with self._lock:
                self.cam_pos = pos
                self.cam_quat = quat
        except Exception:
            pass

    def snapshot(self):
        with self._lock:
            if self.frame is None:
                return None, None, None, None, None
            return (
                self.frame.copy(),
                self.depth.copy() if self.depth is not None else None,
                self.K.copy(),
                self.cam_pos.copy() if self.cam_pos is not None else None,
                self.cam_quat.copy() if self.cam_quat is not None else None,
            )


def debug_camera():
    if _rec is None:
        return None
    frame, depth, K, cam_pos, cam_quat = _rec.snapshot()
    from frame2world import get_r_fix
    return {
        "K": K.copy() if K is not None else None,
        "cam_pos": cam_pos.copy() if cam_pos is not None else None,
        "cam_quat": cam_quat.copy() if cam_quat is not None else None,
        "r_fix": get_r_fix(),
    }


def init_api(camera_frame="right_hand_camera"):
    global _node_initialized, _rec, _limb, _gripper, _solver
    if _node_initialized:
        return
    rospy.init_node("llm_api", anonymous=True)

    rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
    rs.enable()
    rospy.sleep(1.0)

    _rec = receiver(camera_frame=camera_frame)
    _gripper = intera_interface.Gripper("right_gripper")
    _limb = intera_interface.Limb("right")
    _solver = IK(
        IK_BASE_LINK, IK_TIP_LINK,
        timeout=0.05, epsilon=0.001, solve_type="Speed",
    )
    _node_initialized = True
    rospy.loginfo("LLM API initialized.")


def make_scene() -> dict:
    for _ in range(50):
        if _rec.frame is not None:
            break
        rospy.sleep(0.1)

    frame, depth, K, cam_pos, cam_quat = _rec.snapshot()
    if frame is None:
        rospy.logerr("make_scene: no frame received.")
        return {}
    if cam_pos is None:
        rospy.logwarn("make_scene: camera pose not available (TF lookup failed).")

    results = _rec.model.predict(
        source=frame, save=False, show=False, verbose=False,
    )
    detections = []
    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
        detections = get_positions(
            results[0],
            depth_img=depth,
            K=K,
            cam_pos=cam_pos,
            cam_quat=cam_quat,
        )

    eef_pose = _limb.endpoint_pose()
    eef_pos = [
        eef_pose["position"].x,
        eef_pose["position"].y,
        eef_pose["position"].z,
    ]
    eef_quat = [
        eef_pose["orientation"].x,
        eef_pose["orientation"].y,
        eef_pose["orientation"].z,
        eef_pose["orientation"].w,
    ]
    eef_rpy = list(euler_from_quaternion(eef_quat))

    objects = {}
    for d in detections:
        name = d["name"]
        cid = d["class_id"]
        half = OBJECT_CLASSES.get(name, (cid, (0.03, 0.03, 0.03)))[1]
        objects[name] = {
            "center": d["world_xyz"].tolist(),
            "base": d["world_base"].tolist(),
            "depth": d.get("depth", None),
            "depth_source": d.get("depth_source", "unknown"),
            "bbox": {
                "cx_norm": float(d["bbox_norm"][0]),
                "cy_norm": float(d["bbox_norm"][1]),
                "w_norm": float(d["bbox_norm"][2]),
                "h_norm": float(d["bbox_norm"][3]),
            },
            # "orientation": 90-d.get("orientation", 0.0) if name != "banana" else d.get("orientation", 0.0),
            "orientation": d.get("orientation", 0.0),
            "confidence": float(d["confidence"]),
            "half_extents": list(half),
        }

    print(objects.keys())

    return {
        "eef_pos": eef_pos,
        "eef_ori": [math.degrees(v) for v in eef_rpy],
        "objects": objects,
        "x_range": X_RANGE,
        "y_range": Y_RANGE,
        "z_range": Z_RANGE,
        "table_z": TABLE_Z,
    }


def _compute_ik(limb, solver, x, y, z, roll_deg, pitch_deg, yaw_deg):
    z_robot_base = z - SAWYER_BASE_Z
    qx, qy, qz, qw = quaternion_from_euler(
        radians(roll_deg), radians(pitch_deg), radians(yaw_deg),
    )
    current = limb.joint_angles()
    seed = [current.get(j, 0.0) for j in JOINT_NAMES]
    return solver.get_ik(
        seed, x, y, z_robot_base, qx, qy, qz, qw,
        bx=0.001, by=0.001, bz=0.001,
        brx=0.1, bry=0.1, brz=0.1,
    )


def execute_waypoints(waypoints):
    _gripper.open()
    rospy.sleep(0.3)

    ep = _limb.endpoint_pose()
    cur_x = ep["position"].x
    cur_y = ep["position"].y
    cur_z_w = ep["position"].z + SAWYER_BASE_Z

    for wp in waypoints:
        valid = True
        for key in ("pos", "ori", "gripper"):
            if key not in wp:
                rospy.logerr("execute_waypoints: waypoint missing key '%s'", key)
                valid = False
        if valid and (len(wp["pos"]) != 3 or len(wp["ori"]) != 3):
            rospy.logerr("execute_waypoints: 'pos' and 'ori' must be length 3")
            valid = False
        if valid and wp["gripper"] not in ("open", "close"):
            rospy.logerr("execute_waypoints: 'gripper' must be 'open' or 'close'")
            valid = False
        if not valid:
            continue

        x, y, z_t = wp["pos"]
        r, p, ya = wp["ori"]
        if z_t <= 0.001:
            z_t = SAFE_Z

        xy_change = abs(x - cur_x) > 0.005 or abs(y - cur_y) > 0.005
        z_change = abs(z_t - cur_z_w) > 0.005

        if z_change:
            if xy_change:
                move_to_pose(x, y, cur_z_w, r, p, ya)
            move_straight_z(x, y, cur_z_w, z_t, r, p, ya)
        else:
            move_to_pose(x, y, z_t, r, p, ya)

        cur_x, cur_y, cur_z_w = x, y, z_t

        if wp["gripper"] == "open":
            _gripper.open()
        elif wp["gripper"] == "close":
            _gripper.close()
        rospy.sleep(0.3)

    go_to_safe_pose()


def move_to_pose(x, y, z, roll_deg, pitch_deg, yaw_deg, velocity=None):
    if velocity is None:
        velocity = VELOCITY_SCALING
    solution = _compute_ik(_limb, _solver, x, y, z, roll_deg, pitch_deg, yaw_deg)
    if not solution:
        return False
    joint_goal = dict(zip(JOINT_NAMES, solution))
    _limb.set_joint_position_speed(velocity)
    _limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)
    return True


def move_straight_z(x, y, z_start, z_end, roll_deg, pitch_deg, yaw_deg, steps=10):
    distance = z_end - z_start
    step_inc = distance / steps
    for i in range(1, steps + 1):
        z_current = z_start + (step_inc * i)
        success = move_to_pose(x, y, z_current, roll_deg, pitch_deg, yaw_deg, velocity=0.05)
        if not success:
            return False
        rospy.sleep(0.05)
    return True


def adjust_grasp_waypoints(scene, waypoints):
    adjusted = []
    snapped = set()
    last_picked = None
    for wp in waypoints:
        awp = dict(wp)
        gripper = awp["gripper"]

        if gripper == "close":
            x, y, z = awp["pos"]
            best_name = None
            best_obj = None
            best_dist = float("inf")
            for name, obj in scene["objects"].items():
                ox, oy, _ = obj["center"]
                dist = math.hypot(x - ox, y - oy)
                if dist < best_dist:
                    best_dist = dist
                    best_name = name
                    best_obj = obj
            if best_obj is not None and best_dist < 0.15:
                cx, cy, _ = best_obj["center"]
                if best_name not in snapped:
                    base_z = best_obj["base"][2]
                    half_z = best_obj["half_extents"][2]
                    pick_z = base_z + half_z
                    table_z = scene.get("table_z", 0.755)
                    if pick_z < table_z + 0.015:
                        pick_z = table_z + 0.015
                    awp["pos"] = [cx, cy, pick_z]
                    snapped.add(best_name)
                    last_picked = best_name

        elif gripper == "open" and last_picked is not None:
            x, y, z = awp["pos"]
            for name, obj in scene["objects"].items():
                ox, oy, _ = obj["center"]
                dist = math.hypot(x - ox, y - oy)
                if dist < 0.15:
                    ref_top = obj["center"][2] + obj["half_extents"][2]
                    picked_half_z = scene["objects"].get(last_picked, {}).get("half_extents", [0, 0, 0.02])[2]
                    min_z = ref_top + 2.0 * picked_half_z + 0.03
                    if z < min_z:
                        awp["pos"] = [x, y, min_z]
                    break

        adjusted.append(awp)
    return adjusted


def go_to_safe_pose():
    global _limb
    joint_goal = dict(zip(JOINT_NAMES, SAFE_POSE))
    _limb.set_joint_position_speed(VELOCITY_SCALING)
    _limb.move_to_joint_positions(joint_goal, timeout=MOVE_TIMEOUT)


def go_to_neutral():
    global _limb
    _limb.set_joint_position_speed(VELOCITY_SCALING)
    _limb.move_to_neutral(timeout=MOVE_TIMEOUT)


def get_limb():
    global _limb
    return _limb


def get_gripper():
    global _gripper
    return _gripper
