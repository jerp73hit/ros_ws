#!/usr/bin/env python3
import json
import math

import numpy as np
import rospy
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import PointStamped
from gazebo_msgs.srv import GetModelState

from llm_api import (
    init_api,
    make_scene,
    debug_camera,
    move_to_pose,
    move_straight_z,
    go_to_safe_pose,
    get_gripper,
)
from frame2world import get_r_fix, OBJECT_CLASSES

init_api()
rospy.sleep(1.0)

# ── 1. Raw camera data ────────────────────────────────────────────────
cam = debug_camera()
print("=" * 70)
print("  Camera intrinsics K:")
print(np.array2string(cam["K"], precision=1, suppress_small=True))
print(f"\n  Camera position (world): {cam['cam_pos'].tolist()}")
print(f"  Camera quaternion:       {cam['cam_quat'].tolist()}")
R_world_to_cam = quaternion_matrix(cam["cam_quat"])[:3, :3]
print(f"  Rotation world→camera-body (3x3):")
print(np.array2string(R_world_to_cam, precision=4, suppress_small=True))
print(f"\n  _R_FIX (body→optical):")
print(np.array2string(cam["r_fix"], precision=1, suppress_small=True))
print(f"  _R_FIX_INV (optical→body):")
print(np.array2string(cam["r_fix"].T, precision=1, suppress_small=True))

# ── 2. Ground truth from Gazebo ───────────────────────────────────────
print("\n" + "─" * 70)
print("  Ground truth (Gazebo model states):")
rospy.wait_for_service("/gazebo/get_model_state")
get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
gt = {}
for name in OBJECT_CLASSES:
    try:
        resp = get_state(name, "world")
        if resp.success:
            p = resp.pose.position
            gt[name] = np.array([p.x, p.y, p.z])
        else:
            gt[name] = None
    except Exception:
        gt[name] = None

for name, xyz in sorted(gt.items()):
    if xyz is not None:
        print(f"    {name:22s}  ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})")
    else:
        print(f"    {name:22s}  (not found)")

# ── 3. Scene from vision ──────────────────────────────────────────────
scene = make_scene()
if not scene:
    print("\n[test] make_scene returned empty dict — aborting.")
    exit(1)

eef = scene["eef_pos"]
eef_ori = scene["eef_ori"]
objs = scene["objects"]

print("\n" + "─" * 70)
print("  EEF position : ({:.3f}, {:.3f}, {:.3f})  [world]".format(*eef))
print("  EEF RPY deg  : ({:.1f}, {:.1f}, {:.1f})".format(*eef_ori))
print("  Workspace X  : {:.2f} – {:.2f}".format(*scene["x_range"]))
print("  Workspace Y  : {:.2f} – {:.2f}".format(*scene["y_range"]))
print("  Workspace Z  : {:.2f} – {:.2f}".format(*scene["z_range"]))
print("  Table Z      : {:.3f}".format(scene["table_z"]))

if not objs:
    print("\n  No objects detected.")
    exit(1)

# ── 4. Per-detection projection trace ─────────────────────────────────
print("\n" + "─" * 70)
print("  Vision detections (with projection trace):")

# Manually replicate backprojection for trace
K = cam["K"]
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
r_fix_inv = cam["r_fix"].T

for name, obj in sorted(objs.items()):
    c = obj["center"]
    b = obj["base"]
    bbox = obj["bbox"]
    u = bbox["cx_norm"] * cam["K"][0, 2] * 2  # rough img_w via 2*cx
    # better: reconstruct img_w from K
    # img_w ≈ 2*cx, but let's just use the stored values
    # actually bbox_px isn't stored in the scene dict... it is from frame2world but make_scene doesn't pass it through
    # Let's do what we can with what we have
    print(f"\n  {name:22s}")
    print(f"    centre        : ({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f})")
    print(f"    base          : ({b[0]:+.3f}, {b[1]:+.3f}, {b[2]:+.3f})")
    print(f"    depth         : {obj.get('depth', 'N/A'):.4f}  [{obj.get('depth_source', '?')}]")
    print(f"    half-extents  : {obj['half_extents']}")
    print(f"    confidence    : {obj['confidence']:.2f}")

    # GT comparison
    if name in gt and gt[name] is not None:
        err = np.linalg.norm(np.array(c) - gt[name])
        print(f"    GT            : ({gt[name][0]:+.3f}, {gt[name][1]:+.3f}, {gt[name][2]:+.3f})")
        print(f"    centre vs GT  : error = {err:.4f} m")

# ── 5. Pick-and-place (pick_place3 replication) ─────────────────────
pick_names = ["strawberry", "planta_maceta"]
pick_name = next((n for n in pick_names if n in objs), None)
if pick_name is None:
    print("\n[test] None of the preferred objects detected.")
    exit(0)

target = objs[pick_name]
cx, cy, cz = target["center"]
gt_xyz = gt.get(pick_name)
# Drop 5cm right of pick (same as pick_place3's generate_valid_destination but fixed)
drop_x, drop_y = min(cx + 0.05, 0.90), cy
SAFE_Z = 0.93
ROLL = 180.0
PITCH = 0.0
YAW = 180.0

print(f"\n" + "─" * 70)
print(f"  Pick target: {pick_name} @ vision=({cx:.3f}, {cy:.3f}, {cz:.3f})")
if gt_xyz is not None:
    err = math.hypot(cx - gt_xyz[0], cy - gt_xyz[1])
    print(f"  GT position:               @ ({gt_xyz[0]:+.3f}, {gt_xyz[1]:+.3f}, {gt_xyz[2]:+.3f})")
    print(f"  XY error vs GT:  {err:.4f} m")
print(f"  Drop target: ({drop_x:.3f}, {drop_y:.3f})")

# ── pick_place3 exact sequence ─────────────────────────────────────
# Phase 1: Approach at SAFE_Z with yaw=0
print("\n[test] Phase 1: Hovering over target (XY with yaw=0)...")
if not move_to_pose(cx, cy, SAFE_Z, ROLL, PITCH, 0.0):
    print("[test] Phase 1 IK failed. Aborting.")
    exit(1)
rospy.sleep(0.5)

# Phase 2: Align to target yaw at SAFE_Z
print("[test] Phase 2: Aligning yaw...")
if not move_to_pose(cx, cy, SAFE_Z, ROLL, PITCH, YAW):
    print("[test] Phase 2 IK failed. Aborting.")
    exit(1)
rospy.sleep(0.5)

# Phase 3: Linear Z descent → grasp → Z ascent
print("[test] Phase 3: Linear Z descent...")
pick_z = cz + 0.012  # Z_OFFSET above object top (matching pick_place3)
if not move_straight_z(cx, cy, SAFE_Z, pick_z, ROLL, PITCH, YAW, steps=10):
    print("[test] Descent failed. Aborting.")
    exit(1)

print("[test] Grasping...")
get_gripper().close()
rospy.sleep(1.0)

print("[test] Linear Z ascent...")
if not move_straight_z(cx, cy, pick_z, SAFE_Z, ROLL, PITCH, YAW, steps=10):
    print("[test] Ascent failed. Aborting.")
    exit(1)

# Phase 4: Move XY to drop → Z descent → release → Z ascent
print("[test] Phase 4: Moving to drop location...")
if not move_to_pose(drop_x, drop_y, SAFE_Z, ROLL, PITCH, YAW):
    print("[test] Phase 4 IK failed. Aborting.")
    exit(1)
rospy.sleep(0.5)

print("[test] Linear Z descent...")
if not move_straight_z(drop_x, drop_y, SAFE_Z, pick_z, ROLL, PITCH, YAW, steps=10):
    print("[test] Drop descent failed. Aborting.")
    exit(1)

print("[test] Releasing...")
get_gripper().open()
rospy.sleep(1.0)

print("[test] Linear Z ascent...")
if not move_straight_z(drop_x, drop_y, pick_z, SAFE_Z, ROLL, PITCH, YAW, steps=10):
    print("[test] Drop ascent failed. Aborting.")
    exit(1)

# Return to standby
print("[test] Returning to standby...")
go_to_safe_pose()
print("[test] ✓ Pick-and-place cycle completed successfully.")
