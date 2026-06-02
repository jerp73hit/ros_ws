#!/usr/bin/env python3
"""
bbox_to_world.py  — fixed coordinate transform
===============================================
Drop-in replacement for the original.  The only changes are:
  1. _R_FIX is now a parameter you can override from outside
  2. A calibrate_r_fix() helper finds the correct rotation automatically
     given one known detection + its ground-truth Gazebo position
  3. results_to_world() accepts an r_fix= kwarg for easy testing
"""

import math
from typing import Optional, List
import yaml
import numpy as np
import torch
from capture_frame import load_snapshot

# ── Object classes ────────────────────────────────────────────────────────────
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

_ID_TO_CLASS = {cid: (name, ext) for name, (cid, ext) in OBJECT_CLASSES.items()}

# ── Rotation: Gazebo body frame → ROS optical frame ──────────────────────────
#
# This is the matrix you are most likely to need to change.
#
# The dome script assumed the camera is mounted looking straight down the
# Gazebo +X axis with Z up.  If your camera is actually pointing at the table
# from above-and-front (typical for a wrist/head camera), the correct matrix
# is different.
#
# HOW TO FIND YOURS:
#   Run calibrate_r_fix() at the bottom of this file — it prints the best
#   R_FIX for your actual camera pose given the ground-truth object positions
#   from /gazebo/model_states.
#
# Common candidates to try if calibration is not available:
#   Original dome script (looking along +X, Z up):
#     [[ 0, -1,  0],
#      [ 0,  0, -1],
#      [ 1,  0,  0]]
#
#   Camera looking along -Y (table in front, standard Sawyer head camera):
#     [[ 1,  0,  0],
#      [ 0,  0, -1],
#      [ 0,  1,  0]]
#
#   Camera looking along +X, Z down:
#     [[ 0,  1,  0],
#      [ 0,  0,  1],
#      [ 1,  0,  0]]
#
_R_FIX = np.array([
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0],
    [ 0.0, -1.0,  0.0]
], dtype=float)

_R_FIX_INV = _R_FIX.T

DEPTH_CROP_RATIO      = 0.5
DEPTH_MIN_VALID_PIXELS = 5


# ─────────────────────────────────────────────────────────────────────────────
#  Depth sampling 
# ─────────────────────────────────────────────────────────────────────────────

def sample_depth_in_bbox(
    depth_img, u_min, v_min, u_max, v_max,
    crop_ratio=DEPTH_CROP_RATIO, min_valid=DEPTH_MIN_VALID_PIXELS,
):
    h_img, w_img = depth_img.shape[:2]
    u_min = max(0, u_min);  u_max = min(w_img, u_max)
    v_min = max(0, v_min);  v_max = min(h_img, v_max)
    if u_max <= u_min or v_max <= v_min:
        return None
    shrink_u = int((u_max - u_min) * (1.0 - crop_ratio) / 2)
    shrink_v = int((v_max - v_min) * (1.0 - crop_ratio) / 2)
    cu_min = u_min + shrink_u;  cu_max = u_max - shrink_u
    cv_min = v_min + shrink_v;  cv_max = v_max - shrink_v
    if cu_max <= cu_min or cv_max <= cv_min:
        cu_min, cu_max = u_min, u_max
        cv_min, cv_max = v_min, v_max
    roi   = depth_img[cv_min:cv_max, cu_min:cu_max]
    valid = roi[np.isfinite(roi) & (roi > 0.0)]
    if len(valid) < min_valid:
        return None
    return float(np.median(valid))


# ─────────────────────────────────────────────────────────────────────────────
#  Core back-projection
# ─────────────────────────────────────────────────────────────────────────────

def backproject_to_world(u, v, z_optical, cam_pos, cam_quat, K,
                         r_fix_inv=None):
    """
    Back-project pixel (u,v) + depth to world frame.

    r_fix_inv : 3x3 inverse of the Gazebo-body→optical rotation.
                Defaults to the module-level _R_FIX_INV.
                Pass a custom matrix here when testing candidates.
    """
    from tf.transformations import quaternion_matrix

    if r_fix_inv is None:
        r_fix_inv = _R_FIX_INV

    x_opt = (u - K[0, 2]) / K[0, 0] * z_optical
    y_opt = (v - K[1, 2]) / K[1, 1] * z_optical
    p_opt = np.array([x_opt, y_opt, z_optical], dtype=float)

    p_gz    = r_fix_inv @ p_opt
    R       = quaternion_matrix(cam_quat)[:3, :3]
    p_rel   = R @ p_gz
    return cam_pos + p_rel


def bbox_to_world(
    cx_norm, cy_norm, w_norm, h_norm,
    class_id, cam_pos, cam_quat, K, img_w, img_h,
    depth_img=None, depth_K=None, r_fix_inv=None,
):
    name, (hx, hy, hz) = _ID_TO_CLASS[class_id]

    u_min = int((cx_norm - w_norm / 2) * img_w)
    v_min = int((cy_norm - h_norm / 2) * img_h)
    u_max = int((cx_norm + w_norm / 2) * img_w)
    v_max = int((cy_norm + h_norm / 2) * img_h)
    u_ctr = cx_norm * img_w
    v_ctr = cy_norm * img_h
    # NEW: Instead of the middle of the box, target the bottom face!
    # Moving 80% down the bounding box safely hits the base of the object
    v_ctr_base = (cy_norm + (h_norm * 0.40)) * img_h

    depth_source = "size_fallback"
    z_optical    = None

    if depth_img is not None:
        dk = depth_K if depth_K is not None else K
        if depth_img.shape[1] != img_w or depth_img.shape[0] != img_h:
            sx = depth_img.shape[1] / img_w
            sy = depth_img.shape[0] / img_h
            d_u_min = int(u_min * sx);  d_u_max = int(u_max * sx)
            d_v_min = int(v_min * sy);  d_v_max = int(v_max * sy)
            d_u_ctr = u_ctr * sx;       d_v_ctr = v_ctr * sy
        else:
            d_u_min, d_u_max = u_min, u_max
            d_v_min, d_v_max = v_min, v_max
            d_u_ctr, d_v_ctr = u_ctr, v_ctr

        sampled = sample_depth_in_bbox(depth_img, d_u_min, d_v_min,
                                       d_u_max, d_v_max)
        if sampled is not None:
            z_optical    = sampled
            depth_source = "depth_image"
            world_xyz = backproject_to_world(
                d_u_ctr, d_v_ctr, z_optical,
                cam_pos, cam_quat, dk, r_fix_inv=r_fix_inv)

    if z_optical is None:
        h_px      = max(h_norm * img_h, 1e-3)
        z_optical = K[1, 1] * (2.0 * hz) / h_px
        world_xyz = backproject_to_world(
            u_ctr, v_ctr, z_optical,
            cam_pos, cam_quat, K, r_fix_inv=r_fix_inv)

    world_base    = world_xyz.copy()
    world_base[2] -= hz

    return {
        "name":         name,
        "class_id":     class_id,
        "world_xyz":    world_xyz,
        "world_base":   world_base,
        "depth":        float(z_optical),
        "depth_source": depth_source,
        "bbox_norm":    (cx_norm, cy_norm, w_norm, h_norm),
        "bbox_px":      (u_min, v_min, u_max, v_max),
    }


def results_to_world(
    result, cam_pos, cam_quat,
    K=None, fov_rad=1.3962634,
    depth_img=None, depth_K=None,
    r_fix_inv=None,
):
    img_h, img_w = result.orig_shape

    if K is None:
        fx = img_w / (2 * math.tan(fov_rad / 2))
        K  = np.array([[fx,  0, img_w / 2],
                       [ 0, fx, img_h / 2],
                       [ 0,  0,         1]], dtype=float)

    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []

    xywhn = boxes.xywhn.cpu().numpy()
    clss  = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()

    detections = []
    for (cx, cy, w, h), cid, conf in zip(xywhn, clss, confs):
        if cid not in _ID_TO_CLASS:
            continue
        det = bbox_to_world(
            cx_norm=float(cx), cy_norm=float(cy),
            w_norm=float(w),   h_norm=float(h),
            class_id=int(cid),
            cam_pos=cam_pos, cam_quat=cam_quat, K=K,
            img_w=img_w, img_h=img_h,
            depth_img=depth_img, depth_K=depth_K,
            r_fix_inv=r_fix_inv,
        )
        det["confidence"] = float(conf)
        detections.append(det)

    return detections


# ─────────────────────────────────────────────────────────────────────────────
#  Calibration helper
# ─────────────────────────────────────────────────────────────────────────────

# All 24 valid rotation matrices (orthonormal, det=+1, only ±1 entries)
_CANDIDATE_R_FIX = []
for _signs in [(1,1,1),(1,1,-1),(1,-1,1),(1,-1,-1),
               (-1,1,1),(-1,1,-1),(-1,-1,1),(-1,-1,-1)]:
    for _perm in [(0,1,2),(0,2,1),(1,0,2),(1,2,0),(2,0,1),(2,1,0)]:
        _R = np.zeros((3,3))
        for _row, (_col, _s) in enumerate(zip(_perm, _signs)):
            _R[_row, _col] = _s
        if abs(np.linalg.det(_R) - 1.0) < 1e-9:
            _CANDIDATE_R_FIX.append(_R)


def calibrate_r_fix(
    detections_estimated: List[dict],
    ground_truth: dict,
    cam_pos: np.ndarray,
    cam_quat: np.ndarray,
    K: np.ndarray,
    depth_img: np.ndarray,
    img_w: int,
    img_h: int,
    verbose: bool = True,
):
    """
    Find the R_FIX rotation that minimises reprojection error against
    known Gazebo ground-truth positions.

    Parameters
    ----------
    detections_estimated : output of results_to_world() — used only for
                           bbox_norm and class_id fields
    ground_truth         : dict mapping class name → [x, y, z] from
                           /gazebo/model_states
                           e.g. {"coke_can": [0.650, 0.300, 0.772],
                                 "bowl":     [0.850, 0.200, 0.775]}
    cam_pos, cam_quat, K, depth_img, img_w, img_h : same as results_to_world

    Returns
    -------
    best_R  : 3x3 np.ndarray — use as new _R_FIX
    best_err: float — mean position error in metres
    """
    from tf.transformations import quaternion_matrix

    # Build list of (bbox_norm, class_id, gt_xyz) pairs
    pairs = []
    for det in detections_estimated:
        name = det["name"]
        if name in ground_truth:
            pairs.append((det["bbox_norm"], det["class_id"],
                          np.array(ground_truth[name], dtype=float)))

    if not pairs:
        print("[calibrate] No overlap between detections and ground_truth keys.")
        return None, None

    best_R   = None
    best_err = float("inf")
    results_table = []

    for R_fix in _CANDIDATE_R_FIX:
        r_fix_inv = R_fix.T
        errors = []
        for (cx, cy, w, h), cid, gt_xyz in pairs:
            det = bbox_to_world(
                cx_norm=cx, cy_norm=cy, w_norm=w, h_norm=h,
                class_id=cid,
                cam_pos=cam_pos, cam_quat=cam_quat, K=K,
                img_w=img_w, img_h=img_h,
                depth_img=depth_img,
                r_fix_inv=r_fix_inv,
            )
            err = np.linalg.norm(det["world_xyz"] - gt_xyz)
            errors.append(err)
        mean_err = float(np.mean(errors))
        results_table.append((mean_err, R_fix))
        if mean_err < best_err:
            best_err = mean_err
            best_R   = R_fix.copy()

    results_table.sort(key=lambda x: x[0])

    if verbose:
        print("\n── R_FIX calibration results (top 5) ──────────────────────")
        for err, R in results_table[:5]:
            print(f"  mean error = {err:.4f} m")
            print(f"  R_FIX =\n{R}\n")
        print(f"Best R_FIX (mean error {best_err:.4f} m):")
        print(best_R)
        print("\nPaste into your script:")
        print("_R_FIX = np.array([")
        for row in best_R:
            print(f"    {row.tolist()},")
        print("], dtype=float)")
        print("_R_FIX_INV = _R_FIX.T")

    return best_R, best_err


# ─────────────────────────────────────────────────────────────────────────────
#  yaml loader 
# ─────────────────────────────────────────────────────────────────────────────

def _patch_numpy_core():
    import sys, numpy
    if hasattr(numpy, '_core'):
        return
    import numpy.core as _nc, importlib
    sys.modules.setdefault('numpy._core', _nc)
    for _sub in ('multiarray','numeric','fromnumeric','function_base',
                 'records','defchararray','umath','shape_base',
                 'overrides','_multiarray_umath'):
        key = 'numpy._core.' + _sub
        if key not in sys.modules:
            try:
                sys.modules[key] = importlib.import_module('numpy.core.'+_sub)
            except ImportError:
                pass


def load_results_yaml(yaml_path: str):
    _patch_numpy_core()
    original = torch.load
    def cpu_load(*a, **kw):
        kw["map_location"] = torch.device("cpu")
        return original(*a, **kw)
    torch.load = cpu_load
    try:
        with open(yaml_path, "r") as f:
            data = yaml.load(f, Loader=yaml.UnsafeLoader)
    finally:
        torch.load = original
    return data


# ─────────────────────────────────────────────────────────────────────────────
#  __main__
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import json

    result = load_results_yaml(
        "/home/david/ros_ws/src/sawyer_moveit/sawyer_moveit_mycodes/"
        "sawyer-moveit-gazebo-bridge/scripts/res.yaml")
    if isinstance(result, list):
        result = result[0]

    try:
        depth_img = np.load("depth.npy").astype(np.float32)
        print(f"Depth image loaded: {depth_img.shape}  "
              f"range [{np.nanmin(depth_img):.3f}, {np.nanmax(depth_img):.3f}] m")
    except FileNotFoundError:
        depth_img = None
        print("depth.npy not found — using size-fallback.")

    snap     = load_snapshot("~/ros_ws/proof_imgs")
    
    # 1. Get the pure, unadulterated TF coordinates
    cam_pos  = snap["cam_pos"]
    cam_quat = snap["cam_quat"]

    print("printing cam pos and quat")
    print(cam_pos)
    print(cam_quat)
    
    # 2. Add the Gazebo pedestal height!
    cam_pos[2] += 0.930 
    
    K = snap["K"]
    if depth_img is None:
        depth_img = snap["depth"]

    img_h, img_w = result.orig_shape

    # ── Run with current R_FIX ────────────────────────────────────────────
    detections = results_to_world(
        result, cam_pos=cam_pos, cam_quat=cam_quat,
        K=K, depth_img=depth_img,
    )

    print(f"\nFound {len(detections)} object(s):\n")
    for d in detections:
        wx, wy, wz = d["world_xyz"]
        bx, by, bz = d["world_base"]
        print(
            f"  {d['name']:22s}  conf={d['confidence']:.2f}  "
            f"centre=({wx:+.3f}, {wy:+.3f}, {wz:+.3f})  "
            f"base=({bx:+.3f}, {by:+.3f}, {bz:+.3f})  "
            f"depth={d['depth']:.3f}m  [{d['depth_source']}]"
        )

    # ── Calibrate R_FIX against ground truth from /gazebo/model_states ───
    # Paste the real positions from:  rostopic echo -n1 /gazebo/model_states
    ground_truth = {
        "mustard":            [0.512, 0.167, 0.778],
        "coke_can":           [0.650, 0.300, 0.772],
        "bowl":               [0.853, 0.294, 0.775],
        "banana":             [0.450, 0.000, 0.800],
        "strawberry":         [0.650, 0.000, 0.790],
        "planta_maceta":      [0.850, 0.000, 0.775],
        "esponja_lavaplatos": [0.445, -0.300, 0.775],
        "papas_fritas":       [0.650, -0.300, 0.775],
        "block":              [0.550, -0.250, 0.795],
    }

    print("\n── Running R_FIX calibration ────────────────────────────────")
    best_R, best_err = calibrate_r_fix(
        detections_estimated=detections,
        ground_truth=ground_truth,
        cam_pos=cam_pos,
        cam_quat=cam_quat,
        K=K,
        depth_img=depth_img,
        img_w=img_w,
        img_h=img_h,
    )

    if best_R is not None and best_err < 0.2:
        print(f"\nRe-running with calibrated R_FIX (error {best_err:.4f} m):")
        detections = results_to_world(
            result, cam_pos=cam_pos, cam_quat=cam_quat,
            K=K, depth_img=depth_img,
            r_fix_inv=best_R.T,
        )
        for d in detections:
            wx, wy, wz = d["world_xyz"]
            print(
                f"  {d['name']:22s}  conf={d['confidence']:.2f}  "
                f"centre=({wx:+.3f}, {wy:+.3f}, {wz:+.3f})"
            )

    # ── Save JSON ─────────────────────────────────────────────────────────
    out = []
    for d in detections:
        out.append({
            "name":         d["name"],
            "class_id":     d["class_id"],
            "confidence":   d["confidence"],
            "world_xyz":    d["world_xyz"].tolist(),
            "world_base":   d["world_base"].tolist(),
            "depth_m":      d["depth"],
            "depth_source": d["depth_source"],
            "bbox_norm":    list(d["bbox_norm"]),
        })
    with open("detections_world.json", "w") as f:
        json.dump(out, f, indent=2)
    print("\nSaved detections_world.json")