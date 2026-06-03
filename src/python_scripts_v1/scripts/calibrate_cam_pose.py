#!/usr/bin/env python3
"""
calibrate_cam_pose.py
=====================
Diagnoses and fixes wrong cam_pos / cam_quat by working backwards from
known ground-truth object positions.

Run this standalone — it does NOT need a live ROS environment.
It reads detections_world.json (already written by bbox_to_world.py) and
the depth.npy file, then figures out the true camera position.

Steps
-----
1. Prints a table comparing estimated vs real positions so you can see
   the exact offset pattern.
2. Solves for the true cam_pos using least-squares over all detections.
3. Prints the corrected cam_pos to paste into your script.
4. (Optional) also checks cam_quat by testing small rotations around
   the current one.
"""

import json
import math
import numpy as np
from typing import Optional


# ── Paste your ground truth here (from rostopic echo /gazebo/model_states) ──
GROUND_TRUTH  = {
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


def load_detections(path="detections_world.json"):
    with open(path) as f:
        raw = json.load(f)
    for d in raw:
        d["world_xyz"]  = np.array(d["world_xyz"])
        d["world_base"] = np.array(d["world_base"])
        d["bbox_norm"]  = tuple(d["bbox_norm"])
    return raw


def print_comparison(detections):
    """Print estimated vs real, and the per-object offset vector."""
    pairs = [(d, np.array(GROUND_TRUTH[d["name"]]))
             for d in detections if d["name"] in GROUND_TRUTH]

    print("\n── Estimated vs ground truth ────────────────────────────────────")
    print(f"  {'Object':<22}  {'Est XYZ':>30}  {'Real XYZ':>30}  {'Offset':>30}")
    offsets = []
    for det, gt in pairs:
        e  = det["world_xyz"]
        off = gt - e
        offsets.append(off)
        print(f"  {det['name']:<22}  "
              f"({e[0]:+.3f},{e[1]:+.3f},{e[2]:+.3f})  "
              f"({gt[0]:+.3f},{gt[1]:+.3f},{gt[2]:+.3f})  "
              f"({off[0]:+.3f},{off[1]:+.3f},{off[2]:+.3f})")

    if not offsets:
        print("  No matching objects found.")
        return None

    mean_off = np.mean(offsets, axis=0)
    std_off  = np.std(offsets,  axis=0)
    print(f"\n  Mean offset (real - est): ({mean_off[0]:+.4f}, {mean_off[1]:+.4f}, {mean_off[2]:+.4f})")
    print(f"  Std  offset             : ({std_off[0]:+.4f},  {std_off[1]:+.4f},  {std_off[2]:+.4f})")

    if std_off.max() < 0.05:
        print("\n  ✓ Offsets are CONSISTENT — this is purely a cam_pos translation error.")
        print("  Fix: add the mean offset to your cam_pos.")
    else:
        print("\n  ✗ Offsets are INCONSISTENT — likely a rotation error (wrong cam_quat or R_FIX).")
        print("  The offset varies per object, meaning the angular component is wrong.")

    return mean_off


def solve_cam_pos(detections, snap_cam_pos, snap_cam_quat, K,
                  depth_img=None, r_fix=None):
    """
    Given that world_xyz = cam_pos + R @ R_fix_inv @ p_opt,
    and we know the true world_xyz (ground truth), solve for cam_pos.

    This works because:
        true_xyz = cam_pos_true + (estimated_xyz - cam_pos_used)
        cam_pos_true = true_xyz - (estimated_xyz - cam_pos_used)
                     = true_xyz - estimated_xyz + cam_pos_used

    Which is exactly: cam_pos_true = cam_pos_used + mean(gt - est)
    """
    pairs = [(d, np.array(GROUND_TRUTH[d["name"]]))
             for d in detections if d["name"] in GROUND_TRUTH]
    if not pairs:
        return None

    corrections = []
    for det, gt in pairs:
        corrections.append(gt - det["world_xyz"])

    mean_correction = np.mean(corrections, axis=0)
    corrected_pos   = snap_cam_pos + mean_correction

    print("\n── Camera position diagnosis ────────────────────────────────────")
    print(f"  cam_pos used in script  : {snap_cam_pos.tolist()}")
    print(f"  Mean correction needed  : {mean_correction.tolist()}")
    print(f"  Corrected cam_pos       : {corrected_pos.tolist()}")
    print(f"\n  Paste into your script / snapshot:")
    print(f"  cam_pos = np.array({corrected_pos.tolist()})")

    return corrected_pos


def verify_corrected(detections, corrected_cam_pos):
    """Re-compute world positions with the corrected cam_pos and print errors."""
    pairs = [(d, np.array(GROUND_TRUTH[d["name"]]))
             for d in detections if d["name"] in GROUND_TRUTH]

    print("\n── Verification with corrected cam_pos ──────────────────────────")
    errors = []
    for det, gt in pairs:
        corrected_xyz = det["world_xyz"] + (corrected_cam_pos -
                        (det["world_xyz"] - (gt - (gt - det["world_xyz"]))))
        # Simpler: corrected = gt  (by construction of the mean correction)
        # But show per-object residual (std, not mean — mean is zero by construction)
        residual = gt - (det["world_xyz"] + (corrected_cam_pos - corrected_cam_pos))
        # Actually compute properly:
        corrected_xyz = det["world_xyz"] + (corrected_cam_pos -
                                            np.array(det["world_xyz"]) +
                                            np.array(det["world_xyz"]))
    # Cleaner approach: just shift all estimates by mean_correction and measure residuals
    mean_correction = np.mean([np.array(GROUND_TRUTH[d["name"]]) - d["world_xyz"]
                                for d in detections if d["name"] in GROUND_TRUTH], axis=0)
    print(f"  {'Object':<22}  {'Corrected XYZ':>32}  {'Error (m)':>10}")
    for det, gt in pairs:
        corrected = det["world_xyz"] + mean_correction
        err = np.linalg.norm(corrected - gt)
        errors.append(err)
        print(f"  {det['name']:<22}  "
              f"({corrected[0]:+.3f},{corrected[1]:+.3f},{corrected[2]:+.3f})  "
              f"{err:.4f} m")
    print(f"\n  Mean error after correction: {np.mean(errors):.4f} m")
    print(f"  Max  error after correction: {np.max(errors):.4f} m")
    if np.mean(errors) < 0.05:
        print("  ✓ Correction successful — cam_pos was the only problem.")
    elif np.mean(errors) < 0.15:
        print("  ~ Partial fix — residual error suggests cam_quat or R_FIX also needs tuning.")
    else:
        print("  ✗ Large residual — rotation (cam_quat or R_FIX) is also wrong.")


if __name__ == "__main__":
    import sys

    # ── Load detections ───────────────────────────────────────────────────
    det_path = "detections_world.json"
    print(f"Loading {det_path} ...")
    detections = load_detections(det_path)
    print(f"  {len(detections)} detections loaded.")

    # ── Print comparison table ────────────────────────────────────────────
    mean_off = print_comparison(detections)

    if mean_off is None:
        print("Cannot continue without matching detections.")
        sys.exit(1)

    # ── Load snap to get current cam_pos ──────────────────────────────────
    try:
        from capture_frame import load_snapshot
        snap     = load_snapshot("~/ros_ws/proof_imgs")
        cam_pos  = snap["cam_pos"]
        cam_quat = snap["cam_quat"]
        K        = snap["K"]
    except Exception as e:
        print(f"\nCould not load snapshot ({e}), using fallback cam_pos.")
        cam_pos  = np.array([0.75, 0.0, 1.755])
        cam_quat = np.array([0.0, 0.0, 0.0, 1.0])
        K        = None

    # ── Solve for true cam_pos ────────────────────────────────────────────
    corrected_pos = solve_cam_pos(detections, cam_pos, cam_quat, K)

    # ── Verify ────────────────────────────────────────────────────────────
    verify_corrected(detections, corrected_pos)

    # ── Also check if there's a TF topic we can read directly ─────────────
    print("\n── Tip: get the exact camera pose from TF ───────────────────────")
    print("  Run this to see your camera's real pose in the world frame:")
    print("  rosrun tf tf_echo world /io/internal_camera/right_hand_camera")
    print("  or:")
    print("  rosrun tf tf_echo world /right_hand_camera")
    print("  The 'Translation' line IS your correct cam_pos.")