import math
import numpy as np

OBJECT_CLASSES = {
    "mustard":            (0,  (0.038, 0.030, 0.075)),
    "coke_can":           (1,  (0.033, 0.033, 0.058)),
    "bowl":               (2,  (0.095, 0.095, 0.028)),
    "banana":             (3,  (0.110, 0.030, 0.022)),
    "strawberry":         (4,  (0.025, 0.025, 0.022)),
    "plant":      (5,  (0.065, 0.065, 0.090)),
    "sponge":             (6,  (0.0375, 0.02625, 0.01125)),
    "potatoes":           (7,  (0.040, 0.040, 0.040)),
    "block":              (8,  (0.028, 0.028, 0.028)),
}

_ID_TO_CLASS = {cid: (name, ext) for name, (cid, ext) in OBJECT_CLASSES.items()}

# Gazebo body frame → ROS optical frame rotation.
# This is a calibration constant — tune for your camera mount.
_R_FIX = np.eye(3, dtype=float)

_R_FIX_INV = _R_FIX.T

_DEPTH_CROP_RATIO = 0.5
_DEPTH_MIN_VALID_PIXELS = 5
Z_BIAS = 0.91


def set_r_fix(matrix):
    global _R_FIX, _R_FIX_INV
    _R_FIX = np.array(matrix, dtype=float)
    _R_FIX_INV = _R_FIX.T


def get_r_fix():
    return _R_FIX.copy()


def _sample_depth_in_bbox(
    depth_img, u_min, v_min, u_max, v_max,
    crop_ratio=_DEPTH_CROP_RATIO, min_valid=_DEPTH_MIN_VALID_PIXELS,
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
    roi = depth_img[cv_min:cv_max, cu_min:cu_max]
    valid = roi[np.isfinite(roi) & (roi > 0.0)]
    if len(valid) < min_valid:
        return None
    return float(np.median(valid))


def _backproject_to_world(u, v, z_optical, cam_pos, cam_quat, K, r_fix_inv=None):
    from tf.transformations import quaternion_matrix

    if r_fix_inv is None:
        r_fix_inv = _R_FIX_INV

    x_opt = (u - K[0, 2]) / K[0, 0] * z_optical
    y_opt = (v - K[1, 2]) / K[1, 1] * z_optical
    p_opt = np.array([x_opt, y_opt, z_optical], dtype=float)

    p_gz  = r_fix_inv @ p_opt
    R     = quaternion_matrix(cam_quat)[:3, :3]
    p_rel = R @ p_gz
    result = cam_pos + p_rel
    result[2] += Z_BIAS
    return result


def _bbox_to_world(
    cx_norm, cy_norm, w_norm, h_norm,
    class_id, cam_pos, cam_quat, K, img_w, img_h,
    depth_img=None, r_fix_inv=None,
):
    name, (hx, hy, hz) = _ID_TO_CLASS[class_id]

    u_min = int((cx_norm - w_norm / 2) * img_w)
    v_min = int((cy_norm - h_norm / 2) * img_h)
    u_max = int((cx_norm + w_norm / 2) * img_w)
    v_max = int((cy_norm + h_norm / 2) * img_h)
    u_ctr = cx_norm * img_w
    v_ctr = cy_norm * img_h

    depth_source = "size_fallback"
    z_optical = None

    if depth_img is not None:
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

        sampled = _sample_depth_in_bbox(depth_img, d_u_min, d_v_min, d_u_max, d_v_max)
        if sampled is not None:
            z_optical = sampled
            depth_source = "depth_image"
            world_xyz = _backproject_to_world(
                d_u_ctr, d_v_ctr, z_optical, cam_pos, cam_quat, K,
                r_fix_inv=r_fix_inv,
            )

    if z_optical is None:
        h_px = max(h_norm * img_h, 1e-3)
        z_optical = K[1, 1] * (2.0 * hz) / h_px
        world_xyz = _backproject_to_world(
            u_ctr, v_ctr, z_optical, cam_pos, cam_quat, K,
            r_fix_inv=r_fix_inv,
        )

    world_base = world_xyz.copy()
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


def get_positions(results, depth_img, K, cam_pos, cam_quat, r_fix_inv=None):
    img_h, img_w = results.orig_shape

    boxes = results.boxes
    if boxes is None or len(boxes) == 0:
        return []

    xywhn = boxes.xywhn.cpu().numpy()
    clss  = boxes.cls.cpu().numpy().astype(int)
    confs = boxes.conf.cpu().numpy()

    detections = []
    for (cx, cy, w, h), cid, conf in zip(xywhn, clss, confs):
        if cid not in _ID_TO_CLASS:
            continue
        det = _bbox_to_world(
            cx_norm=float(cx), cy_norm=float(cy),
            w_norm=float(w),   h_norm=float(h),
            class_id=int(cid),
            cam_pos=cam_pos, cam_quat=cam_quat, K=K,
            img_w=img_w, img_h=img_h,
            depth_img=depth_img,
            r_fix_inv=r_fix_inv,
        )
        det["confidence"] = float(conf)
        detections.append(det)

    return detections
