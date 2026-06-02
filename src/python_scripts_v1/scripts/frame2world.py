from Bbox_to_world import results_to_world as _results_to_world
from Bbox_to_world import OBJECT_CLASSES


def get_positions(results, depth_img, K, cam_pos, cam_quat, r_fix_inv=None):
    return _results_to_world(
        results,
        cam_pos=cam_pos,
        cam_quat=cam_quat,
        K=K,
        depth_img=depth_img,
        r_fix_inv=r_fix_inv,
    )
