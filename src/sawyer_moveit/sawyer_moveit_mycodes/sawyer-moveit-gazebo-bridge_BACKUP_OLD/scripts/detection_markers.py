#!/usr/bin/env python3
"""
detection_markers.py
====================
Spawns coloured sphere models in Gazebo at the world positions estimated by
bbox_to_world.py.  Uses /gazebo/spawn_sdf_model — no RViz required.

Each call to publish_detections():
  1. Deletes any spheres from the previous call
  2. Spawns one sphere per detection at world_xyz

Usage — live node:
    from detection_markers import attach_marker_pub
    node = WorldPositionNode(model_path="best.pt", cam_pos=..., cam_quat=...)
    attach_marker_pub(node, frame_id="world")
    rospy.spin()

Usage — offline (after bbox_to_world.py writes detections_world.json):
    python detection_markers.py
"""

from typing import List
import numpy as np


# One RGB colour per class id  (0-1 floats)
_CLASS_COLORS = [
    (0.95, 0.70, 0.10),   # 0 mustard        — golden yellow
    (0.85, 0.15, 0.15),   # 1 coke_can        — red
    (0.20, 0.55, 0.95),   # 2 bowl            — blue
    (0.95, 0.88, 0.10),   # 3 banana          — bright yellow
    (0.95, 0.20, 0.45),   # 4 strawberry      — pink-red
    (0.30, 0.80, 0.35),   # 5 planta_maceta   — green
    (0.55, 0.30, 0.90),   # 6 esponja_lavap.  — purple
    (0.95, 0.55, 0.10),   # 7 papas_fritas    — orange
    (0.55, 0.35, 0.20),   # 8 block           — brown
]
_DEFAULT_COLOR = (0.70, 0.70, 0.70)


def _color(class_id: int):
    if 0 <= class_id < len(_CLASS_COLORS):
        return _CLASS_COLORS[class_id]
    return _DEFAULT_COLOR


def _sphere_sdf(radius: float, r: float, g: float, b: float) -> str:
    """Return a minimal SDF string for a semi-transparent sphere."""
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="detection_sphere">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} 0.7</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 0.7</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>{r*0.4:.3f} {g*0.4:.3f} {b*0.4:.3f} 0</emissive>
        </material>
        <transparency>0.3</transparency>
      </visual>
      <!-- No collision — ghost object, does not affect physics -->
    </link>
  </model>
</sdf>"""


class DetectionMarkerPub:
    """
    Spawns / refreshes Gazebo sphere markers at detected object positions.

    Parameters
    ----------
    frame_id   : reference frame for the poses (usually 'world')
    sphere_r   : sphere radius in metres
    prefix     : name prefix for spawned models in Gazebo
                 (models are named  <prefix>_0, <prefix>_1, ...)
    """

    def __init__(
        self,
        frame_id:  str   = "world",
        sphere_r:  float = 0.04,
        prefix:    str   = "det_sphere",
    ):
        import rospy
        from gazebo_msgs.srv import SpawnModel, DeleteModel

        self.frame_id = frame_id
        self.sphere_r = sphere_r
        self.prefix   = prefix
        self._active_names: List[str] = []   # models currently in Gazebo

        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10)
        rospy.wait_for_service("/gazebo/delete_model",    timeout=10)
        self._spawn  = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self._delete = rospy.ServiceProxy("/gazebo/delete_model",    DeleteModel)

        rospy.loginfo("[DetectionMarkers] Connected to Gazebo spawn/delete services.")

    # ── internal helpers ──────────────────────────────────────────────────

    def _delete_all(self):
        """Remove every sphere spawned in the previous cycle."""
        for name in self._active_names:
            try:
                self._delete(name)
            except Exception:
                pass   # already gone — ignore
        self._active_names.clear()

    def _spawn_sphere(self, name: str, xyz: np.ndarray, class_id: int):
        from gazebo_msgs.srv import SpawnModelRequest
        from geometry_msgs.msg import Pose

        r, g, b = _color(class_id)
        sdf = _sphere_sdf(self.sphere_r, r, g, b)

        pose               = Pose()
        pose.position.x    = float(xyz[0])
        pose.position.y    = float(xyz[1])
        pose.position.z    = float(xyz[2])
        pose.orientation.w = 1.0

        req                  = SpawnModelRequest()
        req.model_name       = name
        req.model_xml        = sdf
        req.reference_frame  = self.frame_id
        req.initial_pose     = pose

        resp = self._spawn(req)
        if not resp.success:
            import rospy
            rospy.logwarn(
                f"[DetectionMarkers] spawn failed for {name}: {resp.status_message}")

    # ── public API ────────────────────────────────────────────────────────

    def publish_detections(self, detections: List[dict], stamp=None):
        """
        Delete old spheres and spawn new ones for every detection.

        detections : list of dicts from results_to_world() / bbox_to_world()
                     must have keys: world_xyz, class_id, name, confidence
        stamp      : ignored (kept for API compatibility)
        """
        import rospy

        self._delete_all()

        for i, det in enumerate(detections):
            name = f"{self.prefix}_{i}"
            xyz  = np.asarray(det["world_xyz"], dtype=float)
            cid  = det.get("class_id", -1)
            conf = det.get("confidence", 1.0)

            self._spawn_sphere(name, xyz, cid)
            self._active_names.append(name)

            rospy.loginfo(
                f"[DetectionMarkers] spawned {name}  "
                f"({det.get('name', '?')}  conf={conf:.2f})  "
                f"@ ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})"
            )

    def clear(self):
        """Remove all detection spheres from Gazebo."""
        self._delete_all()


# ─────────────────────────────────────────────────────────────────────────────
#  Monkey-patch helper for WorldPositionNode
# ─────────────────────────────────────────────────────────────────────────────

def attach_marker_pub(node, **kwargs):
    """
    Patch a WorldPositionNode so every inference cycle also spawns Gazebo
    spheres.

    Usage:
        from detection_markers import attach_marker_pub
        node = WorldPositionNode(model_path=..., cam_pos=..., cam_quat=...)
        attach_marker_pub(node, frame_id="world", sphere_r=0.05)
        rospy.spin()
    """
    node.marker_pub = DetectionMarkerPub(**kwargs)
    original_rgb_cb = node._rgb_cb

    def patched_rgb_cb(msg):
        original_rgb_cb(msg)   # original logging still works

        import rospy
        try:
            with node._lock:
                K         = node._K
                depth_img = (node._depth_img.copy()
                             if node._depth_img is not None else None)
            if K is None:
                return
            frame   = node.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            results = node.model(frame, verbose=False)
            if not results:
                return
            from bbox_to_world import results_to_world
            detections = results_to_world(
                results[0],
                cam_pos   = node.cam_pos,
                cam_quat  = node.cam_quat,
                K         = K,
                depth_img = depth_img,
            )
            node.marker_pub.publish_detections(
                detections, stamp=msg.header.stamp)
        except Exception as e:
            rospy.logwarn(f"[DetectionMarkers] {e}")

    node._rgb_cb = patched_rgb_cb


# ─────────────────────────────────────────────────────────────────────────────
#  Offline entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import rospy, json

    rospy.init_node("detection_markers_offline")

    with open("detections_world.json") as f:
        raw = json.load(f)

    detections = []
    for d in raw:
        d["world_xyz"]  = np.array(d["world_xyz"])
        d["world_base"] = np.array(d["world_base"])
        detections.append(d)

    pub = DetectionMarkerPub(frame_id="world")
    pub.publish_detections(detections)

    rospy.loginfo("Spheres are now visible in Gazebo. Ctrl-C to exit (spheres stay).")
    rospy.spin()